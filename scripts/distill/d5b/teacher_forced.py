"""D5b 诊断 (a)(b)(c)：教师强制回放。不改仓库，只读教师数据和权重。"""
import sys, json, numpy as np
WT = sys.argv[1]; weights = sys.argv[2]; out = sys.argv[3]; runs = sys.argv[4:]
sys.path.insert(0, WT)
from yopo_l0.distill import load_batch, convert_run
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_l0.student import load_checkpoint, predict, depth_from_hw, pack_ego_state, pack_neighbor_features
from yopo_l0.policy import StudentPolicy, PolicyContext, Observation, R_yaw
from yopo_l0.dynamics import State
model, meta = load_checkpoint(weights, enabled=True)
man = json.load(open('/workspace/yopo-teacher-data/d5/convert-manifest.json'))
report = {}
for rid in runs:
    rd = f'/workspace/yopo-teacher-data/d2b/runs/{rid}'
    b = load_batch(man['files'][rid]['path']) if rid in man['files'] else convert_run(rd)
    tr = np.load(f'{rd}/trajectory.npz')
    T, Pw, Vw, Aw, Yw, G = (tr[k] for k in ('time_s','position_m','velocity_mps','acceleration_mps2','yaw_rad','goal_m'))
    card = load_card(f'{rd}/scene.json')
    cam_cfg = CameraConfig.from_card({**card.card['camera'], 'noise_std_m': 0.02})
    cam = DepthCamera(cam_cfg, card.obstacles, np.random.default_rng(0))
    idx = np.arange(0, b.n, max(1, b.n // 60))
    rows = []
    for w in idx:
        i, d = int(b.sample_index[w]), int(b.ego_index[w])
        p, v, a, yaw, g, now = Pw[i, d], Vw[i, d], Aw[i, d], float(Yw[i, d]), G[d], float(T[i])
        # A: 训练张量原样
        rA = predict(model, b.depth[w], b.state[w], b.neighbor_rel[w], b.neighbor_mask[w])
        errA = float(np.sqrt(np.mean((rA.position_body[0] - b.position_body[w]) ** 2)))
        # B: 经闭环路径重建输入
        st = pack_ego_state(p, v, a, g, yaw)
        others = [j for j in range(Pw.shape[1]) if j != d]
        nf, nm = pack_neighbor_features(p, v, yaw, Pw[i, others] if others else np.zeros((0,3)), Vw[i, others] if others else np.zeros((0,3)))
        fr = cam.capture(p, yaw, now)
        dep = depth_from_hw(fr.depth, 20.0)
        rB = predict(model, dep, st, nf, nm)
        errB = float(np.sqrt(np.mean((rB.position_body[0] - b.position_body[w]) ** 2)))
        # C: 经 StudentPolicy.plan 得到计划，按时刻取 p/v 与教师比
        pol = StudentPolicy(PolicyContext(owner='uav', physics_dt=0.01, params={'student': {'weights': weights}}))
        if others:
            for j in others: pol.note_neighbor_odom(f'u{j}', {'p': Pw[i, j].tolist(), 'v': Vw[i, j].tolist()})
        obs = Observation(frame=fr, state=State(now, p.copy(), v.copy(), a.copy(), yaw), goal=g, now=now)
        plan = pol.plan(obs)
        cmp = {}
        for h in (0.1, 0.2, 0.5, 1.0, 1.6):
            if now + h > T[-1]: continue
            pt = np.array([np.interp(now + h, T, Pw[:, d, c]) for c in range(3)])
            vt = np.array([np.interp(now + h, T, Vw[:, d, c]) for c in range(3)])
            pp, vp, ap = plan.pva(now + h)
            cmp[h] = (float(np.linalg.norm(pp - pt)), float(np.linalg.norm(vp - vt)))
        # 段内速度峰值 vs 教师
        ts = now + np.linspace(0, 1.6, 81)
        vmax_plan = max(float(np.linalg.norm(plan.pva(t)[1])) for t in ts)
        rows.append(dict(w=int(w), t=now, d=d, errA=errA, errB=errB, st_diff=float(np.abs(st - b.state[w]).max()),
                         nb_diff=float(np.abs(nf - b.neighbor_rel[w]).max()),
                         depth_valid_train=float(b.depth[w][1].mean()), depth_valid_loop=float(dep[1].mean()),
                         depth_absdiff=float(np.abs(dep[0] - b.depth[w][0]).mean()),
                         out_diff=float(np.abs(rB.position_body[0] - rA.position_body[0]).max()),
                         code=pol.last_code, cmp={str(k): v for k, v in cmp.items()}, vmax_plan=vmax_plan,
                         vmax_teacher=float(np.linalg.norm(Vw[(T >= now) & (T <= now + 1.6), d], axis=1).max())))
    def agg(key):
        return float(np.sqrt(np.mean([r[key] ** 2 for r in rows])))
    summ = dict(n=len(rows), rmseA=agg('errA'), rmseB=agg('errB'), st_diff_max=max(r['st_diff'] for r in rows),
                nb_diff_max=max(r['nb_diff'] for r in rows), out_diff_max=max(r['out_diff'] for r in rows),
                depth_valid_train=float(np.mean([r['depth_valid_train'] for r in rows])),
                depth_valid_loop=float(np.mean([r['depth_valid_loop'] for r in rows])),
                depth_absdiff=float(np.mean([r['depth_absdiff'] for r in rows])),
                codes=sorted(set(r['code'] for r in rows)),
                vmax_plan_ratio_p95=float(np.percentile([r['vmax_plan'] / max(r['vmax_teacher'], 0.1) for r in rows], 95)))
    for h in ('0.1', '0.2', '0.5', '1.0', '1.6'):
        e = [r['cmp'][h] for r in rows if h in r['cmp']]
        if e: summ[f'plan_err@{h}s'] = dict(p_rmse=float(np.sqrt(np.mean([x[0]**2 for x in e]))), v_rmse=float(np.sqrt(np.mean([x[1]**2 for x in e]))))
    report[rid] = dict(summary=summ, rows=rows)
    print(rid, json.dumps(summ, ensure_ascii=False))
json.dump(report, open(out, 'w'), ensure_ascii=False)
