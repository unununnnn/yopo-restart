import json, sys, numpy as np
S = '/workspace/yopo-ops/d5b/snap-9216b88'; sys.path.insert(0, S)
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_l0.student import load_checkpoint, predict, depth_from_hw, pack_ego_state, pack_neighbor_features
from yopo_l0.policy import R_yaw
R = '/workspace/yopo-teacher-data/d5b/relabel/runs/'
out = {}
for f, card, j in [('open3-r3', 'open-3drone', 'diag-open3-r3-t1.0'), ('swap-3-open-r3', 'swap-3-open', 'diag-swap-3-open-r3-t1.4'),
                   ('ego-density-014-solo-r3', 'ego-density-014-solo', 'diag-ego-density-014-solo-r3-t0.8')]:
    x = json.load(open(f'tmp/{f}.json')); who = x['reason'].split(':')[0]
    job = json.loads(open(R + j + '/scenario.json').read())
    t = job['dagger']['t_student_s']
    ids = sorted(set(s['id'] for s in x['states'])); row = {s['id']: s for s in x['states'] if abs(s['t'] - t) < 1e-6}
    c = load_card(f'{S}/scenes/{card}.json')
    cam = DepthCamera(CameraConfig.from_card(c.card['camera']), c.obstacles, np.random.default_rng(0))
    for wname in ['r0', 'r3']:
        model, _ = load_checkpoint(f'/workspace/yopo-teacher-data/d5b/{wname}/student.pt')
        s = row[who]; p = np.array(s['p']); v = np.array(s['v']); a = np.array(s['a']); yaw = s['yaw']
        others = [k for k in ids if k != who]
        nf, nm = pack_neighbor_features(p, v, yaw, np.array([row[k]['p'] for k in others]).reshape(-1, 3), np.array([row[k]['v'] for k in others]).reshape(-1, 3))
        dep = depth_from_hw(cam.capture(p, yaw, t).depth, 20.0)
        pr = predict(model, dep, pack_ego_state(p, v, a, s['goal'], yaw), nf, nm).position_body[0]
        tr = np.load(R + j + '/trajectory.npz'); T = tr['time_s']; P = tr['position_m'][:, ids.index(who)]
        kt = 0.2 * np.arange(1, 9)
        pt = np.stack([np.interp(kt, T, P[:, k]) for k in range(3)], 1)
        tb = (R_yaw(yaw).T @ (pt - P[0]).T).T
        err = np.linalg.norm(pr - tb, axis=1)
        out[f + '/' + wname] = dict(t=t, z=float(p[2]), vz=float(v[2]), net_z_1p6=float(pr[-1, 2]), teacher_z_1p6=float(tb[-1, 2]),
                                    err_0p2=float(err[0]), err_1p6=float(err[-1]), net_xy_1p6=pr[-1, :2].round(2).tolist(), teacher_xy_1p6=tb[-1, :2].round(2).tolist())
        print(f, wname, json.dumps(out[f + '/' + wname]))
json.dump(out, open('diag-collide.json', 'w'), indent=1)
