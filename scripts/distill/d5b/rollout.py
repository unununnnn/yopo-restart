#!/usr/bin/env python3
"""D5b：用修正后的学生策略在 L0 里飞一场。--states 时逐步记下每架的 (t, p, v, a_ref, yaw)，供 DAgger 重标注。"""
import argparse, json, os, sys, time, shutil, tempfile
from pathlib import Path
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument("--wt", default=os.environ.get("YOPO_WT", "/workspace/wt/yopo-restart/yopo-restart-bot-d5-student-loop"))
ap.add_argument("--card", required=True); ap.add_argument("--seed", type=int, required=True)
ap.add_argument("--weights", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--duration", type=float, default=None); ap.add_argument("--states", action="store_true")
ap.add_argument("--policy", default="student"); ap.add_argument("--variant", default="b", choices=["b", "d5", "repo"])
ap.add_argument("--student-params", default=os.environ.get("D5B_STUDENT_PARAMS", "{}"))
a = ap.parse_args()
sys.path.insert(0, a.wt); sys.path.insert(0, "/workspace/yopo-ops/d5b")
os.environ["YOPO_STUDENT_WEIGHTS"] = a.weights
import torch; torch.set_num_threads(1)
if a.variant == "b" and a.policy == "student":
    import student_fix; student_fix.install()
import yopo_l0.policy as P
if a.variant == "repo":
    P.POLICIES["student"].DEFAULTS = {**P.POLICIES["student"].DEFAULTS, **json.loads(a.student_params)}
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card
card_path = a.card
tmpdir = None
if a.duration is not None:
    c = json.loads(Path(a.card).read_text()); c["task"]["duration_s"] = float(a.duration)
    tmpdir = tempfile.mkdtemp(prefix="d5b-card-", dir="/workspace/yopo-ops/d5b/tmp")
    geo = Path(a.card).parent / c["geometry"]; shutil.copy(geo, Path(tmpdir) / Path(c["geometry"]).name)
    c["geometry"] = Path(c["geometry"]).name
    card_path = str(Path(tmpdir) / "card.json"); Path(card_path).write_text(json.dumps(c))
states = []
if a.states:
    cls = P.POLICIES["student"]; orig = cls.plan
    def plan(self, obs):
        r = orig(self, obs)
        s = obs.state
        a_ref = np.asarray(self.plan_obj.pva(obs.now)[2], float) if self.plan_obj is not None else np.zeros(3)
        states.append({"id": self.ctx.owner, "t": float(obs.now), "p": s.p.tolist(), "v": s.v.tolist(), "a": a_ref.tolist(),
                       "yaw": float(s.yaw), "goal": np.asarray(obs.goal, float).tolist(), "code": self.last_code})
        return r
    cls.plan = plan
t0 = time.perf_counter()
o = run(load_card(card_path), RunConfig(seed=a.seed, policy=a.policy, shadow=True))
res = o.result
keep = ("outcome", "reason", "arrived", "n_drones", "arrival_time_s", "min_obstacle_distance_m", "min_inter_drone_distance_m",
        "final_goal_error_m", "student_infer", "network", "scene_card_hash", "sim_duration_s")
row = {k: res.get(k) for k in keep}
row.update(card=a.card, seed=a.seed, policy=a.policy, variant=a.variant, student_params=json.loads(a.student_params), weights=a.weights, duration_override=a.duration, worker_s=time.perf_counter() - t0)
if a.states: row["states"] = states
Path(a.out + ".tmp").write_text(json.dumps(row)); os.replace(a.out + ".tmp", a.out)
if tmpdir: shutil.rmtree(tmpdir, ignore_errors=True)
print(row["outcome"], row["reason"], f"{row['arrived']}/{row['n_drones']}", row["final_goal_error_m"])
