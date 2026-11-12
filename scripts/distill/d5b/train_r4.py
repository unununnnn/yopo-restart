#!/usr/bin/env python3
"""D5b r4：低维动作（只学水平）+ DART 扰动重标注 + 安全加权。基础成功场 + DAgger r1–r3 + DART 窗口。
用法 train_r4.py OUT"""
import json, os, sys
from pathlib import Path
WT = os.environ.get("YOPO_WT", "/workspace/yopo-ops/d5b/snap-planar")
sys.path.insert(0, WT)
from yopo_l0.student_fit import FitConfig, train_from_paths
D = Path("/workspace/yopo-teacher-data/d5b")
out = Path(sys.argv[1])
man = json.loads((D / "convert-base-manifest.json").read_text())
train = [Path(man["files"][i]["path"]) for i in man["train_ids"]]
hold = [Path(man["files"][i]["path"]) for i in man["hold_ids"]]
extra = {}
for r in ("r1", "r2", "r3", "dart"):
    dm = json.loads((D / "windows-dagger" / r / "manifest.json").read_text())
    extra[r] = [Path(v["path"]) for v in dm["files"].values()]
EXTRA_W = 4.0
pw = {str(p): EXTRA_W for ps in extra.values() for p in ps}
cfg = FitConfig(num_threads=2, planar=True, near_weight=3.0, near_depth_m=1.2, shuffle_batches=64)
summary = train_from_paths(train + [p for ps in extra.values() for p in ps], hold, out, cfg=cfg, path_weights=pw)
summary["data"] = {"base_train_runs": len(train), "hold_runs": len(hold), "extra_files": {k: len(v) for k, v in extra.items()},
                   "extra_weight": EXTRA_W, "cfg": {"planar": True, "near_weight": 3.0, "near_depth_m": 1.2, "shuffle_batches": 64}}
(out / "d5b-train.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
print(json.dumps({"weights_sha256": summary.get("weights_sha256"), "holdout": summary.get("holdout")}, ensure_ascii=False))
