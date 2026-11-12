#!/usr/bin/env python3
"""r3 权重在留出集上的 x/y 位置 RMSE，和 r4 同口径。用法 holdout_xy.py WEIGHTS OUT.json"""
import json, sys
sys.path.insert(0, "/workspace/yopo-ops/d5b/snap-planar")
from pathlib import Path
from yopo_l0.student import load_checkpoint
from yopo_l0.student_fit import evaluate_paths
m = json.loads(Path("/workspace/yopo-teacher-data/d5b/convert-base-manifest.json").read_text())
model, _ = load_checkpoint(sys.argv[1], enabled=True)
r = evaluate_paths(model, [Path(m["files"][i]["path"]) for i in m["hold_ids"]], batch_size=256)
Path(sys.argv[2]).write_text(json.dumps(r)); print(r)
