#!/usr/bin/env python3
"""D5b 训练：基础教师窗口（全量成功场）+ 指定轮次的 DAgger 窗口。用法 train.py OUT ROUNDS(逗号分隔，可空)"""
import json, os, sys
from pathlib import Path
WT = os.environ.get("YOPO_WT", "/workspace/wt/yopo-restart/yopo-restart-bot-d5-student-loop")
sys.path.insert(0, WT)
from yopo_l0.student_fit import FitConfig, train_from_paths
D = Path("/workspace/yopo-teacher-data/d5b")
out = Path(sys.argv[1]); rounds = [r for r in (sys.argv[2] if len(sys.argv) > 2 else "").split(",") if r]
man = json.loads((D / "convert-base-manifest.json").read_text())
train = [Path(man["files"][i]["path"]) for i in man["train_ids"]]
hold = [Path(man["files"][i]["path"]) for i in man["hold_ids"]]
extra = []
for r in rounds:
    dm = json.loads((D / "windows-dagger" / r / "manifest.json").read_text())
    extra += [Path(v["path"]) for v in dm["files"].values()]
summary = train_from_paths(train + extra, hold, out, cfg=FitConfig(num_threads=2))
summary["data"] = {"base_train_runs": len(train), "hold_runs": len(hold), "dagger_rounds": rounds, "dagger_files": len(extra)}
(out / "d5b-train.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
print(json.dumps({"weights_sha256": summary.get("weights_sha256"), "holdout": summary.get("holdout")}, ensure_ascii=False))
