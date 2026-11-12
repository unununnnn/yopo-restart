#!/usr/bin/env bash
# D5b 改训练：DART 重标注 → 转换（净空过滤）→ r4 训练（只学水平 + 加权）→ 评估 r4 与 r3+定高对照。每步完成标记，可续跑。
set -u
cd /workspace/yopo-ops/d5b
WT=/workspace/yopo-ops/d5b/snap-planar; PY=$WT/.venv/bin/python
export YOPO_WT=$WT PYTHONPATH=$WT:$WT/core/src
D=/workspace/yopo-teacher-data/d5b; M=$D/markers; mkdir -p $M
log(){ echo "$(date '+%F %T %z') $*"; }
step(){ local name=$1; shift; [ -f $M/$name ] && { log "skip $name"; return 0; }; log "start $name"; "$@" && touch $M/$name && log "done $name" || { log "FAIL $name"; exit 1; }; }
SP='{"knots":"position","yaw_source":"ego_lookahead","accel_source":"reference","vertical":"hold_goal_z"}'
relabel(){ for i in 0 1 2; do python3 relabel.py $D/jobs/dart.jsonl --container-index $i --stride 3 > $D/jobs/dart-relabel-$i.log 2>&1 & done; wait; }
step relabel-dart relabel
step convert-dart nice -n 10 $PY dagger_convert.py dart $D/windows-dagger/dart
step train-r4 nice -n 10 $PY train_r4.py $D/r4
step eval-r4 nice -n 10 $PY eval.py $D/r4/student.pt $D/eval/r4-planar repo "$SP"
step eval-r3-planar nice -n 10 $PY eval.py $D/r3/student.pt $D/eval/r3-planar repo "$SP"
log ALL-DONE
