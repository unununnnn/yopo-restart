#!/usr/bin/env bash
# D5b 编排：基础转换 → 基础训练 → 评估 → DAgger 三轮。每步有完成标记，可重入续跑。
set -u
cd /workspace/yopo-ops/d5b
WT=/workspace/yopo-ops/d5b/snap-9216b88; PY=$WT/.venv/bin/python
export YOPO_WT=$WT PYTHONPATH=$WT:$WT/core/src
D=/workspace/yopo-teacher-data/d5b; M=$D/markers; mkdir -p $M $D/rollouts $D/jobs
log(){ echo "$(date '+%F %T %z') $*"; }
step(){ local name=$1; shift; [ -f $M/$name ] && { log "skip $name"; return 0; }; log "start $name"; "$@" && touch $M/$name && log "done $name" || { log "FAIL $name"; exit 1; }; }
step convert-base nice -n 10 $PY convert_full.py
step eval-d5-fixed nice -n 10 $PY eval.py /workspace/yopo-teacher-data/d5/student.pt $D/eval/d5-fixed b
step train-r0 nice -n 10 $PY train.py $D/r0 ""
step eval-r0 nice -n 10 $PY eval.py $D/r0/student.pt $D/eval/r0 b
prev=r0; rounds=""
for k in 1 2 3; do
  r=r$k; W=$D/$prev/student.pt
  rollouts(){
    mkdir -p $D/rollouts/$r
    local seeds; case $k in 1) seeds="1 2 3 4 6";; 2) seeds="7 8 9 11 12";; 3) seeds="13 14 16 17 18";; esac
    for cell in d000-n1-parallel d000-n1-headon d000-n3-parallel d000-n3-headon d014-n1-parallel d014-n1-headon d014-n3-parallel d014-n3-headon; do
      for s in $seeds; do echo "$cell-s$s"; done; done | xargs -P 3 -I{} bash -c "[ -s $D/rollouts/$r/{}.json ] || nice -n 10 timeout 1200 $PY rollout.py --card /workspace/yopo-teacher-data/d2b/runs/{}/scene.json --seed 0 --weights $W --duration 15 --states --out $D/rollouts/$r/{}.json > $D/rollouts/$r/{}.log 2>&1"
    ls $D/rollouts/$r/*.json >/dev/null
  }
  relabel(){
    python3 gen_jobs.py $r $D/rollouts/$r $D/jobs/$r.jsonl || return 1
    for i in 0 1 2; do python3 relabel.py $D/jobs/$r.jsonl --container-index $i --stride 3 > $D/jobs/$r-relabel-$i.log 2>&1 & done; wait
  }
  step rollouts-$r rollouts
  step relabel-$r relabel
  step convert-$r nice -n 10 $PY dagger_convert.py $r $D/windows-dagger/$r
  rounds=${rounds:+$rounds,}$r
  step train-$r nice -n 10 $PY train.py $D/$r "$rounds"
  step eval-$r nice -n 10 $PY eval.py $D/$r/student.pt $D/eval/$r b
  prev=$r
done
log ALL-DONE
