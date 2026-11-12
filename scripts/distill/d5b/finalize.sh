#!/usr/bin/env bash
# D5b 收口：等 planar.sh 结束 → r3 留出 x/y → 生成文档 → 合 origin/main → 回归 → 提交/推送 → gh pr create → 入合并队列。
# 每步有完成标记，可重入续跑。
set -u
cd /workspace/yopo-ops/d5b
WT=/workspace/wt/yopo-restart/yopo-restart-bot-d5b-planar; BR=yopo-restart-bot/d5b-planar
D=/workspace/yopo-teacher-data/d5b; M=$D/markers-final; mkdir -p $M
PY=/workspace/yopo-ops/d5b/snap-planar/.venv/bin/python
log(){ echo "$(TZ=America/Argentina/San_Juan date '+%F %T UTC-3') $*"; }
step(){ local name=$1; shift; [ -f $M/$name ] && { log "skip $name"; return 0; }; log "start $name"; "$@" && touch $M/$name && log "done $name" || { log "FAIL $name"; exit 1; }; }
P=$(cat planar.pid); while kill -0 $P 2>/dev/null; do sleep 30; done
grep -q ALL-DONE planar.log || { log "planar.sh 未完成（无 ALL-DONE），停"; exit 1; }
step holdout-r3 nice -n 10 $PY holdout_xy.py $D/r3/student.pt $D/r3/holdout-xy.json
step doc python3 make_doc.py $WT
merge(){ cd $WT && git fetch -q origin && git merge --no-edit origin/main; }
step merge merge
regress(){ cd $WT && nice -n 10 scripts/regress.sh > /workspace/yopo-ops/d5b/regress.log 2>&1; }
step regress regress
commit(){
  cd $WT && cp /workspace/yopo-ops/d5b/{make_doc.py,holdout_xy.py,finalize.sh} scripts/distill/d5b/ && git add -A yopo_l0 tests scripts/distill/d5b docs/research/d5b-student-fix.md docs/research/d5b-student-fix-runs.json \
  && git -c user.name=unununnnn -c user.email=107021589+unununnnn@users.noreply.github.com commit -q \
     -m "票 D5b：学生闭环接口修正、DAgger 诊断与改训练（低维动作 + DART + 加权，默认关，report-only）" \
     --author "unununnnn <107021589+unununnnn@users.noreply.github.com>"
}
step commit commit
step push bash -c "cd $WT && git push -q -u origin $BR"
pr(){
  cd $WT
  n=$(gh pr list -R unununnnn/yopo-restart --head $BR --state open --json number -q '.[0].number')
  if [ -z "$n" ]; then
    body=$(printf '票 D5b（report-only，新选项默认关，关时逐位不变）。\n\n详见 docs/research/d5b-student-fix.md：接口诊断、全量 D2b 成功场数据与按格用量、DAgger r0–r3 失败原因（数字）、改训练（vertical=hold_goal_z 低维动作 + DART 扰动重标注 + 安全加权）与四场景评测、教师在密集格不可靠。\n\n回归：scripts/regress.sh 通过（日志 /workspace/yopo-ops/d5b/regress.log）。权重在仓库外 /workspace/yopo-teacher-data/d5b/。')
    gh pr create -R unununnnn/yopo-restart --base main --head $BR --title "票 D5b：学生闭环接口修正与改训练（默认关，report-only）" --body "$body" || return 1
    n=$(gh pr list -R unununnnn/yopo-restart --head $BR --state open --json number -q '.[0].number')
  fi
  echo $n > /workspace/yopo-ops/d5b/pr.txt; log "PR $n"
}
step pr pr
step queue bash -c "/workspace/yopo-ops/merge-queue.sh add \$(cat /workspace/yopo-ops/d5b/pr.txt)"
/workspace/yopo-ops/merge-queue.sh status
log FINAL-DONE
