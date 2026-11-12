#!/usr/bin/env bash
# 票 S1b：同一任务下默认定位与 RTK 各跑 2 次，打印对照表（report-only，不冻结判据）。
# 只开 1 个 PX4 实例，串行跑。用法：scripts/sitl/compare.sh [对照表路径]
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"
mkdir -p "$SITL_STATE/runs"
OUT="${1:-$SITL_STATE/smoke-compare.md}"
LIST="$SITL_STATE/smoke-compare.runs"
: > "$LIST"
n_fail=0
i_default=0
i_rtk=0
for mode in default default rtk rtk; do
  if [[ "$mode" == default ]]; then i_default=$((i_default + 1)); idx=$i_default
  else i_rtk=$((i_rtk + 1)); idx=$i_rtk; fi
  echo "=== compare positioning=$mode run=$idx ==="
  set +e
  log=$("$SITL_DIR/smoke.sh" --positioning "$mode")
  rc=$?
  set -e
  printf '%s\n' "$log"
  run=$(printf '%s\n' "$log" | awk '/^run_dir /{print $2; exit}')
  if [[ -z "${run:-}" || ! -f "$run/result.json" ]]; then
    echo "冒烟没有 result.json（rc=$rc）" >&2
    n_fail=$((n_fail + 1))
    continue
  fi
  printf '%s\t%s\t%s\n' "$run" "$mode" "$idx" >> "$LIST"
  # 以 result.json 的 ok 为准。任务完成后 python/gz 拆卸偶发 SIGABRT（rc=134）不算失败。
  ok=$(python3 -c 'import json,sys; print("1" if json.load(open(sys.argv[1])).get("ok") else "0")' "$run/result.json")
  if [[ "$ok" != 1 ]]; then
    echo "冒烟任务未完成 rc=$rc $run" >&2
    n_fail=$((n_fail + 1))
  elif [[ $rc -ne 0 ]]; then
    echo "冒烟任务成功但进程 rc=$rc（拆卸），仍计为通过 $run"
  fi
done
python3 - "$SITL_DIR" "$LIST" "$OUT" << 'PY'
import importlib.util
import json
import sys
from pathlib import Path

sitl, list_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
spec = importlib.util.spec_from_file_location("sitl_summarize", Path(sitl) / "summarize.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
rows = []
for line in Path(list_path).read_text().splitlines():
    if not line.strip():
        continue
    run, mode, idx = line.split("\t")
    res = json.loads((Path(run) / "result.json").read_text())
    rows.append({
        "positioning": res.get("positioning") or mode,
        "run": idx,
        "ok": res.get("ok"),
        "phase": res.get("phase"),
        "metrics": res.get("metrics") or {},
    })
text = mod.render_compare(rows)
Path(out_path).write_text(text)
print(text, end="")
PY
echo "compare_out $OUT fail_runs=$n_fail"
# 对照表写出来就算完成；有失败次数时以非零退出，方便收口脚本决定要不要开 PR。
exit $((n_fail > 0 ? 1 : 0))
