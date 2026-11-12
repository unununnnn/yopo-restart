#!/usr/bin/env bash
# 将一次已经完成的 regress 登记为已验 tree。
set -euo pipefail

[[ $# -eq 1 ]] || { printf '用法：scripts/record-verified-tree.sh <OUT 目录>\n' >&2; exit 2; }
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
# shellcheck source=scripts/premerge.sh
source "$SCRIPT_DIR/premerge.sh"
OUT=$(cd "$1" 2>/dev/null && pwd) || { printf 'OUT 目录不存在：%s\n' "$1" >&2; exit 1; }
cd "$ROOT"
[[ -z "$(git status --porcelain)" ]] || { printf '工作区不干净，拒绝登记（包括未跟踪文件）。\n' >&2; exit 1; }

HEAD_SHA=$(git rev-parse HEAD)
premerge_fetch_base "$ROOT" main
git rev-parse --verify --quiet refs/remotes/origin/main >/dev/null || {
  printf '找不到 origin/main，拒绝登记。\n' >&2
  exit 1
}
BASE_SHA=$(git rev-parse refs/remotes/origin/main)
TREE_SHA=$(git rev-parse 'HEAD^{tree}')
if ! git merge-base --is-ancestor "$BASE_SHA" "$HEAD_SHA"; then
  printf 'HEAD 未包含 origin/main，premerge 时合并后 tree 不同，缓存不会命中；先 git merge origin/main 再跑回归登记。\n' >&2
fi
LOG="$OUT/regress.log"
[[ -f "$LOG" ]] || {
  printf '缺少 %s；请保存 regress 的完整 stdout/stderr 为 OUT/regress.log。\n' "$LOG" >&2
  exit 1
}

PARSED=$(mktemp -d "${TMPDIR:-/tmp}/record-verified.XXXXXX")
trap 'rm -rf "$PARSED"' EXIT
premerge_split_regress "$LOG" "$PARSED"
read -r L0P L0S L0F L0E L0PARSED <<<"$(premerge_counts_file "$PARSED/l0.txt")"
read -r CP CS CF CE CPARSED <<<"$(premerge_counts_file "$PARSED/core.txt")"
FINAL=$(tail -n 20 "$LOG" | awk '/^pytest=(通过|失败) 批跑=(通过|失败) 输出 / {line=$0} END {print line}')
[[ "$FINAL" == "pytest=通过 批跑=通过 输出 "* ]] || { printf 'regress 最终结果不是全部通过或无法解析。\n' >&2; exit 1; }
[[ "$L0PARSED" == 1 && "$CPARSED" == 1 && "$L0F" == 0 && "$L0E" == 0 && "$CF" == 0 && "$CE" == 0 ]] || {
  printf 'pytest 日志无法解析或包含失败。\n' >&2
  exit 1
}
[[ -s "$PARSED/batch.md" && "$FINAL" == *"批跑=通过"* ]] || { printf '批跑摘要无法解析。\n' >&2; exit 1; }

# regress.sh 当前不把 git sha 写进 pytest 日志；至少要求 OUT 在当前提交之后更新。
COMMIT_TIME=$(git show -s --format=%ct HEAD)
OUT_TIME=$(stat -c %Y "$OUT")
[[ "$OUT_TIME" -ge "$COMMIT_TIME" ]] || { printf 'OUT 早于当前 HEAD，拒绝登记。\n' >&2; exit 1; }

FILE=${PREMERGE_VERIFIED_TREES:-/workspace/yopo-ops/verified-trees.tsv}
BRANCH=$(git symbolic-ref --short -q HEAD || printf 'detached')
L0=$(premerge_fmt_counts "$L0P" "$L0S" "$L0F" "$L0E" "$L0PARSED")
CORE=$(premerge_fmt_counts "$CP" "$CS" "$CF" "$CE" "$CPARSED")
BATCH=$(printf '%s' "$FINAL" | sed -n 's/^.*批跑=\([^ ]*\).*$/批跑\1/p')
premerge_append_verified "$FILE" "$TREE_SHA" \
  "$(TZ=Etc/GMT+3 date '+%Y-%m-%d %H:%M:%S UTC-3')" "$HEAD_SHA" "record ${BRANCH}" \
  "$L0" "$CORE" "$BATCH" "0"
printf '已登记 tree %s（HEAD %s，来源 %s）。\n' "$TREE_SHA" "$HEAD_SHA" "$BRANCH"
