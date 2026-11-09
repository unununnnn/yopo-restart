#!/usr/bin/env bash
# 合 main 前的本机门。不接 GitHub Actions。
# 用法：scripts/premerge.sh <PR号> [--merge|--squash]
#
# 1. gh pr view 取 head sha 与 mergeable，不是 MERGEABLE 就立刻失败（不建 worktree）。
# 2. 在临时目录建 PR head 的 git worktree。
# 3. 在 worktree 里 make venv（共享依赖 venv + uv 缓存）并跑 scripts/regress.sh。
# 4. 解析 L0 / core 的 passed、skipped，以及批跑 Markdown 表。
# 5. 在 PR 上留一条评论：head sha、计数、退出码、批跑表。
# 6. 检查 PR 相对基线的每个提交都没有 Co-authored-by trailer，并写进评论。
# 7. 任一检查失败则退出码非 0。
#
# 不带 --merge / --squash 时绝不合并。不会推送分支，也不会强行改写远端历史。
# --merge / --squash 只在全部检查通过之后执行：
#   gh pr ready
#   gh pr merge --merge|--squash --match-head-commit <sha>
# --squash 的说明由各提交说明拼成，并删掉其中的 Co-authored-by 行。
# 结束时删除 worktree。
#
# 测试可设 PREMERGE_TEST=1 与 PREMERGE_ROOT=<仓库>，把 git 操作指到临时仓库。
# 不跳过 regress：worktree 里跑的就是该提交的 scripts/regress.sh。

premerge_strip_coauthor_lines() {
  # 删掉整行都是 Co-authored-by trailer 的行（大小写不敏感）。正文里顺带提到的不删。
  local line
  while IFS= read -r line || [[ -n "$line" ]]; do
    local low
    low=$(printf '%s' "$line" | tr '[:upper:]' '[:lower:]')
    if [[ "$low" =~ ^[[:space:]]*co-authored-by[[:space:]]*: ]]; then
      continue
    fi
    printf '%s\n' "$line"
  done
}

premerge_squash_body() {
  # $1 仓库 $2 git revision range（base..head）
  local repo=$1 range=$2
  git -C "$repo" log --reverse --format='%B' "$range" | premerge_strip_coauthor_lines
}

premerge_coauthor_lines() {
  # $1 仓库 $2 range。每行：<sha><TAB>Co-authored-by: ...
  # 只用 git trailer 块，正文中部的同名句子不算。
  local repo=$1 range=$2
  git -C "$repo" log --format='%H%n%(trailers:key=Co-authored-by)' "$range" \
    | awk '
      /^[0-9a-f]{40}$/ { sha=$0; next }
      {
        low=tolower($0)
        if (low ~ /^co-authored-by:/) print sha "\t" $0
      }
    '
}

premerge_last_summary() {
  local line last=""
  while IFS= read -r line || [[ -n "$line" ]]; do
    if [[ "$line" =~ [0-9]+[[:space:]]+(passed|failed|skipped|errors?) ]]; then
      last=$line
    fi
  done
  printf '%s' "$last"
}

premerge_line_count() {
  # $1 摘要行 $2 passed|skipped|failed|errors
  local line=$1 word=$2
  local re
  if [[ "$word" == "errors" ]]; then
    re='([0-9]+)[[:space:]]+errors?'
  else
    re="([0-9]+)[[:space:]]+${word}"
  fi
  if [[ "$line" =~ $re ]]; then
    printf '%s' "${BASH_REMATCH[1]}"
  else
    printf '0'
  fi
}

premerge_split_regress() {
  # $1 regress 日志 $2 输出目录。写出 l0.txt core.txt batch.md。
  local src=$1 dest=$2
  mkdir -p "$dest"
  awk -v dest="$dest" '
    BEGIN { sec = "" }
    {
      if ($0 ~ /^## pytest（L0/) { sec = "l0"; next }
      if ($0 ~ /^## pytest（core/) { sec = "core"; next }
      if ($0 ~ /^## 批跑/) { sec = "batch" }
      if ($0 ~ /^pytest=/) { sec = "end"; next }
      if (sec == "l0") l0 = l0 $0 "\n"
      else if (sec == "core") core = core $0 "\n"
      else if (sec == "batch") batch = batch $0 "\n"
    }
    END {
      printf "%s", l0 > (dest "/l0.txt")
      printf "%s", core > (dest "/core.txt")
      printf "%s", batch > (dest "/batch.md")
    }
  ' "$src"
}

premerge_counts_file() {
  # 读一个 pytest 片段，打印：passed skipped failed errors parsed
  local file=$1
  local text line parsed=0 passed=0 skipped=0 failed=0 errors=0
  text=$(cat "$file")
  line=$(printf '%s\n' "$text" | premerge_last_summary)
  if [[ -n "$line" ]]; then
    parsed=1
    passed=$(premerge_line_count "$line" passed)
    skipped=$(premerge_line_count "$line" skipped)
    failed=$(premerge_line_count "$line" failed)
    errors=$(premerge_line_count "$line" errors)
  fi
  printf '%s %s %s %s %s' "$passed" "$skipped" "$failed" "$errors" "$parsed"
}

premerge_fmt_counts() {
  # $1 passed $2 skipped $3 failed $4 errors $5 parsed
  local passed=$1 skipped=$2 failed=$3 errors=$4 parsed=$5
  if [[ "$parsed" != 1 ]]; then
    printf '未能解析'
    return
  fi
  local out="${passed} passed, ${skipped} skipped"
  if [[ "$failed" != 0 ]]; then
    out="${failed} failed, ${out}"
  fi
  if [[ "$errors" != 0 ]]; then
    out="${errors} errors, ${out}"
  fi
  printf '%s' "$out"
}

premerge_usage() {
  cat <<'EOF'
用法：scripts/premerge.sh <PR号> [--merge|--squash]

在临时 worktree 里跑 scripts/regress.sh，把 head sha、L0/core 的
passed/skipped、退出码和批跑表写到 PR 评论。检查每个提交都没有
Co-authored-by trailer。

  --merge   全部通过后：gh pr ready && gh pr merge --merge --match-head-commit <sha>
  --squash  全部通过后：gh pr ready && gh pr merge --squash --match-head-commit <sha>
            提交说明由各提交拼成，并去掉 Co-authored-by 行

不带这两个参数时只检查、不合并。不会推送分支。
EOF
}

premerge_write_view() {
  local json_file=$1 dest=$2
  python3 - "$json_file" "$dest" <<'PY'
import json, pathlib, sys
src, dest = sys.argv[1], pathlib.Path(sys.argv[2])
data = json.loads(pathlib.Path(src).read_text())
dest.mkdir(parents=True, exist_ok=True)
keys = ("headRefOid", "mergeable", "baseRefName", "headRefName", "isDraft", "title")
missing = [k for k in keys if k not in data]
if missing:
    sys.stderr.write("gh pr view 缺少字段: %s\n" % ", ".join(missing))
    sys.exit(1)
for k in keys:
    v = data[k]
    if isinstance(v, bool):
        text = "true" if v else "false"
    elif v is None:
        text = ""
    else:
        text = str(v)
    (dest / k).write_text(text)
PY
}

premerge_ready() {
  local pr=$1 err rc
  set +e
  err=$(gh pr ready "$pr" 2>&1)
  rc=$?
  set -e
  if [[ "$rc" -eq 0 ]]; then
    return 0
  fi
  if [[ "$err" =~ [Aa]lready ]]; then
    return 0
  fi
  printf '%s\n' "$err" >&2
  return 1
}

premerge_do_merge() {
  local pr=$1 mode=$2 sha=$3 title=$4 body_file=$5
  premerge_ready "$pr"
  if [[ "$mode" == "merge" ]]; then
    gh pr merge "$pr" --merge --match-head-commit "$sha"
  else
    gh pr merge "$pr" --squash --match-head-commit "$sha" --subject "$title" --body-file "$body_file"
  fi
}

premerge_cleanup() {
  if [[ -n "${ROOT:-}" && -n "${WT:-}" ]]; then
    git -C "$ROOT" worktree remove --force "$WT" >/dev/null 2>&1 || true
  fi
  if [[ -n "${TMP:-}" && -d "${TMP:-}" ]]; then
    case "$TMP" in
      /tmp/premerge.*|"${TMPDIR:-/tmp}"/premerge.*)
        rm -rf "$TMP" || true
        ;;
      *)
        printf '拒绝删除意外路径 %s\n' "$TMP" >&2
        ;;
    esac
  fi
  if [[ -n "${ROOT:-}" ]]; then
    git -C "$ROOT" worktree prune >/dev/null 2>&1 || true
  fi
}

premerge_main() {
  set -euo pipefail
  local pr="" mode="" arg
  while [[ $# -gt 0 ]]; do
    arg=$1
    shift
    case "$arg" in
      -h|--help)
        premerge_usage
        return 0
        ;;
      --merge)
        [[ -z "$mode" ]] || { printf '只能选 --merge 或 --squash 之一\n' >&2; return 2; }
        mode=merge
        ;;
      --squash)
        [[ -z "$mode" ]] || { printf '只能选 --merge 或 --squash 之一\n' >&2; return 2; }
        mode=squash
        ;;
      --)
        break
        ;;
      -*)
        printf '未知参数 %s\n' "$arg" >&2
        premerge_usage >&2
        return 2
        ;;
      *)
        [[ -z "$pr" ]] || { printf '多余参数 %s\n' "$arg" >&2; return 2; }
        pr=$arg
        ;;
    esac
  done
  if [[ $# -gt 0 ]]; then
    printf '多余参数 %s\n' "$1" >&2
    return 2
  fi
  [[ "$pr" =~ ^[0-9]+$ ]] || { printf '需要 PR 号\n' >&2; premerge_usage >&2; return 2; }

  command -v gh >/dev/null 2>&1 || { printf '找不到 gh\n' >&2; return 1; }
  command -v git >/dev/null 2>&1 || { printf '找不到 git\n' >&2; return 1; }
  command -v python3 >/dev/null 2>&1 || { printf '找不到 python3\n' >&2; return 1; }
  command -v make >/dev/null 2>&1 || { printf '找不到 make\n' >&2; return 1; }
  if ! command -v uv >/dev/null 2>&1 && [[ -x "${HOME}/.local/bin/uv" ]]; then
    export PATH="${HOME}/.local/bin:${PATH}"
  fi

  local script_dir root_default
  script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  root_default=$(cd "$script_dir/.." && pwd)
  if [[ "${PREMERGE_TEST:-}" == 1 ]]; then
    [[ -n "${PREMERGE_ROOT:-}" ]] || { printf 'PREMERGE_TEST=1 需要 PREMERGE_ROOT\n' >&2; return 2; }
    ROOT=$(cd "$PREMERGE_ROOT" && pwd)
  else
    ROOT=$root_default
  fi
  cd "$ROOT"

  export UV_CACHE_DIR="${UV_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/uv}"
  export YOPO_DEPS_ROOT="${YOPO_DEPS_ROOT:-${XDG_CACHE_HOME:-$HOME/.cache}/yopo-restart/deps}"

  TMP=$(mktemp -d "${TMPDIR:-/tmp}/premerge.XXXXXX")
  WT=""
  trap premerge_cleanup EXIT

  local view_json="$TMP/view.json"
  gh pr view "$pr" --json headRefOid,mergeable,baseRefName,headRefName,isDraft,title >"$view_json"
  premerge_write_view "$view_json" "$TMP/view"

  local sha mergeable base_name title
  sha=$(cat "$TMP/view/headRefOid")
  mergeable=$(cat "$TMP/view/mergeable")
  base_name=$(cat "$TMP/view/baseRefName")
  title=$(tr '\n' ' ' <"$TMP/view/title" | sed 's/[[:space:]]*$//')
  [[ -n "$sha" && "$sha" =~ ^[0-9a-fA-F]{40}$ ]] || { printf 'head sha 无效：%s\n' "$sha" >&2; return 1; }

  if [[ "$mergeable" != "MERGEABLE" ]]; then
    printf 'PR #%s 不可合并：mergeable=%s（需要 MERGEABLE）。未建 worktree。\n' "$pr" "${mergeable:-空}" >&2
    return 1
  fi

  if ! git cat-file -e "${sha}^{commit}" 2>/dev/null; then
    git fetch --no-tags origin "pull/${pr}/head" || { printf 'fetch pull/%s/head 失败\n' "$pr" >&2; return 1; }
  fi
  git cat-file -e "${sha}^{commit}" 2>/dev/null || { printf '仓库里没有 commit %s\n' "$sha" >&2; return 1; }

  local baseref
  if git rev-parse --verify --quiet "refs/remotes/origin/${base_name}" >/dev/null; then
    baseref="origin/${base_name}"
  elif [[ "${PREMERGE_TEST:-}" == 1 ]] && git rev-parse --verify --quiet "refs/heads/${base_name}" >/dev/null; then
    baseref="refs/heads/${base_name}"
  else
    git fetch --no-tags origin "$base_name" || { printf 'fetch %s 失败\n' "$base_name" >&2; return 1; }
    baseref="origin/${base_name}"
  fi

  local range
  range="${baseref}..${sha}"

  WT="$TMP/wt"
  git worktree add --detach "$WT" "$sha" >&2
  printf 'worktree %s @ %s\n' "$WT" "$sha" >&2

  local regress_log="$TMP/regress.log" regress_rc
  set +e
  (
    set -euo pipefail
    cd "$WT"
    make venv
    scripts/regress.sh
  ) 2>&1 | tee "$regress_log"
  regress_rc=${PIPESTATUS[0]}
  set -e

  premerge_split_regress "$regress_log" "$TMP/parsed"
  local l0_line core_line
  l0_line=$(premerge_counts_file "$TMP/parsed/l0.txt")
  core_line=$(premerge_counts_file "$TMP/parsed/core.txt")
  # shellcheck disable=SC2086
  local l0p l0s l0f l0e l0parsed cp cs cf ce cparsed
  read -r l0p l0s l0f l0e l0parsed <<<"$l0_line"
  read -r cp cs cf ce cparsed <<<"$core_line"
  local batch_ok=0
  if grep -q '| seed |' "$TMP/parsed/batch.md"; then
    batch_ok=1
  fi

  local co_rc coauthors ncommits
  set +e
  coauthors=$(premerge_coauthor_lines "$ROOT" "$range")
  co_rc=$?
  ncommits=$(git rev-list --count "$range")
  set -e
  [[ -n "${ncommits:-}" ]] || ncommits=0

  local gate=0
  if [[ "$regress_rc" -ne 0 ]]; then gate=1; fi
  if [[ "$l0parsed" != 1 || "$cparsed" != 1 ]]; then gate=1; fi
  if [[ "$l0f" != 0 || "$cf" != 0 || "$l0e" != 0 || "$ce" != 0 ]]; then gate=1; fi
  if [[ "$batch_ok" != 1 ]]; then gate=1; fi
  if [[ "$co_rc" -ne 0 ]]; then gate=1; fi
  if [[ -n "$coauthors" ]]; then gate=1; fi

  local merge_note="未请求" final=$gate body_file="$TMP/squash.md" merge_rc=0
  if [[ -n "$mode" ]]; then
    if [[ "$gate" -ne 0 ]]; then
      merge_note="未执行（检查未过）"
    else
      if [[ "$mode" == "squash" ]]; then
        premerge_squash_body "$ROOT" "$range" >"$body_file"
        if [[ ! -s "$body_file" ]]; then
          printf '%s\n' "（无提交说明）" >"$body_file"
        fi
        if [[ -z "$title" ]]; then
          title="PR #${pr}"
        fi
      fi
      set +e
      premerge_do_merge "$pr" "$mode" "$sha" "$title" "$body_file"
      merge_rc=$?
      set -e
      if [[ "$merge_rc" -eq 0 ]]; then
        if [[ "$mode" == "merge" ]]; then
          merge_note="已合并（merge）"
        else
          merge_note="已合并（squash）"
        fi
      else
        merge_note="合并失败"
        final=1
      fi
    fi
  fi

  local l0_txt core_txt co_txt
  l0_txt=$(premerge_fmt_counts "$l0p" "$l0s" "$l0f" "$l0e" "$l0parsed")
  core_txt=$(premerge_fmt_counts "$cp" "$cs" "$cf" "$ce" "$cparsed")
  if [[ "$co_rc" -ne 0 ]]; then
    co_txt="检查失败"
  elif [[ -z "$coauthors" ]]; then
    co_txt="无（${ncommits} 个提交）"
  else
    co_txt="有"
  fi

  local comment="$TMP/comment.md"
  {
    printf '## premerge `%s`\n\n' "${sha:0:12}"
    printf '| 项 | 值 |\n|---|---|\n'
    printf '| head | `%s` |\n' "$sha"
    printf '| pytest L0 | %s |\n' "$l0_txt"
    printf '| pytest core | %s |\n' "$core_txt"
    printf '| regress 退出码 | %s |\n' "$regress_rc"
    printf '| Co-authored-by | %s |\n' "$co_txt"
    printf '| 合并 | %s |\n' "$merge_note"
    printf '| 退出码 | %s |\n' "$final"
    if [[ -n "$coauthors" ]]; then
      printf '\n'
      local row sha_i rest
      while IFS=$'\t' read -r sha_i rest; do
        [[ -n "$sha_i" ]] || continue
        printf -- '- `%s` %s\n' "$sha_i" "$rest"
      done <<<"$coauthors"
    fi
    printf '\n## 批跑\n\n'
    if [[ "$batch_ok" == 1 ]]; then
      cat "$TMP/parsed/batch.md"
    else
      printf '（没有批跑表）\n'
    fi
  } >"$comment"

  if ! gh pr comment "$pr" --body-file "$comment"; then
    printf '评论 PR 失败\n' >&2
    final=1
  fi
  printf 'premerge PR #%s head %s 退出码 %s 合并 %s\n' "$pr" "$sha" "$final" "$merge_note" >&2
  return "$final"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  premerge_main "$@"
fi
