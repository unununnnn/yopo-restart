#!/usr/bin/env bash
# 合 main 前的本机门。不接 GitHub Actions。
# 用法：scripts/premerge.sh <PR号> [--merge|--squash]
#
# 1. gh pr view 取 head sha 与 mergeable，不是 MERGEABLE 就立刻失败（不建 worktree）。
# 2. 在临时目录建 PR head 的 git worktree。
# 3. 在 worktree 里 make venv（共享依赖 venv + uv 缓存）并跑 scripts/regress.sh。
# 4. 解析 L0 / core 的 passed、skipped，以及批跑 Markdown 表。
# 5. 在 PR 上留一条评论：head sha、计数、退出码、批跑表。
# 6. 检查 PR 相对基线各提交的 Co-authored-by trailer，并写进评论。
#    --squash 时正文已用 premerge_strip_coauthor_lines 去掉 trailer，有 trailer 不把 gate 置 1。
#    --merge（以及不带合并旗标的检查）仍然把 trailer 当失败。
# 7. 任一检查失败则退出码非 0。
# 8. --merge / --squash 在合入前会 fetch 基线；基线已前进则先把 origin/<base>
#    合进 PR 分支（普通 push，不 force）。push 之后轮询
#    gh pr view <PR> --json mergeable,mergeStateStatus，每 5 s 一次、最多 120 s，
#    等到 mergeable=MERGEABLE 再 gh pr merge；若为 CONFLICTING 立刻失败；
#    超时也失败。gh pr merge 若报 not mergeable，再等 10 s 重试一次。
#    追上失败、等待失败或 gh pr merge 失败：退出码非 0，并在 PR 留评说明原因。
# 9. EXIT 陷阱清理 worktree 时保留原退出码，避免合入失败被收成 0。
#
# 不带 --merge / --squash 时绝不合并，也不会推送分支。
# --merge / --squash 只在全部检查通过之后执行：
#   如基线已前进：git merge origin/<base>，再 git push origin <sha>:refs/heads/<head>
#   push 后等待 MERGEABLE（见上）
#   gh pr ready
#   gh pr merge --merge|--squash --match-head-commit <sha>
#   若报 not mergeable：等 10 s 再试一次
# --squash 的说明由各提交说明拼成，并删掉其中的 Co-authored-by 行。
# 结束时删除 worktree。
#
# 测试可设 PREMERGE_TEST=1 与 PREMERGE_ROOT=<仓库>，把 git 操作指到临时仓库。
# 不跳过 regress：worktree 里跑的就是该提交的 scripts/regress.sh。
# 等待间隔可用 PREMERGE_MERGEABLE_POLL_S（默认 5）、PREMERGE_MERGEABLE_TIMEOUT_S
# （默认 120）、PREMERGE_MERGE_RETRY_S（默认 10）覆盖，便于测试。

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

PREMERGE_VERIFIED_HEADER=$'tree_sha\ttime\thead_sha\tsource\tl0\tcore\tbatch\trc'

premerge_merged_tree() {
  # $1 仓库 $2 基线 commit $3 head commit。输出合入后的 tree，不创建 commit。
  local repo=$1 base=$2 head=$3
  if git -C "$repo" merge-base --is-ancestor "$base" "$head"; then
    git -C "$repo" rev-parse "${head}^{tree}"
  else
    git -C "$repo" merge-tree --write-tree "$base" "$head"
  fi
}

premerge_verified_lookup() {
  # 严格校验整份 TSV；命中输出该行，未命中返回 1，损坏返回 2 并输出原因。
  python3 - "$1" "$2" <<'PY'
import pathlib, re, sys
path, wanted = pathlib.Path(sys.argv[1]), sys.argv[2]
header = "tree_sha\ttime\thead_sha\tsource\tl0\tcore\tbatch\trc"
if not path.is_file():
    print("缓存文件不存在")
    raise SystemExit(2)
try:
    lines = path.read_text(encoding="utf-8").splitlines()
except Exception as exc:
    print(f"缓存读取失败：{exc}")
    raise SystemExit(2)
if not lines or lines[0] != header:
    print("缓存表头错误")
    raise SystemExit(2)
seen = set()
match = None
for number, line in enumerate(lines[1:], 2):
    cols = line.split("\t")
    if len(cols) != 8:
        print(f"缓存第 {number} 行列数错误")
        raise SystemExit(2)
    tree, _time, head, _source, _l0, _core, _batch, rc = cols
    if not re.fullmatch(r"[0-9a-fA-F]{40}", tree):
        print(f"缓存第 {number} 行 tree_sha 无效")
        raise SystemExit(2)
    if not re.fullmatch(r"[0-9a-fA-F]{40}", head):
        print(f"缓存第 {number} 行 head_sha 无效")
        raise SystemExit(2)
    if rc != "0":
        print(f"缓存第 {number} 行 rc 不是 0")
        raise SystemExit(2)
    key = tree.lower()
    if key in seen:
        print(f"缓存第 {number} 行 tree_sha 重复")
        raise SystemExit(2)
    seen.add(key)
    if key == wanted.lower():
        match = line
if match is None:
    raise SystemExit(1)
print(match)
PY
}

premerge_append_verified() {
  # $1 文件，后 8 个参数为列。flock 覆盖初始化/追加的整个临界区。
  local file=$1 tree=$2 time=$3 head=$4 source=$5 l0=$6 core=$7 batch=$8 rc=$9
  local value lock="${file}.lock"
  for value in "$tree" "$time" "$head" "$source" "$l0" "$core" "$batch" "$rc"; do
    [[ "$value" != *$'\t'* && "$value" != *$'\n'* && "$value" != *$'\r'* ]] || return 1
  done
  mkdir -p "$(dirname "$file")"
  (
    flock -x 9
    [[ -e "$file" ]] || printf '%s\n' "$PREMERGE_VERIFIED_HEADER" >"$file"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$tree" "$time" "$head" "$source" "$l0" "$core" "$batch" "$rc" >>"$file"
  ) 9>"$lock"
}

premerge_usage() {
  cat <<'EOF'
用法：scripts/premerge.sh <PR号> [--merge|--squash]

在临时 worktree 里跑 scripts/regress.sh，把 head sha、L0/core 的
passed/skipped、退出码和批跑表写到 PR 评论。各提交的 Co-authored-by
trailer 会写进评论：--squash 时不挡门（正文已去掉 trailer），--merge
时仍然拦截。

  --merge   全部通过后：gh pr ready && gh pr merge --merge --match-head-commit <sha>
  --squash  全部通过后：gh pr ready && gh pr merge --squash --match-head-commit <sha>
            提交说明由各提交拼成，并去掉 Co-authored-by 行；原提交里的 trailer 不挡门

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
  # 不用 set -e / set +e，避免把调用方的 errexit 状态改掉。
  # 以前 premerge_ready 末尾的 set -e 会让后面的 gh pr merge 一失败
  # 就直接退出主函数，评论写不上，EXIT 清理还可能把退出码收成 0。
  local pr=$1 err rc=0
  err=$(gh pr ready "$pr" 2>&1) || rc=$?
  if [[ "$rc" -eq 0 ]]; then
    return 0
  fi
  if [[ "$err" =~ [Aa]lready ]]; then
    return 0
  fi
  printf '%s\n' "$err" >&2
  return 1
}

premerge_parse_mergeable() {
  # $1 json 文件。打印：mergeable<TAB>mergeStateStatus
  python3 - "$1" <<'PY'
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text() or "{}")
mergeable = data.get("mergeable")
state = data.get("mergeStateStatus")
if mergeable is None:
    mergeable = ""
if state is None:
    state = ""
print("%s\t%s" % (mergeable, state))
PY
}

premerge_wait_mergeable() {
  # push 新 head 后，等 GitHub 算完可合并状态。
  # $1 PR 号 $2 错误文件（失败时写入原因）
  # mergeable=MERGEABLE 返回 0；CONFLICTING 立刻失败；其它状态每
  # PREMERGE_MERGEABLE_POLL_S 秒再查，直到 PREMERGE_MERGEABLE_TIMEOUT_S。
  local pr=$1 err_file=$2
  local poll timeout started now elapsed json view_err line mergeable state
  poll=${PREMERGE_MERGEABLE_POLL_S:-5}
  timeout=${PREMERGE_MERGEABLE_TIMEOUT_S:-120}
  started=$(date +%s)
  json="${err_file}.view.json"
  view_err="${err_file}.view.err"
  while true; do
    if ! gh pr view "$pr" --json mergeable,mergeStateStatus >"$json" 2>"$view_err"; then
      {
        printf '查询可合并状态失败\n'
        cat "$view_err"
      } >"$err_file"
      return 1
    fi
    line=$(premerge_parse_mergeable "$json")
    mergeable=${line%%$'\t'*}
    state=${line#*$'\t'}
    if [[ "$mergeable" == "MERGEABLE" ]]; then
      return 0
    fi
    if [[ "$mergeable" == "CONFLICTING" ]]; then
      printf '推送后 GitHub 报告冲突：mergeable=CONFLICTING mergeStateStatus=%s。\n' "${state:-空}" >"$err_file"
      return 1
    fi
    now=$(date +%s)
    elapsed=$((now - started))
    if [[ "$elapsed" -ge "$timeout" ]]; then
      printf '推送后等待 GitHub 计算可合并状态超时（%s s），最后 mergeable=%s mergeStateStatus=%s。\n' \
        "$timeout" "${mergeable:-空}" "${state:-空}" >"$err_file"
      return 1
    fi
    sleep "$poll"
  done
}

premerge_gh_merge() {
  local pr=$1 mode=$2 sha=$3 title=$4 body_file=$5
  if [[ "$mode" == "merge" ]]; then
    gh pr merge "$pr" --merge --match-head-commit "$sha"
  else
    gh pr merge "$pr" --squash --match-head-commit "$sha" --subject "$title" --body-file "$body_file"
  fi
}

premerge_do_merge() {
  local pr=$1 mode=$2 sha=$3 title=$4 body_file=$5
  local err_file=${6:-}
  local err rc=0 retry_s low
  retry_s=${PREMERGE_MERGE_RETRY_S:-10}
  premerge_ready "$pr" || return $?
  err=$(premerge_gh_merge "$pr" "$mode" "$sha" "$title" "$body_file" 2>&1) || rc=$?
  if [[ "$rc" -ne 0 ]]; then
    low=$(printf '%s' "$err" | tr '[:upper:]' '[:lower:]')
    if [[ "$low" == *"not mergeable"* ]]; then
      sleep "$retry_s"
      rc=0
      err=$(premerge_gh_merge "$pr" "$mode" "$sha" "$title" "$body_file" 2>&1) || rc=$?
    fi
  fi
  if [[ "$rc" -ne 0 ]]; then
    printf '%s\n' "$err" >&2
    if [[ -n "$err_file" ]]; then
      printf '%s\n' "$err" >"$err_file"
    fi
    return "$rc"
  fi
  return 0
}

premerge_has_origin() {
  git -C "$1" remote get-url origin >/dev/null 2>&1
}

premerge_base_sha() {
  # $1 仓库 $2 基线分支名
  local repo=$1 base_name=$2
  if git -C "$repo" rev-parse --verify --quiet "refs/remotes/origin/${base_name}" >/dev/null; then
    git -C "$repo" rev-parse "refs/remotes/origin/${base_name}"
  else
    git -C "$repo" rev-parse "refs/heads/${base_name}"
  fi
}

premerge_fetch_base() {
  # $1 仓库 $2 基线分支名。没有 origin 时（测试仓库）跳过 fetch。
  local repo=$1 base_name=$2
  if premerge_has_origin "$repo"; then
    git -C "$repo" fetch --no-tags origin "$base_name"
  fi
}

premerge_catch_up_base() {
  # 基线已前进时，把新基线合进 PR head，并普通推送到 head 分支。
  # 参数：仓库 基线名 旧基线 sha 当前 head sha 分支名 错误文件
  # 成功时把新 head sha 打到 stdout；无需追上则原样打印 head sha。
  # 冲突或 push 失败返回非 0，原因写进错误文件。
  local repo=$1 base_name=$2 old_base=$3 head_sha=$4 head_name=$5 err_file=$6
  local new_base catch new_sha
  premerge_fetch_base "$repo" "$base_name" || {
    printf 'fetch %s 失败\n' "$base_name" >"$err_file"
    return 1
  }
  new_base=$(premerge_base_sha "$repo" "$base_name")
  if [[ "$new_base" == "$old_base" ]]; then
    printf '%s' "$head_sha"
    return 0
  fi
  catch=$(mktemp -d "${TMPDIR:-/tmp}/premerge-catch.XXXXXX")
  rmdir "$catch"
  if ! git -C "$repo" worktree add --detach "$catch" "$head_sha" >"$err_file" 2>&1; then
    printf '无法为追上基线建立 worktree\n%s\n' "$(cat "$err_file")" >"${err_file}.tmp"
    mv "${err_file}.tmp" "$err_file"
    rm -rf "$catch"
    return 1
  fi
  if ! git -C "$catch" merge --no-edit "$new_base" >"$err_file" 2>&1; then
    {
      printf '基线已前进（%s → %s），合入 PR 分支时发生冲突。\n' "${old_base:0:12}" "${new_base:0:12}"
      cat "$err_file"
    } >"${err_file}.tmp"
    mv "${err_file}.tmp" "$err_file"
    git -C "$catch" merge --abort >/dev/null 2>&1 || true
    git -C "$repo" worktree remove --force "$catch" >/dev/null 2>&1 || true
    rm -rf "$catch"
    return 1
  fi
  new_sha=$(git -C "$catch" rev-parse HEAD)
  git -C "$repo" worktree remove --force "$catch" >/dev/null 2>&1 || true
  rm -rf "$catch"
  if premerge_has_origin "$repo"; then
    if ! git -C "$repo" push origin "${new_sha}:refs/heads/${head_name}" >"${err_file}" 2>&1; then
      {
        printf '追上基线后 push %s 失败（非 force）。\n' "$head_name"
        cat "$err_file"
      } >"${err_file}.tmp"
      mv "${err_file}.tmp" "$err_file"
      return 1
    fi
  else
    git -C "$repo" update-ref "refs/heads/${head_name}" "$new_sha"
  fi
  printf '%s\n' "基线已前进（${old_base:0:12} → ${new_base:0:12}），已合入 PR 并更新 ${head_name} 到 ${new_sha:0:12}。" >&2
  printf '%s' "$new_sha"
  return 0
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
  premerge_on_exit() {
    local rc=$?
    set +e
    premerge_cleanup
    exit "$rc"
  }
  trap premerge_on_exit EXIT

  local view_json="$TMP/view.json"
  gh pr view "$pr" --json headRefOid,mergeable,baseRefName,headRefName,isDraft,title >"$view_json"
  premerge_write_view "$view_json" "$TMP/view"

  local sha mergeable base_name head_name title
  sha=$(cat "$TMP/view/headRefOid")
  mergeable=$(cat "$TMP/view/mergeable")
  base_name=$(cat "$TMP/view/baseRefName")
  head_name=$(cat "$TMP/view/headRefName")
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

  local range base_sha
  base_sha=$(git rev-parse "$baseref")
  range="${baseref}..${sha}"

  WT="$TMP/wt"
  git worktree add --detach "$WT" "$sha" >&2
  printf 'worktree %s @ %s\n' "$WT" "$sha" >&2

  local regress_log="$TMP/regress.log" regress_rc verified_file verified_tree tested_tree cache_row="" cache_reason="" cache_hit=0 cache_append=0 lookup_rc
  verified_file=${PREMERGE_VERIFIED_TREES:-/workspace/yopo-ops/verified-trees.tsv}
  verified_tree=$(premerge_merged_tree "$ROOT" "$base_sha" "$sha") || { printf '无法计算待合入 tree\n' >&2; return 1; }
  tested_tree=$(git -C "$ROOT" rev-parse "${sha}^{tree}")
  set +e
  cache_row=$(premerge_verified_lookup "$verified_file" "$verified_tree")
  lookup_rc=$?
  set -e
  if [[ "$lookup_rc" -eq 0 ]]; then
    cache_hit=1
    printf '已验树缓存命中：%s\n' "$verified_tree" >&2
    : >"$regress_log"
    regress_rc=0
  else
    if [[ "$lookup_rc" -eq 2 ]]; then
      cache_reason=$cache_row
      printf '已验树缓存不可信，退回全量回归：%s\n' "$cache_reason" >&2
    else
      cache_reason="没有 tree ${verified_tree} 的记录"
      cache_append=1
      printf '已验树缓存未命中，运行全量回归：%s\n' "$cache_reason" >&2
    fi
    if [[ "$cache_reason" == "缓存文件不存在" ]]; then cache_append=1; fi
    set +e
    (
      set -euo pipefail
      cd "$WT"
      make venv
      scripts/regress.sh
    ) 2>&1 | tee "$regress_log"
    regress_rc=${PIPESTATUS[0]}
    set -e
  fi

  premerge_split_regress "$regress_log" "$TMP/parsed"
  local l0_line core_line
  l0_line=$(premerge_counts_file "$TMP/parsed/l0.txt")
  core_line=$(premerge_counts_file "$TMP/parsed/core.txt")
  # shellcheck disable=SC2086
  local l0p l0s l0f l0e l0parsed cp cs cf ce cparsed
  read -r l0p l0s l0f l0e l0parsed <<<"$l0_line"
  read -r cp cs cf ce cparsed <<<"$core_line"
  local batch_ok=0
  local l0_txt_cached="" core_txt_cached="" batch_txt_cached=""
  if [[ "$cache_hit" -eq 1 ]]; then
    IFS=$'\t' read -r _ _ _ _ l0_txt_cached core_txt_cached batch_txt_cached _ <<<"$cache_row"
    l0p=0; l0s=0; l0f=0; l0e=0; l0parsed=1
    cp=0; cs=0; cf=0; ce=0; cparsed=1
    batch_ok=1
  elif grep -q '| seed |' "$TMP/parsed/batch.md"; then
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
  # squash 正文已经去掉 trailer，有 trailer 只写进评论，不挡门。
  # --merge 会保留原提交，无旗标的检查同样把 trailer 当失败。
  if [[ -n "$coauthors" && "$mode" != "squash" ]]; then gate=1; fi

  if [[ "$gate" -eq 0 && "$cache_hit" -eq 0 && "$cache_append" -eq 1 ]]; then
    # 回归在 PR head worktree 上跑，只能登记真正测过的 head tree。
    premerge_append_verified "$verified_file" "$tested_tree" \
      "$(TZ=Etc/GMT+3 date '+%Y-%m-%d %H:%M:%S UTC-3')" "$sha" "premerge ${pr}" \
      "$(premerge_fmt_counts "$l0p" "$l0s" "$l0f" "$l0e" "$l0parsed")" \
      "$(premerge_fmt_counts "$cp" "$cs" "$cf" "$ce" "$cparsed")" "批跑通过" "0"
  fi

  local merge_note="未请求" final=$gate body_file="$TMP/squash.md" merge_rc=0
  local merge_err_file="$TMP/merge.err" catchup_note="" caught_sha=""
  : >"$merge_err_file"
  if [[ -n "$mode" ]]; then
    if [[ "$gate" -ne 0 ]]; then
      merge_note="未执行（检查未过）"
    else
      caught_sha=""
      head_before=$sha
      if caught_sha=$(premerge_catch_up_base "$ROOT" "$base_name" "$base_sha" "$sha" "$head_name" "$merge_err_file"); then
        if [[ "$caught_sha" != "$sha" ]]; then
          catchup_note="基线已前进，已先合入并更新 head ${caught_sha:0:12}。"
          sha=$caught_sha
          if git rev-parse --verify --quiet "refs/remotes/origin/${base_name}" >/dev/null; then
            baseref="origin/${base_name}"
          else
            baseref="refs/heads/${base_name}"
          fi
          base_sha=$(git rev-parse "$baseref")
          range="${baseref}..${sha}"
          final_tree=$(premerge_merged_tree "$ROOT" "$base_sha" "$sha") || final_tree=""
          if [[ "$cache_hit" -eq 1 && "$final_tree" != "$verified_tree" ]]; then
            printf '基线前进后最终 tree %s 与已验 tree %s 不同；拒绝使用缓存合入。\n' "$final_tree" "$verified_tree" >"$merge_err_file"
            merge_note="合并失败（已验 tree 已变化）"
            merge_rc=1
            final=1
          fi
        fi
        if [[ "$mode" == "squash" ]]; then
          premerge_squash_body "$ROOT" "$range" >"$body_file"
          if [[ ! -s "$body_file" ]]; then
            printf '%s\n' "（无提交说明）" >"$body_file"
          fi
          if [[ -z "$title" ]]; then
            title="PR #${pr}"
          fi
        fi
        if [[ "$merge_rc" -eq 0 ]]; then : >"$merge_err_file"; fi
        if [[ "$merge_rc" -eq 0 && "$caught_sha" != "$head_before" ]]; then
          if ! premerge_wait_mergeable "$pr" "$merge_err_file"; then
            merge_note="合并失败（等待可合并状态失败）"
            merge_rc=1
            final=1
          fi
        fi
        if [[ "$merge_rc" -eq 0 ]]; then
          premerge_do_merge "$pr" "$mode" "$sha" "$title" "$body_file" "$merge_err_file" || merge_rc=$?
          if [[ "$merge_rc" -eq 0 ]]; then
            if [[ "$mode" == "merge" ]]; then
              merge_note="已合并（merge）"
            else
              merge_note="已合并（squash）"
            fi
            if [[ -n "$catchup_note" ]]; then
              merge_note="${merge_note}；${catchup_note}"
            fi
          else
            merge_note="合并失败"
            final=1
          fi
        fi
      else
        merge_note="合并失败（追上基线失败）"
        final=1
      fi
    fi
  fi

  local l0_txt core_txt co_txt
  if [[ "$cache_hit" -eq 1 ]]; then
    l0_txt=$l0_txt_cached
    core_txt=$core_txt_cached
  else
    l0_txt=$(premerge_fmt_counts "$l0p" "$l0s" "$l0f" "$l0e" "$l0parsed")
    core_txt=$(premerge_fmt_counts "$cp" "$cs" "$cf" "$ce" "$cparsed")
  fi
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
    if [[ "$cache_hit" -eq 1 ]]; then
      local cache_tree cache_time cache_head cache_source cache_l0 cache_core cache_batch cache_rc
      IFS=$'\t' read -r cache_tree cache_time cache_head cache_source cache_l0 cache_core cache_batch cache_rc <<<"$cache_row"
      printf '\n## 已验树缓存命中\n\n'
      printf -- '- tree: `%s`\n- 时间: %s\n- 来源: %s\n- L0: %s\n- core: %s\n- batch: %s\n' \
        "$cache_tree" "$cache_time" "$cache_source" "$cache_l0" "$cache_core" "$cache_batch"
    elif [[ -n "$cache_reason" ]]; then
      printf '\n## 已验树缓存未使用\n\n%s\n' "$cache_reason"
    fi
    if [[ -s "$merge_err_file" ]]; then
      printf '\n## 合并失败\n\n```\n'
      cat "$merge_err_file"
      printf '\n```\n'
    fi
    if [[ -n "$coauthors" ]]; then
      printf '\n'
      local row sha_i rest
      while IFS=$'\t' read -r sha_i rest; do
        [[ -n "$sha_i" ]] || continue
        printf -- '- `%s` %s\n' "$sha_i" "$rest"
      done <<<"$coauthors"
    fi
    printf '\n## 批跑\n\n'
    if [[ "$cache_hit" -eq 1 ]]; then
      printf '%s\n' "$batch_txt_cached"
    elif [[ "$batch_ok" == 1 ]]; then
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
