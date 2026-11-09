"""scripts/premerge.sh：回归输出解析、Co-authored-by trailer、假 gh 下的评论与合并。"""

import json
import os
import stat
import subprocess
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PREMERGE = ROOT / "scripts" / "premerge.sh"

FIXTURE = textwrap.dedent(
    """\
    ## pytest（L0）
    ......                                                                   [100%]
    12 passed, 3 skipped in 1.50s
    ## pytest（core，vendored，含上游金样；不含 slow 与联网的上游 head 检查）
    .........                                                                [100%]
    40 passed, 1 skipped in 9.00s

    ## 批跑（suite.txt：hover 判过；g2-pillar@upstream 与 open-3drone@hover 判据未冻结只报数）
    commit `abcdefabcdef`，层级 L0

    ### g2-hover @ hover（hover，层级 L0，影子盾 开，场景卡 `abc`）：1/1 通过
    墙钟 最大 1.0 s / 平均 1.0 s

    | seed | 层级 | 策略 | 结果 |
    |---|---|---|---|
    | 0 | L0 | hover | SUCCESS |

    确定性（同 seed 重跑，墙钟除外）：g2-hover@hover seed0 一致

    ## certified 冒烟（单 seed、仿真 4 s；全部门每步约 2.4 s 墙钟）
    commit `abcdefabcdef`，层级 L0

    | seed | 层级 | 策略 | 结果 |
    |---|---|---|---|
    | 0 | L0 | certified | SUCCESS |

    pytest=通过 批跑=通过 输出 out/regress/20261109T000000
    """
)

FAIL_FIXTURE = textwrap.dedent(
    """\
    ## pytest（L0）
    FAILED tests/test_x.py::test_y - assert 0
    1 failed, 4 passed, 2 skipped in 0.20s
    ## pytest（core，vendored，含上游金样；不含 slow 与联网的上游 head 检查）
    5 passed in 0.10s

    ## 批跑（suite.txt）
    | seed | 层级 | 策略 | 结果 |
    |---|---|---|---|
    | 0 | L0 | hover | SUCCESS |

    pytest=失败 批跑=通过 输出 out/regress/x
    """
)

TEST_MAKEFILE = textwrap.dedent(
    """\
    venv:
    \t@printf '%s\\n' venv >> "$$PREMERGE_MAKE_LOG"
    """
)

TEST_REGRESS = textwrap.dedent(
    """\
    #!/bin/sh
    cat "$PREMERGE_FIXTURE_LOG" || exit 1
    exit "$PREMERGE_FIXTURE_RC"
    """
)

GH_STUB = textwrap.dedent(
    """\
    #!/usr/bin/env python3
    import json, os, pathlib, sys
    log = pathlib.Path(os.environ["GH_LOG"])
    args = sys.argv[1:]
    rec = {"argv": args}
    def take(flag):
        if flag in args:
            return args[args.index(flag) + 1]
        return None
    body_path = take("--body-file")
    if body_path and body_path != "-":
        rec["body"] = pathlib.Path(body_path).read_text()
    if "--body" in args:
        rec["body"] = take("--body")
    with log.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec, ensure_ascii=False) + "\\n")
    if len(args) >= 2 and args[0] == "pr" and args[1] == "view":
        sys.stdout.write(pathlib.Path(os.environ["GH_VIEW"]).read_text())
        raise SystemExit(0)
    if len(args) >= 2 and args[0] == "pr" and args[1] == "ready":
        rc = int(os.environ.get("GH_READY_RC", "0"))
        if rc != 0:
            sys.stderr.write(os.environ.get("GH_READY_STDERR", "ready failed"))
        raise SystemExit(rc)
    if len(args) >= 2 and args[0] == "pr" and args[1] in ("comment", "merge"):
        raise SystemExit(int(os.environ.get("GH_ACTION_RC", "0")))
    sys.stderr.write("unexpected gh " + " ".join(args) + "\\n")
    raise SystemExit(2)
    """
)


def _bash(snippet, *args, env=None):
    cmd = ["bash", "-c", snippet, "bash", *map(str, args)]
    return subprocess.run(cmd, capture_output=True, text=True, env=env, check=False)


def _git(repo, *args, check=True):
    proc = subprocess.run(
        ["git", "-C", str(repo), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    if check and proc.returncode != 0:
        raise AssertionError(proc.stderr or proc.stdout)
    return proc


def _init_repo(path):
    path.mkdir(parents=True)
    subprocess.run(["git", "init", "-b", "main", str(path)], check=True, capture_output=True)
    _git(path, "config", "user.email", "t@example.com")
    _git(path, "config", "user.name", "Tester")
    _git(path, "config", "commit.gpgsign", "false")
    return path


def _commit(repo, *paths, message="feat", trailer=None):
    if paths:
        _git(repo, "add", *[str(p) for p in paths])
    else:
        marker = repo / "marker.txt"
        marker.write_text(marker.read_text() + "x\n" if marker.exists() else "x\n")
        _git(repo, "add", "marker.txt")
    cmd = ["commit", "-m", message]
    if trailer:
        cmd += ["-m", trailer]
    _git(repo, *cmd)
    return _git(repo, "rev-parse", "HEAD").stdout.strip()


def test_parse_regress_counts_and_batch_table(tmp_path):
    log = tmp_path / "regress.log"
    log.write_text(FIXTURE, encoding="utf-8")
    out = tmp_path / "parsed"
    proc = _bash(
        """
        set -euo pipefail
        source "$1"
        premerge_split_regress "$2" "$3"
        premerge_counts_file "$3/l0.txt"
        printf '\\n'
        premerge_counts_file "$3/core.txt"
        printf '\\n---BATCH---\\n'
        cat "$3/batch.md"
        """,
        PREMERGE,
        log,
        out,
    )
    assert proc.returncode == 0, proc.stderr
    head, batch = proc.stdout.split("---BATCH---", 1)
    l0, core = [ln for ln in head.splitlines() if ln.strip()]
    assert l0 == "12 3 0 0 1"
    assert core == "40 1 0 0 1"
    assert "| seed | 层级 | 策略 | 结果 |" in batch
    assert "| 0 | L0 | hover | SUCCESS |" in batch
    assert "| 0 | L0 | certified | SUCCESS |" in batch
    assert "pytest=通过" not in batch
    assert "12 passed" not in batch

    failed = tmp_path / "fail.log"
    failed.write_text(FAIL_FIXTURE, encoding="utf-8")
    proc = _bash(
        """
        set -euo pipefail
        source "$1"
        premerge_split_regress "$2" "$3"
        premerge_counts_file "$3/l0.txt"; printf '\\n'
        premerge_counts_file "$3/core.txt"; printf '\\n'
        premerge_fmt_counts 1 2 1 0 1; printf '\\n'
        premerge_fmt_counts 5 0 0 0 1; printf '\\n'
        premerge_fmt_counts 0 0 0 0 0; printf '\\n'
        """,
        PREMERGE,
        failed,
        tmp_path / "fail-parsed",
    )
    assert proc.returncode == 0, proc.stderr
    lines = proc.stdout.splitlines()
    assert lines[0] == "4 2 1 0 1"
    assert lines[1] == "5 0 0 0 1"
    assert lines[2] == "1 failed, 1 passed, 2 skipped"
    assert lines[3] == "5 passed, 0 skipped"
    assert lines[4] == "未能解析"


def test_coauthor_trailer_detected_and_squash_body_strips(tmp_path):
    repo = _init_repo(tmp_path / "repo")
    (repo / "README").write_text("base\n")
    base = _commit(repo, "README", message="base")
    _git(repo, "checkout", "-b", "feature")
    prose = _commit(
        repo,
        message="mention in prose",
        trailer="See Co-authored-by: not a trailer\n\nSigned-off-by: Tester <t@example.com>",
    )
    middle = _commit(
        repo,
        message="middle line",
        trailer="Co-authored-by: Not Trail <not@example.com>\n\nstill body",
    )
    real = _commit(
        repo,
        message="real trailer",
        trailer="Co-authored-by: Bot <bot@example.com>",
    )
    lower = _commit(
        repo,
        message="lower trailer",
        trailer="co-authored-by: Lower <lower@example.com>",
    )
    proc = _bash(
        """
        set -euo pipefail
        source "$1"
        premerge_coauthor_lines "$2" "$3"
        printf '\\n---BODY---\\n'
        premerge_squash_body "$2" "$3"
        """,
        PREMERGE,
        repo,
        f"{base}..{lower}",
    )
    assert proc.returncode == 0, proc.stderr
    found, body = proc.stdout.split("---BODY---", 1)
    lines = [ln for ln in found.splitlines() if ln.strip()]
    flagged = {ln.split("\t", 1)[0] for ln in lines}
    assert real in flagged
    assert lower in flagged
    assert prose not in flagged
    assert middle not in flagged
    assert base not in flagged
    assert any("Bot <bot@example.com>" in ln for ln in lines)
    assert any("Lower <lower@example.com>" in ln for ln in lines)
    assert "See Co-authored-by: not a trailer" in body
    assert "Not Trail" not in body
    for line in body.splitlines():
        assert not line.lower().lstrip().startswith("co-authored-by:")
    assert "real trailer" in body
    assert "still body" in body


def _install_gh(tmp_path, view):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    gh = bin_dir / "gh"
    gh.write_text(GH_STUB)
    gh.chmod(gh.stat().st_mode | stat.S_IEXEC)
    view_path = tmp_path / "view.json"
    view_path.write_text(json.dumps(view), encoding="utf-8")
    log = tmp_path / "gh.log"
    log.write_text("", encoding="utf-8")
    return bin_dir, view_path, log


def _read_gh(log):
    rows = []
    for line in log.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _pr_repo(tmp_path, *, trailer=None, fixture=FIXTURE):
    repo = _init_repo(tmp_path / "repo")
    (repo / "README").write_text("base\n")
    _commit(repo, "README", message="base")
    _git(repo, "checkout", "-b", "feature")
    (repo / "Makefile").write_text(TEST_MAKEFILE)
    (repo / "scripts").mkdir()
    regress = repo / "scripts" / "regress.sh"
    regress.write_text(TEST_REGRESS)
    regress.chmod(0o755)
    message_trailer = trailer
    sha = _commit(
        repo,
        "Makefile",
        "scripts/regress.sh",
        message="feat: candidate",
        trailer=message_trailer,
    )
    fixture_path = tmp_path / "regress.log"
    fixture_path.write_text(fixture, encoding="utf-8")
    make_log = tmp_path / "make.log"
    return repo, sha, fixture_path, make_log


def _run_premerge(tmp_path, repo, sha, fixture_path, make_log, *args, mergeable="MERGEABLE", ready_rc="0", ready_err="", title="feat: candidate", regress_rc="0"):
    view = {
        "headRefOid": sha,
        "mergeable": mergeable,
        "baseRefName": "main",
        "headRefName": "feature",
        "isDraft": True,
        "title": title,
    }
    bin_dir, view_path, gh_log = _install_gh(tmp_path, view)
    env = os.environ.copy()
    env["PATH"] = str(bin_dir) + os.pathsep + env.get("PATH", "")
    env["PREMERGE_TEST"] = "1"
    env["PREMERGE_ROOT"] = str(repo)
    env["PREMERGE_FIXTURE_LOG"] = str(fixture_path)
    env["PREMERGE_FIXTURE_RC"] = str(regress_rc)
    env["PREMERGE_MAKE_LOG"] = str(make_log)
    env["GH_LOG"] = str(gh_log)
    env["GH_VIEW"] = str(view_path)
    env["GH_READY_RC"] = ready_rc
    env["GH_READY_STDERR"] = ready_err
    proc = subprocess.run(
        ["bash", str(PREMERGE), *args],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    return proc, _read_gh(gh_log)


def test_not_mergeable_fails_fast(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(tmp_path)
    proc, calls = _run_premerge(
        tmp_path, repo, sha, fixture_path, make_log, "42", mergeable="CONFLICTING"
    )
    assert proc.returncode != 0
    assert "MERGEABLE" in proc.stderr
    assert calls and calls[0]["argv"][:2] == ["pr", "view"]
    assert all(c["argv"][1] != "comment" for c in calls)
    assert all(c["argv"][1] != "merge" for c in calls)
    assert not make_log.exists()
    listed = _git(repo, "worktree", "list").stdout.strip().splitlines()
    assert len(listed) == 1


def test_unknown_mergeable_fails_fast(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(tmp_path)
    proc, calls = _run_premerge(
        tmp_path, repo, sha, fixture_path, make_log, "7", mergeable="UNKNOWN"
    )
    assert proc.returncode != 0
    assert all(c["argv"][1] != "comment" for c in calls)


def test_comment_reports_sha_counts_table_and_blocks_merge_on_trailer(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(
        tmp_path, trailer="Co-authored-by: Bot <bot@example.com>"
    )
    proc, calls = _run_premerge(
        tmp_path, repo, sha, fixture_path, make_log, "42", "--merge"
    )
    assert proc.returncode != 0
    comments = [c for c in calls if c["argv"][1] == "comment"]
    merges = [c for c in calls if c["argv"][1] == "merge"]
    assert len(comments) == 1
    assert merges == []
    assert "pr" == comments[0]["argv"][0]
    assert "--body-file" in comments[0]["argv"]
    body = comments[0]["body"]
    assert f"`{sha}`" in body
    assert "12 passed, 3 skipped" in body
    assert "40 passed, 1 skipped" in body
    assert "| regress 退出码 | 0 |" in body
    assert "| 退出码 | 1 |" in body
    assert "| seed | 层级 | 策略 | 结果 |" in body
    assert "| 0 | L0 | hover | SUCCESS |" in body
    assert "| 0 | L0 | certified | SUCCESS |" in body
    assert "pytest=通过" not in body
    assert f"`{sha}` Co-authored-by: Bot <bot@example.com>" in body
    assert "未执行（检查未过）" in body
    assert "venv\n" == make_log.read_text()
    assert len(_git(repo, "worktree", "list").stdout.strip().splitlines()) == 1


def test_merge_flag_ready_and_match_head_when_clean(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(tmp_path)
    proc, calls = _run_premerge(
        tmp_path,
        repo,
        sha,
        fixture_path,
        make_log,
        "9",
        "--merge",
        ready_rc="1",
        ready_err='Pull request #9 is already "ready for review"',
    )
    assert proc.returncode == 0, proc.stderr
    ready = [c for c in calls if c["argv"][1] == "ready"]
    merges = [c for c in calls if c["argv"][1] == "merge"]
    assert len(ready) == 1 and ready[0]["argv"][2] == "9"
    assert len(merges) == 1
    argv = merges[0]["argv"]
    assert "--merge" in argv
    assert "--squash" not in argv
    assert "--match-head-commit" in argv
    assert argv[argv.index("--match-head-commit") + 1] == sha
    body = [c for c in calls if c["argv"][1] == "comment"][0]["body"]
    assert "| 退出码 | 0 |" in body
    assert "已合并（merge）" in body
    assert "Co-authored-by | 无" in body


def test_squash_strips_body_and_matches_head(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(
        tmp_path,
        trailer="See Co-authored-by: kept in prose\n\nSigned-off-by: Tester <t@example.com>",
    )
    proc, calls = _run_premerge(
        tmp_path, repo, sha, fixture_path, make_log, "3", "--squash", title="票：premerge"
    )
    assert proc.returncode == 0, proc.stderr
    merges = [c for c in calls if c["argv"][1] == "merge"]
    assert len(merges) == 1
    argv = merges[0]["argv"]
    assert "--squash" in argv
    assert "--merge" not in argv
    assert argv[argv.index("--match-head-commit") + 1] == sha
    assert argv[argv.index("--subject") + 1] == "票：premerge"
    body = merges[0]["body"]
    assert "See Co-authored-by: kept in prose" in body
    for line in body.splitlines():
        assert not line.lower().lstrip().startswith("co-authored-by:")
    assert "feat: candidate" in body
    comment = [c for c in calls if c["argv"][1] == "comment"][0]["body"]
    assert "已合并（squash）" in comment
    assert "| 退出码 | 0 |" in comment


def test_no_flag_never_merges(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(tmp_path)
    proc, calls = _run_premerge(tmp_path, repo, sha, fixture_path, make_log, "4")
    assert proc.returncode == 0, proc.stderr
    assert all(c["argv"][1] != "merge" for c in calls)
    assert all(c["argv"][1] != "ready" for c in calls)
    body = [c for c in calls if c["argv"][1] == "comment"][0]["body"]
    assert "未请求" in body


def test_regress_failure_skips_merge_and_reports_exit_code(tmp_path):
    repo, sha, fixture_path, make_log = _pr_repo(tmp_path, fixture=FAIL_FIXTURE)
    proc, calls = _run_premerge(
        tmp_path, repo, sha, fixture_path, make_log, "8", "--squash", regress_rc="1"
    )
    assert proc.returncode != 0
    assert all(c["argv"][1] != "merge" for c in calls)
    body = [c for c in calls if c["argv"][1] == "comment"][0]["body"]
    assert "1 failed, 4 passed, 2 skipped" in body
    assert "5 passed, 0 skipped" in body
    assert "| regress 退出码 | 1 |" in body
    assert "| 退出码 | 1 |" in body
    assert "| 0 | L0 | hover | SUCCESS |" in body


def test_usage_and_script_never_pushes():
    proc = subprocess.run(
        ["bash", str(PREMERGE)], capture_output=True, text=True, check=False
    )
    assert proc.returncode == 2
    both = subprocess.run(
        ["bash", str(PREMERGE), "1", "--merge", "--squash"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert both.returncode == 2
    text = PREMERGE.read_text(encoding="utf-8")
    code = "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))
    assert "git push" not in code
    assert "push --force" not in code
    assert "--force-with-lease" not in code
    assert "push -f" not in code
    assert "--admin" not in code


def test_make_venv_idempotent_when_present():
    if not (ROOT / ".venv" / ".deps-hash").exists():
        pytest.skip("需要已建好的 .venv")
    proc = subprocess.run(
        ["make", "venv"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    combined = proc.stdout + proc.stderr
    assert proc.returncode == 0, combined
    assert "already exists" not in combined.lower()
    assert "virtual environment" not in combined.lower()
