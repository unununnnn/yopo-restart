"""已验 tree 缓存的命中、回退、登记保护。"""

import os
import shutil
import subprocess
from pathlib import Path

from test_premerge import FIXTURE, PREMERGE, _git, _pr_repo, _run_premerge

HEADER = "tree_sha\ttime\thead_sha\tsource\tl0\tcore\tbatch\trc\n"


def _tree(repo, sha):
    return _git(repo, "rev-parse", f"{sha}^{{tree}}").stdout.strip()


def _row(tree, head, rc="0"):
    return (
        f"{tree}\t2026-11-12 11:40:00 UTC-3\t{head}\trecord feature\t"
        f"12 passed, 3 skipped\t40 passed, 1 skipped\t批跑通过\t{rc}\n"
    )


def test_cache_hit_skips_regress(tmp_path):
    repo, sha, fixture, make_log = _pr_repo(tmp_path)
    cache = tmp_path / "verified-trees.tsv"
    cache.write_text(HEADER + _row(_tree(repo, sha), sha), encoding="utf-8")
    proc, calls = _run_premerge(tmp_path, repo, sha, fixture, make_log, "21")
    assert proc.returncode == 0, proc.stderr
    assert not make_log.exists()
    assert "已验树缓存命中" in proc.stderr
    body = [c for c in calls if c["argv"][1] == "comment"][0]["body"]
    assert "已验树缓存命中" in body
    assert _tree(repo, sha) in body
    assert "record feature" in body


def test_cache_miss_runs_regress_and_appends(tmp_path):
    repo, sha, fixture, make_log = _pr_repo(tmp_path)
    proc, _ = _run_premerge(tmp_path, repo, sha, fixture, make_log, "22")
    assert proc.returncode == 0, proc.stderr
    assert make_log.read_text() == "venv\n"
    rows = (tmp_path / "verified-trees.tsv").read_text().splitlines()
    assert rows[0] == HEADER.rstrip("\n")
    assert len(rows) == 2
    assert rows[1].split("\t")[0] == _tree(repo, sha)
    assert rows[1].split("\t")[3] == "premerge 22"


def test_damaged_cache_falls_back_to_regress(tmp_path):
    repo, sha, fixture, make_log = _pr_repo(tmp_path)
    (tmp_path / "verified-trees.tsv").write_text("wrong\theader\n", encoding="utf-8")
    proc, calls = _run_premerge(tmp_path, repo, sha, fixture, make_log, "23")
    assert proc.returncode == 0, proc.stderr
    assert make_log.exists()
    assert "不可信" in proc.stderr
    body = [c for c in calls if c["argv"][1] == "comment"][0]["body"]
    assert "缓存表头错误" in body


def test_one_file_different_tree_does_not_hit(tmp_path):
    repo, sha, fixture, make_log = _pr_repo(tmp_path)
    other = "0" * 40
    (tmp_path / "verified-trees.tsv").write_text(HEADER + _row(other, sha), encoding="utf-8")
    proc, _ = _run_premerge(tmp_path, repo, sha, fixture, make_log, "24")
    assert proc.returncode == 0, proc.stderr
    assert make_log.exists()
    assert "未命中" in proc.stderr


def test_nonzero_rc_invalidates_entire_cache(tmp_path):
    repo, sha, fixture, make_log = _pr_repo(tmp_path)
    tree = _tree(repo, sha)
    (tmp_path / "verified-trees.tsv").write_text(HEADER + _row(tree, sha, "1"), encoding="utf-8")
    proc, _ = _run_premerge(tmp_path, repo, sha, fixture, make_log, "25")
    assert proc.returncode == 0, proc.stderr
    assert make_log.exists()
    assert "rc 不是 0" in proc.stderr


def test_record_rejects_dirty_worktree(tmp_path):
    repo, sha, _fixture, _make_log = _pr_repo(tmp_path)
    scripts = repo / "scripts"
    shutil.copy2(PREMERGE, scripts / "premerge.sh")
    shutil.copy2(PREMERGE.parent / "record-verified-tree.sh", scripts / "record-verified-tree.sh")
    out = tmp_path / "out"
    out.mkdir()
    (out / "regress.log").write_text(FIXTURE, encoding="utf-8")
    (repo / "untracked.txt").write_text("dirty\n")
    env = os.environ.copy()
    env["PREMERGE_VERIFIED_TREES"] = str(tmp_path / "verified-trees.tsv")
    proc = subprocess.run(
        ["bash", str(scripts / "record-verified-tree.sh"), str(out)],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode != 0
    assert "工作区不干净" in proc.stderr
    assert not (tmp_path / "verified-trees.tsv").exists()
