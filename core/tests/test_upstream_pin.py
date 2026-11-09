"""#33（Y0）上游钉定不漂移：YOPO-MINCO @ d5e1e60，见 ADR-0006 2026-10-26 修订。

离线部分不需要网络和 torch，核对四类钉定是否一致：
- 代码常量（baseline / candidates）；
- 金样文件的字节哈希与内嵌元数据；
- ADR-0006 与参考文档里写的值；
- 保真登记里引用的测试。

联网部分带 `upstream` 标记（`pytest -m 'not upstream'` 可跳过；设置 CI 环境变量时也会跳过），
用 `git ls-remote` 确认上游 YOPO-MINCO 分支 head 仍是钉定 SHA。上游若推进，这个用例会失败，
提醒先修订 ADR-0006，再决定是否跟进。
"""

import hashlib
import json
import os
import re
import subprocess
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core import baseline
from yopo_swarm_core.candidates import UPSTREAM_DECODER, DecoderParams

PIN_SHA = "d5e1e60f83cc991602f142d9b6b96204a24d6b9f"
PIN_BRANCH = "YOPO-MINCO"
PIN_URL = "https://github.com/TJU-Aerial-Robotics/YOPO.git"
PIN_WEIGHTS_RELPATH = "YOPO/saved/YOPO_1/epoch50.pth"
PIN_WEIGHTS_SHA256 = "2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f"
PIN_CONFIG_HASH = "18de07db2c1482948b858ee32b64ddc5782d6c53a1e0ca2eb9dce5b7863281d8"
PIN_DECODER_CONFIG_SHA256 = "ac295f3381339d94f5c04aa79266e99df850226f278aa255d8df856c508894c9"
# 金样文件字节哈希：重新生成金样必须同时改这里，并在 PR 里说明原因（docs/ref/golden.md）。
PIN_GOLDEN_FILES = {
    "minco_d5e1e60_cpu.npz": "3707d1ec9314f6b41e71274cdede05267efc2583cd51e36fcb13dcd9a692bfe6",
    "minco_decode_d5e1e60.npz": "92b0083ae7cc1ecad6b54b96aeb419c9733912f883365b81e1d5d5b7aa437d15",
    "minco_node_d5e1e60.npz": "97a96be596b53543f2b89149c76c5474b18512465436e645301abaee3f197c9f",
}

TESTS = Path(__file__).resolve().parent
GOLDEN = TESTS / "golden"
REPO = TESTS.parents[1]
ADR = REPO / "docs" / "adr" / "0006-yopo-minco-baseline.md"
PORT_MAP = REPO / "docs" / "ref" / "minco-port-map.md"
UPSTREAM_DOC = REPO / "docs" / "ref" / "upstream.md"
AMENDMENT = "## 修订（2026-10-26，#33）"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _amendment() -> str:
    text = ADR.read_text(encoding="utf-8")
    assert AMENDMENT in text, "ADR-0006 缺少 2026-10-26 修订"
    return text[text.index(AMENDMENT) :]


# --- 代码常量 ---------------------------------------------------------------


def test_code_constants_match_pin():
    assert baseline.UPSTREAM_URL == PIN_URL
    assert baseline.UPSTREAM_SHA == PIN_SHA
    assert baseline.WEIGHTS_RELPATH == PIN_WEIGHTS_RELPATH
    assert baseline.WEIGHTS_SHA256 == PIN_WEIGHTS_SHA256


def test_pinned_decoder_matches_golden_config_hash():
    cfg = json.loads(str(np.load(GOLDEN / "minco_decode_d5e1e60.npz")["decoder_config"]))
    canon = json.dumps(cfg, sort_keys=True, separators=(",", ":")).encode()
    assert hashlib.sha256(canon).hexdigest() == PIN_DECODER_CONFIG_SHA256
    assert DecoderParams.from_config(cfg) == UPSTREAM_DECODER


# --- 金样 -------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(PIN_GOLDEN_FILES))
def test_golden_file_bytes_pinned(name):
    assert _sha256(GOLDEN / name) == PIN_GOLDEN_FILES[name]


def test_cpu_golden_metadata_matches_pin():
    g = np.load(GOLDEN / "minco_d5e1e60_cpu.npz")
    assert str(g["upstream_sha"]) == PIN_SHA
    assert str(g["weights_sha256"]) == PIN_WEIGHTS_SHA256
    assert str(g["config_hash"]) == PIN_CONFIG_HASH


def test_golden_filenames_carry_short_sha():
    for name in PIN_GOLDEN_FILES:
        assert PIN_SHA[:7] in name


# --- 文档 -------------------------------------------------------------------


@pytest.mark.parametrize("doc", [ADR, PORT_MAP, UPSTREAM_DOC], ids=lambda p: p.name)
def test_docs_state_the_pin(doc):
    text = doc.read_text(encoding="utf-8")
    assert PIN_SHA in text
    assert PIN_WEIGHTS_SHA256[:8] in text


def test_adr_amendment_appended_not_superseding():
    text = ADR.read_text(encoding="utf-8")
    assert text.startswith("---\nStatus: accepted\n---")
    assert text.index("## 修订（2026-10-24，ADR-0011）") < text.index(AMENDMENT)


def test_fidelity_register_lists_exactly_a_to_g():
    rows = re.findall(r"^\| \(([a-z])\) ", _amendment(), flags=re.MULTILINE)
    assert rows == list("abcdefg")


def test_fidelity_register_tests_exist():
    refs = re.findall(r"`(test_[a-z_]+\.py)::(test_[a-z0-9_]+)`", _amendment())
    assert refs, "保真登记至少引用一个测试"
    for fname, func in refs:
        src = (TESTS / fname).read_text(encoding="utf-8")
        assert re.search(rf"^def {func}\(", src, flags=re.MULTILINE), f"{fname}::{func} 不存在"


def test_port_map_has_27_rows_each_with_target_and_winner():
    rows = [
        line
        for line in PORT_MAP.read_text(encoding="utf-8").splitlines()
        if re.match(r"^\| \d+ \|", line)
    ]
    assert [int(r.split("|")[1]) for r in rows] == list(range(1, 28))
    for r in rows:
        cells = [c.strip() for c in r.strip().strip("|").split("|")]
        assert len(cells) == 6, r
        assert cells[4] and cells[5], r  # ys target、胜出方与理由


# --- 联网（可选） ------------------------------------------------------------


@pytest.mark.upstream
def test_upstream_branch_head_has_not_moved():
    if os.environ.get("CI"):
        pytest.skip("CI 不联网检查上游 head")
    try:
        r = subprocess.run(
            ["git", "ls-remote", PIN_URL, f"refs/heads/{PIN_BRANCH}"],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
            env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        pytest.skip(f"无法访问上游：{e}")
    if r.returncode != 0 or not r.stdout.strip():
        pytest.skip(f"无法访问上游：{r.stderr.strip()}")
    head = r.stdout.split()[0]
    assert head == PIN_SHA, (
        f"上游 {PIN_BRANCH} 已推进到 {head}；先修订 ADR-0006，再决定是否跟进（不要直接改钉定）"
    )
