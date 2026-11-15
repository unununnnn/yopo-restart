"""票 E-B2q：诊断脚本干跑。

票 E-B2r 之后库缺省是 20 次。诊断各臂用显式 time_budget_s 保住 6/10/20/40。
"""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

from conftest import ROOT

SCRIPT = ROOT / "scripts" / "e-b2q-evals-diag.py"


def _load():
    spec = importlib.util.spec_from_file_location("e_b2q_evals_diag", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_default_budget_is_twenty_and_explicit_seconds_keep_their_count(monkeypatch):
    """缺省期望从 E-B2q 写的 10 改成 20。显式秒数按 1 ms/次折算，不被改成 20。

    诊断当时 0.040 s 是 20 次、0.080 s 是 40 次。E-B2r 之后这两档是 40 和 80。
    要回 10 次，显式传 0.010 s。
    """
    monkeypatch.delenv("YOPO_BACKEND_BUDGET", raising=False)
    from yopo_l0.backend.bspline import _eval_cap
    from yopo_l0.backend.config import DEFAULT_BUDGET, BackendConfig, config_from_params

    assert DEFAULT_BUDGET == "evals"
    cfg = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    assert cfg is not None
    assert cfg.budget == "evals"
    assert cfg.time_budget_s == pytest.approx(0.020)
    assert _eval_cap(cfg) == 20
    assert _eval_cap(BackendConfig(time_budget_s=0.010)) == 10
    assert _eval_cap(BackendConfig(time_budget_s=0.040)) == 40
    assert _eval_cap(BackendConfig(time_budget_s=0.080)) == 80
    assert config_from_params({}, a_max=6.0) is None
    disk = json.loads((ROOT / "scenes" / "crossing-forest-dense-bspline.json").read_text())
    backend = disk["policy_params"]["backend"]
    assert backend == {"enabled": True}
    assert "budget" not in backend
    assert "time_budget_s" not in backend


def test_check_is_a_dry_run_and_keeps_wall_budget_off():
    mod = _load()
    assert mod.arms_table()["off"]["enabled"] is False
    assert mod.arms_table()["off"]["upstream_deviation"] is False
    for arm_id in ("e6", "e10", "e20", "e40"):
        arm = mod.arms_table()[arm_id]
        assert arm["enabled"] is True
        assert arm["budget"] == "evals"
        assert arm["upstream_deviation"] is True
        assert "偏离上游" in arm["label"]
        assert arm["budget"] != "wall"
    assert mod.arms_table()["e10"]["evals"] == 10
    assert mod.arms_table()["e20"]["evals"] == 20
    assert mod.arms_table()["e40"]["evals"] == 40
    before = {rel: (ROOT / rel).read_bytes() for rel in (
        "scenes/crossing-forest-dense-bspline.json",
        "scenes/suite.txt",
        "README.md",
        "docs/tickets.md",
        "yopo_l0/backend/config.py",
        "yopo_l0/backend/bspline.py",
    ) if (ROOT / rel).is_file()}
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--check", "--case", "forest,c4b014,b6medium"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    report = json.loads(proc.stdout)
    assert report["simulation"] is False
    assert report["default_budget"] == "evals"
    assert report["default_eval_cap"] == 20
    after = {rel: (ROOT / rel).read_bytes() for rel in before}
    assert after == before


def test_in_memory_cards_do_not_write_scenes(tmp_path, monkeypatch):
    monkeypatch.delenv("YOPO_BACKEND_BUDGET", raising=False)
    mod = _load()
    forest = ROOT / "scenes" / "crossing-forest-dense-bspline.json"
    blob = forest.read_bytes()
    from yopo_l0.backend.bspline import _eval_cap
    from yopo_l0.backend.config import config_from_params

    off = mod._scene_card("forest", "off")
    assert config_from_params(off.card.get("policy_params"), a_max=6.0) is None
    e10 = mod._scene_card("forest", "e10")
    cfg10 = config_from_params(e10.card.get("policy_params"), a_max=6.0)
    assert cfg10 is not None and cfg10.budget == "evals" and _eval_cap(cfg10) == 10
    e40 = mod._scene_card("forest", "e40")
    cfg40 = config_from_params(e40.card.get("policy_params"), a_max=6.0)
    assert cfg40 is not None and _eval_cap(cfg40) == 40 and cfg40.budget == "evals"
    assert forest.read_bytes() == blob
    assert not (tmp_path / "unused").exists()
