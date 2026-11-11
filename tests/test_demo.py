"""票 12a：README 顶部一键命令，干净检出按该命令可复现。"""

import json
import os
import shutil
import stat
import subprocess

from conftest import ROOT

DEMO_SCENES = {
    "g2-pillar@upstream": 1,
    "formation-hold@formation": 3,
    "formation-6-open@formation": 6,
}


def test_readme_leads_with_one_click_command():
    text = (ROOT / "README.md").read_text()
    head = "\n".join(text.splitlines()[:30])
    assert "scripts/demo.sh" in head
    assert "单机" in head and "三机" in head and "六机" in head
    assert text.index("scripts/demo.sh") < text.index("scripts/regress.sh")


def test_demo_script_is_executable_and_covers_three_sizes():
    path = ROOT / "scripts/demo.sh"
    assert path.is_file()
    assert path.stat().st_mode & stat.S_IXUSR
    text = path.read_text()
    assert "scenes/g2-pillar.json" in text
    assert "scenes/formation-hold.json" in text
    assert "scenes/formation-6-open.json" in text
    assert "yopo_l0.batch" in text


def _export_clean_tree(dest):
    listed = subprocess.check_output(
        ["git", "-C", str(ROOT), "ls-files", "-c", "-o", "--exclude-standard"],
        text=True,
    )
    for rel in listed.splitlines():
        src = ROOT / rel
        if not src.is_file():
            continue
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)


def test_fresh_checkout_runs_readme_demo(tmp_path):
    dest = tmp_path / "yopo-restart"
    dest.mkdir()
    _export_clean_tree(dest)
    assert (dest / "scripts/demo.sh").is_file()
    assert not (dest / ".venv").exists()
    assert not (dest / "out").exists()
    proc = subprocess.run(
        ["bash", "scripts/demo.sh"],
        cwd=dest,
        capture_output=True,
        text=True,
        timeout=600,
        env=os.environ.copy(),
    )
    assert proc.returncode == 0, proc.stdout + "\n" + proc.stderr
    for name, n_drones in DEMO_SCENES.items():
        result_path = dest / "out" / "demo" / name / "seed0" / "result.json"
        assert result_path.is_file(), f"缺少 {result_path.relative_to(dest)}"
        row = json.loads(result_path.read_text())
        assert row["tier"] == "L0"
        assert "commit" in row and row["commit"]
        assert row["outcome"] == "SUCCESS"
        assert int(row["n_drones"]) == n_drones
        arrived = row.get("arrived")
        if arrived is not None:
            assert int(arrived) == n_drones
        elif row.get("arrival_rate") is not None:
            assert float(row["arrival_rate"]) == 1.0
