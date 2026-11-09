import copy
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
G2_CARD = ROOT / "scenes/g2-hover.json"


@pytest.fixture
def g2_geometry():
    return json.loads((ROOT / "scenes/g2-scene.json").read_text())


@pytest.fixture
def make_card(tmp_path):
    """基于 g2-hover 改字段后另存一张卡。"""
    from yopo_l0.scene import load_card

    def _make(geometry=None, **changes):
        card = json.loads(G2_CARD.read_text())
        card = copy.deepcopy(card)
        card.update(changes)
        (tmp_path / "g2-scene.json").write_text((ROOT / "scenes/g2-scene.json").read_text())
        p = tmp_path / f"card{len(list(tmp_path.glob('card*')))}.json"
        p.write_text(json.dumps(card))
        return load_card(p, geometry_override=geometry)

    return _make
