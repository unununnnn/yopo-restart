.PHONY: venv run test regress nightly
venv:
	uv venv -q -p 3.12 .venv
	uv pip install -q -p .venv/bin/python --index-strategy unsafe-best-match -r requirements.txt
	uv pip install -q -p .venv/bin/python --no-deps -e ./core -e .
run:
	.venv/bin/python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0
test:
	.venv/bin/python -m pytest -q tests
	cd core && ../.venv/bin/python -m pytest -q -p no:cacheprovider -m 'not slow'
regress:
	scripts/regress.sh
nightly:
	scripts/nightly.sh
