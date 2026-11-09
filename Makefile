.PHONY: venv run test regress
venv:
	python3 -m venv .venv && .venv/bin/pip -q install -e '.[test]'
run:
	.venv/bin/python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0
test:
	.venv/bin/python -m pytest -q
regress:
	scripts/regress.sh
