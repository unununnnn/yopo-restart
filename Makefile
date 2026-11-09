.PHONY: venv run test regress nightly

# 第三方依赖按 requirements.txt + pyproject 的哈希放进共享 venv
# （YOPO_DEPS_ROOT/<hash>）。本目录 .venv 只做 editable，用 .pth 挂上共享
# site-packages。哈希未变且 .venv/bin/python、共享 venv 都在则直接跳过，
# 避免 `uv venv` 在 .venv 已存在时报 virtual environment already exists。
# wheel 走 UV_CACHE_DIR，新 worktree 不重装 torch。
CACHE_HOME := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)
UV_CACHE_DIR ?= $(CACHE_HOME)/uv
YOPO_DEPS_ROOT ?= $(CACHE_HOME)/yopo-restart/deps
export UV_CACHE_DIR

venv:
	@set -eu; \
	if ! command -v uv >/dev/null 2>&1 && [ -x "$(HOME)/.local/bin/uv" ]; then \
	  PATH="$(HOME)/.local/bin:$$PATH"; \
	  export PATH; \
	fi; \
	hash=$$(cat requirements.txt pyproject.toml core/pyproject.toml | sha256sum | awk '{print $$1}'); \
	if [ -x .venv/bin/python ] && [ -f .venv/.deps-hash ] && [ "$$(cat .venv/.deps-hash)" = "$$hash" ] && [ -f .venv/.deps-shared ] && [ -x "$$(cat .venv/.deps-shared)/bin/python" ]; then \
	  exit 0; \
	fi; \
	echo "make venv: 安装依赖 $$hash"; \
	mkdir -p "$(YOPO_DEPS_ROOT)"; \
	exec 9>"$(YOPO_DEPS_ROOT)/.lock"; \
	flock 9; \
	shared="$(YOPO_DEPS_ROOT)/$$hash"; \
	if [ ! -x "$$shared/bin/python" ] || [ ! -f "$$shared/.complete" ]; then \
	  rm -rf "$$shared"; \
	  uv venv -q -p 3.12 "$$shared"; \
	  uv pip install -q -p "$$shared/bin/python" --index-strategy unsafe-best-match -r requirements.txt; \
	  touch "$$shared/.complete"; \
	fi; \
	if [ ! -x .venv/bin/python ]; then \
	  rm -rf .venv; \
	  uv venv -q -p "$$shared/bin/python" .venv; \
	fi; \
	pure=$$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))'); \
	shared_pure=$$("$$shared/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))'); \
	mkdir -p "$$pure"; \
	printf '%s\n' "$$shared_pure" > "$$pure/_yopo_shared.pth"; \
	uv pip install -q -p .venv/bin/python --no-deps -e ./core -e .; \
	printf '%s\n' "$$hash" > .venv/.deps-hash; \
	printf '%s\n' "$$shared" > .venv/.deps-shared

run:
	.venv/bin/python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0
test:
	.venv/bin/python -m pytest -q tests
	cd core && ../.venv/bin/python -m pytest -q -p no:cacheprovider -m 'not slow'
regress:
	scripts/regress.sh
nightly:
	scripts/nightly.sh
