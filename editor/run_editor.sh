#!/bin/sh
# 에디터 실행: 처음 한 번 .venv 를 만들고 PyQt5 5.12 를 설치한 뒤 simul_editor.py 를 실행한다.
cd "$(dirname "$0")/.." || exit 1
[ -x .venv/bin/python ] || python3 -m venv .venv || exit 1
.venv/bin/python -c "import PyQt5" 2>/dev/null || .venv/bin/pip install "PyQt5>=5.12,<5.16" || exit 1
export SIMUL_LIB="${SIMUL_LIB:-$PWD/build/lib}"
exec .venv/bin/python editor/simul_editor.py "$@"
