#!/usr/bin/env bash
# Launches metaboard_osc_monitor.py with a Python that has _tkinter (Homebrew python@X alone does not).

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pick_python() {
  if [[ -x "$ROOT/venv/bin/python3" ]]; then
    if "$ROOT/venv/bin/python3" -c "import _tkinter" 2>/dev/null; then
      echo "$ROOT/venv/bin/python3"
      return 0
    fi
  fi
  for p in \
    "/usr/local/opt/python@3.14/bin/python3.14" \
    "/opt/homebrew/opt/python@3.14/bin/python3.14" \
    "/usr/local/opt/python@3.13/bin/python3.13" \
    "/opt/homebrew/opt/python@3.13/bin/python3.13" \
    "/usr/local/opt/python@3.12/bin/python3.12" \
    "/opt/homebrew/opt/python@3.12/bin/python3.12"
  do
    if [[ -x "$p" ]] && "$p" -c "import _tkinter" 2>/dev/null; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

PY="$(pick_python)" || {
  echo "No Python with Tk (_tkinter) found." >&2
  echo "If you use Homebrew Python 3.14, also run: brew install python-tk@3.14" >&2
  echo "Then recreate the venv: rm -rf venv && /usr/local/opt/python@3.14/bin/python3.14 -m venv venv && source venv/bin/activate && pip install -r requirements.txt" >&2
  exit 1
}

exec "$PY" "$ROOT/metaboard_osc_monitor.py" "$@"
