#!/usr/bin/env bash
# Create a local venv and install Phase 1 dependencies.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

python3 -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .

echo "Environment ready. Activate with: source ${ROOT}/.venv/bin/activate"
echo "Run tests with: pytest tests"
