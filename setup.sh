#!/usr/bin/env bash
#
# Development environment setup.
#
# Creates a virtual environment in .venv and installs the package in editable
# mode with its development dependencies.
#
# For plain use you do not need this script at all:
#     pip install dc-motor-fuzzy-sim

set -euo pipefail

VENV_DIR="${VENV_DIR:-.venv}"
PYTHON="${PYTHON:-python3}"

echo "=========================================="
echo "DC Motor Position Control - Dev Setup"
echo "=========================================="

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment in $VENV_DIR ..."
    "$PYTHON" -m venv "$VENV_DIR"
else
    echo "Reusing existing virtual environment in $VENV_DIR"
fi

echo "Upgrading pip ..."
"$VENV_DIR/bin/python" -m pip install --upgrade pip

echo "Installing the package with development dependencies ..."
"$VENV_DIR/bin/python" -m pip install -e '.[dev]'

if "$VENV_DIR/bin/python" -c 'import pre_commit' 2>/dev/null; then
    echo "Installing pre-commit hooks ..."
    "$VENV_DIR/bin/pre-commit" install
fi

echo
echo "=========================================="
echo "Setup complete."
echo "=========================================="
echo
echo "Activate the environment:"
echo "  source $VENV_DIR/bin/activate"
echo
echo "Run the simulation:"
echo "  dc-motor-sim -90 45"
echo "  dc-motor-sim -90 45 --controller pid"
echo
echo "Run the tests:"
echo "  pytest"
