#!/usr/bin/env bash
#
# Convenience wrapper around the dc-motor-sim console script, kept for the
# key=value calling convention used before the package restructure.
#
# Usage:
#   ./run_simulation.sh start_position=<deg> end_position=<deg> [controller=<fuzzy|pid>]
#
# Examples:
#   ./run_simulation.sh start_position=-90 end_position=45
#   ./run_simulation.sh start_position=-90 end_position=45 controller=pid
#
# Any other argument is forwarded to dc-motor-sim unchanged, so flags such as
# --no-plot, --seed, and --output-dir work here too.

set -euo pipefail

VENV_DIR="${VENV_DIR:-.venv}"

START_POSITION=""
END_POSITION=""
CONTROLLER="fuzzy"
EXTRA_ARGS=()

usage() {
    echo "Usage: $0 start_position=<deg> end_position=<deg> [controller=<fuzzy|pid>] [extra flags]"
    echo "Example: $0 start_position=-90 end_position=45 controller=pid"
}

for arg in "$@"; do
    case "$arg" in
        start_position=*) START_POSITION="${arg#*=}" ;;
        end_position=*)   END_POSITION="${arg#*=}" ;;
        controller=*)     CONTROLLER="${arg#*=}" ;;
        -h|--help)        usage; exit 0 ;;
        *)                EXTRA_ARGS+=("$arg") ;;
    esac
done

if [ -z "$START_POSITION" ] || [ -z "$END_POSITION" ]; then
    echo "Error: both start_position and end_position must be provided" >&2
    usage >&2
    exit 2
fi

if [ "$CONTROLLER" != "fuzzy" ] && [ "$CONTROLLER" != "pid" ]; then
    echo "Error: invalid controller '$CONTROLLER'; use 'fuzzy' or 'pid'" >&2
    exit 2
fi

# Prefer the project virtualenv, then anything already on PATH.
if [ -x "$VENV_DIR/bin/dc-motor-sim" ]; then
    RUNNER=("$VENV_DIR/bin/dc-motor-sim")
elif command -v dc-motor-sim >/dev/null 2>&1; then
    RUNNER=(dc-motor-sim)
else
    echo "Error: dc-motor-sim is not installed." >&2
    echo "Run ./setup.sh first, or 'pip install dc-motor-fuzzy-sim'." >&2
    exit 1
fi

echo "=========================================="
echo "DC Motor Position Control Simulation"
echo "  Start position: ${START_POSITION} deg"
echo "  Target position: ${END_POSITION} deg"
echo "  Controller: ${CONTROLLER}"
echo "=========================================="

exec "${RUNNER[@]}" "$START_POSITION" "$END_POSITION" \
    --controller "$CONTROLLER" ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
