#!/usr/bin/env bash

set -e

# Verify 10 command-line arguments are passed 
if [ $# -ne 10 ]; then
    echo "Usage: $0 --config [path] --width [width] --height [height] --count [count] --gamma [gamma]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Interpret relative paths from the repository root so the wrapper can be run
# from any working directory.
cd "$REPO_ROOT"

# Create the XML of the zigzag corridor
xmlpath=$(PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" python3 "$SCRIPT_DIR/zigzag_corridor.py" \
    --dir-path "$REPO_ROOT/config/env" \
    --config "$2" \
    --width "$4" \
    --height "$6" \
    --count "$8" \
    --gamma "${10}")

# Start a simulation with the same configuration used to generate the corridor.
mjpython "$REPO_ROOT/run.py" --config "$2" --env "$xmlpath" --gui --log-level DEBUG

# width=0.45 and height=0.25 work well.
