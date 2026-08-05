#!/usr/bin/env bash

# Verify 10 command-line arguments are passed 
if [ $# -ne 10 ]; then
    echo "Usage: ./zigzag_corridor.sh --config [path] --width [width] --height [height] --count [count] --gamma [gamma]"
    exit 1
fi

# Create the XML of the zigzag corridor
xmlpath=$(python3 zigzag_corridor.py --config "$2" --width "$4" --height "$6" --count "$8" --gamma "${10}")

# Start simulation
mjpython run.py --config config/paper_experiments/obstacle_course.json --env "$xmlpath" --gui --log-level DEBUG

# width=0.45 and height=0.25 work well.
