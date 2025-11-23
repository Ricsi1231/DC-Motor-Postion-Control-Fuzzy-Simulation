#!/bin/bash

# DC Motor Position Control - Run Simulation Script
# Usage: ./run_simulation.sh start_position=<value> end_position=<value>
# Example: ./run_simulation.sh start_position=-90 end_position=45

# Parse arguments
START_POSITION=""
END_POSITION=""

for arg in "$@"; do
    case $arg in
        start_position=*)
            START_POSITION="${arg#*=}"
            ;;
        end_position=*)
            END_POSITION="${arg#*=}"
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: ./run_simulation.sh start_position=<value> end_position=<value>"
            echo "Example: ./run_simulation.sh start_position=-90 end_position=45"
            exit 1
            ;;
    esac
done

# Check if both arguments are provided
if [ -z "$START_POSITION" ] || [ -z "$END_POSITION" ]; then
    echo "Error: Both start_position and end_position must be provided"
    echo "Usage: ./run_simulation.sh start_position=<value> end_position=<value>"
    echo "Example: ./run_simulation.sh start_position=-90 end_position=45"
    exit 1
fi

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Error: Virtual environment not found"
    echo "Please run ./setup.sh first to set up the environment"
    exit 1
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

if [ $? -ne 0 ]; then
    echo "Error: Failed to activate virtual environment"
    exit 1
fi

# Run the simulation
echo "=========================================="
echo "Starting DC Motor Position Control Simulation"
echo "Start Position: $START_POSITION degrees"
echo "End Position: $END_POSITION degrees"
echo "=========================================="
echo ""

python main.py "$START_POSITION" "$END_POSITION"

# Capture exit status
EXIT_STATUS=$?

if [ $EXIT_STATUS -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Simulation completed successfully!"
    echo "=========================================="
else
    echo ""
    echo "=========================================="
    echo "Simulation failed with exit code: $EXIT_STATUS"
    echo "=========================================="
fi

exit $EXIT_STATUS
