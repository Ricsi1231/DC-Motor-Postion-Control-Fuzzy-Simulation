#!/bin/bash

# DC Motor Position Control - Setup Script
# This script sets up the virtual environment and installs dependencies

echo "=========================================="
echo "DC Motor Position Control - Setup"
echo "=========================================="

echo "Creating virtual environment..."
python3 -m venv venv

if [ $? -ne 0 ]; then
    echo "Error: Failed to create virtual environment"
    exit 1
fi

echo "Activating virtual environment..."
source venv/bin/activate

if [ $? -ne 0 ]; then
    echo "Error: Failed to activate virtual environment"
    exit 1
fi

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install required packages
echo "Installing required packages from requirements.txt..."
pip install -r requirements.txt

if [ $? -ne 0 ]; then
    echo "Error: Failed to install packages"
    exit 1
fi

echo "=========================================="
echo "Setup completed successfully!"
echo "=========================================="
echo ""
echo "To activate the virtual environment manually, run:"
echo "  source venv/bin/activate"
echo ""
echo "To run the simulation, use:"
echo "  ./run_simulation.sh start_position=<value> end_position=<value>"
echo "  Example: ./run_simulation.sh start_position=-90 end_position=45"
