#!/bin/bash

# Define the environment name and the path where the virtual environment will be created
ENV_NAME="shaq_and_koby_venv"
ENV_DIR="$PWD/$ENV_NAME"  # You can change this to any other folder path if needed

# Check if the virtual environment exists
if [ ! -d "$ENV_DIR" ]; then
    echo "Virtual environment not found. Creating a new one..."
    python3 -m venv $ENV_DIR  # Create a new virtual environment in the specified folder
else
    echo "Virtual environment already exists."
fi

# Activate the virtual environment
source $ENV_DIR/bin/activate

# Install the requirements from requirements.txt
if [ -f "requirements.txt" ]; then
    echo "Installing dependencies from requirements.txt..."
    pip install -r requirements.txt
else
    echo "No requirements.txt found!"
fi

# Confirmation message
echo "Environment setup complete!"
