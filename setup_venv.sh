#!/bin/bash

# Define the environment name and directory
ENV_NAME="shaq_and_koby_venv"
ENV_DIR="$PWD/$ENV_NAME"

# Create virtual environment if it doesn't exist
if [ ! -d "$ENV_DIR" ]; then
    echo "🔧 Virtual environment not found. Creating..."
    if ! python3 -m venv "$ENV_DIR"; then
        echo "❌ Failed to create virtual environment."
        exit 1
    fi
else
    echo "✅ Virtual environment already exists."
fi

# Check if activation line already exists in ~/.bashrc to avoid duplicates
ACTIVATION_LINE="source $ENV_DIR/bin/activate"
if ! grep -Fxq "$ACTIVATION_LINE" ~/.bashrc; then
    echo "Adding virtual environment activation to ~/.bashrc"
    echo "$ACTIVATION_LINE" >> ~/.bashrc
else
    echo "Virtual environment activation already in ~/.bashrc"
fi

echo "⚠️ To activate the virtual environment in your current shell, run:"
echo "source $ENV_DIR/bin/activate"
echo "Or restart your terminal to auto-activate it."

# # Install dependencies if requirements.txt exists
# if [ -f "requirements.txt" ]; then
#     echo "📦 Installing dependencies from requirements.txt..."
#     # Activate environment temporarily for this session to install packages
#     source "$ENV_DIR/bin/activate"
#     if ! pip install --upgrade pip; then
#         echo "⚠️ Failed to upgrade pip."
#     fi
#     if ! pip install -r requirements.txt; then
#         echo "❌ Failed to install some or all dependencies."
#         # deactivate
#         exit 1
#     fi
#     # deactivate
# else
#     echo "⚠️ No requirements.txt found. Skipping dependency installation."
# fi

# echo "✅ Environment setup complete!"
