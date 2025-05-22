# Install dependencies if requirements.txt exists
if [ -f "requirements.txt" ]; then
    echo "📦 Installing dependencies from requirements.txt..."
    # Activate environment temporarily for this session to install packages
    source "$ENV_DIR/bin/activate"
    if ! pip install --upgrade pip; then
        echo "⚠️ Failed to upgrade pip."
    fi
    if ! pip install -r requirements.txt; then
        echo "❌ Failed to install some or all dependencies."
        # deactivate
        exit 1
    fi
    # deactivate
else
    echo "⚠️ No requirements.txt found. Skipping dependency installation."
fi

echo "✅ Environment setup complete!"
