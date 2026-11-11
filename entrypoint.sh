#!/bin/bash

# Function to handle shutdown
cleanup() {
    echo "Shutting down services..."
    pkill -f "uvicorn"
    pkill -f "basalt_configurable"
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

export PYTHONPATH=/app/depthai-python

echo "🚀 Starting OAK-D VIO Extension"
PORT="${PORT:-8095}"
echo "📋 Web UI available at http://localhost:${PORT}"

# Start the FastAPI web server
cd /app
/app/venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port "${PORT}"
