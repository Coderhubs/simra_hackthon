#!/usr/bin/env python3
"""
Test script to run the RAG Chatbot with mock services for testing
"""
import os
import threading
import time
import requests
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Set up mock environment variables for testing
os.environ.setdefault('GEMINI_API_KEY', 'mock-key-for-testing')
os.environ.setdefault('QDRANT_URL', 'http://localhost:6333')
os.environ.setdefault('QDRANT_API_KEY', '')

def check_server_health():
    """Check if the server is running"""
    try:
        response = requests.get("http://localhost:8000/health", timeout=5)
        if response.status_code == 200:
            print("✓ Server is running and healthy")
            print(f"Response: {response.json()}")
            return True
    except requests.exceptions.RequestException:
        pass
    return False

def run_server():
    """Run the server in a separate thread"""
    import uvicorn
    from app.main import app

    print("Starting RAG Chatbot server...")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

if __name__ == "__main__":
    print("Setting up test environment...")

    # Start the server in a separate thread
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    # Wait a bit for the server to start
    print("Waiting for server to start...")
    time.sleep(3)

    # Check if the server is running
    if check_server_health():
        print("\n✓ RAG Chatbot server is running successfully!")
        print("✓ You can access the API at http://localhost:8000")
        print("✓ API documentation available at http://localhost:8000/docs")

        # Show some example endpoints
        print("\nExample endpoints:")
        print("  - GET /health - Health check")
        print("  - GET / - Root endpoint")
        print("  - POST /chat - Chat with the RAG system")
        print("  - POST /api/v1/search-live-content - Search content")

        print("\nPress Ctrl+C to stop the server")

        try:
            # Keep the main thread alive
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down server...")
    else:
        print("✗ Failed to start the server")
        print("Note: This could be because Qdrant is not running.")
        print("For full functionality, you need to:")
        print("1. Install Docker")
        print("2. Run: docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant")
        print("3. Set your GEMINI_API_KEY in the .env file")