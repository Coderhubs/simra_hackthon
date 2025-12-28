#!/usr/bin/env python3
"""
Script to start the RAG Chatbot backend server
"""
import uvicorn
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    print(f"Starting RAG Chatbot backend server on {host}:{port}")
    print("Environment variables loaded:")
    print(f"  - GEMINI_API_KEY: {'Set' if os.getenv('GEMINI_API_KEY') else 'Not set'}")
    print(f"  - QDRANT_URL: {os.getenv('QDRANT_URL', 'Not set')}")
    print(f"  - QDRANT_API_KEY: {'Set' if os.getenv('QDRANT_API_KEY') else 'Not set'}")

    uvicorn.run(
        "app.main:app",
        host=host,
        port=port,
        reload=True,  # Enable auto-reload for development
        log_level="info"
    )