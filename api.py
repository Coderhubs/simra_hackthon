"""
Vercel entry point for the RAG Chatbot backend
This file serves as the single entry point for Vercel deployment
"""
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'RAG_Chatbot'))

# Import the main FastAPI app from the app module
from backend.RAG_Chatbot.app.main import app

# This ensures Vercel can properly serve the FastAPI application
if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)