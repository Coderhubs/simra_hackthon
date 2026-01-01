"""
Vercel entry point for the RAG Chatbot backend
This file serves as the single entry point for Vercel deployment
"""
import sys
import os

# Add the backend directory to the Python path to ensure imports work
current_dir = os.path.dirname(__file__)
backend_dir = os.path.join(current_dir, 'backend', 'RAG_Chatbot')
sys.path.insert(0, backend_dir)
sys.path.insert(0, os.path.join(backend_dir, 'app'))

# Import the main FastAPI app from the app module
from app.main import app

# Export the app for Vercel Python runtime
handler = app