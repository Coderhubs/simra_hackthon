"""
Configuration file for the Physical AI & Humanoid Robotics RAG Chatbot
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# API Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Model Configuration
EMBEDDING_MODEL_NAME = "all-MiniLM-L6-v2"
GENERATIVE_MODEL_NAME = "gemini-2.5-flash"

# Collection Configuration
COLLECTION_NAME = "physical_ai_humanoid_robots"

# Server Configuration
DEFAULT_PORT = int(os.getenv("PORT", 8003))
DEFAULT_HOST = os.getenv("HOST", "0.0.0.0")

# API Configuration
API_V1_STR = "/api/v1"