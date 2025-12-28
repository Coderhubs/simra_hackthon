#!/usr/bin/env python3
"""
Simple test to verify the RAG chatbot backend can be imported and started
"""
import sys
import os

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_imports():
    """Test that all modules can be imported without errors"""
    try:
        from app import schemas, utils, retriever, generator, main
        print("+ All modules imported successfully")
        return True
    except ImportError as e:
        print(f"- Import error: {e}")
        return False
    except Exception as e:
        print(f"- Unexpected error during import: {e}")
        return False

def test_environment():
    """Test that required environment variables are available"""
    import os
    from dotenv import load_dotenv
    load_dotenv()

    required_vars = ['GEMINI_API_KEY', 'QDRANT_URL']
    missing_vars = []

    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"? Missing environment variables: {missing_vars} (these are needed for full functionality)")
    else:
        print("+ All required environment variables are set")

    return True

def test_embedding():
    """Test that embedding functionality works"""
    try:
        from app.utils import embed_text
        test_text = "This is a test sentence for embedding."
        embedding = embed_text(test_text)

        if isinstance(embedding, list) and len(embedding) > 0:
            print(f"+ Embedding works: generated {len(embedding)}-dimensional vector")
            return True
        else:
            print("- Embedding failed: returned invalid result")
            return False
    except Exception as e:
        print(f"- Embedding error: {e}")
        return False

if __name__ == "__main__":
    print("Testing RAG Chatbot Backend...")
    print()

    success = True
    success &= test_imports()
    print()

    success &= test_environment()
    print()

    success &= test_embedding()
    print()

    if success:
        print("+ All tests passed! The RAG chatbot backend is ready.")
    else:
        print("- Some tests failed. Please check the output above.")
        sys.exit(1)