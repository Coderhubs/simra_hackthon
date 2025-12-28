#!/usr/bin/env python3
"""
Script to preload the textbook content into the vector database
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the backend directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    print("Preloading textbook content into vector database...")

    # Import the preload functionality
    from scripts.preload import main as preload_main

    try:
        preload_main()
        print("\nPreloading completed successfully!")
    except Exception as e:
        print(f"\nError during preloading: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()