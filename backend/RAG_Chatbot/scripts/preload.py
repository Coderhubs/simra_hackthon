import os
import glob
from typing import List, Dict, Any
from app.retriever import upsert_points, create_collection_if_not_exists
from dotenv import load_dotenv

load_dotenv()

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """
    Split text into overlapping chunks
    """
    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)

        # Move start forward by chunk_size - overlap to create overlap
        start = end - overlap

        # If the remaining text is less than chunk_size, take the rest
        if len(text) - start < chunk_size:
            if start < len(text):
                chunks.append(text[start:])
            break

    return chunks

def load_markdown_files(directory: str) -> List[Dict[str, Any]]:
    """
    Load all markdown files from the specified directory
    """
    md_files = glob.glob(os.path.join(directory, "**/*.md"), recursive=True)
    md_files += glob.glob(os.path.join(directory, "**/*.markdown"), recursive=True)

    documents = []
    for file_path in md_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

            # Create chunks from the content
            chunks = chunk_text(content)

            for i, chunk in enumerate(chunks):
                documents.append({
                    "text": chunk,
                    "metadata": {
                        "source": file_path,
                        "chunk_index": i,
                        "file_name": os.path.basename(file_path)
                    }
                })

    return documents

def main():
    """
    Main function to load markdown files, chunk them, embed them, and upsert to Qdrant
    """
    # Create collection if it doesn't exist
    create_collection_if_not_exists()

    # Load markdown files from the textbook directory
    textbook_dir = os.getenv("TEXTBOOK_DIR", ".")  # Default to current directory
    documents = load_markdown_files(textbook_dir)

    print(f"Loaded {len(documents)} document chunks from markdown files")

    if not documents:
        print("No documents found to ingest. Make sure markdown files exist in the specified directory.")
        return

    # Extract texts and metadatas
    texts = [doc["text"] for doc in documents]
    metadatas = [doc["metadata"] for doc in documents]

    print("Starting ingestion process...")

    # Upsert to Qdrant
    try:
        ids = upsert_points(texts, metadatas)
        print(f"Successfully ingested {len(ids)} documents with IDs: {ids[:5]}{'...' if len(ids) > 5 else ''}")
    except Exception as e:
        print(f"Error during ingestion: {e}")
        raise

if __name__ == "__main__":
    main()