from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from uuid import uuid4
import os
from dotenv import load_dotenv

load_dotenv()

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_api_key:
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
else:
    qdrant_client = QdrantClient(url=qdrant_url)

COLLECTION_NAME = "physical_ai_humanoid_robots"


def get_collection_name() -> str:
    """Return the collection name"""
    return COLLECTION_NAME


def create_collection_if_not_exists():
    """Create the collection if it doesn't exist"""
    try:
        collections = qdrant_client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        if COLLECTION_NAME not in collection_names:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=384,  # all-MiniLM-L6-v2 produces 384-dimensional vectors
                    distance=models.Distance.COSINE
                )
            )
            print(f"Collection '{COLLECTION_NAME}' created successfully.")
        else:
            print(f"Collection '{COLLECTION_NAME}' already exists.")
    except Exception as e:
        print(f"Warning: Could not connect to Qdrant or create collection: {e}")
        # Don't raise the exception, just warn - this allows the app to continue running


def upsert_points(texts: List[str], metadatas: List[Dict[str, Any]]) -> List[str]:
    """
    Upsert points to Qdrant collection
    """
    # Create embeddings for the texts
    from app.utils import embed_text

    try:
        points = []
        ids = []

        for text, metadata in zip(texts, metadatas):
            vector = embed_text(text)
            point_id = str(uuid4())
            ids.append(point_id)

            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        "text": text,
                        "metadata": metadata
                    }
                )
            )

        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )

        return ids
    except Exception as e:
        print(f"Warning: Could not upsert points to Qdrant: {e}")
        # Return empty list if Qdrant is not available
        return []


def search(query: str, filters: Optional[Dict[str, Any]] = None, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Search in Qdrant collection
    """
    from app.utils import embed_text, build_filter

    try:
        query_vector = embed_text(query)
        qdrant_filter = build_filter(filters)

        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            query_filter=qdrant_filter,
            limit=top_k
        )

        results = []
        for hit in search_results:
            results.append({
                "id": hit.id,
                "text": hit.payload.get("text", ""),
                "metadata": hit.payload.get("metadata", {}),
                "score": hit.score
            })

        return results
    except Exception as e:
        print(f"Warning: Could not perform search in Qdrant: {e}")
        # Return empty results if Qdrant is not available
        return []