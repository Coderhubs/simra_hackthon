from sentence_transformers import SentenceTransformer
from qdrant_client.http import models
from typing import Dict, Any, Optional, List
import logging

# Initialize the embedding model (lazy loading)
embedding_model = None

logger = logging.getLogger(__name__)


def embed_text(text: str) -> List[float]:
    """
    Generate embeddings for the given text using SentenceTransformer
    """
    global embedding_model
    if embedding_model is None:
        embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
    embedding = embedding_model.encode(text)
    return embedding.tolist()


def build_filter(filters: Optional[Dict[str, Any]] = None) -> Optional[models.Filter]:
    """
    Build Qdrant filter from dictionary
    """
    if not filters:
        return None

    conditions = []
    for key, value in filters.items():
        if isinstance(value, list):
            # Handle list of values (OR condition)
            or_conditions = []
            for val in value:
                or_conditions.append(
                    models.FieldCondition(
                        key=f"metadata.{key}",
                        match=models.MatchValue(value=val)
                    )
                )
            conditions.append(models.Should(conditions=or_conditions))
        else:
            # Handle single value
            conditions.append(
                models.FieldCondition(
                    key=f"metadata.{key}",
                    match=models.MatchValue(value=value)
                )
            )

    if conditions:
        return models.Filter(must=conditions)

    return None


def generate_rag_prompt(query: str, contexts: List[str], selected_text: Optional[str] = None) -> str:
    """
    Generate a RAG prompt combining the query with retrieved contexts or selected text
    """
    if selected_text:
        # Use selected text as the sole context
        return f"Context: {selected_text}\n\nQuestion: {query}\n\nAnswer:"

    if contexts:
        # Combine retrieved contexts
        context_str = "\n\n".join(contexts)
        return f"Context:\n{context_str}\n\nQuestion: {query}\n\nAnswer:"

    # No context available
    return f"Question: {query}\n\nAnswer:"