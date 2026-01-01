from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any
import os
from dotenv import load_dotenv

# Load configuration
from app.config import DEFAULT_HOST, DEFAULT_PORT

# Load environment variables
load_dotenv()

# Import schemas
from app.schemas import (
    LiveContentRequest, IngestionResponse, QueryWithMetadata,
    IngestRequest, ChatRequest, ChatResponse
)

# Import modules
from app.retriever import (
    create_collection_if_not_exists, upsert_points, search, get_collection_name
)
from app.generator import get_agent_response

# Create FastAPI app

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot API",
    description="API for the Physical AI & Humanoid Robotics book RAG system",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/api/openapi.json"
)


# Initialize the collection when the application starts
@app.on_event("startup")
async def startup_event():
    create_collection_if_not_exists()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """
    Root endpoint to check if the API is running
    """
    return {
        "message": "Physical AI & Humanoid Robotics RAG Chatbot API",
        "collection": get_collection_name(),
        "status": "running"
    }

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {
        "status": "healthy",
        "service": "RAG Chatbot Backend"
    }

@app.post("/api/v1/ingest-from-frontend", response_model=IngestionResponse)
async def ingest_from_frontend(request: LiveContentRequest):
    """
    Ingest content from frontend with metadata
    """
    try:
        # Upsert the content to Qdrant
        ids = upsert_points([request.content], [request.metadata])

        return IngestionResponse(
            success=True,
            message="Content ingested successfully",
            ids=ids
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to ingest content: {str(e)}")

@app.post("/api/v1/search-live-content", response_model=ChatResponse)
async def search_live_content(request: QueryWithMetadata):
    """
    Search live content and generate response using RAG
    """
    try:
        # Perform search if no selected_text is provided
        contexts = []
        if not request.selected_text:
            search_results = search(
                query=request.query,
                filters=request.filters,
                top_k=request.top_k
            )
            contexts = [result["text"] for result in search_results]

        # Get response from the agent
        agent_response = get_agent_response(
            query=request.query,
            contexts=contexts if not request.selected_text else None,
            selected_text=request.selected_text
        )

        return ChatResponse(
            response=agent_response["response"],
            context=[{"text": ctx, "score": 1.0} for ctx in contexts] if contexts else None
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to search content: {str(e)}")

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint using RAG
    """
    try:
        # Perform search if no selected_text is provided
        contexts = []
        if not request.selected_text:
            search_results = search(
                query=request.message,
                filters=request.filters,
                top_k=5  # Default top_k
            )
            contexts = [result["text"] for result in search_results]

        # Get response from the agent
        agent_response = get_agent_response(
            query=request.message,
            contexts=contexts if not request.selected_text else None,
            selected_text=request.selected_text
        )

        return ChatResponse(
            response=agent_response["response"],
            context=[{"text": ctx, "score": 1.0} for ctx in contexts] if contexts else None
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to generate chat response: {str(e)}")

@app.post("/ingest", response_model=IngestionResponse)
async def ingest(request: IngestRequest):
    """
    Ingest endpoint for general content
    """
    try:
        # Upsert the content to Qdrant
        ids = upsert_points([request.content], [request.metadata])

        return IngestionResponse(
            success=True,
            message="Content ingested successfully",
            ids=ids
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to ingest content: {str(e)}")

# Include the OpenAPI spec
if __name__ == "__main__":
    import uvicorn
    from app.config import DEFAULT_HOST, DEFAULT_PORT
    uvicorn.run(app, host=DEFAULT_HOST, port=DEFAULT_PORT)