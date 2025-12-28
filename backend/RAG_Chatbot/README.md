# Physical AI & Humanoid Robotics RAG Chatbot

This is a Retrieval-Augmented Generation (RAG) chatbot backend for the "Physical AI & Humanoid Robotics" book. The system combines vector search with Google's Gemini AI model to provide accurate, context-aware responses based on the book's content.

## Features

- **Vector Search**: Uses Qdrant for efficient similarity search
- **Google Gemini Integration**: Leverages Google's Gemini Pro model for responses
- **Flexible Ingestion**: Accepts content from various sources with metadata
- **Filtering**: Supports filtering by chapter, section, URL, and other metadata
- **Real-time Chat**: Provides conversational interface with context-aware responses

## Setup

### Prerequisites

- Python 3.8+
- Qdrant vector database (can be run locally or hosted)
- Google Gemini API key

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd rag_chatbot_backend
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables:
   ```bash
   cp .env.example .env
   ```

   Add your API keys:
   ```env
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

### Running the Application

1. Start the Qdrant server (if running locally):
   ```bash
   docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant
   ```

2. Run the FastAPI application:
   ```bash
   python run_server.py
   ```
   Or directly with uvicorn:
   ```bash
   uvicorn app.main:app --reload --port 8000
   ```

### Initial Data Ingestion

To preload the book content from markdown files:

1. Place your markdown files in the appropriate directory
2. Run the preload script:
   ```bash
   python scripts/preload.py
   ```

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/v1/ingest-from-frontend` - Ingest content from frontend
- `POST /api/v1/search-live-content` - Search and get RAG response
- `POST /chat` - Chat endpoint with RAG
- `POST /ingest` - General ingestion endpoint

## Architecture

The system is built with the following components:

- **FastAPI**: Web framework for API endpoints
- **Qdrant**: Vector database for similarity search
- **Sentence Transformers**: For generating text embeddings
- **Google Generative AI (Gemini)**: For language model responses
- **Pydantic**: For request/response validation

## Environment Variables

- `GEMINI_API_KEY`: Your Google Gemini API key
- `QDRANT_URL`: URL of your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `PORT`: Port to run the application on (default: 8000)

## Usage Examples

### Ingesting Content

```bash
curl -X POST "http://localhost:8000/api/v1/ingest-from-frontend" \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Your content here",
    "metadata": {
      "chapter": "1",
      "section": "1.1",
      "url": "https://example.com/chapter1"
    }
  }'
```

### Searching Content

```bash
curl -X POST "http://localhost:8000/api/v1/search-live-content" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is humanoid robotics?",
    "filters": {
      "chapter": "2"
    },
    "top_k": 5
  }'
```

### Chat Interface

```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain the key concepts of physical AI",
    "filters": {
      "section": "3.2"
    }
  }'
```

## Development

For development, run the application with auto-reload:

```bash
uvicorn app.main:app --reload
```

## License

[Specify your license here]