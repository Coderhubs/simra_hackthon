# Feature: Integrated RAG Chatbot for Book Content with Stunning UI

## Phase 1: Backend Infrastructure Setup

- [x] T001 Create the base directory structure for the FastAPI application (e.g., `backend/RAG_Chatbot/src/`).
- [x] T002 Initialize `uv` project with `pyproject.toml` including `fastapi`, `uvicorn`, `openai[agents]`, `qdrant-client`, `sqlmodel`, `alembic`, `python-dotenv`, `sentence-transformers`, `tqdm`, `pymupdf`, `pydantic-settings`.
- [x] T003 Configure `python-dotenv` for managing environment variables (e.g., API keys, database URLs).
- [x] T004 Define application settings using `pydantic-settings` in `backend/RAG_Chatbot/src/settings.py`.
- [x] T005 Set up Neon Postgres database connection using SQLModel in `backend/RAG_Chatbot/src/database.py`.
- [x] T006 Initialize Alembic for database migrations in `backend/RAG_Chatbot/migrations/` with `alembic.ini` and `env.py`.
- [x] T007 Create an initial Alembic migration script for base database schema.
- [x] T008 Implement the core FastAPI application instance in `backend/RAG_Chatbot/src/main.py`.

## Phase 2: Qdrant Integration and RAG Tool Development

**Goal**: Enable the chatbot to retrieve relevant information from Qdrant Cloud.

- [ ] T009 [P] Initialize `QdrantClient` for Qdrant Cloud Free Tier in `backend/RAG_Chatbot/src/qdrant_client.py`.
- [ ] T010 [P] Implement embedding model initialization using `sentence-transformers` for document and query embeddings in `backend/RAG_Chatbot/src/embeddings.py`.
- [ ] T011 Develop the `search_qdrant` function that queries Qdrant with `top-k=5` and formats the retrieved context in `backend/RAG_Chatbot/src/rag_tool.py`.
- [ ] T012 Wrap the `search_qdrant` function as a custom tool for the OpenAI Agents SDK in `backend/RAG_Chatbot/src/rag_tool.py`.
- [ ] T013 Implement document ingestion logic using `pymupdf` to parse book content and generate embeddings, then store in Qdrant, in `backend/RAG_Chatbot/src/document_processor.py`.
- [ ] T014 Create a `POST /upload-book-content` endpoint in `backend/RAG_Chatbot/src/main.py` for ingesting book content into Qdrant.

## Phase 3: OpenAI Agent and Core Chat Endpoint

**Goal**: Integrate the OpenAI Agent and establish the primary chat interface.

- [ ] T015 Initialize the OpenAI `AgentExecutor` instance in `backend/RAG_Chatbot/src/agent.py`.
- [ ] T016 Add the custom RAG tool (from T012) to the `AgentExecutor`'s list of available tools in `backend/RAG_Chatbot/src/agent.py`.
- [ ] T017 Implement logic for streaming responses from the `AgentExecutor` to the client in `backend/RAG_Chatbot/src/agent.py`.
- [ ] T018 Create a `POST /chat` endpoint in `backend/RAG_Chatbot/src/main.py` for handling chat interactions.
- [ ] T019 Integrate the `AgentExecutor` with the `POST /chat` endpoint to process user queries and generate responses in `backend/RAG_Chatbot/src/main.py`.

## Phase 4: Chat Modes Implementation (`full` and `selection`)

**Goal**: Implement the two distinct chat interaction modes as required.

- [ ] T020 Modify the `POST /chat` endpoint in `backend/RAG_Chatbot/src/main.py` to accept a `mode` parameter (`"full"` or `"selection"`) and an optional `selected_text` parameter.
- [ ] T021 Implement the `mode="full"` logic within `POST /chat`, allowing the agent to automatically decide when to use the RAG tool for context.
- [ ] T022 Implement the `mode="selection"` logic within `POST /chat`, injecting `selected_text` as forced context for the agent and explicitly preventing the RAG tool from being called.

## Phase 5: Data Persistence with Neon Postgres

**Goal**: Store chat history and book content metadata in the Neon Postgres database.

- [ ] T023 Define SQLModel for `ChatMessage` in `backend/RAG_Chatbot/src/models/chat.py` (e.g., fields for `session_id`, `role`, `content`, `timestamp`).
- [ ] T024 Define SQLModel for `BookContentMetadata` in `backend/RAG_Chatbot/src/models/book.py` (e.g., fields for `filename`, `upload_timestamp`, `qdrant_collection_id`).
- [ ] T025 Generate a new Alembic migration script to add the `ChatMessage` and `BookContentMetadata` tables to the database.
- [ ] T026 Implement CRUD operations for `ChatMessage` in `backend/RAG_Chatbot/src/services/chat_history_service.py`.
- [ ] T027 Integrate `chat_history_service` into the `POST /chat` endpoint to save chat messages in `backend/RAG_Chatbot/src/main.py`.
- [ ] T028 Implement CRUD operations for `BookContentMetadata` in `backend/RAG_Chatbot/src/services/book_content_service.py`.
- [ ] T029 Integrate `book_content_service` into the `POST /upload-book-content` endpoint to save metadata after ingestion in `backend/RAG_Chatbot/src/main.py`.

## Phase 6: Backend Polish and Documentation

- [ ] T030 Implement comprehensive error handling and logging across the backend application.
- [ ] T031 Create a `README.md` for the `backend/RAG_Chatbot/` directory detailing setup, environment variables, running the application, and API endpoints.
- [ ] T032 Review and refine `pyproject.toml` to ensure all dependencies are correctly specified and project metadata is accurate.

## Phase 7: Frontend UI Development (React)

**Goal**: Build a stunning, interactive chat interface for the RAG chatbot.

- [ ] T033 Create the frontend project structure (e.g., `frontend/rag-chatbot-ui/`).
- [ ] T034 Initialize a React project within `frontend/rag-chatbot-ui/` using Vite or Create React App.
- [ ] T035 [P] Design and implement the main chat layout component in `frontend/rag-chatbot-ui/src/components/ChatLayout.tsx`.
- [ ] T036 [P] Develop a `MessageDisplay` component to render chat messages, including user queries and agent responses (streaming), in `frontend/rag-chatbot-ui/src/components/MessageDisplay.tsx`.
- [ ] T037 [P] Create a `ChatInput` component for user text input and sending messages in `frontend/rag-chatbot-ui/src/components/ChatInput.tsx`.
- [ ] T038 Implement a `SelectedTextContext` or similar mechanism to handle and inject user-selected text for the "selection" chat mode in `frontend/rag-chatbot-ui/src/context/SelectedTextContext.tsx`.
- [ ] T039 Implement API integration logic to communicate with the FastAPI backend (`/chat`, `/upload-book-content` endpoints) in `frontend/rag-chatbot-ui/src/services/api.ts`.
- [ ] T040 Develop the main `App` component to orchestrate `ChatLayout`, `MessageDisplay`, `ChatInput`, and integrate with API services in `frontend/rag-chatbot-ui/src/App.tsx`.
- [ ] T041 Implement styling (e.g., using CSS modules, Tailwind CSS, or a UI library like Chakra UI/Material-UI) to achieve a stunning and responsive design in `frontend/rag-chatbot-ui/src/styles/` and component-specific styles.
- [ ] T042 Add functionality for uploading book content via the UI to the `/upload-book-content` endpoint in `frontend/rag-chatbot-ui/src/components/BookUploader.tsx`.
- [ ] T043 Implement a toggle or selection mechanism for "full" vs. "selection" chat modes in `frontend/rag-chatbot-ui/src/components/ChatModeSelector.tsx`.
- [ ] T044 Ensure robust error handling and user feedback for API calls and streaming responses in the UI.
- [ ] T045 Implement responsive design for various screen sizes (desktop, tablet, mobile).
- [ ] T046 Create a `README.md` for the `frontend/rag-chatbot-ui/` directory with setup and usage instructions.

## Dependencies

- Phase 1 must complete before Phase 2, Phase 3, Phase 5.
- Phase 2 (Qdrant) and Phase 3 (OpenAI Agent) can proceed in parallel once Phase 1 is done.
- Phase 4 (Chat Modes) depends on Phase 3.
- Phase 5 (Persistence) depends on Phase 1 and its services will integrate with endpoints from Phase 3 and Phase 2.
- Phase 6 (Backend Polish) depends on all previous backend phases.
- Phase 7 (Frontend) depends on backend APIs (from Phases 3, 4, 5) being available, but initial UI structure can be developed in parallel with later backend phases.

## Parallel Execution Examples

- **Backend Phases 2 & 3 Parallelism**:
    - T009-T010 (Qdrant client & embeddings) can be developed alongside T015 (AgentExecutor setup).
    - T011 (search_qdrant) can be developed in parallel with T017 (streaming response logic).
- **Backend Phase 5 (Persistence) Parallelism**:
    - T023 (ChatMessage model) and T024 (BookContentMetadata model) can be defined independently and in parallel.
    - T026 (Chat history service) and T028 (Book content service) can be implemented in parallel after their respective models are defined.
- **Frontend Phase 7 Parallelism**:
    - T035 (ChatLayout), T036 (MessageDisplay), T037 (ChatInput) can be developed in parallel.
    - T038 (SelectedTextContext) and T039 (API integration) can be developed concurrently after core backend endpoints are defined.

## Implementation Strategy

The implementation will follow a modular and iterative approach:

1.  **Core Backend Infrastructure**: Establish the FastAPI application, settings, and database connection.
2.  **RAG Foundation**: Get Qdrant integrated and the custom RAG tool working.
3.  **Core Chat Backend**: Set up the OpenAI Agent and link it with the RAG tool and the main chat endpoint.
4.  **Chat Modes**: Implement the specific "full" and "selection" chat behaviors on the backend.
5.  **Data Persistence**: Add chat history and book content metadata storage on the backend.
6.  **Basic Frontend UI**: Create the core React application and basic chat interface components.
7.  **Frontend-Backend Integration**: Connect the frontend UI to the backend APIs.
8.  **Stunning UI/UX**: Focus on styling, responsiveness, and user experience enhancements for the frontend.
9.  **Refinement**: Address comprehensive error handling, logging, and documentation for both backend and frontend.

## Report

- tasks.md generated successfully, specifically addressing the backend and frontend requirements for "Integrated RAG Chatbot Development with Stunning UI."
- Total tasks: 46 (32 backend + 14 frontend)
- Tasks per phase:
    - Backend Infrastructure Setup: 8
    - Qdrant Integration and RAG Tool Development: 6
    - OpenAI Agent and Core Chat Endpoint: 5
    - Chat Modes Implementation: 3
    - Data Persistence with Neon Postgres: 7
    - Backend Polish and Documentation: 3
    - Frontend UI Development (React): 14
- Parallel opportunities are identified across both backend and frontend to optimize development flow.
- Each task is designed to be specific and independently actionable.
- The implementation strategy emphasizes a structured, iterative build-out, starting with backend core, then basic frontend, then integration and UI polish.