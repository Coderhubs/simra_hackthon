# Chatbot Integration for Physical AI & Humanoid Robotics Textbook

## Overview
This document explains how the AI chatbot is integrated into the digital textbook frontend. The chatbot allows users to ask questions about the book content and get answers based on the embedded textbook data.

## Features
- **Floating Chatbot Icon**: A chatbot icon appears on all textbook pages
- **Text Selection Context**: Users can select text on the page and ask questions about it
- **Book Content Knowledge**: The chatbot can answer questions about the entire book content
- **RAG (Retrieval-Augmented Generation)**: Uses vector search to find relevant content before generating responses

## Technical Implementation

### Frontend Components
1. **Chatbot Component** (`src/components/Chatbot/index.tsx`): The main chat interface with:
   - Floating icon that expands to a chat window
   - Message history display
   - Text input with send functionality
   - Selected text context display

2. **Integration** (`src/components/BookLayout/index.tsx`): The chatbot is integrated into the book layout component so it appears on all textbook pages.

### Backend Integration
- The frontend communicates with the RAG backend via API routes (development: `http://localhost:8002`, production: relative path `/api`)
- Uses the `/api/v1/search-live-content` endpoint for queries
- When text is selected, it's sent as `selected_text` parameter for focused context

### How Text Selection Works
1. Users can select any text on the textbook page
2. The selected text is captured and displayed in the chatbot interface
3. When asking questions, the selected text is used as context for more accurate answers
4. Users can clear the context if needed

## Running the Application

### Prerequisites
- Development: Backend RAG server running on `http://localhost:8002` and Frontend development server running
- Production: Backend deployed and accessible via API routes

### Steps
1. Start the backend RAG server:
   ```bash
   cd backend/RAG_Chatbot
   python run_server.py
   ```

2. In a separate terminal, start the frontend:
   ```bash
   cd frontend
   npm install
   npm run start
   ```

### Important Note about CORS
Since the frontend and backend run on different ports, there may be CORS issues. To avoid these:

1. The backend already has CORS middleware enabled to allow all origins
2. If you still encounter issues, you can start Chrome with disabled security for development:
   ```bash
   # Windows
   start chrome --user-data-dir="C:/Chrome dev session" --disable-web-security

   # Mac
   open -n -a /Applications/Google\ Chrome.app/Contents/MacOS/Google\ Chrome --args --user-data-dir="/tmp/chrome_dev_session" --disable-web-security
   ```

## Usage
1. Navigate to any textbook chapter
2. Click the chatbot icon (bottom-right corner) to open the chat interface
3. Select text on the page to provide context (optional)
4. Type your question in the input field
5. Press Enter or click Send to get a response
6. The chatbot will use both the selected text and its knowledge of the book to answer

## Troubleshooting
- Development: If the chatbot doesn't respond, ensure the backend server is running on port 8002
- Production: If the chatbot doesn't respond, check that API routes are properly configured and the backend is accessible
- If you see CORS errors, check that the backend is properly configured with CORS enabled
- If selected text isn't being captured, try refreshing the page