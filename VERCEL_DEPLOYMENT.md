# Vercel Deployment Guide

This document explains how to deploy the Physical AI & Humanoid Robotics textbook project on Vercel with both frontend and backend components.

## Project Structure

The project consists of:
- **Frontend**: Docusaurus-based textbook interface in the `frontend/` directory
- **Backend**: FastAPI-based RAG chatbot in the `backend/RAG_Chatbot/` directory
- **Configuration**: `vercel.json` for deployment routing

## Deployment Process

### 1. Repository Setup
1. Push your code to a Git repository (GitHub, GitLab, or Bitbucket)
2. The repository should contain both frontend and backend code

### 2. Vercel Project Creation
1. Go to [vercel.com](https://vercel.com) and sign in
2. Click "New Project" and import your repository
3. Vercel will automatically detect this as a monorepo with multiple build steps

### 3. Environment Variables
In the Vercel dashboard, under your project settings, add the following environment variables:

- `GEMINI_API_KEY` - Your Google Gemini API key
- `QDRANT_URL` - Your Qdrant vector database URL
- `QDRANT_API_KEY` - Your Qdrant database API key

### 4. Build Configuration
The `vercel.json` file is already configured to:
- Build the frontend using Docusaurus
- Deploy the backend as a Python serverless function
- Route API requests (`/api/*`) to the backend
- Serve static frontend content for all other routes

### 5. API Routing
- Frontend routes: All routes except `/api/*` are served as static content
- Backend routes: All `/api/*` routes are routed to the FastAPI backend
- The chatbot component in the frontend automatically uses relative paths that work in both development and production

## Important Notes

1. **API Keys**: Never commit API keys to the repository. Always use Vercel's environment variable system.

2. **Database**: Ensure your Qdrant vector database is accessible from the deployed backend.

3. **CORS**: The backend is configured with permissive CORS settings for development, but you may want to restrict this in production.

4. **Scaling**: The Python backend runs as serverless functions, which may have cold start delays. Consider this for production use.

## Troubleshooting

- If API calls fail, check that environment variables are properly set in Vercel
- If the frontend builds but API calls don't work, verify the routing configuration
- Monitor the Vercel logs for any deployment or runtime errors