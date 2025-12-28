# Physical AI & Humanoid Robotics Textbook - Frontend

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator. This is the frontend for the Physical AI & Humanoid Robotics textbook with an integrated AI chatbot.

## Features

- Complete textbook content in digital format
- Interactive documentation with code examples
- Integrated AI chatbot for answering questions about the content
- Responsive design for all devices

## AI Chatbot Integration

The frontend includes an integrated AI chatbot that can answer questions about the textbook content. The chatbot:

- Appears as a floating icon on all pages
- Can answer questions about the entire book content
- Supports text selection context for specific questions
- Uses the backend RAG system to provide accurate, context-aware responses

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Running with the Backend

To use the full AI chatbot functionality, you need to run the backend RAG server as well:

1. First, start the backend RAG server:
   ```bash
   cd backend/RAG_Chatbot
   python run_server.py
   ```

2. In a separate terminal, start the frontend:
   ```bash
   cd frontend
   yarn start
   ```

See `CHATBOT_INTEGRATION.md` for more details about the chatbot functionality and troubleshooting.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
