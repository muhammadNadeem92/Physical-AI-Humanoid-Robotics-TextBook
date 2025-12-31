# Deploying on Hugging Face Spaces

This guide explains how to deploy the Physical AI & Humanoid Robotics Textbook RAG Chatbot API on Hugging Face Spaces.

## Prerequisites

Before deploying, make sure you have:

1. A Hugging Face account
2. API keys for all required services:
   - Cohere API key
   - Qdrant Cloud API key and URL
   - Google Gemini API key
   - PostgreSQL database (Neon or other provider)
   - Optional: OpenAI API key

## Deployment Steps

### 1. Create a New Space

1. Go to [huggingface.co/spaces](https://huggingface.co/spaces)
2. Click "Create new Space"
3. Fill in the details:
   - Name: `physical-ai-humanoid-robotics-textbook-api`
   - License: MIT
   - SDK: Docker
   - Hardware: Choose appropriate CPU/Memory based on your needs
4. Click "Create Space"

### 2. Configure Environment Variables

In your Space settings:

1. Go to your Space page
2. Click on "Settings" → "Secrets"
3. Add the following secrets:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `QDRANT_URL`: Your Qdrant endpoint URL
   - `GEMINI_API_KEY`: Your Google Gemini API key
   - `DATABASE_URL`: Your PostgreSQL database URL
   - `OPENAI_API_KEY`: Your OpenAI API key (optional)

### 3. Upload Files

You have two options to upload the files:

#### Option A: Direct Upload (Recommended for small projects)

1. Go to your Space repository
2. Upload the following files:
   - `app.py` (the entry point)
   - `requirements.txt` (dependencies)
   - `backend/` directory (the backend code)
   - `pyproject.toml` (if needed)

#### Option B: Git Clone

1. Clone your Space repository locally
2. Copy all the necessary files to the repository
3. Push the changes:
   ```bash
   git add .
   git commit -m "Initial deployment"
   git push
   ```

### 4. Verify Deployment

1. Check the Space logs to ensure the application starts correctly
2. Access your Space URL to verify the API is running
3. Test the endpoints using the interactive documentation at `/docs`

## Important Notes

- The application will run on port 7860 (standard for Hugging Face Spaces)
- Make sure your Qdrant and database endpoints are accessible from the Space
- The Space will restart periodically, so ensure your session data is persisted in the database
- Monitor your Space's resource usage and adjust hardware settings if needed

## Troubleshooting

If the Space fails to start:

1. Check the logs for error messages
2. Verify all required environment variables are set
3. Ensure your database and Qdrant endpoints are accessible
4. Check that your requirements.txt doesn't have conflicting dependencies

## Updating the Space

To update your Space:

1. Make changes to your local files
2. Commit and push to the repository
3. The Space will automatically rebuild and redeploy