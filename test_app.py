"""
Simple test to verify that the RAG Chatbot API application can be imported and initialized.
"""

def test_imports():
    """Test that all major components can be imported without errors."""
    try:
        # Test main application import
        from src.main import app
        print("✓ Main application imported successfully")

        # Test configuration imports
        from src.config import settings, db_config, qdrant_config, llm_config
        print("✓ Configuration components imported successfully")

        # Test model imports
        from src.models import ChatSession, UserQuery, RetrievedChunk, ChatResponse, Feedback, TokenUsageLog
        print("✓ Model components imported successfully")

        # Test service imports
        from src.services import (
            retrieval_service,
            agent_service,
            session_service,
            history_service,
            feedback_service,
            rag_orchestrator,
            query_processor
        )
        print("✓ Service components imported successfully")

        # Test API imports
        from src.api import chat_router
        print("✓ API components imported successfully")

        print("\n✓ All components imported successfully! The RAG Chatbot API is ready.")
        return True

    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == "__main__":
    success = test_imports()
    if success:
        print("\nThe RAG Chatbot API structure is complete and ready for use!")
    else:
        print("\nThere were issues with the API structure.")
        exit(1)