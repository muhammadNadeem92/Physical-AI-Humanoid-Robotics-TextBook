import React, { createContext, useContext, useReducer, ReactNode } from 'react';
import { Session } from '../types/session';
import { Message } from '../types/message';
import { TextSelection } from '../types/selection';

// Define the shape of our global state
interface ChatState {
  currentSession: Session | null;
  messages: Message[];
  textSelection: TextSelection | null;
  uiState: {
    isWidgetOpen: boolean;
    isLoading: boolean;
    error: string | null;
    selectedText: string | null;
  };
}

// Define the actions that can modify the state
type ChatAction =
  | { type: 'SET_SESSION'; payload: Session }
  | { type: 'ADD_MESSAGE'; payload: Message }
  | { type: 'SET_MESSAGES'; payload: Message[] }
  | { type: 'SET_TEXT_SELECTION'; payload: TextSelection | null }
  | { type: 'SET_WIDGET_OPEN'; payload: boolean }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'SET_SELECTED_TEXT'; payload: string | null }
  | { type: 'CLEAR_SESSION' };

// Initial state
const initialState: ChatState = {
  currentSession: null,
  messages: [],
  textSelection: null,
  uiState: {
    isWidgetOpen: false,
    isLoading: false,
    error: null,
    selectedText: null,
  },
};

// Reducer function
const chatReducer = (state: ChatState, action: ChatAction): ChatState => {
  switch (action.type) {
    case 'SET_SESSION':
      return { ...state, currentSession: action.payload };
    case 'ADD_MESSAGE':
      return { ...state, messages: [...state.messages, action.payload] };
    case 'SET_MESSAGES':
      return { ...state, messages: action.payload };
    case 'SET_TEXT_SELECTION':
      return { ...state, textSelection: action.payload };
    case 'SET_WIDGET_OPEN':
      return { ...state, uiState: { ...state.uiState, isWidgetOpen: action.payload } };
    case 'SET_LOADING':
      return { ...state, uiState: { ...state.uiState, isLoading: action.payload } };
    case 'SET_ERROR':
      return { ...state, uiState: { ...state.uiState, error: action.payload } };
    case 'SET_SELECTED_TEXT':
      return { ...state, uiState: { ...state.uiState, selectedText: action.payload } };
    case 'CLEAR_SESSION':
      return { ...initialState };
    default:
      return state;
  }
};

// Create context
interface ChatContextType extends ChatState {
  dispatch: React.Dispatch<ChatAction>;
  // Add helper methods here
  openWidget: () => void;
  closeWidget: () => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  setSelectedText: (text: string | null) => void;
  addMessage: (message: Message) => void;
  setMessages: (messages: Message[]) => void;
  setSession: (session: Session) => void;
  clearSession: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

// Provider component
interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  // Helper methods
  const openWidget = () => {
    dispatch({ type: 'SET_WIDGET_OPEN', payload: true });
  };

  const closeWidget = () => {
    dispatch({ type: 'SET_WIDGET_OPEN', payload: false });
  };

  const setLoading = (loading: boolean) => {
    dispatch({ type: 'SET_LOADING', payload: loading });
  };

  const setError = (error: string | null) => {
    dispatch({ type: 'SET_ERROR', payload: error });
  };

  const setSelectedText = (text: string | null) => {
    dispatch({ type: 'SET_SELECTED_TEXT', payload: text });
  };

  const addMessage = (message: Message) => {
    dispatch({ type: 'ADD_MESSAGE', payload: message });
  };

  const setMessages = (messages: Message[]) => {
    dispatch({ type: 'SET_MESSAGES', payload: messages });
  };

  const setSession = (session: Session) => {
    dispatch({ type: 'SET_SESSION', payload: session });
  };

  const clearSession = () => {
    dispatch({ type: 'CLEAR_SESSION' });
  };

  const value: ChatContextType = {
    ...state,
    dispatch,
    openWidget,
    closeWidget,
    setLoading,
    setError,
    setSelectedText,
    addMessage,
    setMessages,
    setSession,
    clearSession,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
};

// Custom hook to use the ChatContext
export const useChatContext = (): ChatContextType => {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
};