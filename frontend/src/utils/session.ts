/**
 * Session management utilities
 */

// Session data structure
export interface SessionData {
  id: string;
  createdAt: number;
  lastAccessed: number;
  ttl: number; // Time to live in milliseconds
  data: any; // Additional session data
}

/**
 * Generate a new session ID
 */
export const generateSessionId = (): string => {
  return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
};

/**
 * Create a new session
 */
export const createSession = (ttl: number = 3600000): SessionData => { // Default TTL: 1 hour
  const sessionId = generateSessionId();
  const now = Date.now();

  const session: SessionData = {
    id: sessionId,
    createdAt: now,
    lastAccessed: now,
    ttl,
    data: {}
  };

  return session;
};

/**
 * Save session to localStorage
 */
export const saveSession = (session: SessionData): void => {
  try {
    localStorage.setItem(`chat_session_${session.id}`, JSON.stringify(session));
  } catch (error) {
    console.error('Failed to save session to localStorage:', error);
    throw new Error('Unable to save session data');
  }
};

/**
 * Load session from localStorage
 */
export const loadSession = (sessionId: string): SessionData | null => {
  try {
    const sessionData = localStorage.getItem(`chat_session_${sessionId}`);
    if (!sessionData) {
      return null;
    }

    const session: SessionData = JSON.parse(sessionData);

    // Check if session has expired
    if (isSessionExpired(session)) {
      removeSession(sessionId);
      return null;
    }

    // Update last accessed time
    session.lastAccessed = Date.now();
    saveSession(session);

    return session;
  } catch (error) {
    console.error('Failed to load session from localStorage:', error);
    return null;
  }
};

/**
 * Remove session from localStorage
 */
export const removeSession = (sessionId: string): void => {
  try {
    localStorage.removeItem(`chat_session_${sessionId}`);
  } catch (error) {
    console.error('Failed to remove session from localStorage:', error);
  }
};

/**
 * Check if a session is expired
 */
export const isSessionExpired = (session: SessionData): boolean => {
  const now = Date.now();
  const sessionAge = now - session.lastAccessed;
  return sessionAge > session.ttl;
};

/**
 * Get or create session
 */
export const getOrCreateSession = (ttl: number = 3600000): SessionData => {
  // Check for existing session in URL or localStorage
  const urlParams = new URLSearchParams(window.location.search);
  const urlSessionId = urlParams.get('sessionId');

  if (urlSessionId) {
    const existingSession = loadSession(urlSessionId);
    if (existingSession && !isSessionExpired(existingSession)) {
      return existingSession;
    }
  }

  // Try to get session from localStorage
  const storedSessionId = localStorage.getItem('current_chat_session_id');
  if (storedSessionId) {
    const existingSession = loadSession(storedSessionId);
    if (existingSession && !isSessionExpired(existingSession)) {
      return existingSession;
    }
  }

  // Create new session
  const newSession = createSession(ttl);
  saveSession(newSession);
  localStorage.setItem('current_chat_session_id', newSession.id);
  return newSession;
};

/**
 * Update session data
 */
export const updateSessionData = (sessionId: string, data: any): void => {
  const session = loadSession(sessionId);
  if (!session) {
    throw new Error('Session not found');
  }

  session.data = { ...session.data, ...data };
  session.lastAccessed = Date.now();
  saveSession(session);
};

/**
 * Get session data
 */
export const getSessionData = <T>(sessionId: string): T | null => {
  const session = loadSession(sessionId);
  if (!session) {
    return null;
  }

  return session.data as T;
};

/**
 * Extend session TTL
 */
export const extendSession = (sessionId: string, additionalTime: number): void => {
  const session = loadSession(sessionId);
  if (!session) {
    throw new Error('Session not found');
  }

  session.ttl += additionalTime;
  session.lastAccessed = Date.now();
  saveSession(session);
};

/**
 * Clear all expired sessions
 */
export const clearExpiredSessions = (): void => {
  const keys = Object.keys(localStorage);
  const sessionKeys = keys.filter(key => key.startsWith('chat_session_'));

  sessionKeys.forEach(key => {
    const sessionId = key.replace('chat_session_', '');
    const session = loadSession(sessionId);
    if (!session || isSessionExpired(session)) {
      removeSession(sessionId);
    }
  });
};

/**
 * Get all active sessions
 */
export const getActiveSessions = (): SessionData[] => {
  const keys = Object.keys(localStorage);
  const sessionKeys = keys.filter(key => key.startsWith('chat_session_'));
  const sessions: SessionData[] = [];

  sessionKeys.forEach(key => {
    const sessionId = key.replace('chat_session_', '');
    const session = loadSession(sessionId);
    if (session && !isSessionExpired(session)) {
      sessions.push(session);
    }
  });

  return sessions;
};

/**
 * Initialize session cleanup
 */
export const initSessionCleanup = (cleanupInterval: number = 300000): void => { // Default: 5 minutes
  setInterval(clearExpiredSessions, cleanupInterval);
};