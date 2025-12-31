import React, { useState, useEffect } from 'react';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  createdAt: Date;
}

interface ChatKitProps {
  messages: Message[];
  input: string;
  handleInputChange: (value: string) => void;
  handleSubmit: (e: React.FormEvent) => void;
  isLoading: boolean;
}

const MockChatKit: React.FC<ChatKitProps> = ({
  messages,
  input,
  handleInputChange,
  handleSubmit,
  isLoading
}) => {
  const [localInput, setLocalInput] = useState(input);

  useEffect(() => {
    setLocalInput(input);
  }, [input]);

  const handleLocalSubmit = (e: React.FormEvent) => {
    handleSubmit(e);
    setLocalInput('');
  };

  return (
    <div className="mock-chatkit">
      <div className="chat-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.role}`}
            style={{
              alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
              backgroundColor: message.role === 'user' ? '#4f46e5' : '#f3f4f6',
              color: message.role === 'user' ? 'white' : 'black',
              padding: '8px 12px',
              borderRadius: '18px',
              margin: '4px 0',
              maxWidth: '80%',
            }}
          >
            {message.content}
          </div>
        ))}
        {isLoading && (
          <div
            className="loading-indicator"
            style={{
              alignSelf: 'flex-start',
              backgroundColor: '#f3f4f6',
              color: 'black',
              padding: '8px 12px',
              borderRadius: '18px',
              margin: '4px 0',
            }}
          >
            Typing...
          </div>
        )}
      </div>

      <form onSubmit={handleLocalSubmit} className="chat-input-form" style={{ display: 'flex', gap: '8px', marginTop: '12px' }}>
        <input
          value={localInput}
          onChange={(e) => {
            setLocalInput(e.target.value);
            handleInputChange(e.target.value);
          }}
          placeholder="Type your message..."
          style={{
            flex: 1,
            padding: '8px 12px',
            border: '1px solid #d1d5db',
            borderRadius: '24px',
            fontSize: '14px',
          }}
        />
        <button
          type="submit"
          disabled={isLoading || !localInput.trim()}
          style={{
            padding: '8px 16px',
            backgroundColor: '#4f46e5',
            color: 'white',
            border: 'none',
            borderRadius: '24px',
            cursor: 'pointer',
            fontSize: '14px',
          }}
        >
          Send
        </button>
      </form>
    </div>
  );
};

export default MockChatKit;