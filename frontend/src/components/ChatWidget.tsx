import React, { useState } from 'react';
import styles from './ChatWidget.module.css';

interface ChatWidgetProps {
  // Props can be added here as needed in the future
}

const ChatWidget: React.FC<ChatWidgetProps> = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<string[]>([]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = (message: string) => {
    // Placeholder for future chat functionality
    setMessages([...messages, message]);
  };

  return (
    <div className={styles.chatWidget}>
      <button className={styles.chatToggle} onClick={toggleChat}>
        {isOpen ? 'Close Chat' : 'Open Chat'}
      </button>
      {isOpen && (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={styles.message}>
                {msg}
              </div>
            ))}
          </div>
          <div className={styles.chatInput}>
            <input
              type="text"
              placeholder="Type your question..."
              onKeyPress={(e) => {
                if (e.key === 'Enter') {
                  sendMessage(e.currentTarget.value);
                  e.currentTarget.value = '';
                }
              }}
            />
            <button onClick={(e) => {
              const input = document.querySelector(`.${styles.chatInput} input`) as HTMLInputElement;
              if (input && input.value) {
                sendMessage(input.value);
                input.value = '';
              }
            }}>
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;