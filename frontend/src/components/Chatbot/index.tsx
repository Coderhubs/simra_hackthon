import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';

type Message = {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
};

type ChatbotProps = {
  // Optional: allow passing selected text from parent component
  selectedText?: string;
};

const Chatbot: React.FC<ChatbotProps> = ({ selectedText = '' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedTextState, setSelectedTextState] = useState(selectedText);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selected = window.getSelection()?.toString().trim();
    if (selected) {
      setSelectedTextState(selected);
    }
    return selected;
  };

  // Handle sending a message to the backend
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Get current selection to ensure we have the latest selected text
    const currentSelection = window.getSelection()?.toString().trim();
    const textToSend = currentSelection || selectedTextState || null;

    // Add user message to chat
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    console.log('Sending message:', { query: inputValue, selected_text: textToSend });

    try {
      // API call to backend
      const response = await fetch('http://localhost:8002/api/v1/search-live-content', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          selected_text: textToSend,
        }),
      });

      const data = await response.json();

      console.log('Received response:', data);

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.response || 'Sorry, I could not process your request.',
        sender: 'bot',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Handle sending message on Enter key
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selected = window.getSelection()?.toString().trim();
      if (selected) {
        setSelectedTextState(selected);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Initialize with a welcome message
  useEffect(() => {
    if (messages.length === 0 && isOpen) {
      const welcomeMessage: Message = {
        id: 'welcome',
        text: 'Hello! I\'m your Physical AI & Humanoid Robotics assistant. Ask me anything about the book content or select text to get specific explanations.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages([welcomeMessage]);
    }
  }, [isOpen, messages.length]);

  return (
    <div className={styles.chatbotContainer}>
      {/* Chatbot Icon */}
      {!isOpen && (
        <button
          className={styles.chatbotIcon}
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            width="24px"
            height="24px"
          >
            <path d="M0 0h24v24H0z" fill="none" />
            <path d="M20 2H4c-1.1 0-1.99.9-1.99 2L2 22l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm-2 12H6v-2h12v2zm0-3H6V9h12v2zm0-3H6V6h12v2z" />
          </svg>
        </button>
      )}

      {/* Chatbot Window */}
      {isOpen && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatbotHeader}>
            <h3>AI Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chatbot"
            >
              ×
            </button>
          </div>

          <div className={styles.chatbotMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  styles.message,
                  message.sender === 'user' ? styles.userMessage : styles.botMessage
                )}
              >
                <div className={styles.messageText}>{message.text}</div>
                <div className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={clsx(styles.message, styles.botMessage)}>
                <div className={styles.typingIndicator}>
                  <div></div>
                  <div></div>
                  <div></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedTextState && (
            <div className={styles.selectedTextPreview}>
              <strong>Context:</strong> "{selectedTextState.substring(0, 60)}{selectedTextState.length > 60 ? '...' : ''}"
              <button
                className={styles.clearContextButton}
                onClick={() => setSelectedTextState('')}
                title="Clear context"
              >
                ×
              </button>
            </div>
          )}

          <div className={styles.chatbotInput}>
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the book content..."
              rows={1}
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              className={styles.sendButton}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;