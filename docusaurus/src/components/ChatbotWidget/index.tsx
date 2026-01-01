import React, { useState, useRef, useEffect } from 'react';
import './styles.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  citations?: Array<{
    document_id?: string;
    title?: string;
    chapter?: string;
    section?: string;
    page_reference?: string;
  }>;
}

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      role: 'assistant',
      content: 'Hello! I\'m your Physical AI & Humanoid Robotics assistant. I can answer questions about the curriculum, modules, and concepts covered in the book. What would you like to know?',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call backend API
      
      const response = await fetch('https://syedfarooqali-backend-deploy.hf.space/api', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          message: inputValue,
          session_id: 'chatbot-widget-session-' + Date.now(),
        }),
      });


      if (!response.ok) {
        if (response.status === 422) {
          throw new Error('Validation error: Required fields are missing. Please check your input.');
        } else if (response.status === 500) {
          throw new Error('Server error: The backend encountered an error. Please try again later.');
        } else {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.response || 'Sorry, I couldn\'t process your request.',
        timestamp: new Date(),
        citations: data.citations || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const formatTime = (date: Date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Floating Chat Icon */}
      <div className={`chatbot-widget ${isOpen ? 'open' : ''}`}>
        {!isOpen ? (
          <button
            className="chatbot-toggle"
            onClick={toggleChat}
            aria-label="Open chat"
          >
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.33L2 22L7.67 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM9 17C9 17.55 8.55 18 8 18C7.45 18 7 17.55 7 17C7 16.45 7.45 16 8 16C8.55 16 9 16.45 9 17ZM12 17C12 17.55 11.55 18 11 18C10.45 18 10 17.55 10 17C10 16.45 10.45 16 11 16C11.55 16 12 16.45 12 17ZM15 17C15 17.55 14.55 18 14 18C13.45 18 13 17.55 13 17C13 16.45 13.45 16 14 16C14.55 16 15 16.45 15 17Z" fill="white"/>
            </svg>
          </button>
        ) : (
          <div className="chatbot-container">
            <div className="chatbot-header">
              <div className="chatbot-title">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" style={{marginRight: '8px'}}>
                  <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.33L2 22L7.67 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM9 17C9 17.55 8.55 18 8 18C7.45 18 7 17.55 7 17C7 16.45 7.45 16 8 16C8.55 16 9 16.45 9 17ZM12 17C12 17.55 11.55 18 11 18C10.45 18 10 17.55 10 17C10 16.45 10.45 16 11 16C11.55 16 12 16.45 12 17ZM15 17C15 17.55 14.55 18 14 18C13.45 18 13 17.55 13 17C13 16.45 13.45 16 14 16C14.55 16 15 16.45 15 17Z" fill="white"/>
                </svg>
                <span>Physical AI Assistant</span>
              </div>
              <button
                className="chatbot-close"
                onClick={toggleChat}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>

            <div className="chatbot-messages">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                  style={{
                    display: 'flex',
                    justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
                  }}
                >
                  <div
                    className={`message-bubble ${message.role}`}
                  >
                    <div style={{ whiteSpace: 'pre-wrap' }}>{message.content}</div>
                    {message.role === 'assistant' && message.citations && message.citations.length > 0 && (
                      <div style={{ marginTop: '0.5rem', fontSize: '0.8rem', opacity: 0.8 }}>
                        <strong>Sources:</strong>
                        <ul style={{ margin: '0.25rem 0 0 1rem', padding: 0 }}>
                          {message.citations.slice(0, 3).map((citation, index) => (
                            <li key={index}>
                              {citation.chapter || citation.title || 'Source'}
                              {citation.section ? ` - ${citation.section}` : ''}
                            </li>
                          ))}
                          {message.citations.length > 3 && (
                            <li>... and {message.citations.length - 3} more</li>
                          )}
                        </ul>
                      </div>
                    )}
                    <div
                      className="message-time"
                    >
                      {formatTime(message.timestamp)}
                    </div>
                  </div>
                </div>
              ))}
              {isLoading && (
                <div
                  style={{
                    display: 'flex',
                    justifyContent: 'flex-start',
                  }}
                >
                  <div
                    className="message-bubble assistant"
                  >
                    Thinking...
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            <form onSubmit={handleSubmit} className="chatbot-input-form">
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask about Physical AI & Humanoid Robotics..."
                disabled={isLoading}
                className="chatbot-input"
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                className="chatbot-send-button"
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M2.01 21L23 12L2.01 3L2 10L17 12L2 14L2.01 21Z" fill="currentColor"/>
                </svg>
              </button>
            </form>
          </div>
        )}
      </div>
    </>
  );
};

export default ChatbotWidget;