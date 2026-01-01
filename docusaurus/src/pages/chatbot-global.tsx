import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';

// Define TypeScript interfaces
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

const ChatbotGlobal = () => {
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
          session_id: 'chatbot-global-session-' + Date.now(),
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

  return (
    <Layout title="Physical AI & Humanoid Robotics Chatbot" description="Interactive Q&A chatbot for the Physical AI & Humanoid Robotics curriculum">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <div className="card">
              <div className="card__header">
                <h1>Physical AI & Humanoid Robotics Assistant</h1>
                <p>Ask me anything about the curriculum, modules, or concepts!</p>
              </div>

              <div className="card__body" style={{
                display: 'flex',
                flexDirection: 'column',
                height: '600px',
                backgroundColor: 'var(--ifm-color-emphasis-100, #f8f9fa)'
              }}>
                <div
                  style={{
                    flex: 1,
                    overflowY: 'auto',
                    padding: '1rem',
                    backgroundColor: 'var(--ifm-background-color, white)',
                    borderRadius: '8px',
                    marginBottom: '1rem'
                  }}
                >
                  {messages.map((message) => (
                    <div
                      key={message.id}
                      className={`margin-bottom--md ${message.role === 'user' ? 'text--right' : ''}`}
                      style={{
                        display: 'flex',
                        justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
                      }}
                    >
                      <div
                        style={{
                          maxWidth: '80%',
                          padding: '0.75rem 1rem',
                          borderRadius: '18px',
                          backgroundColor: message.role === 'user'
                            ? 'var(--ifm-color-primary, #3498db)'
                            : 'var(--ifm-color-emphasis-200, #e9ecef)',
                          color: message.role === 'user' ? 'white' : 'var(--ifm-font-color-base, #212529)',
                          wordWrap: 'break-word',
                        }}
                      >
                        <div style={{ whiteSpace: 'pre-wrap' }}>{message.content}</div>
                        {message.role === 'assistant' && message.citations && message.citations.length > 0 && (
                          <div style={{ marginTop: '0.5rem', fontSize: '0.85rem', opacity: 0.8 }}>
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
                          style={{
                            fontSize: '0.7rem',
                            opacity: 0.7,
                            marginTop: '0.25rem',
                            textAlign: 'right'
                          }}
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
                        style={{
                          maxWidth: '80%',
                          padding: '0.75rem 1rem',
                          borderRadius: '18px',
                          backgroundColor: 'var(--ifm-color-emphasis-200, #e9ecef)',
                          color: 'var(--ifm-font-color-base, #212529)',
                        }}
                      >
                        Thinking...
                      </div>
                    </div>
                  )}
                  <div ref={messagesEndRef} />
                </div>

                <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '0.5rem' }}>
                  <input
                    type="text"
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    placeholder="Ask about Physical AI & Humanoid Robotics..."
                    disabled={isLoading}
                    style={{
                      flex: 1,
                      padding: '0.75rem',
                      borderRadius: '24px',
                      border: '1px solid var(--ifm-color-emphasis-300, #ddd)',
                      backgroundColor: 'var(--ifm-background-color, white)',
                      color: 'var(--ifm-font-color-base, #212529)',
                    }}
                  />
                  <button
                    type="submit"
                    disabled={!inputValue.trim() || isLoading}
                    style={{
                      padding: '0.75rem 1.5rem',
                      borderRadius: '24px',
                      border: 'none',
                      backgroundColor: isLoading
                        ? 'var(--ifm-color-emphasis-400, #ccc)'
                        : 'var(--ifm-color-primary, #3498db)',
                      color: 'white',
                      cursor: isLoading ? 'not-allowed' : 'pointer',
                      fontWeight: 'bold',
                    }}
                  >
                    {isLoading ? '...' : 'Send'}
                  </button>
                </form>
              </div>

              <div className="card__footer">
                <small>
                  This chatbot uses AI to answer questions based on the Physical AI & Humanoid Robotics curriculum.
                  Responses are generated using content from the book modules covering ROS 2, Digital Twins, AI-Brain, and VLA.
                </small>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ChatbotGlobal;