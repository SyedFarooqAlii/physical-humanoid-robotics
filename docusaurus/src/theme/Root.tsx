import React from 'react';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      {props.children}
      <ChatbotWidget />
    </>
  );
}