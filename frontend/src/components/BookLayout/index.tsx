import React, { ReactNode } from 'react';
import clsx from 'clsx';
import Chatbot from '@site/src/components/Chatbot'; // Import Chatbot component
import styles from './styles.module.css';

type BookLayoutProps = {
  children: ReactNode;
};

export default function BookLayout({ children }: BookLayoutProps): ReactNode {
  return (
    <div className={clsx('book-layout', styles.bookLayout)}>
      <header className={clsx('book-header', styles.bookHeader)}>
        {/* You can add a title, logo, or other header elements here */}
        <h1>Physical AI & Humanoid Robotics</h1>
      </header>
      <main className={clsx('book-main', styles.bookMain)}>
        <div className={clsx('book-content', styles.bookContent)}>
          {children}
        </div>
      </main>
      <footer className={clsx('book-footer', styles.bookFooter)}>
        {/* Optional footer content */}
        <p>&copy; {new Date().getFullYear()} My AI Textbook</p>
      </footer>
      <div className={styles.chatbotContainer}>
        <Chatbot /> {/* Render Chatbot component as fixed element */}
      </div>
    </div>
  );
}
