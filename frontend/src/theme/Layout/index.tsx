import React, { ReactNode } from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot';

type LayoutProps = {
  children: ReactNode;
  [key: string]: any;
};

export default function Layout(props: LayoutProps): ReactNode {
  return (
    <>
      <OriginalLayout {...props} />
      <Chatbot />
    </>
  );
}