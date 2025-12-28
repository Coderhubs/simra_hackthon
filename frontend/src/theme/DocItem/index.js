import React from 'react';
import DocItem from '@theme-original/DocItem';
import BookLayout from '@site/src/components/BookLayout';

export default function DocItemWrapper(props) {
  return (
    <BookLayout>
      <DocItem {...props} />
    </BookLayout>
  );
}