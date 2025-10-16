
import React from 'react';

type IconProps = {
  className?: string;
};

export const LoadingIcon: React.FC<IconProps> = ({ className }) => (
  <svg xmlns="http://www.w3.org/2000/svg" className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor">
    <style>{`
      @keyframes spin {
        from { transform: rotate(0deg); }
        to { transform: rotate(360deg); }
      }
      .spinner {
        animation: spin 1s linear infinite;
        transform-origin: center;
      }
    `}</style>
    <path
      className="spinner"
      strokeWidth={2}
      strokeLinecap="round"
      strokeLinejoin="round"
      d="M12 3a9 9 0 1 0 9 9"
    />
  </svg>
);