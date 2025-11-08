import React from 'react';
import './ProgressBar.css';

interface ProgressBarProps {
  progress: number;        // 0-100
  status?: 'idle' | 'loading' | 'success' | 'error';
  animated?: boolean;
  showPercentage?: boolean;
  label?: string;
  size?: 'small' | 'medium' | 'large';
}

export const ProgressBar: React.FC<ProgressBarProps> = ({
  progress = 0,
  status = 'loading',
  animated = true,
  showPercentage = true,
  label,
  size = 'medium'
}) => {
  const percentage = Math.min(Math.max(progress, 0), 100);
  
  return (
    <div className={`progress-container progress-${size}`}>
      {label && <label className="progress-label">{label}</label>}
      
      <div 
        className={`progress-bar progress-${status} ${animated ? 'animated' : ''}`}
        role="progressbar"
        aria-valuenow={percentage}
        aria-valuemin={0}
        aria-valuemax={100}
        aria-label={label || 'Progress'}
      >
        <div 
          className="progress-fill"
          style={{ width: `${percentage}%` }}
        />
        {showPercentage && (
          <span className="progress-text">{Math.round(percentage)}%</span>
        )}
      </div>
    </div>
  );
};

export default ProgressBar;
