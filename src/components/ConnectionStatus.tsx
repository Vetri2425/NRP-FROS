import React from 'react';
import { useConnectionHealth, ConnectionStatus as Status } from '../hooks/useConnectionHealth';
import './ConnectionStatus.css';

interface ConnectionStatusProps {
  className?: string;
  showLabel?: boolean;
  checkInterval?: number;
}

/**
 * Visual indicator for backend connection status
 * Shows a colored dot with tooltip displaying connection details
 */
export const ConnectionStatus: React.FC<ConnectionStatusProps> = ({
  className = '',
  showLabel = false,
  checkInterval = 5000
}) => {
  const { status, latency, lastChecked, isChecking, refresh } = useConnectionHealth({
    checkInterval
  });

  const getStatusColor = (status: Status): string => {
    switch (status) {
      case 'connected':
        return 'status-connected';
      case 'degraded':
        return 'status-degraded';
      case 'disconnected':
        return 'status-disconnected';
      case 'checking':
        return 'status-checking';
      default:
        return 'status-unknown';
    }
  };

  const getStatusLabel = (status: Status): string => {
    switch (status) {
      case 'connected':
        return 'Connected';
      case 'degraded':
        return 'Slow Connection';
      case 'disconnected':
        return 'Disconnected';
      case 'checking':
        return 'Checking...';
      default:
        return 'Unknown';
    }
  };

  const getTooltipText = (): string => {
    const statusText = getStatusLabel(status);
    
    if (status === 'disconnected') {
      return `Backend: ${statusText}\nClick to retry`;
    }
    
    if (status === 'checking') {
      return 'Checking connection...';
    }
    
    const latencyText = latency !== null ? `${latency}ms` : 'N/A';
    const timeText = lastChecked 
      ? new Date(lastChecked).toLocaleTimeString()
      : 'Never';
    
    return `Backend: ${statusText}\nLatency: ${latencyText}\nLast checked: ${timeText}\nClick to refresh`;
  };

  const handleClick = (e: React.MouseEvent) => {
    e.preventDefault();
    refresh();
  };

  return (
    <div 
      className={`connection-status ${className}`}
      onClick={handleClick}
      title={getTooltipText()}
      role="status"
      aria-label={`Backend connection: ${getStatusLabel(status)}`}
    >
      <div className={`status-indicator ${getStatusColor(status)}`}>
        <div className="status-dot" />
        {isChecking && <div className="status-pulse" />}
      </div>
      
      {showLabel && (
        <span className="status-label">
          {getStatusLabel(status)}
        </span>
      )}
    </div>
  );
};
