import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { ConnectionStatus } from '../components/ConnectionStatus';
import * as connectionHealthHook from '../hooks/useConnectionHealth';

// Mock the useConnectionHealth hook
vi.mock('../hooks/useConnectionHealth');

describe('ConnectionStatus', () => {
  const mockRefresh = vi.fn();

  const mockConnectionHealth = (overrides = {}) => {
    vi.spyOn(connectionHealthHook, 'useConnectionHealth').mockReturnValue({
      status: 'connected',
      latency: 50,
      lastChecked: new Date('2024-01-01T12:00:00'),
      isChecking: false,
      errorCount: 0,
      refresh: mockRefresh,
      ...overrides
    });
  };

  beforeEach(() => {
    vi.clearAllMocks();
  });

  describe('Rendering', () => {
    it('should render connection status indicator', () => {
      mockConnectionHealth();
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status).toBeInTheDocument();
    });

    it('should render with custom className', () => {
      mockConnectionHealth();
      
      const { container } = render(<ConnectionStatus className="custom-class" />);
      
      const statusElement = container.querySelector('.custom-class');
      expect(statusElement).toBeInTheDocument();
    });

    it('should show label when showLabel is true', () => {
      mockConnectionHealth({ status: 'connected' });
      
      render(<ConnectionStatus showLabel={true} />);
      
      expect(screen.getByText('Connected')).toBeInTheDocument();
    });

    it('should not show label when showLabel is false', () => {
      mockConnectionHealth({ status: 'connected' });
      
      render(<ConnectionStatus showLabel={false} />);
      
      expect(screen.queryByText('Connected')).not.toBeInTheDocument();
    });
  });

  describe('Status colors', () => {
    it('should show green indicator when connected', () => {
      mockConnectionHealth({ status: 'connected' });
      
      const { container } = render(<ConnectionStatus />);
      
      const indicator = container.querySelector('.status-connected');
      expect(indicator).toBeInTheDocument();
    });

    it('should show yellow indicator when degraded', () => {
      mockConnectionHealth({ status: 'degraded' });
      
      const { container } = render(<ConnectionStatus />);
      
      const indicator = container.querySelector('.status-degraded');
      expect(indicator).toBeInTheDocument();
    });

    it('should show red indicator when disconnected', () => {
      mockConnectionHealth({ status: 'disconnected' });
      
      const { container } = render(<ConnectionStatus />);
      
      const indicator = container.querySelector('.status-disconnected');
      expect(indicator).toBeInTheDocument();
    });

    it('should show blue indicator when checking', () => {
      mockConnectionHealth({ status: 'checking' });
      
      const { container } = render(<ConnectionStatus />);
      
      const indicator = container.querySelector('.status-checking');
      expect(indicator).toBeInTheDocument();
    });
  });

  describe('Status labels', () => {
    it('should display "Connected" for connected status', () => {
      mockConnectionHealth({ status: 'connected' });
      
      render(<ConnectionStatus showLabel={true} />);
      
      expect(screen.getByText('Connected')).toBeInTheDocument();
    });

    it('should display "Slow Connection" for degraded status', () => {
      mockConnectionHealth({ status: 'degraded' });
      
      render(<ConnectionStatus showLabel={true} />);
      
      expect(screen.getByText('Slow Connection')).toBeInTheDocument();
    });

    it('should display "Disconnected" for disconnected status', () => {
      mockConnectionHealth({ status: 'disconnected' });
      
      render(<ConnectionStatus showLabel={true} />);
      
      expect(screen.getByText('Disconnected')).toBeInTheDocument();
    });

    it('should display "Checking..." for checking status', () => {
      mockConnectionHealth({ status: 'checking' });
      
      render(<ConnectionStatus showLabel={true} />);
      
      expect(screen.getByText('Checking...')).toBeInTheDocument();
    });
  });

  describe('Accessibility', () => {
    it('should have proper ARIA label for connected status', () => {
      mockConnectionHealth({ status: 'connected' });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status).toHaveAttribute('aria-label', 'Backend connection: Connected');
    });

    it('should have proper ARIA label for disconnected status', () => {
      mockConnectionHealth({ status: 'disconnected' });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status).toHaveAttribute('aria-label', 'Backend connection: Disconnected');
    });

    it('should have tooltip with connection details', () => {
      mockConnectionHealth({
        status: 'connected',
        latency: 50,
        lastChecked: new Date('2024-01-01T12:00:00')
      });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status).toHaveAttribute('title');
      expect(status.getAttribute('title')).toContain('Connected');
      expect(status.getAttribute('title')).toContain('50ms');
    });
  });

  describe('User interactions', () => {
    it('should call refresh when clicked', async () => {
      mockConnectionHealth();
      const user = userEvent.setup();
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      await user.click(status);
      
      expect(mockRefresh).toHaveBeenCalledTimes(1);
    });

    it('should be clickable even when disconnected', async () => {
      mockConnectionHealth({ status: 'disconnected' });
      const user = userEvent.setup();
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      await user.click(status);
      
      expect(mockRefresh).toHaveBeenCalledTimes(1);
    });
  });

  describe('Checking animation', () => {
    it('should show pulse animation when checking', () => {
      mockConnectionHealth({ status: 'checking', isChecking: true });
      
      const { container } = render(<ConnectionStatus />);
      
      const pulse = container.querySelector('.status-pulse');
      expect(pulse).toBeInTheDocument();
    });

    it('should not show pulse animation when not checking', () => {
      mockConnectionHealth({ status: 'connected', isChecking: false });
      
      const { container } = render(<ConnectionStatus />);
      
      const pulse = container.querySelector('.status-pulse');
      expect(pulse).not.toBeInTheDocument();
    });
  });

  describe('Tooltip content', () => {
    it('should show latency in tooltip when connected', () => {
      mockConnectionHealth({
        status: 'connected',
        latency: 123
      });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status.getAttribute('title')).toContain('123ms');
    });

    it('should show retry message when disconnected', () => {
      mockConnectionHealth({ status: 'disconnected' });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status.getAttribute('title')).toContain('Click to retry');
    });

    it('should show refresh message when connected', () => {
      mockConnectionHealth({ status: 'connected' });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status.getAttribute('title')).toContain('Click to refresh');
    });

    it('should handle null latency', () => {
      mockConnectionHealth({
        status: 'disconnected',
        latency: null
      });
      
      render(<ConnectionStatus />);
      
      const status = screen.getByRole('status');
      expect(status).toBeInTheDocument();
      // Should not crash with null latency
    });
  });

  describe('Hook integration', () => {
    it('should pass checkInterval to hook', () => {
      const spy = vi.spyOn(connectionHealthHook, 'useConnectionHealth');
      mockConnectionHealth();
      
      render(<ConnectionStatus checkInterval={10000} />);
      
      expect(spy).toHaveBeenCalledWith({
        checkInterval: 10000
      });
    });

    it('should use default checkInterval when not specified', () => {
      const spy = vi.spyOn(connectionHealthHook, 'useConnectionHealth');
      mockConnectionHealth();
      
      render(<ConnectionStatus />);
      
      expect(spy).toHaveBeenCalledWith({
        checkInterval: 5000
      });
    });
  });
});
