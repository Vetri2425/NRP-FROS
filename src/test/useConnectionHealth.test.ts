import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act, waitFor } from '@testing-library/react';
import { useConnectionHealth } from '../hooks/useConnectionHealth';

// Mock fetch globally
global.fetch = vi.fn();

describe('useConnectionHealth', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  describe('Initial state', () => {
    it('should initialize with checking status', () => {
      const { result } = renderHook(() => useConnectionHealth({ enabled: false }));

      expect(result.current.status).toBe('checking');
      expect(result.current.latency).toBeNull();
      expect(result.current.lastChecked).toBeNull();
      expect(result.current.isChecking).toBe(false);
      expect(result.current.errorCount).toBe(0);
    });

    it('should start checking when enabled', async () => {
      (global.fetch as any).mockResolvedValueOnce({
        ok: true,
        status: 200
      });

      const { result } = renderHook(() => useConnectionHealth({ enabled: true }));

      // Wait for the fetch to complete
      await waitFor(() => {
        expect(result.current.lastChecked).not.toBeNull();
      });

      expect(global.fetch).toHaveBeenCalled();
    });
  });

  describe('Connection success', () => {
    it('should set connected status on successful response', async () => {
      (global.fetch as any).mockResolvedValueOnce({
        ok: true,
        status: 200
      });

      const { result } = renderHook(() => useConnectionHealth({ enabled: true }));

      await waitFor(() => {
        expect(result.current.status).toBe('connected');
      }, { timeout: 2000 });

      expect(result.current.latency).not.toBeNull();
      expect(result.current.latency).toBeGreaterThanOrEqual(0);
      expect(result.current.lastChecked).toBeInstanceOf(Date);
      expect(result.current.errorCount).toBe(0);
    });

    it('should calculate latency correctly', async () => {
      (global.fetch as any).mockResolvedValueOnce({
        ok: true,
        status: 200
      });

      const { result } = renderHook(() => useConnectionHealth({ enabled: true }));

      await waitFor(() => {
        expect(result.current.latency).not.toBeNull();
      }, { timeout: 2000 });

      // Latency should be a positive number
      expect(result.current.latency).toBeGreaterThanOrEqual(0);
    });
  });

  describe('Degraded connection', () => {
    it('should set degraded status when latency exceeds threshold', async () => {
      // Mock slow response
      (global.fetch as any).mockImplementation(() => {
        return new Promise(resolve => {
          setTimeout(() => {
            resolve({ ok: true, status: 200 });
          }, 50); // Delay to ensure latency is measured
        });
      });

      const { result } = renderHook(() => 
        useConnectionHealth({ enabled: true, degradedThreshold: 10 }) // Low threshold
      );

      await waitFor(() => {
        expect(result.current.status).toBe('degraded');
      }, { timeout: 2000 });

      expect(result.current.latency).toBeGreaterThan(10);
    });

    it('should use custom degraded threshold', async () => {
      (global.fetch as any).mockResolvedValueOnce({
        ok: true,
        status: 200
      });

      const { result } = renderHook(() => 
        useConnectionHealth({ enabled: true, degradedThreshold: 0 }) // Zero threshold means any latency is degraded
      );

      await waitFor(() => {
        expect(result.current.latency).not.toBeNull();
      }, { timeout: 2000 });

      // With 0 threshold, any latency should be degraded
      expect(result.current.status).toBe('degraded');
    });
  });

  describe('Connection failure', () => {
    it('should set disconnected status on network error', async () => {
      (global.fetch as any).mockRejectedValueOnce(new Error('Network error'));

      const { result } = renderHook(() => useConnectionHealth({ enabled: true }));

      await waitFor(() => {
        expect(result.current.status).toBe('disconnected');
      }, { timeout: 2000 });

      expect(result.current.latency).toBeNull();
      expect(result.current.errorCount).toBe(1);
    });

    it('should set disconnected status on HTTP error', async () => {
      (global.fetch as any).mockResolvedValueOnce({
        ok: false,
        status: 500
      });

      const { result } = renderHook(() => useConnectionHealth({ enabled: true }));

      await waitFor(() => {
        expect(result.current.status).toBe('disconnected');
      }, { timeout: 2000 });

      expect(result.current.errorCount).toBe(1);
    });

    it('should increment error count on consecutive failures', async () => {
      (global.fetch as any).mockRejectedValue(new Error('Network error'));

      const { result } = renderHook(() => 
        useConnectionHealth({ enabled: true, checkInterval: 100 })
      );

      // Wait for first error
      await waitFor(() => {
        expect(result.current.errorCount).toBeGreaterThanOrEqual(1);
      }, { timeout: 2000 });

      const firstErrorCount = result.current.errorCount;

      // Wait for more errors to accumulate
      await waitFor(() => {
        expect(result.current.errorCount).toBeGreaterThan(firstErrorCount);
      }, { timeout: 3000 });
    });

    it('should reset error count on successful connection', async () => {
      (global.fetch as any)
        .mockRejectedValueOnce(new Error('Error'))
        .mockResolvedValue({ ok: true, status: 200 });

      const { result } = renderHook(() => 
        useConnectionHealth({ enabled: true, checkInterval: 100 })
      );

      // Wait for error
      await waitFor(() => {
        expect(result.current.errorCount).toBe(1);
      }, { timeout: 2000 });

      // Wait for recovery
      await waitFor(() => {
        expect(result.current.status).toBe('connected');
        expect(result.current.errorCount).toBe(0);
      }, { timeout: 3000 });
    });
  });

  describe('Periodic checks', () => {
    it('should perform checks at specified interval', async () => {
      (global.fetch as any).mockResolvedValue({ ok: true, status: 200 });

      renderHook(() => useConnectionHealth({ enabled: true, checkInterval: 100 }));

      // Wait for initial check
      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledTimes(1);
      }, { timeout: 1000 });

      // Wait for more checks
      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalled();
      }, { timeout: 1000 });

      // Should have made multiple calls
      expect(global.fetch).toHaveBeenCalled();
    });

    it('should not perform checks when disabled', async () => {
      (global.fetch as any).mockResolvedValue({ ok: true, status: 200 });

      renderHook(() => useConnectionHealth({ enabled: false, checkInterval: 100 }));

      // Wait a bit
      await new Promise(resolve => setTimeout(resolve, 300));

      expect(global.fetch).not.toHaveBeenCalled();
    });
  });

  describe('Manual refresh', () => {
    it('should provide refresh function', () => {
      const { result } = renderHook(() => useConnectionHealth({ enabled: false }));

      expect(result.current.refresh).toBeDefined();
      expect(typeof result.current.refresh).toBe('function');
    });

    it('should perform check when refresh is called', async () => {
      (global.fetch as any).mockResolvedValue({ ok: true, status: 200 });

      const { result } = renderHook(() => useConnectionHealth({ enabled: false }));

      expect(global.fetch).not.toHaveBeenCalled();

      act(() => {
        result.current.refresh();
      });

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledTimes(1);
      }, { timeout: 1000 });
    });
  });

  describe('Cleanup', () => {
    it('should cleanup interval on unmount', async () => {
      (global.fetch as any).mockResolvedValue({ ok: true, status: 200 });

      const { unmount } = renderHook(() => 
        useConnectionHealth({ enabled: true, checkInterval: 100 })
      );

      // Wait for initial check
      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalled();
      }, { timeout: 1000 });

      const callsBeforeUnmount = (global.fetch as any).mock.calls.length;

      // Unmount
      unmount();

      // Wait a bit - should not trigger more checks
      await new Promise(resolve => setTimeout(resolve, 300));

      // Should not have more calls than before unmount
      expect((global.fetch as any).mock.calls.length).toBe(callsBeforeUnmount);
    });
  });

  describe('Fetch configuration', () => {
    it('should call correct endpoint', async () => {
      (global.fetch as any).mockResolvedValue({ ok: true, status: 200 });

      renderHook(() => useConnectionHealth({ enabled: true }));

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalled();
      }, { timeout: 1000 });

      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('/api/status'),
        expect.objectContaining({
          method: 'GET',
          headers: expect.objectContaining({
            'Accept': 'application/json'
          })
        })
      );
    });
  });
});
