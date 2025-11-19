import { useState, useEffect, useCallback, useRef } from 'react';
import { BACKEND_URL } from '../config';

export type ConnectionStatus = 'connected' | 'degraded' | 'disconnected' | 'checking';

export interface ConnectionHealth {
  status: ConnectionStatus;
  latency: number | null; // in milliseconds
  lastChecked: Date | null;
  isChecking: boolean;
  errorCount: number;
}

interface UseConnectionHealthOptions {
  checkInterval?: number; // in milliseconds, default 5000 (5 seconds)
  timeout?: number; // in milliseconds, default 3000 (3 seconds)
  degradedThreshold?: number; // latency threshold for degraded status, default 1000ms
  enabled?: boolean; // whether to run health checks, default true
}

/**
 * Hook for monitoring backend connection health
 * Periodically pings the backend and tracks connection status
 */
export function useConnectionHealth(options: UseConnectionHealthOptions = {}) {
  const {
    checkInterval = 5000,
    timeout = 3000,
    degradedThreshold = 1000,
    enabled = true
  } = options;

  const [health, setHealth] = useState<ConnectionHealth>({
    status: 'checking',
    latency: null,
    lastChecked: null,
    isChecking: false,
    errorCount: 0
  });

  const abortControllerRef = useRef<AbortController | null>(null);
  const consecutiveErrors = useRef(0);

  /**
   * Perform a health check by pinging the backend
   */
  const checkHealth = useCallback(async () => {
    // Cancel any ongoing request
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }

    const controller = new AbortController();
    abortControllerRef.current = controller;

    setHealth(prev => ({ ...prev, isChecking: true }));

    const startTime = Date.now();

    try {
      // Use a lightweight endpoint for health checks
      // Try /api/health first, fallback to base URL
      const response = await fetch(`${BACKEND_URL}/api/health`, {
        method: 'GET',
        signal: controller.signal,
        headers: {
          'Accept': 'application/json',
        },
        // Add timeout using AbortSignal.timeout if available (modern browsers)
        ...(AbortSignal.timeout ? { signal: AbortSignal.timeout(timeout) } : {})
      });

      const endTime = Date.now();
      const latency = endTime - startTime;

      if (response.ok) {
        // Successful connection
        consecutiveErrors.current = 0;
        
        const status: ConnectionStatus = 
          latency > degradedThreshold ? 'degraded' : 'connected';

        setHealth({
          status,
          latency,
          lastChecked: new Date(),
          isChecking: false,
          errorCount: 0
        });
      } else {
        // HTTP error
        throw new Error(`HTTP ${response.status}`);
      }
    } catch (error: unknown) {
      // Connection failed
      if (error instanceof Error && error.name === 'AbortError') {
        // Request was aborted, don't count as error
        return;
      }

      consecutiveErrors.current += 1;

      setHealth({
        status: 'disconnected',
        latency: null,
        lastChecked: new Date(),
        isChecking: false,
        errorCount: consecutiveErrors.current
      });
    }
  }, [timeout, degradedThreshold]);

  /**
   * Manual refresh of connection status
   */
  const refresh = useCallback(() => {
    checkHealth();
  }, [checkHealth]);

  // Set up periodic health checks
  useEffect(() => {
    if (!enabled) {
      return;
    }

    // Initial check
    checkHealth();

    // Set up interval for periodic checks
    const intervalId = setInterval(checkHealth, checkInterval);

    // Cleanup
    return () => {
      clearInterval(intervalId);
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, [enabled, checkInterval, checkHealth]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, []);

  return {
    ...health,
    refresh
  };
}
