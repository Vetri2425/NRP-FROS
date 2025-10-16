import { useState, useEffect, Dispatch, SetStateAction } from 'react';

/**
 * React state that mirrors its value in localStorage so data survives refreshes.
 * Only simple serialisable data should be stored.
 */
export function usePersistentState<T>(key: string, defaultValue: T): [T, Dispatch<SetStateAction<T>>] {
  const [state, setState] = useState<T>(() => {
    if (typeof window === 'undefined') {
      return defaultValue;
    }
    try {
      const stored = window.localStorage.getItem(key);
      if (stored === null) {
        return defaultValue;
      }
      return JSON.parse(stored) as T;
    } catch {
      return defaultValue;
    }
  });

  useEffect(() => {
    if (typeof window === 'undefined') {
      return;
    }
    try {
      window.localStorage.setItem(key, JSON.stringify(state));
    } catch {
      // Ignore storage write failures (e.g., quota exceeded or private mode)
    }
  }, [key, state]);

  return [state, setState];
}
