import { useEffect, useRef } from 'react';

// Animation-frame ticker with callback. Runs at display refresh rate without triggering re-renders.
export function useFrameTicker(callback: (timestamp: number) => void) {
  const callbackRef = useRef(callback);
  const rafRef = useRef<number | null>(null);

  // Keep the callback reference fresh without causing re-renders
  useEffect(() => {
    callbackRef.current = callback;
  }, [callback]);

  useEffect(() => {
    const loop = (time: number) => {
      // Call the callback without state updates
      callbackRef.current(time);
      rafRef.current = requestAnimationFrame(loop);
    };

    rafRef.current = requestAnimationFrame(loop);

    return () => {
      if (rafRef.current !== null) {
        cancelAnimationFrame(rafRef.current);
      }
    };
  }, []); // Empty dependency array
}

