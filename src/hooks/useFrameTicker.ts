import { useEffect, useRef, useState } from 'react';

// Simple animation-frame ticker. Re-renders the component at display refresh rate
// and returns the current high-resolution timestamp (ms).
export function useFrameTicker(): number {
  const [now, setNow] = useState(() => performance.now());
  const rafRef = useRef<number | null>(null);

  useEffect(() => {
    const loop = () => {
      setNow(performance.now());
      rafRef.current = requestAnimationFrame(loop);
    };
    rafRef.current = requestAnimationFrame(loop);
    return () => {
      if (rafRef.current !== null) cancelAnimationFrame(rafRef.current);
    };
  }, []);

  return now;
}

