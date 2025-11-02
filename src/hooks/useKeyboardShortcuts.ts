import { useEffect, useCallback, useRef } from 'react';

export interface KeyboardShortcut {
  key: string;
  ctrl?: boolean;
  shift?: boolean;
  alt?: boolean;
  meta?: boolean;
  description: string;
  action: () => void;
  disabled?: boolean;
}

interface UseKeyboardShortcutsOptions {
  shortcuts: KeyboardShortcut[];
  enabled?: boolean;
  preventDefault?: boolean;
}

/**
 * Hook for registering and handling keyboard shortcuts
 * Supports modifier keys (Ctrl, Shift, Alt, Meta) and prevents conflicts with browser shortcuts
 */
export function useKeyboardShortcuts({
  shortcuts,
  enabled = true,
  preventDefault = true
}: UseKeyboardShortcutsOptions) {
  const shortcutsRef = useRef<KeyboardShortcut[]>(shortcuts);

  // Update ref when shortcuts change
  useEffect(() => {
    shortcutsRef.current = shortcuts;
  }, [shortcuts]);

  const handleKeyDown = useCallback((event: KeyboardEvent) => {
    if (!enabled) {
      return;
    }

    // Don't trigger shortcuts when typing in input fields
    const target = event.target as HTMLElement;
    if (
      target.tagName === 'INPUT' ||
      target.tagName === 'TEXTAREA' ||
      target.isContentEditable
    ) {
      return;
    }

    // Find matching shortcut
    const matchingShortcut = shortcutsRef.current.find(shortcut => {
      if (shortcut.disabled) {
        return false;
      }

      // Normalize key comparison (case-insensitive)
      const keyMatches = event.key.toLowerCase() === shortcut.key.toLowerCase();
      
      // Check modifiers
      const ctrlMatches = !!shortcut.ctrl === (event.ctrlKey || event.metaKey);
      const shiftMatches = !!shortcut.shift === event.shiftKey;
      const altMatches = !!shortcut.alt === event.altKey;
      const metaMatches = !!shortcut.meta === event.metaKey;

      return keyMatches && ctrlMatches && shiftMatches && altMatches && metaMatches;
    });

    if (matchingShortcut) {
      if (preventDefault) {
        event.preventDefault();
      }
      matchingShortcut.action();
    }
  }, [enabled, preventDefault]);

  useEffect(() => {
    if (!enabled) {
      return;
    }

    window.addEventListener('keydown', handleKeyDown);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [enabled, handleKeyDown]);

  /**
   * Get formatted shortcut string for display (e.g., "Ctrl+S")
   */
  const getShortcutString = useCallback((shortcut: KeyboardShortcut): string => {
    const parts: string[] = [];
    
    if (shortcut.ctrl) parts.push('Ctrl');
    if (shortcut.shift) parts.push('Shift');
    if (shortcut.alt) parts.push('Alt');
    if (shortcut.meta) parts.push('Meta');
    
    parts.push(shortcut.key.toUpperCase());
    
    return parts.join('+');
  }, []);

  /**
   * Get all registered shortcuts with their display strings
   */
  const getAllShortcuts = useCallback(() => {
    return shortcutsRef.current
      .filter(s => !s.disabled)
      .map(shortcut => ({
        shortcut: getShortcutString(shortcut),
        description: shortcut.description
      }));
  }, [getShortcutString]);

  return {
    getShortcutString,
    getAllShortcuts
  };
}
