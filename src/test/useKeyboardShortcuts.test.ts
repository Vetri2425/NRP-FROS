import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useKeyboardShortcuts, KeyboardShortcut } from '../hooks/useKeyboardShortcuts';

describe('useKeyboardShortcuts', () => {
  const mockAction1 = vi.fn();
  const mockAction2 = vi.fn();
  const mockAction3 = vi.fn();

  const testShortcuts: KeyboardShortcut[] = [
    {
      key: 's',
      ctrl: true,
      description: 'Save',
      action: mockAction1
    },
    {
      key: 'o',
      ctrl: true,
      description: 'Open',
      action: mockAction2
    },
    {
      key: 'delete',
      description: 'Delete',
      action: mockAction3
    }
  ];

  beforeEach(() => {
    vi.clearAllMocks();
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  describe('Hook registration', () => {
    it('should register keyboard shortcuts without errors', () => {
      const { result } = renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      expect(result.current.getAllShortcuts).toBeDefined();
      expect(result.current.getShortcutString).toBeDefined();
    });

    it('should not register shortcuts when disabled', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts, enabled: false })
      );

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true
      });
      window.dispatchEvent(event);

      expect(mockAction1).not.toHaveBeenCalled();
    });
  });

  describe('Shortcut triggering', () => {
    it('should trigger action on matching shortcut', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).toHaveBeenCalledTimes(1);
    });

    it('should handle shortcuts without modifiers', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const event = new KeyboardEvent('keydown', {
        key: 'delete',
        bubbles: true
      });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction3).toHaveBeenCalledTimes(1);
    });

    it('should not trigger action when modifiers do not match', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      // Press 's' without Ctrl
      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: false,
        bubbles: true
      });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).not.toHaveBeenCalled();
    });

    it('should not trigger disabled shortcuts', () => {
      const disabledShortcuts = [
        { ...testShortcuts[0], disabled: true }
      ];

      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: disabledShortcuts })
      );

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).not.toHaveBeenCalled();
    });
  });

  describe('Input field handling', () => {
    it('should not trigger shortcuts when typing in input fields', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const input = document.createElement('input');
      document.body.appendChild(input);

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });
      
      Object.defineProperty(event, 'target', { value: input, enumerable: true });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).not.toHaveBeenCalled();
      
      document.body.removeChild(input);
    });

    it('should not trigger shortcuts when typing in textarea', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const textarea = document.createElement('textarea');
      document.body.appendChild(textarea);

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });
      
      Object.defineProperty(event, 'target', { value: textarea, enumerable: true });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).not.toHaveBeenCalled();
      
      document.body.removeChild(textarea);
    });
  });

  describe('Utility functions', () => {
    it('should format shortcut string correctly', () => {
      const { result } = renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const formatted = result.current.getShortcutString(testShortcuts[0]);
      expect(formatted).toBe('Ctrl+S');
    });

    it('should format shortcut with multiple modifiers', () => {
      const { result } = renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const complexShortcut: KeyboardShortcut = {
        key: 'z',
        ctrl: true,
        shift: true,
        description: 'Redo',
        action: vi.fn()
      };

      const formatted = result.current.getShortcutString(complexShortcut);
      expect(formatted).toBe('Ctrl+Shift+Z');
    });

    it('should get all shortcuts with descriptions', () => {
      const { result } = renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      const all = result.current.getAllShortcuts();
      
      expect(all).toHaveLength(3);
      expect(all[0]).toEqual({ shortcut: 'Ctrl+S', description: 'Save' });
      expect(all[1]).toEqual({ shortcut: 'Ctrl+O', description: 'Open' });
      expect(all[2]).toEqual({ shortcut: 'DELETE', description: 'Delete' });
    });

    it('should exclude disabled shortcuts from getAllShortcuts', () => {
      const mixedShortcuts = [
        testShortcuts[0],
        { ...testShortcuts[1], disabled: true },
        testShortcuts[2]
      ];

      const { result } = renderHook(() =>
        useKeyboardShortcuts({ shortcuts: mixedShortcuts })
      );

      const all = result.current.getAllShortcuts();
      
      expect(all).toHaveLength(2);
      expect(all[0].description).toBe('Save');
      expect(all[1].description).toBe('Delete');
    });
  });

  describe('Cleanup', () => {
    it('should remove event listeners on unmount', () => {
      const { unmount } = renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      unmount();

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).not.toHaveBeenCalled();
    });
  });

  describe('Case insensitivity', () => {
    it('should match keys case-insensitively', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts })
      );

      // Press uppercase 'S' with Ctrl (common browser behavior)
      const event = new KeyboardEvent('keydown', {
        key: 'S',
        ctrlKey: true,
        bubbles: true
      });
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(mockAction1).toHaveBeenCalledTimes(1);
    });
  });

  describe('Prevent default behavior', () => {
    it('should prevent default when preventDefault is true', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts, preventDefault: true })
      );

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });

      const preventDefaultSpy = vi.spyOn(event, 'preventDefault');
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(preventDefaultSpy).toHaveBeenCalled();
    });

    it('should not prevent default when preventDefault is false', () => {
      renderHook(() =>
        useKeyboardShortcuts({ shortcuts: testShortcuts, preventDefault: false })
      );

      const event = new KeyboardEvent('keydown', {
        key: 's',
        ctrlKey: true,
        bubbles: true
      });

      const preventDefaultSpy = vi.spyOn(event, 'preventDefault');
      
      act(() => {
        window.dispatchEvent(event);
      });

      expect(preventDefaultSpy).not.toHaveBeenCalled();
    });
  });
});
