import { describe, it, expect, beforeEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useMissionHistory } from '../hooks/useMissionHistory';
import { Waypoint, MissionFileInfo } from '../types';

describe('useMissionHistory', () => {
  const createMockWaypoints = (count: number): Waypoint[] => {
    return Array.from({ length: count }, (_, i) => ({
      id: i + 1,
      lat: 40.7128 + i * 0.01,
      lng: -74.006 + i * 0.01,
      alt: 100 + i * 10,
      command: 'WAYPOINT',
      frame: 3
    }));
  };

  const createMockFileInfo = (name: string): MissionFileInfo => ({
    name,
    size: 1024,
    type: 'waypoint',
    uploadedAt: new Date().toISOString(),
    waypointCount: 3,
    source: 'file'
  });

  describe('Initial state', () => {
    it('should initialize with no undo/redo available', () => {
      const { result } = renderHook(() => useMissionHistory());

      expect(result.current.canUndo).toBe(false);
      expect(result.current.canRedo).toBe(false);
    });

    it('should initialize with empty history', () => {
      const { result } = renderHook(() => useMissionHistory());

      const historySize = result.current.getHistorySize();
      expect(historySize.past).toBe(0);
      expect(historySize.future).toBe(0);
    });
  });

  describe('pushSnapshot', () => {
    it('should add snapshot to history', () => {
      const { result } = renderHook(() => useMissionHistory());
      const waypoints = createMockWaypoints(3);
      const fileInfo = createMockFileInfo('test.waypoint');

      act(() => {
        result.current.pushSnapshot(waypoints, fileInfo);
      });

      // Need at least 2 snapshots to undo (current and previous)
      expect(result.current.canUndo).toBe(false);
      expect(result.current.getHistorySize().past).toBe(1);
      
      // Add another snapshot
      act(() => {
        result.current.pushSnapshot(createMockWaypoints(4), fileInfo);
      });
      
      expect(result.current.canUndo).toBe(true);
      expect(result.current.getHistorySize().past).toBe(2);
    });

    it('should handle multiple snapshots', () => {
      const { result } = renderHook(() => useMissionHistory());

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), null);
        result.current.pushSnapshot(createMockWaypoints(2), null);
        result.current.pushSnapshot(createMockWaypoints(3), null);
      });

      expect(result.current.getHistorySize().past).toBe(3);
    });

    it('should limit history to maxHistorySize', () => {
      const { result } = renderHook(() => useMissionHistory(5));

      act(() => {
        for (let i = 0; i < 10; i++) {
          result.current.pushSnapshot(createMockWaypoints(i + 1), null);
        }
      });

      const historySize = result.current.getHistorySize();
      expect(historySize.past).toBe(5);
    });

    it('should clear future history when new snapshot is added', () => {
      const { result } = renderHook(() => useMissionHistory());

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(2), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(3), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(4), null);
      });

      // Undo twice
      act(() => {
        result.current.undo();
      });

      act(() => {
        result.current.undo();
      });

      expect(result.current.getHistorySize().future).toBe(2);

      // Add new snapshot should clear future
      act(() => {
        result.current.pushSnapshot(createMockWaypoints(5), null);
      });

      expect(result.current.getHistorySize().future).toBe(0);
    });
  });

  describe('undo', () => {
    it('should undo to previous snapshot', () => {
      const { result } = renderHook(() => useMissionHistory());
      const waypoints1 = createMockWaypoints(1);
      const waypoints2 = createMockWaypoints(2);
      const waypoints3 = createMockWaypoints(3);

      act(() => {
        result.current.pushSnapshot(waypoints1, null);
        result.current.pushSnapshot(waypoints2, null);
        result.current.pushSnapshot(waypoints3, null);
      });

      let snapshot;
      act(() => {
        snapshot = result.current.undo();
      });

      expect(snapshot).not.toBeNull();
      expect(snapshot?.waypoints.length).toBe(2); // Should return to waypoints2
      expect(result.current.canRedo).toBe(true);
    });

    it('should return null when no history to undo', () => {
      const { result } = renderHook(() => useMissionHistory());

      let snapshot;
      act(() => {
        snapshot = result.current.undo();
      });

      expect(snapshot).toBeNull();
    });

    it('should move snapshot from past to future', () => {
      const { result } = renderHook(() => useMissionHistory());

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), null);
        result.current.pushSnapshot(createMockWaypoints(2), null);
        result.current.pushSnapshot(createMockWaypoints(3), null);
      });

      expect(result.current.getHistorySize()).toEqual({ past: 3, future: 0 });

      act(() => {
        result.current.undo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 2, future: 1 });
    });
  });

  describe('redo', () => {
    it('should redo to next snapshot', () => {
      const { result } = renderHook(() => useMissionHistory());
      const waypoints1 = createMockWaypoints(1);
      const waypoints2 = createMockWaypoints(2);
      const waypoints3 = createMockWaypoints(3);

      act(() => {
        result.current.pushSnapshot(waypoints1, null);
      });

      act(() => {
        result.current.pushSnapshot(waypoints2, null);
      });

      act(() => {
        result.current.pushSnapshot(waypoints3, null);
      });

      act(() => {
        result.current.undo();
      });

      let snapshot;
      act(() => {
        snapshot = result.current.redo();
      });

      expect(snapshot).not.toBeNull();
      expect(snapshot?.waypoints.length).toBe(3); // Should redo to waypoints3
    });

    it('should return null when no future history', () => {
      const { result } = renderHook(() => useMissionHistory());

      let snapshot;
      act(() => {
        snapshot = result.current.redo();
      });

      expect(snapshot).toBeNull();
    });

    it('should move snapshot from future to past', () => {
      const { result } = renderHook(() => useMissionHistory());

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(2), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(3), null);
      });

      act(() => {
        result.current.undo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 2, future: 1 });

      act(() => {
        result.current.redo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 3, future: 0 });
    });
  });

  describe('clearHistory', () => {
    it('should clear all history', () => {
      const { result } = renderHook(() => useMissionHistory());

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(2), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(3), null);
      });

      act(() => {
        result.current.undo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 2, future: 1 });

      act(() => {
        result.current.clearHistory();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 0, future: 0 });
      expect(result.current.canUndo).toBe(false);
      expect(result.current.canRedo).toBe(false);
    });
  });

  describe('Data integrity', () => {
    it('should deep clone waypoints to prevent mutations', () => {
      const { result } = renderHook(() => useMissionHistory());
      const originalWaypoints = createMockWaypoints(2);

      act(() => {
        result.current.pushSnapshot(originalWaypoints, null);
      });

      // Mutate original
      originalWaypoints[0].alt = 999;

      let snapshot;
      act(() => {
        snapshot = result.current.undo();
      });

      // Snapshot should not be affected by mutation
      expect(snapshot?.waypoints[0].alt).not.toBe(999);
    });

    it('should deep clone file info', () => {
      const { result } = renderHook(() => useMissionHistory());
      const fileInfo = createMockFileInfo('test.waypoint');

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), fileInfo);
        result.current.pushSnapshot(createMockWaypoints(2), fileInfo);
      });

      // Mutate original
      fileInfo.name = 'modified.waypoint';

      let snapshot;
      act(() => {
        snapshot = result.current.undo();
      });

      // Snapshot should not be affected by mutation
      expect(snapshot?.fileInfo?.name).toBe('test.waypoint');
    });
  });

  describe('Complex undo/redo sequences', () => {
    it('should handle multiple undo/redo operations', () => {
      const { result } = renderHook(() => useMissionHistory());

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(1), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(2), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(3), null);
      });

      act(() => {
        result.current.pushSnapshot(createMockWaypoints(4), null);
      });

      // Undo twice
      act(() => {
        result.current.undo();
      });

      act(() => {
        result.current.undo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 2, future: 2 });

      // Redo once
      act(() => {
        result.current.redo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 3, future: 1 });

      // Undo once more
      act(() => {
        result.current.undo();
      });

      expect(result.current.getHistorySize()).toEqual({ past: 2, future: 2 });
    });
  });
});
