import { useState, useCallback, useRef } from 'react';
import { Waypoint, MissionFileInfo } from '../types';

interface MissionSnapshot {
  waypoints: Waypoint[];
  fileInfo: MissionFileInfo | null;
  timestamp: number;
}

interface UseMissionHistoryReturn {
  canUndo: boolean;
  canRedo: boolean;
  undo: () => MissionSnapshot | null;
  redo: () => MissionSnapshot | null;
  pushSnapshot: (waypoints: Waypoint[], fileInfo: MissionFileInfo | null) => void;
  clearHistory: () => void;
  getHistorySize: () => { past: number; future: number };
}

/**
 * Hook for managing mission waypoint history with undo/redo functionality.
 * 
 * Features:
 * - Maintains up to 20 snapshots in history
 * - Supports undo/redo operations
 * - Automatically clears future history when new changes are made
 * - Provides history size information
 * 
 * @param maxHistorySize - Maximum number of snapshots to keep (default: 20)
 * @returns Object with undo/redo controls and state
 */
export const useMissionHistory = (maxHistorySize: number = 20): UseMissionHistoryReturn => {
  const [pastSnapshots, setPastSnapshots] = useState<MissionSnapshot[]>([]);
  const [futureSnapshots, setFutureSnapshots] = useState<MissionSnapshot[]>([]);
  
  // Use ref to track if we're currently applying undo/redo to prevent duplicate snapshots
  const isApplyingHistory = useRef(false);

  /**
   * Push a new snapshot to history
   */
  const pushSnapshot = useCallback((waypoints: Waypoint[], fileInfo: MissionFileInfo | null) => {
    // Don't create snapshot if we're applying undo/redo
    if (isApplyingHistory.current) {
      return;
    }

    const snapshot: MissionSnapshot = {
      waypoints: JSON.parse(JSON.stringify(waypoints)), // Deep clone
      fileInfo: fileInfo ? JSON.parse(JSON.stringify(fileInfo)) : null,
      timestamp: Date.now()
    };

    setPastSnapshots(prev => {
      const updated = [...prev, snapshot];
      // Limit history size
      if (updated.length > maxHistorySize) {
        return updated.slice(updated.length - maxHistorySize);
      }
      return updated;
    });

    // Clear future history when new change is made
    setFutureSnapshots([]);
  }, [maxHistorySize]);

  /**
   * Undo to previous snapshot
   */
  const undo = useCallback((): MissionSnapshot | null => {
    if (pastSnapshots.length < 2) {
      // Need at least 2 snapshots: current and previous
      return null;
    }

    isApplyingHistory.current = true;

    // Move current snapshot (last) to future
    const currentSnapshot = pastSnapshots[pastSnapshots.length - 1];
    const previousSnapshot = pastSnapshots[pastSnapshots.length - 2];
    const newPast = pastSnapshots.slice(0, -1); // Remove current from past

    setPastSnapshots(newPast);
    setFutureSnapshots(prev => [currentSnapshot, ...prev]); // Add to start of future

    // Reset flag immediately - the state updates will trigger re-render
    isApplyingHistory.current = false;

    return previousSnapshot; // Return the snapshot to restore to
  }, [pastSnapshots]);

  /**
   * Redo to next snapshot
   */
  const redo = useCallback((): MissionSnapshot | null => {
    if (futureSnapshots.length === 0) {
      return null;
    }

    isApplyingHistory.current = true;

    // Get the first snapshot from future
    const nextSnapshot = futureSnapshots[0];
    const newFuture = futureSnapshots.slice(1);

    setFutureSnapshots(newFuture);
    setPastSnapshots(prev => [...prev, nextSnapshot]);

    // Reset flag immediately - the state updates will trigger re-render
    isApplyingHistory.current = false;

    return nextSnapshot;
  }, [futureSnapshots]);

  /**
   * Clear all history
   */
  const clearHistory = useCallback(() => {
    setPastSnapshots([]);
    setFutureSnapshots([]);
  }, []);

  /**
   * Get current history sizes
   */
  const getHistorySize = useCallback(() => {
    return {
      past: pastSnapshots.length,
      future: futureSnapshots.length
    };
  }, [pastSnapshots.length, futureSnapshots.length]);

  return {
    canUndo: pastSnapshots.length > 1, // Need at least 2: current and previous
    canRedo: futureSnapshots.length > 0,
    undo,
    redo,
    pushSnapshot,
    clearHistory,
    getHistorySize
  };
};
