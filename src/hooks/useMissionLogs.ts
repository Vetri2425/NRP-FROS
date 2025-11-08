import { useCallback, useMemo, useRef, SetStateAction } from 'react';
import { MissionLog, LogEntry } from '../types';
import { usePersistentState } from './usePersistentState';

type PersistedMissionLog = Omit<MissionLog, 'timestamp'> & { timestamp: string };

export const useMissionLogs = () => {
  const [missionLogsStore, setMissionLogsStore] = usePersistentState<PersistedMissionLog[]>('app:missionLogs', []);
  const [activeLogId, setActiveLogId] = usePersistentState<string | null>('app:activeMissionLogId', null);
  const activeLogIdRef = useRef<string | null>(activeLogId);
  activeLogIdRef.current = activeLogId;

  const missionLogs = useMemo<MissionLog[]>(() => {
    return missionLogsStore.map(log => ({
      ...log,
      timestamp: new Date(log.timestamp),
    }));
  }, [missionLogsStore]);

  const setMissionLogs = useCallback((updater: SetStateAction<MissionLog[]>) => {
    setMissionLogsStore(prevStore => {
      const previous = prevStore.map(log => ({
        ...log,
        timestamp: new Date(log.timestamp),
      })) as MissionLog[];

      const next = typeof updater === 'function' ? (updater as (prev: MissionLog[]) => MissionLog[])(previous) : updater;

      return next.map(log => ({
        ...log,
        timestamp: (log.timestamp instanceof Date ? log.timestamp : new Date(log.timestamp)).toISOString(),
      }));
    });
  }, [setMissionLogsStore]);

  const createNewLog = useCallback((name: string) => {
    if (activeLogIdRef.current) {
      setMissionLogs(prev => prev.map(log =>
        log.id === activeLogIdRef.current && log.status === 'In Progress'
          ? { ...log, status: 'Incomplete' }
          : log
      ));
    }

    const newLog: MissionLog = {
      id: `log-${Date.now()}`,
      name: name || 'Unnamed Mission',
      status: 'In Progress',
      timestamp: new Date(),
      entries: [],
    };
    setMissionLogs(prev => [newLog, ...prev]);
    activeLogIdRef.current = newLog.id;
    setActiveLogId(newLog.id);
  }, [setMissionLogs, setActiveLogId]);

  const addLogEntry = useCallback((entry: Omit<LogEntry, 'timestamp'> & { timestamp?: string }) => {
    if (!activeLogIdRef.current) return;

    const newLogEntry: LogEntry = {
      ...entry,
      lat: entry.lat ?? null,
      lng: entry.lng ?? null,
      timestamp: entry.timestamp ?? new Date().toISOString(),
    };

    setMissionLogs(prevLogs => prevLogs.map(log =>
      log.id === activeLogIdRef.current
        ? { ...log, entries: [...log.entries, newLogEntry] }
        : log
    ));
  }, [setMissionLogs]);

  const updateActiveLogStatus = useCallback((status: 'Completed' | 'Incomplete') => {
    if (activeLogIdRef.current) {
      setMissionLogs(prev => prev.map(log =>
        log.id === activeLogIdRef.current && log.status === 'In Progress'
          ? { ...log, status }
          : log
      ));
      activeLogIdRef.current = null;
      setActiveLogId(null);
    }
  }, [setMissionLogs, setActiveLogId]);

  const getActiveLogEntries = useCallback((): LogEntry[] => {
    if (activeLogIdRef.current) {
      const activeLog = missionLogs.find(log => log.id === activeLogIdRef.current && log.status === 'In Progress');
      if (activeLog) {
        return activeLog.entries;
      }
    }
    return missionLogs.length > 0 ? missionLogs[0].entries : [];
  }, [missionLogs]);

  const clearLogs = useCallback(() => {
    setMissionLogs(() => []);
    activeLogIdRef.current = null;
    setActiveLogId(null);
  }, [setMissionLogs, setActiveLogId]);

  return {
    missionLogs,
    createNewLog,
    addLogEntry,
    updateActiveLogStatus,
    getActiveLogEntries,
    clearLogs,
  };
};
