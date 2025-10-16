import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useGlobalSocket } from './SocketContext';

// Define the shape of our caster credentials
interface CasterCredentials {
  host: string;
  port: string;
  mountpoint: string;
  user: string;
  password?: string;
}

// Define the shape of the context's state and methods
interface RtkContextType {
  // State
  isConnected: boolean;
  isConnecting: boolean;
  status: string;
  bytes: number;
  logs: string[];
  caster: CasterCredentials;
  
  // Methods
  setCaster: (caster: CasterCredentials) => void;
  startStream: () => void;
  stopStream: () => void;
  clearLogs: () => void;
}

// Create the context with a default undefined value
const RtkContext = createContext<RtkContextType | undefined>(undefined);

// Helper function to add a timestamp to log messages
const createLogEntry = (msg: string) => `[${new Date().toLocaleTimeString()}] ${msg}`;

// The main provider component
export const RtkProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const socket = useGlobalSocket();

  // All RTK state now lives here, globally
  const [caster, setCasterState] = useState<CasterCredentials>({
    host: 'caster.emlid.com',
    port: '2101',
    mountpoint: 'YOUR_MOUNTPOINT',
    user: '',
    password: '',
  });
  const [status, setStatus] = useState('Disconnected');
  const [isConnecting, setIsConnecting] = useState(false);
  const [isConnected, setIsConnected] = useState(false);
  const [logs, setLogs] = useState<string[]>([]);
  const [bytes, setBytes] = useState(0);

  // Function to add a new log entry
  const addLog = useCallback((msg: string) => {
    setLogs((prev) => [...prev.slice(-99), createLogEntry(msg)]);
  }, []);
  
  const clearLogs = useCallback(() => {
    setLogs([createLogEntry('Console cleared')]);
  }, []);

  // Effect to load saved credentials on startup
  useEffect(() => {
    try {
      const savedCaster = localStorage.getItem('rtk_caster');
      if (savedCaster) {
        setCasterState(JSON.parse(savedCaster));
      }
    } catch (e) {
      console.error("Failed to load saved caster credentials:", e);
    }
  }, []);

  // This single useEffect hook manages all RTK socket events for the entire app lifecycle
  useEffect(() => {
    if (!socket) return;

    const onLog = (data: { message: string }) => addLog(data.message);
    const onRtkForwarded = (msg: { total: number }) => setBytes(msg.total);
    const onCasterStatus = (msg: { status: string; message: string }) => {
      addLog(`📡 ${msg.status}: ${msg.message}`);
      const statusLower = msg.status.toLowerCase();

      if (statusLower === 'connecting') {
        setIsConnecting(true);
        setIsConnected(false);
        setStatus('Connecting...');
      } else if (statusLower === 'connected') {
        setIsConnecting(false);
        setIsConnected(true);
        setStatus('Connected');
      } else { // 'disconnected', 'error', etc.
        setIsConnecting(false);
        setIsConnected(false);
        setStatus(statusLower === 'error' ? 'Error' : 'Disconnected');
      }
    };

    // Attach listeners
    socket.on('rtk_log', onLog);
    socket.on('caster_status', onCasterStatus);
    socket.on('rtk_forwarded', onRtkForwarded);

    // Auto-reconnect logic
    const shouldStream = localStorage.getItem('rtk_should_stream') === 'true';
    if (shouldStream) {
      addLog('🚀 Auto-reconnecting to previous RTK stream...');
      socket.emit('connect_caster', caster);
    }

    // Cleanup listeners on socket disconnect, not on component unmount
    return () => {
      socket.off('rtk_log', onLog);
      socket.off('caster_status', onCasterStatus);
      socket.off('rtk_forwarded', onRtkForwarded);
    };
  }, [socket, addLog, caster]);

  // Method to update caster and save to local storage
  const setCaster = (newCaster: CasterCredentials) => {
    setCasterState(newCaster);
    try {
      localStorage.setItem('rtk_caster', JSON.stringify(newCaster));
    } catch (e) {
      console.error("Failed to save caster credentials:", e);
    }
  };

  // Method to start the stream
  const startStream = () => {
    if (!socket) return addLog('❌ Cannot start stream: Backend not connected.');
    addLog(`🔄 Connecting to ${caster.host}:${caster.port}/${caster.mountpoint}...`);
    setBytes(0); // Reset byte count on new connection
    localStorage.setItem('rtk_should_stream', 'true');
    socket.emit('connect_caster', {
      ...caster,
      port: parseInt(caster.port) || 2101,
    });
  };

  // Method to stop the stream
  const stopStream = () => {
    if (!socket) return addLog('❌ Cannot stop stream: Backend not connected.');
    addLog('🛑 Stopping RTK stream...');
    localStorage.setItem('rtk_should_stream', 'false');
    socket.emit('disconnect_caster');
  };

  const contextValue: RtkContextType = {
    isConnected, isConnecting, status, bytes, logs, caster,
    setCaster, startStream, stopStream, clearLogs
  };

  return <RtkContext.Provider value={contextValue}>{children}</RtkContext.Provider>;
};

// Custom hook to easily consume the context
export const useRtk = () => {
  const context = useContext(RtkContext);
  if (context === undefined) {
    throw new Error('useRtk must be used within an RtkProvider');
  }
  return context;
};