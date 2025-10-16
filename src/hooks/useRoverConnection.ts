import { useState, useRef, useCallback, useEffect } from 'react';
import { io, Socket } from "socket.io-client";
import { BackendLogEntry, RoverData, Waypoint } from '../types';
export type { RoverData } from '../types';

// --- Servo Configuration Type ---
export type ServoConfig =
  | {
      mode: 'MARK_AT_WAYPOINT';
      servoNumber: number;
      pwmOn: number;
      pwmOff: number;
      fromWp: number;
      toWp: number;
      sprayDuration: number;
    }
  | {
      mode: 'CONTINUOUS_LINE';
      servoNumber: number;
      pwmOn: number;
      pwmOff: number;
      startPoint: number;
      endPoint: number;
    }
  | {
      mode: 'INTERVAL_SPRAY';
      servoNumber: number;
      pwmOn: number;
      pwmOff: number;
      distanceOnMeters: number;
      distanceOffMeters: number;
      startWp: number;
      endWp: number;
    };

// Expose the active socket instance for module-level helpers
let sharedSocket: Socket | null = null;

// Normalize union to backend-friendly payload (maps mode names to backend)
function normalizeServoConfig(config: ServoConfig | undefined): Record<string, any> | undefined {
    if (!config) return undefined;
    switch (config.mode) {
        case 'MARK_AT_WAYPOINT':
            return {
                mode: 'MARK_AT_WAYPOINT',
                servoNumber: config.servoNumber,
                pwmOn: config.pwmOn,
                pwmOff: config.pwmOff,
                fromWp: (config as any).fromWp,
                toWp: (config as any).toWp,
                sprayDuration: (config as any).sprayDuration,
            };
        case 'CONTINUOUS_LINE':
            return {
                mode: 'CONTINUOUS_LINE',
                servoNumber: config.servoNumber,
                pwmOn: config.pwmOn,
                pwmOff: config.pwmOff,
                startPoint: (config as any).startPoint,
                endPoint: (config as any).endPoint,
            };
        case 'INTERVAL_SPRAY':
            return {
                mode: 'INTERVAL_SPRAY',
                servoNumber: config.servoNumber,
                pwmOn: config.pwmOn,
                pwmOff: config.pwmOff,
                distanceOnMeters: (config as any).distanceOnMeters,
                distanceOffMeters: (config as any).distanceOffMeters,
                startWp: (config as any).startWp,
                endWp: (config as any).endWp,
            };
    }
}

// Replace write function with a simple boolean-returning helper using the active socket
export async function writeMissionToRover(
    waypoints: Waypoint[],
    servoConfig?: ServoConfig
): Promise<boolean> {
    return new Promise<boolean>((resolve) => {
        const sock = sharedSocket;
        if (!sock) {
            resolve(false);
            return;
        }
        const normalized = normalizeServoConfig(servoConfig);
        const handler = (resp: any) => {
            resolve(resp?.ok === true);
        };
        sock.once('mission_uploaded', handler);
        sock.emit('mission_upload', { waypoints, servoConfig: normalized });
    });
}

type RoverConnectionOptions = {
    onMissionEvent?: (entry: BackendLogEntry) => void;
    onMissionLogsSnapshot?: (entries: BackendLogEntry[]) => void;
};

const JETSON_BACKEND_URL = import.meta.env.VITE_JETSON_BACKEND_URL;
// Default to forcing WebSocket transport (prevents Engine.IO polling 400s)
const FORCE_WEBSOCKET = (import.meta.env.VITE_FORCE_WEBSOCKET ?? 'true').toString().toLowerCase() === 'true';

export type ConnectionStatus = 'DISCONNECTED' | 'CONNECTING' | 'WAITING_FOR_ROVER' | 'CONNECTED_TO_ROVER' | 'ERROR';
export type BackendLinkStatus = 'OFFLINE' | 'CONNECTING' | 'ONLINE' | 'ERROR';
export type CommandResponse = { status: 'success' | 'error' | 'pending', message: string, waypoints?: Waypoint[], command?: string, result?: number };

const initialRoverData: RoverData = {
    position: null, 
    heading: 0, 
    battery: -1, 
    status: 'disarmed', 
    mode: 'UNKNOWN',
    rtk_status: 'N/A', 
    signal_strength: 'N/A', 
    current_waypoint_id: null, 
    rc_connected: false,
    hrms: '0.000', 
    vrms: '0.000', 
    imu_status: 'UNALIGNED', 
    activeWaypointIndex: null,
    completedWaypointIds: [], 
    distanceToNext: 0,
    lastUpdate: undefined,
    telemetryAgeMs: undefined,
};

const clampBattery = (value: unknown, fallback: number) => {
    if (typeof value === 'number' && Number.isFinite(value)) {
        return Math.max(0, Math.min(100, Math.round(value)));
    }
    if (typeof value === 'string') {
        const parsed = Number(value);
        if (!Number.isNaN(parsed)) {
            return Math.max(0, Math.min(100, Math.round(parsed)));
        }
    }
    return fallback;
};

const toBoolean = (value: unknown, fallback: boolean) => {
    if (typeof value === 'boolean') return value;
    if (typeof value === 'number') return value !== 0;
    return fallback;
};

const mapIncomingPayload = (payload: Record<string, unknown>): Partial<RoverData> => {
    const mapped: Partial<RoverData> = {};

    Object.entries(payload).forEach(([key, value]) => {
        if (key === 'last_update') {
            if (typeof value === 'number') {
                mapped.lastUpdate = new Date(value * 1000).toISOString();
            } else if (typeof value === 'string') {
                mapped.lastUpdate = value;
            }
            return;
        }

        (mapped as Record<string, unknown>)[key] = value;
    });

    return mapped;
};

const deriveSignalStrength = (
    incoming: unknown,
    rcConnected: boolean,
    fallback: string,
    status: ConnectionStatus
): string => {
    if (status !== 'CONNECTED_TO_ROVER') return 'No Link';
    if (typeof incoming === 'string' && incoming.trim().length > 0) {
        return incoming;
    }
    return rcConnected ? 'Connected' : fallback || 'Weak';
};

const normaliseHorizontalAccuracy = (value: unknown, fallback: string | number) => {
    if (typeof value === 'number') {
        return value.toFixed(3);
    }
    if (typeof value === 'string' && value.trim().length > 0) {
        return value;
    }
    return fallback;
};

const normaliseRoverData = (
    prevData: RoverData,
    rawPayload: Record<string, unknown>,
    status: ConnectionStatus
): RoverData => {
    const payload = mapIncomingPayload(rawPayload);
    
    // Start fresh - don't carry over old data
    const merged: RoverData = {
        ...prevData,
        ...payload,
    } as RoverData;

    // FIXED: Always use the incoming mode if provided
    if (payload.mode !== undefined && payload.mode !== null) {
        merged.mode = typeof payload.mode === 'string' ? payload.mode.toUpperCase() : String(payload.mode).toUpperCase();
    }

    // FIXED: Always use the incoming status if provided
    if (payload.status !== undefined) {
        merged.status = (payload as any).status === 'armed' ? 'armed' : 'disarmed';
    }

    merged.battery = clampBattery((payload as any).battery ?? merged.battery, merged.battery);
    merged.rc_connected = toBoolean((payload as any).rc_connected, merged.rc_connected);
    merged.signal_strength = deriveSignalStrength(
        (payload as any).signal_strength,
        merged.rc_connected,
        typeof prevData.signal_strength === 'string' ? prevData.signal_strength : 'Weak',
        status
    );

    merged.hrms = normaliseHorizontalAccuracy((payload as any).hrms, merged.hrms);
    merged.vrms = normaliseHorizontalAccuracy((payload as any).vrms, merged.vrms);

    if ((payload as any).position === null) {
        merged.position = null;
    } else if ((payload as any).position && typeof (payload as any).position === 'object') {
        const incomingPosition = (payload as any).position as { lat?: unknown; lng?: unknown };
        const lat = typeof incomingPosition.lat === 'number' ? incomingPosition.lat : merged.position?.lat ?? 0;
        const lng = typeof incomingPosition.lng === 'number' ? incomingPosition.lng : merged.position?.lng ?? 0;
        merged.position = typeof incomingPosition.lat === 'number' && typeof incomingPosition.lng === 'number'
            ? { lat, lng }
            : merged.position;
    }

    if ((payload as any).lastUpdate) {
        merged.lastUpdate = (payload as any).lastUpdate as any;
    } else if (!merged.lastUpdate) {
        merged.lastUpdate = new Date().toISOString();
    }

    merged.telemetryAgeMs = merged.lastUpdate ? Math.max(0, Date.now() - new Date(merged.lastUpdate).getTime()) : undefined;

    return merged;
};

const normaliseBackendLogEntry = (payload: Record<string, unknown>): BackendLogEntry => {
    const rawTimestamp = payload?.timestamp;
    let isoTimestamp: string;
    if (typeof rawTimestamp === 'number') {
        isoTimestamp = new Date(rawTimestamp * 1000).toISOString();
    } else if (typeof rawTimestamp === 'string') {
        const parsed = new Date(rawTimestamp);
        isoTimestamp = Number.isNaN(parsed.getTime()) ? new Date().toISOString() : parsed.toISOString();
    } else {
        isoTimestamp = new Date().toISOString();
    }

    const message = typeof payload?.message === 'string'
        ? payload.message.trim()
        : String(payload?.message ?? '').trim();

    return {
        timestamp: isoTimestamp,
        message,
        lat: typeof payload?.lat === 'number' ? payload.lat : null,
        lng: typeof payload?.lng === 'number' ? payload.lng : null,
        waypointId: typeof payload?.waypointId === 'number'
            ? payload.waypointId
            : (typeof (payload as any)?.waypoint_id === 'number' ? (payload as any).waypoint_id : null),
        status: typeof payload?.status === 'string' ? payload.status : null,
        servoAction: typeof payload?.servoAction === 'string' ? payload.servoAction : null,
    };
};

export const useRoverConnection = (options: RoverConnectionOptions = {}) => {
    const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>('DISCONNECTED');
    const [roverData, setRoverData] = useState<RoverData>(initialRoverData);
    const [backendStatus, setBackendStatus] = useState<BackendLinkStatus>('OFFLINE');
    const [logs, setLogs] = useState<string[]>([]);
    const [uploadProgress, setUploadProgress] = useState(0);
    const socketRef = useRef<Socket | null>(null);
    const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
    const connectionAttemptRef = useRef<number>(0);
    const commandTimeoutRef = useRef<{ [key: string]: NodeJS.Timeout }>({});
    // Optimistic UI override for local, user-initiated changes (mode/arm)
    type OptimisticOverlay = { mode?: string; status?: 'armed' | 'disarmed'; until?: number };
    const optimisticRef = useRef<OptimisticOverlay>({});
    const connectionStatusRef = useRef<ConnectionStatus>('DISCONNECTED');
    const backendStatusRef = useRef<BackendLinkStatus>('OFFLINE');
    const manualDisconnectRef = useRef<boolean>(false);
    const optionsRef = useRef<RoverConnectionOptions>(options);

    useEffect(() => {
        optionsRef.current = options;
    }, [options]);

    const updateBackendStatus = useCallback((next: BackendLinkStatus) => {
        if (backendStatusRef.current === next) {
            return;
        }
        backendStatusRef.current = next;
        setBackendStatus(next);
    }, [setBackendStatus]);

    useEffect(() => {
        connectionStatusRef.current = connectionStatus;
    }, [connectionStatus]);

    // Logging system with frontend timestamps
    const addLog = useCallback((message: string, type: 'info' | 'error' | 'success' = 'info') => {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = `[${timestamp}] ${type.toUpperCase()}: ${message}`;
        setLogs(prev => [...prev.slice(-49), logEntry]);
    }, []);

    // Safe connection status update with state validation
    const updateConnectionStatus = useCallback((newStatus: ConnectionStatus, reason?: string) => {
        setConnectionStatus(prevStatus => {
            if (prevStatus === newStatus) return prevStatus;
            
            const logLevel = newStatus === 'ERROR' ? 'error' : 
                           newStatus === 'CONNECTED_TO_ROVER' ? 'success' : 'info';
            
            addLog(`Status: ${prevStatus} → ${newStatus}${reason ? ` (${reason})` : ''}`, logLevel);
            return newStatus;
        });
    }, [addLog]);

    // Apply local optimistic overrides (if not expired) to a RoverData snapshot
    const applyOptimistic = useCallback((data: RoverData): RoverData => {
        const opt = optimisticRef.current;
        if (!opt) return data;
        if (opt.until && Date.now() > opt.until) {
            optimisticRef.current = {};
            return data;
        }
        const patched = { ...data } as RoverData;
        if (opt.status) patched.status = opt.status;
        if (opt.mode) patched.mode = (opt.mode || '').toUpperCase();
        return patched;
    }, []);

    const setOptimistic = useCallback((overlay: Partial<OptimisticOverlay>, ttlMs = 8000) => {
        optimisticRef.current = { ...optimisticRef.current, ...overlay, until: Date.now() + Math.max(1000, ttlMs) };
        // Trigger a re-render with the overlay applied immediately
        setRoverData(prev => applyOptimistic(prev));
    }, [applyOptimistic]);

    const clearOptimistic = useCallback((keys?: Array<'mode' | 'status'>) => {
        if (!keys || keys.length === 0) {
            optimisticRef.current = {};
        } else {
            const next = { ...optimisticRef.current } as any;
            keys.forEach(k => delete next[k]);
            optimisticRef.current = next;
        }
        setRoverData(prev => ({ ...prev } as RoverData));
    }, []);

    // Clean up socket and timers
    const cleanupSocket = useCallback((options: { resetData?: boolean } = {}) => {
        const { resetData = false } = options;
        const socket = socketRef.current;
        if (socket) {
            addLog('Cleaning up socket connection...');
            socket.removeAllListeners();
        }
        socketRef.current = null;
        sharedSocket = null;

        if (reconnectTimeoutRef.current) {
            clearTimeout(reconnectTimeoutRef.current);
            reconnectTimeoutRef.current = null;
        }

        Object.values(commandTimeoutRef.current).forEach(timeout => clearTimeout(timeout));
        commandTimeoutRef.current = {};

        if (resetData) {
            setRoverData(initialRoverData);
        }

        updateBackendStatus('OFFLINE');
    }, [addLog, updateBackendStatus]);

    // Main connection function with improved error handling
    const connect = useCallback(() => {
        if (!JETSON_BACKEND_URL) {
            addLog('Backend URL is not configured (VITE_JETSON_BACKEND_URL)', 'error');
            updateBackendStatus('ERROR');
            updateConnectionStatus('ERROR', 'backend URL missing');
            return;
        }

        const existingSocket = socketRef.current;
        manualDisconnectRef.current = false;

        if (existingSocket) {
            if (existingSocket.connected || (existingSocket as any).active) {
                addLog('Backend socket already active, skipping new connection attempt');
                updateBackendStatus(existingSocket.connected ? 'ONLINE' : 'CONNECTING');
                return;
            }

            addLog('Reusing existing socket instance for reconnection');
            connectionAttemptRef.current += 1;
            updateBackendStatus('CONNECTING');
            updateConnectionStatus('CONNECTING');
            existingSocket.connect();
            return;
        }

        connectionAttemptRef.current += 1;
        addLog(`Connection attempt #${connectionAttemptRef.current} to: ${JETSON_BACKEND_URL}`);

        updateBackendStatus('CONNECTING');
        updateConnectionStatus('CONNECTING');

        try {
            const socket = io(JETSON_BACKEND_URL, {
                // Force WebSocket to avoid polling fallback errors (HTTP 400)
                transports: FORCE_WEBSOCKET ? ['websocket'] : undefined,
                upgrade: FORCE_WEBSOCKET ? false : undefined,
                reconnection: true,
                reconnectionAttempts: Infinity,
                reconnectionDelay: 2000,
                reconnectionDelayMax: 10000,
                timeout: 20000,
                forceNew: false,
                autoConnect: true
            });

            socketRef.current = socket;
            sharedSocket = socket;

            // Connection event handlers
            socket.on('connect', () => {
                addLog('WebSocket connected to backend server', 'success');
                connectionAttemptRef.current = 0;
                updateBackendStatus('ONLINE');
                try {
                    socket.emit('request_mission_logs');
                } catch {}
            });

            socket.on('disconnect', (reason: string) => {
                addLog(`Backend socket disconnected: ${reason}`, manualDisconnectRef.current ? 'info' : 'error');

                updateBackendStatus('OFFLINE');

                Object.values(commandTimeoutRef.current).forEach(timeout => clearTimeout(timeout));
                commandTimeoutRef.current = {};
                
                if (manualDisconnectRef.current) {
                    manualDisconnectRef.current = false;
                    updateConnectionStatus('DISCONNECTED', reason || 'client disconnect');
                    setRoverData(initialRoverData);
                    cleanupSocket();
                    return;
                }

                if (reconnectTimeoutRef.current) {
                    return; // reconnect already scheduled
                }

                const delay = Math.min(5000, 2000 * Math.max(1, connectionAttemptRef.current));
                addLog(`Lost backend link (${reason}). Reconnecting in ${delay / 1000}s`, 'error');
                reconnectTimeoutRef.current = setTimeout(() => {
                    reconnectTimeoutRef.current = null;
                    connect();
                }, delay);
            });

            socket.on('connect_error', (error: Error) => {
                addLog(`Connection error: ${error.message}`, 'error');
                updateBackendStatus('ERROR');
                updateConnectionStatus('ERROR', error.message);

                if (reconnectTimeoutRef.current) {
                    return;
                }
                const delay = Math.min(2000 * Math.pow(1.5, connectionAttemptRef.current - 1), 30000);
                addLog(`Retrying in ${Math.round(delay / 1000)}s...`);
                reconnectTimeoutRef.current = setTimeout(() => {
                    reconnectTimeoutRef.current = null;
                    connect();
                }, delay);
            });

            // Backend-specific event handlers
            socket.on('connection_status', (data: { status: ConnectionStatus; message?: string }) => {
                addLog(`Backend reports: ${data.status}${data.message ? ` - ${data.message}` : ''}`);
                updateConnectionStatus(data.status, data.message || 'backend update');

                if (data.status === 'CONNECTED_TO_ROVER') {
                    updateBackendStatus('ONLINE');
                }

                if (data.status === 'ERROR' && data.message) {
                    addLog(`Backend error details: ${data.message}`, 'error');
                }
            });

            // RTK caster status handler (global broadcast for UI toasts)
            socket.on('caster_status', (msg: { status: string; message: string }) => {
                try {
                    const statusLower = (msg.status || '').toLowerCase();
                    if (statusLower === 'disconnected' || statusLower === 'error') {
                        window.dispatchEvent(new CustomEvent('rtk_stream_status', { detail: msg }));
                    }
                } catch {}
            });

            // Enhanced rover_data handler
            socket.on('rover_data', (data: Partial<RoverData>) => {
                const payload = data as Record<string, unknown>;

                setRoverData(prevData => {
                    const normalized = normaliseRoverData(prevData, payload, connectionStatusRef.current);
                    const withOverlay = applyOptimistic(normalized);
                    // Force new object reference
                    const freshData = {
                        ...withOverlay,
                        _timestamp: Date.now()
                    } as RoverData;

                    return freshData;
                });
            });

            // Command response handler
            socket.on('command_response', (response: CommandResponse & { command?: string }) => {
                if ((response as any).status === 'pending') {
                    addLog(`Command ${response.command ?? ''} pending: ${response.message}`, 'info');
                    return;
                }
                const logLevel = response.status === 'success' ? 'success' : 'error';
                addLog(`${response.command ? `${response.command}: ` : ''}${response.message}`, logLevel);
            });

            socket.on('vehicle_status_text', (event: { severity: number; text: string }) => {
                const level = event.severity >= 4 ? 'error' : 'info';
                addLog(`Vehicle: ${event.text}`, level as any);
            });

            socket.on('rover_reconnect_ack', (payload: { status: string; message?: string }) => {
                addLog(`Rover reconnect: ${payload.status}${payload.message ? ` - ${payload.message}` : ''}`, payload.status === 'success' ? 'success' : 'info');
            });

            // Reconnection handlers
            socket.on('reconnect', (attemptNumber: number) => {
                addLog(`Backend socket reconnected after ${attemptNumber} attempts`, 'success');
                connectionAttemptRef.current = 0;
                updateBackendStatus('ONLINE');
                try {
                    socket.emit('request_mission_logs');
                } catch {}
            });

            socket.on('reconnect_error', (error: Error) => {
                addLog(`Reconnection failed: ${error.message}`, 'error');
                updateBackendStatus('ERROR');
            });

            socket.on('mission_event', (payload: Record<string, unknown>) => {
                try {
                    const entry = normaliseBackendLogEntry(payload);
                    optionsRef.current.onMissionEvent?.(entry);
                } catch (err) {
                    addLog(`Failed to process mission event payload: ${String(err)}`, 'error');
                }
            });

            socket.on('mission_logs_snapshot', (payload: unknown) => {
                if (!payload) return;
                try {
                    const entriesArray = Array.isArray(payload) ? payload : [];
                    const entries = entriesArray
                        .map(item => normaliseBackendLogEntry(item as Record<string, unknown>));
                    optionsRef.current.onMissionLogsSnapshot?.(entries);
                } catch (err) {
                    addLog(`Failed to process mission logs snapshot: ${String(err)}`, 'error');
                }
            });

            socket.on('reconnect_failed', () => {
                addLog('All reconnection attempts exhausted', 'error');
                updateBackendStatus('ERROR');
                updateConnectionStatus('ERROR', 'reconnection failed');
            });

            // Mission upload progress
            socket.on('mission_upload_progress', (data: { progress: number }) => {
                setUploadProgress(data.progress);
            });

            addLog('Socket created with all event listeners attached');

        } catch (error) {
            addLog(`Socket creation failed: ${error}`, 'error');
            updateBackendStatus('ERROR');
            updateConnectionStatus('ERROR', 'socket creation failed');
        }
    }, [JETSON_BACKEND_URL, addLog, updateBackendStatus, updateConnectionStatus]);

    useEffect(() => {
        connect();

        return () => {
            manualDisconnectRef.current = true;
            if (socketRef.current) {
                try {
                    socketRef.current.disconnect();
                } catch (error) {
                }
            }
            cleanupSocket({ resetData: true });
        };
    }, [connect, cleanupSocket]);

    // Disconnect function
    const disconnect = useCallback(() => {
        const socket = socketRef.current;
        if (socket) {
            manualDisconnectRef.current = true;
            addLog('User requested backend disconnect');
            socket.disconnect();
            cleanupSocket({ resetData: true });
            connectionAttemptRef.current = 0;
            updateBackendStatus('OFFLINE');
            updateConnectionStatus('DISCONNECTED', 'client disconnect');
        }
    }, [addLog, cleanupSocket, updateBackendStatus, updateConnectionStatus]);

    // Send command with timeout and error handling
    const sendCommand = useCallback((command: any): Promise<CommandResponse> => {
        return new Promise((resolve) => {
            const socket = socketRef.current;
            const target = command?.command;
            const cmdStr = JSON.stringify(command, null, 0);

            if (!socket?.connected) {
                const error = { status: 'error' as const, message: 'Socket not connected' };
                addLog(`Command blocked: ${error.message}`, 'error');
                resolve(error);
                return;
            }

            if (connectionStatus !== 'CONNECTED_TO_ROVER') {
                const error = { status: 'error' as const, message: `Cannot send command in ${connectionStatus} state` };
                addLog(`Command rejected: ${error.message}`, 'error');
                resolve(error);
                return;
            }

            const commandId = `cmd_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
            addLog(`Sending: ${cmdStr}`);

            // Optimistic UI overlay for local status bar updates
            try {
                if (target === 'ARM_DISARM') {
                    const arm = !!command?.arm;
                    setOptimistic({ status: arm ? 'armed' as const : 'disarmed' as const });
                } else if (target === 'SET_MODE') {
                    const mode = String(command?.mode || '').toUpperCase();
                    if (mode) setOptimistic({ mode });
                }
            } catch {}

            // CHANGE: compute a smarter timeout per command type, aligned with backend behavior
            const calcTimeoutMs = () => {
                if (target === 'UPLOAD_MISSION') {
                    const n = Array.isArray(command?.waypoints) ? command.waypoints.length : 0;
                    // Backend window ~ max(20s, 3s * N). Add buffer +10s.
                    const seconds = Math.max(20, 3 * n) + 10;
                    return seconds * 1000;
                }
                if (target === 'GET_MISSION') {
                    return 30000; // reading may take a while
                }
                // Default for simple commands relying on COMMAND_ACK
                return 45000; // extend to 45s for slower ACKs
            };

            const timeoutMs = calcTimeoutMs();

            // CHANGE: attach listener BEFORE emit to avoid race with fast responses
            const responseHandler = (response: CommandResponse & { command?: string }) => {
                // For ACK-based commands (ARM_DISARM, SET_MODE), ignore 'pending' and only resolve on matching final
                if ((response as any).status === 'pending') {
                    // Only log pending for matching command types
                    if (response.command && response.command === target) {
                        addLog(`Command ${response.command} pending: ${response.message}`, 'info');
                    }
                    return;
                }

                // If backend includes 'command', ensure it matches the request for ACK-based flows
                if (response.command && target && response.command !== target) {
                    return; // not for us
                }

                if (commandTimeoutRef.current[commandId]) {
                    clearTimeout(commandTimeoutRef.current[commandId]);
                    delete commandTimeoutRef.current[commandId];
                }
                socket?.off('command_response', responseHandler);
                // Clear optimistic overlay for this command on final response
                try {
                    if (target === 'ARM_DISARM') {
                        clearOptimistic(['status']);
                    } else if (target === 'SET_MODE') {
                        clearOptimistic(['mode']);
                    }
                } catch {}
                resolve(response);
            };

            // For commands that emit a single, final response (UPLOAD_MISSION, GET_MISSION, GOTO),
            // we can safely use a filtered handler that resolves on the next non-pending event.
            socket.on('command_response', responseHandler);

            // Now emit the command
            socket.emit('send_command', command);

            // Arm/disarm/mode may take a moment to ACK; mission upload needs a longer dynamic window.
            commandTimeoutRef.current[commandId] = setTimeout(() => {
                socket?.off('command_response', responseHandler);
                delete commandTimeoutRef.current[commandId];
                const timeoutError = { status: 'error' as const, message: `Command timeout (${Math.round(timeoutMs / 1000)}s)` };
                addLog(`Command timeout: ${cmdStr}`, 'error');
                try {
                    if (target === 'ARM_DISARM') clearOptimistic(['status']);
                    else if (target === 'SET_MODE') clearOptimistic(['mode']);
                } catch {}
                resolve(timeoutError);
            }, timeoutMs);
        });
    }, [connectionStatus, addLog, setOptimistic, clearOptimistic]);

    // Write mission to rover (direct event API for reliability)
    // --- Write mission to rover with servo config (boolean wrapper) ---
    const writeMissionToRoverHook = useCallback(async (waypoints: Waypoint[], servoConfig?: ServoConfig): Promise<boolean> => {
        const socket = socketRef.current;
        if (!socket?.connected || connectionStatus !== 'CONNECTED_TO_ROVER') {
            addLog('Not connected to rover', 'error');
            return false;
        }
        addLog(`Uploading mission: ${waypoints.length} waypoints${servoConfig ? ` (servo mode: ${servoConfig.mode})` : ''}`);
        setUploadProgress(0);
        return await writeMissionToRover(waypoints, servoConfig);
    }, [connectionStatus, addLog]);

    const requestRoverReconnect = useCallback(() => {
        const socket = socketRef.current;
        if (!socket?.connected) {
            addLog('Cannot request rover reconnect - backend offline', 'error');
            return;
        }

        addLog('Requesting rover reconnect from backend');
        socket.emit('request_rover_reconnect');
    }, [addLog]);

    // Read mission from rover
    const readMissionFromRover = useCallback(async (): Promise<CommandResponse> => {
        if (!socketRef.current?.connected || connectionStatus !== 'CONNECTED_TO_ROVER') {
            return { status: 'error', message: 'Not connected to rover' };
        }

        addLog('Downloading mission from rover...');
        
        try {
            const result = await sendCommand({
                command: 'GET_MISSION'
            });

            if (result.status === 'success') {
                const waypointCount = result.waypoints?.length || 0;
                addLog(`Mission download successful: ${waypointCount} waypoints received`, 'success');
            } else {
                addLog(`Mission download failed: ${result.message}`, 'error');
            }

            return result;
        } catch (error) {
            const errorMsg = `Mission download error: ${error}`;
            addLog(errorMsg, 'error');
            return { status: 'error', message: errorMsg };
        }
    }, [connectionStatus, sendCommand, addLog]);

    // Command response listener for App.tsx
    const addCommandResponseListener = useCallback((callback: (response: CommandResponse) => void) => {
        const socket = socketRef.current;
        if (socket) {
            addLog('Adding external command response listener');
            
            const responseHandler = (response: CommandResponse) => {
                if (response.status === 'pending') {
                    return;
                }
                callback(response);
            };
            
            socket.on('command_response', responseHandler);
            
            return () => {
                addLog('Removing external command response listener');
                socket.off('command_response', responseHandler);
            };
        }
        return () => {};
    }, [addLog]);

    // Cleanup on unmount
    useEffect(() => {
        addLog('Hook initialized - ready for connections');
        
        return () => {
            addLog('Hook unmounting - cleaning up all resources');
            cleanupSocket();
        };
    }, [addLog, cleanupSocket]);

    const clearLogs = useCallback(() => {
        setLogs([]);
        addLog('Logs cleared by user');
    }, [addLog]);
    const returnObject = {
        // Connection state
        connectionStatus,
        roverData,
        backendStatus,
        logs,
        uploadProgress,

        // Connection control
        connect,
        disconnect,
        requestRoverReconnect,

        // Command functions
        sendCommand,
        addCommandResponseListener,

        // Mission functions  
        writeMissionToRover: writeMissionToRoverHook,
        readMissionFromRover,

        // Utilities
        clearLogs,
        isConnected: connectionStatus === 'CONNECTED_TO_ROVER',
        isConnecting: connectionStatus === 'CONNECTING' || connectionStatus === 'WAITING_FOR_ROVER',
        isBackendOnline: backendStatus === 'ONLINE',

        // Debug info
        connectionAttempts: connectionAttemptRef.current,
        hasSocket: !!socketRef.current,
        socketConnected: socketRef.current?.connected || false
    };

    return returnObject;
};

export default useRoverConnection;
