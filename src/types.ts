export interface LogEntry {
  timestamp: string;
  lat: number | null;
  lng: number | null;
  event: string;
  waypointId?: number | null;
  status?: string | null;
  servoAction?: string | null;
}

export interface BackendLogEntry {
  timestamp: string;
  message: string;
  lat?: number | null;
  lng?: number | null;
  waypointId?: number | null;
  status?: string | null;
  servoAction?: string | null;
}

export interface MissionFileInfo {
  name: string;
  size: number;
  type: string;
  uploadedAt: string;
  waypointCount: number;
  source?: 'file' | 'generated' | 'drawn' | 'downloaded';
}

export interface MissionLog {
  id: string;
  name: string;
  status: 'Completed' | 'Incomplete' | 'In Progress';
  timestamp: Date;
  entries: LogEntry[];
}

export type Waypoint = {
  id: number;
  command: string;
  lat: number;
  lng: number;
  alt: number;
  frame: number;
  current?: number; // 0 or 1
  autocontinue?: number; // 0 or 1
  param1?: number;
  param2?: number;
  param3?: number;
  param4?: number;
  action?: string;
};

export type ViewMode = 'dashboard' | 'planning' | 'live' | 'logs' | 'map' | 'setup' | 'servo';

/**
 * A single, unified data structure for all rover data,
 * combining real telemetry with UI-specific values.
 * This is now the single source of truth, replacing LiveRoverData.
 */
export interface RoverData {
  // --- Core Telemetry from Backend ---
  position: { lat: number; lng: number } | null;
  heading: number;
  battery: number;
  status: 'armed' | 'disarmed';
  mode: string;
  rtk_status: string;
  signal_strength: string;
  current_waypoint_id: number | null;
  rc_connected: boolean; // RC connection status

  // --- UI / Live Report Values ---
  hrms: string | number;
  vrms: string | number;
  imu_status: string;
  activeWaypointIndex: number | null;
  completedWaypointIds: number[];
  distanceToNext: number;
  lastUpdate?: string;
  telemetryAgeMs?: number;
}
