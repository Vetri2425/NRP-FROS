import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';

// Types for the store
export interface RoverState {
  position: { lat: number; lng: number } | null;
  heading: number | null;
  isConnected: boolean;
  telemetry: {
    speed?: number;
    battery?: number;
    signalStrength?: number;
    altitude?: number;
    satellites?: number;
  };
}

export interface MapState {
  bounds: any;
  zoom: number;
  center: { lat: number; lng: number };
  isFullScreen: boolean;
  activeTool: string | null;
  viewMode: 'planning' | 'live';
}

export interface GeofenceZone {
  id: string;
  center: { lat: number; lng: number };
  radius: number;
  name: string;
  type: 'include' | 'exclude';
  alertLevel: 'info' | 'warning' | 'critical';
  isActive: boolean;
}

export interface GeofenceAlert {
  id: string;
  message: string;
  level: 'info' | 'warning' | 'critical';
  timestamp: number;
}

export interface AppState {
  // Rover state
  rover: RoverState;
  
  // Map state
  map: MapState;
  
  // Waypoints
  waypoints: any[];
  activeWaypointIndex: number | null;
  
  // Geofencing
  geofenceZones: GeofenceZone[];
  geofenceAlerts: GeofenceAlert[];
  showGeofencePanel: boolean;
  editingGeofence: string | null;
  
  // UI state
  showKeyboardHelp: boolean;
  trailHistory: Array<{ lat: number; lng: number; timestamp: number }>;
  
  // Performance metrics
  performanceMetrics: {
    renderTime: number;
    markerCount: number;
    lastUpdate: number;
  };
  
  // Actions
  updateRoverPosition: (position: { lat: number; lng: number }, heading?: number) => void;
  updateRoverTelemetry: (telemetry: Partial<RoverState['telemetry']>) => void;
  setRoverConnection: (connected: boolean) => void;
  
  updateMapState: (state: Partial<MapState>) => void;
  setActiveTool: (tool: string | null) => void;
  
  addWaypoint: (waypoint: any) => void;
  updateWaypoint: (id: number, updates: any) => void;
  deleteWaypoint: (id: number) => void;
  setActiveWaypoint: (index: number | null) => void;
  
  addGeofenceZone: (zone: GeofenceZone) => void;
  updateGeofenceZone: (id: string, updates: Partial<GeofenceZone>) => void;
  deleteGeofenceZone: (id: string) => void;
  addGeofenceAlert: (alert: GeofenceAlert) => void;
  clearOldAlerts: () => void;
  setShowGeofencePanel: (show: boolean) => void;
  setEditingGeofence: (id: string | null) => void;
  
  setShowKeyboardHelp: (show: boolean) => void;
  updateTrailHistory: (position: { lat: number; lng: number }) => void;
  
  updatePerformanceMetrics: (metrics: Partial<AppState['performanceMetrics']>) => void;
}

export const useAppStore = create<AppState>()(
  subscribeWithSelector((set, get) => ({
    // Initial state
    rover: {
      position: null,
      heading: null,
      isConnected: false,
      telemetry: {},
    },
    
    map: {
      bounds: null,
      zoom: 13,
      center: { lat: 13.0827, lng: 80.2707 },
      isFullScreen: false,
      activeTool: null,
      viewMode: 'planning',
    },
    
    waypoints: [],
    activeWaypointIndex: null,
    
    geofenceZones: [],
    geofenceAlerts: [],
    showGeofencePanel: false,
    editingGeofence: null,
    
    showKeyboardHelp: false,
    trailHistory: [],
    
    performanceMetrics: {
      renderTime: 0,
      markerCount: 0,
      lastUpdate: Date.now(),
    },
    
    // Rover actions
    updateRoverPosition: (position, heading) =>
      set((state) => ({
        rover: {
          ...state.rover,
          position,
          heading: heading ?? state.rover.heading,
        },
      })),
    
    updateRoverTelemetry: (telemetry) =>
      set((state) => ({
        rover: {
          ...state.rover,
          telemetry: { ...state.rover.telemetry, ...telemetry },
        },
      })),
    
    setRoverConnection: (connected) =>
      set((state) => ({
        rover: { ...state.rover, isConnected: connected },
      })),
    
    // Map actions
    updateMapState: (mapState) =>
      set((state) => ({
        map: { ...state.map, ...mapState },
      })),
    
    setActiveTool: (tool) =>
      set((state) => ({
        map: { ...state.map, activeTool: tool },
      })),
    
    // Waypoint actions
    addWaypoint: (waypoint) =>
      set((state) => ({
        waypoints: [...state.waypoints, waypoint],
      })),
    
    updateWaypoint: (id, updates) =>
      set((state) => ({
        waypoints: state.waypoints.map((wp) =>
          wp.id === id ? { ...wp, ...updates } : wp
        ),
      })),
    
    deleteWaypoint: (id) =>
      set((state) => ({
        waypoints: state.waypoints.filter((wp) => wp.id !== id),
      })),
    
    setActiveWaypoint: (index) =>
      set({ activeWaypointIndex: index }),
    
    // Geofence actions
    addGeofenceZone: (zone) =>
      set((state) => ({
        geofenceZones: [...state.geofenceZones, zone],
      })),
    
    updateGeofenceZone: (id, updates) =>
      set((state) => ({
        geofenceZones: state.geofenceZones.map((zone) =>
          zone.id === id ? { ...zone, ...updates } : zone
        ),
      })),
    
    deleteGeofenceZone: (id) =>
      set((state) => ({
        geofenceZones: state.geofenceZones.filter((zone) => zone.id !== id),
        editingGeofence: state.editingGeofence === id ? null : state.editingGeofence,
      })),
    
    addGeofenceAlert: (alert) =>
      set((state) => ({
        geofenceAlerts: [...state.geofenceAlerts.slice(-9), alert], // Keep last 10
      })),
    
    clearOldAlerts: () =>
      set((state) => {
        const now = Date.now();
        return {
          geofenceAlerts: state.geofenceAlerts.filter(
            (alert) => now - alert.timestamp < 120000 // 2 minutes
          ),
        };
      }),
    
    setShowGeofencePanel: (show) => set({ showGeofencePanel: show }),
    setEditingGeofence: (id) => set({ editingGeofence: id }),
    
    // UI actions
    setShowKeyboardHelp: (show) => set({ showKeyboardHelp: show }),
    
    updateTrailHistory: (position) =>
      set((state) => {
        const now = Date.now();
        const MAX_TRAIL_POINTS = 100;
        const MAX_AGE_MS = 60000; // 1 minute
        
        // Filter out old points
        const filtered = state.trailHistory.filter(
          (p) => now - p.timestamp < MAX_AGE_MS
        );
        
        // Add new point if different from last
        const lastPoint = filtered[filtered.length - 1];
        if (
          !lastPoint ||
          Math.abs(lastPoint.lat - position.lat) > 0.00001 ||
          Math.abs(lastPoint.lng - position.lng) > 0.00001
        ) {
          filtered.push({ ...position, timestamp: now });
        }
        
        return {
          trailHistory: filtered.slice(-MAX_TRAIL_POINTS),
        };
      }),
    
    // Performance actions
    updatePerformanceMetrics: (metrics) =>
      set((state) => ({
        performanceMetrics: { ...state.performanceMetrics, ...metrics },
      })),
  }))
);

// Selectors for optimized re-renders
export const useRoverState = () => useAppStore((state) => state.rover);
export const useMapState = () => useAppStore((state) => state.map);
export const useWaypoints = () => useAppStore((state) => state.waypoints);
export const useActiveWaypoint = () => useAppStore((state) => state.activeWaypointIndex);
export const useGeofenceZones = () => useAppStore((state) => state.geofenceZones);
export const useGeofenceAlerts = () => useAppStore((state) => state.geofenceAlerts);
export const usePerformanceMetrics = () => useAppStore((state) => state.performanceMetrics);

// Action selectors
export const useRoverActions = () => useAppStore((state) => ({
  updateRoverPosition: state.updateRoverPosition,
  updateRoverTelemetry: state.updateRoverTelemetry,
  setRoverConnection: state.setRoverConnection,
}));

export const useMapActions = () => useAppStore((state) => ({
  updateMapState: state.updateMapState,
  setActiveTool: state.setActiveTool,
}));

export const useWaypointActions = () => useAppStore((state) => ({
  addWaypoint: state.addWaypoint,
  updateWaypoint: state.updateWaypoint,
  deleteWaypoint: state.deleteWaypoint,
  setActiveWaypoint: state.setActiveWaypoint,
}));

export const useGeofenceActions = () => useAppStore((state) => ({
  addGeofenceZone: state.addGeofenceZone,
  updateGeofenceZone: state.updateGeofenceZone,
  deleteGeofenceZone: state.deleteGeofenceZone,
  addGeofenceAlert: state.addGeofenceAlert,
  clearOldAlerts: state.clearOldAlerts,
  setShowGeofencePanel: state.setShowGeofencePanel,
  setEditingGeofence: state.setEditingGeofence,
}));