import React, { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { CrosshairIcon } from './icons/CrosshairIcon';
import { ExpandIcon } from './icons/ExpandIcon';
import { FullScreenToggleIcon } from './icons/FullScreenToggleIcon';
import { RoverIcon } from './icons/RoverIcon';
import { NorthArrowIcon } from './icons/NorthArrowIcon';
import { Waypoint, ViewMode } from '../types';
import { RulerIcon } from './icons/RulerIcon';
import { LineIcon } from './icons/LineIcon';
import { RectangleIcon } from './icons/RectangleIcon';
import { CircleIcon } from './icons/CircleIcon';
import { PolygonIcon } from './icons/PolygonIcon';
import { HexagonIcon } from './icons/HexagonIcon';
import { GeofenceIcon } from './icons/GeofenceIcon';
import { DebugIcon } from './icons/DebugIcon';
import { generateCircleWaypoints, generateRegularPolygonWaypoints, calculateDistance, calculateBearing } from '../utils/geo';
import DrawingInstructions from './DrawingInstructions';
import { useFrameTicker } from '../hooks/useFrameTicker';
import { useRover } from '../context/RoverContext';
import useMapTelemetry from '../hooks/useMapTelemetry';
import type { MapLayer } from '../types/map';


declare var L: any;

type MapViewProps = {
  missionWaypoints: Waypoint[];
  onMapClick: (lat: number, lng: number) => void;
  roverPosition?: { lat: number; lng: number; timestamp?: number } | null;
  activeWaypointIndex?: number | null;
  heading?: number | null;
  viewMode: ViewMode;
  isFullScreen: boolean;
  onNewMissionDrawn: (points: { lat: number, lng: number }[]) => void;
  isConnectedToRover: boolean;
  onUpdateWaypointPosition: (waypointId: number, newPosition: { lat: number, lng: number }) => void;
  onDeleteWaypoint?: (waypointId: number) => void;
  onInsertWaypoint?: (afterId: number, position: { lat: number, lng: number }) => void;
  telemetry?: {
    speed?: number;
    battery?: number;
    signalStrength?: number;
    altitude?: number;
    satellites?: number;
  };
};

type Tool = 'measure' | 'profile' | 'line' | 'rectangle' | 'circle' | 'polygon' | 'hexagon' | 'geofence' | null;

const MAP_COMMANDS = new Set<string>([
  'WAYPOINT',
  'SPLINE_WAYPOINT',
  'TAKEOFF',
  'LAND',
  'RETURN_TO_LAUNCH',
  'LOITER_TIME',
  'LOITER_TURNS',
]);

// Improved rover icon with clear directional indicator
const ROVER_SVG = `
<svg viewBox="0 0 64 64" xmlns="http://www.w3.org/2000/svg" width="36" height="36" style="display:block">
  <!-- Main body -->
  <rect x="6" y="14" width="52" height="36" rx="6" ry="6" fill="#e11d1d" stroke="#8b0000" stroke-width="2" />
  <!-- Windshield -->
  <rect x="12" y="20" width="40" height="16" rx="2" ry="2" fill="#111" opacity="0.9" />
  <!-- Wheels -->
  <rect x="14" y="38" width="10" height="6" rx="1" fill="#222" />
  <rect x="40" y="38" width="10" height="6" rx="1" fill="#222" />
  <!-- Headlights -->
  <rect x="20" y="18" width="8" height="6" fill="#fff" opacity="0.3" />
  <rect x="36" y="18" width="8" height="6" fill="#fff" opacity="0.3" />
  <!-- Front indicator arrow -->
  <path d="M 32 6 L 38 14 L 26 14 Z" fill="#ffeb3b" stroke="#f57f17" stroke-width="1" />
  <!-- Center dot for precise positioning -->
  <circle cx="32" cy="32" r="2" fill="#fff" opacity="0.5" />
</svg>
`;

const getWaypointIcon = (waypoint: Waypoint, index: number, total: number, activeId: number | null | undefined): any => {
    const isActive = waypoint.id === activeId;
    const isStart = index === 0;
    const isEnd = index === total - 1 && total > 1;

    let fill = '#f97316'; // Default orange
    if (isStart) fill = '#16a34a';
    if (isEnd) fill = '#dc2626';
    if (isActive) fill = '#22c55e';

    const size = isActive ? 32 : 24;

    const svgIcon = `
      <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="${size}" height="${size}" fill="${fill}" class="drop-shadow-lg">
        <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"/>
        <text x="12" y="10.5" font-family="sans-serif" font-size="8" font-weight="bold" fill="white" text-anchor="middle" dy=".3em">${waypoint.id}</text>
      </svg>
    `;

    return L.divIcon({
        html: svgIcon,
        className: 'bg-transparent border-0',
        iconSize: [size, size],
        iconAnchor: [size / 2, size]
    });
};

const MapView: React.FC<MapViewProps> = ({
  missionWaypoints,
  onMapClick,
  roverPosition,
  activeWaypointIndex,
  heading,
  viewMode,
  isFullScreen,
  onNewMissionDrawn,
  isConnectedToRover,
  onUpdateWaypointPosition,
  onDeleteWaypoint,
  onInsertWaypoint,
  telemetry,
}) => {
  const pathWaypoints = useMemo(
    () =>
      missionWaypoints.filter(
        (wp) =>
          MAP_COMMANDS.has(wp.command) &&
          Number.isFinite(wp.lat) &&
          Number.isFinite(wp.lng)
      ),
    [missionWaypoints]
  );

  const mapWrapperRef = useRef<HTMLDivElement>(null);
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<any>(null);
  const userLocationMarkerRef = useRef<any>(null);
  const missionLayerRef = useRef<any>(null);
  const missionMarkersRef = useRef<any[]>([]);
  const roverMarkerRef = useRef<any>(null);
  const vehicleLayerGroupRef = useRef<any>(null);
  const trailLayerGroupRef = useRef<any>(null);
  // 3D rover resources removed; using lightweight SVG icon instead
  const traveledPathLayerRef = useRef<any>(null);
  const headingLineRef = useRef<any>(null);
  const drawingLayerRef = useRef<any>(null);
  const geofenceLayerRef = useRef<any>(null);
  const isDrawingRef = useRef(false);

  // Keep previous and last rover samples for interpolation
  const prevSampleRef = useRef<{ t: number, lat: number, lng: number, heading: number } | null>(null);
  const lastSampleRef = useRef<{ t: number, lat: number, lng: number, heading: number } | null>(null);
  const lastIconUpdateRef = useRef<number>(0);
  const lastPositionUpdateRef = useRef<number>(0);
  useFrameTicker((timestamp) => {
    // RAF ticker runs at 60 FPS for smooth animations
    // No need to log every frame - use only for performance-critical updates
  });
  
  // Throttle settings for optimized real-time accuracy
  const POSITION_UPDATE_THROTTLE_MS = 50; // Throttle position updates to 50ms
  const ICON_UPDATE_THROTTLE_MS = 100; // Throttle icon rotation updates to 100ms
  // Track whether we've already auto-fitted to the current mission to avoid repeated zoom jitter
  const hasFittedMissionRef = useRef(false);
  // Stable signature for mission geometry to reset auto-fit when a new mission loads/changes
  const missionKey = useMemo(
    () => pathWaypoints.map((wp) => `${wp.lat.toFixed(6)},${wp.lng.toFixed(6)}`).join('|'),
    [pathWaypoints]
  );

  const [isMapFullScreen, setIsMapFullScreen] = useState(false);
  const [activeTool, setActiveTool] = useState<Tool>(null);
  const [drawnPoints, setDrawnPoints] = useState<any[]>([]);
  const [mousePos, setMousePos] = useState<any>(null);
  const [measurementText, setMeasurementText] = useState<string | null>(null);
  const [geofenceZones, setGeofenceZones] = useState<Array<{ 
    id: string;
    center: {lat: number, lng: number}; 
    radius: number; 
    name: string;
    type: 'include' | 'exclude';
    alertLevel: 'info' | 'warning' | 'critical';
    isActive: boolean;
  }>>([]);
  const [selectedWaypointId, setSelectedWaypointId] = useState<number | null>(null);
  const [contextMenu, setContextMenu] = useState<{ x: number, y: number, waypointId: number } | null>(null);
  const [showKeyboardHelp, setShowKeyboardHelp] = useState(false);
  // Trail is now managed internally via useMapTelemetry to avoid React re-renders
  const [geofenceAlerts, setGeofenceAlerts] = useState<Array<{ id: string, message: string, level: 'info' | 'warning' | 'critical', timestamp: number }>>([]);
  const [showGeofencePanel, setShowGeofencePanel] = useState(false);
  const [editingGeofence, setEditingGeofence] = useState<string | null>(null);
  
  // Debug and testing state
  const [showDebugPanel, setShowDebugPanel] = useState(false);
  const [simulationMode, setSimulationMode] = useState(false);
  const [debugMetrics, setDebugMetrics] = useState({
    renderTime: 0,
    markerCount: 0,
    memoryUsage: 0,
    frameRate: 0,
    lastUpdate: Date.now()
  });
  
  // Lazy loading state
  const [visibleMarkers, setVisibleMarkers] = useState<Set<number>>(new Set());
  const [mapBounds, setMapBounds] = useState<any>(null);
  const [currentZoom, setCurrentZoom] = useState(13);
  const lastBoundsUpdateRef = useRef<number>(0);

  // Layer visibility management
  const [layers, setLayers] = useState<MapLayer[]>([
    { name: 'Vehicle', type: 'vehicle', visible: true },
    { name: 'Trail', type: 'trail', visible: true },
    { name: 'Geofence', type: 'geofence', visible: true },
    { name: 'Waypoints', type: 'waypoints', visible: true },
  ]);

  // Vehicle and trail updates handled via useMapTelemetry (non-react updates)
  useMapTelemetry({
    mapRef: mapRef as any,
    vehicleLayerRef: vehicleLayerGroupRef as any,
    trailLayerRef: trailLayerGroupRef as any,
    throttleMs: 250,
    maxTrailPoints: 500,
  });

  const invalidateAndFitBounds = useCallback(() => {
    if (!mapRef.current) return;
    setTimeout(() => {
      if (!mapRef.current) return;
      mapRef.current.invalidateSize();
      if (pathWaypoints.length > 0) {
        const missionBounds = L.latLngBounds(pathWaypoints.map((wp) => [wp.lat, wp.lng]));
        if (missionBounds.isValid()) {
          mapRef.current.fitBounds(missionBounds.pad(0.2));
        }
      }
    }, 150);
  }, [pathWaypoints]);

  // Lazy loading for waypoints - only render visible markers
  const visibleWaypoints = useMemo(() => {
    if (!mapBounds || pathWaypoints.length === 0) return pathWaypoints;
    
    // For high zoom levels, show all waypoints
    if (currentZoom >= 15) return pathWaypoints;
    
    // For lower zoom levels, filter by viewport with buffer
    const bufferedBounds = mapBounds.pad(0.5); // 50% buffer around viewport
    
    return pathWaypoints.filter(wp => 
      bufferedBounds.contains([wp.lat, wp.lng])
    );
  }, [pathWaypoints, mapBounds, currentZoom]);

  useEffect(() => {
    if (mapContainerRef.current && !mapRef.current) {
      // Increase max zoom and initial zoom by ~20%
      const map = L.map(mapContainerRef.current, { zoomControl: true, maxZoom: 26 }).setView([13.0827, 80.2707], 16);
      mapRef.current = map;
      
      // Optimized tile layer with lazy loading
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
          attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
          loading: 'lazy',
          crossOrigin: true,
          updateWhenZooming: false, // Reduce tile requests during zoom
          updateWhenIdle: true, // Only update tiles when map is idle
          keepBuffer: 2, // Keep 2 tile buffers around viewport
          // Allow over-zooming beyond native tile zoom for closer view
          maxNativeZoom: 19,
          maxZoom: 26
      }).addTo(map);
      
      // Initialize vehicle and trail layer groups for rover marker and path
      if (!vehicleLayerGroupRef.current) {
        vehicleLayerGroupRef.current = L.layerGroup().addTo(map);
        console.log('🚗 [MapView] Vehicle layer group created and added to map');
      }
      
      if (!trailLayerGroupRef.current) {
        trailLayerGroupRef.current = L.layerGroup().addTo(map);
        console.log('🛤️ [MapView] Trail layer group created and added to map');
      }
      
      // Add map event listeners for lazy loading
      map.on('moveend zoomend', () => {
        const now = performance.now();
        // Throttle bounds updates to avoid excessive recalculations
        if (now - lastBoundsUpdateRef.current > 250) {
          setMapBounds(map.getBounds());
          setCurrentZoom(map.getZoom());
          lastBoundsUpdateRef.current = now;
        }
      });
      
      // Initial bounds
      setMapBounds(map.getBounds());
      setCurrentZoom(map.getZoom());
    }
  }, []);

  // Keyboard navigation support
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!mapRef.current) return;
      
      const map = mapRef.current;
      const currentZoom = map.getZoom();
      const currentCenter = map.getCenter();
      const panStep = 0.001; // Degrees to pan per key press
      
      switch(e.key) {
        case '+':
        case '=':
          map.setZoom(currentZoom + 1);
          e.preventDefault();
          break;
        case '-':
        case '_':
          map.setZoom(currentZoom - 1);
          e.preventDefault();
          break;
        case 'ArrowUp':
          if (e.ctrlKey) {
            map.panTo([currentCenter.lat + panStep, currentCenter.lng]);
            e.preventDefault();
          }
          break;
        case 'ArrowDown':
          if (e.ctrlKey) {
            map.panTo([currentCenter.lat - panStep, currentCenter.lng]);
            e.preventDefault();
          }
          break;
        case 'ArrowLeft':
          if (e.ctrlKey) {
            map.panTo([currentCenter.lat, currentCenter.lng - panStep]);
            e.preventDefault();
          }
          break;
        case 'ArrowRight':
          if (e.ctrlKey) {
            map.panTo([currentCenter.lat, currentCenter.lng + panStep]);
            e.preventDefault();
          }
          break;
        case 'f':
          if (e.ctrlKey) {
            handleToggleMapFullScreen();
            e.preventDefault();
          }
          break;
        case 'r':
          if (e.ctrlKey && roverPosition) {
            map.panTo([roverPosition.lat, roverPosition.lng]);
            e.preventDefault();
          }
          break;
        case 'm':
          if (e.ctrlKey && pathWaypoints.length > 0) {
            invalidateAndFitBounds();
            e.preventDefault();
          }
          break;
        case 'g':
          if (e.ctrlKey) {
            setShowGeofencePanel(prev => !prev);
            e.preventDefault();
          }
          break;
        case 'd':
          if (e.ctrlKey) {
            setShowDebugPanel(prev => !prev);
            e.preventDefault();
          }
          break;
        case 's':
          if (e.ctrlKey && e.shiftKey) {
            setSimulationMode(prev => !prev);
            e.preventDefault();
          }
          break;
        case 'Escape':
          if (showKeyboardHelp) {
            setShowKeyboardHelp(false);
            e.preventDefault();
          }
          break;
        case '?':
        case '/':
          if (e.shiftKey) {
            setShowKeyboardHelp(prev => !prev);
            e.preventDefault();
          }
          break;
      }
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [roverPosition, pathWaypoints, invalidateAndFitBounds, showKeyboardHelp]);

  // Debug performance monitoring
  useEffect(() => {
    if (!showDebugPanel) return;
    
    const interval = setInterval(() => {
      // Memory usage (approximate)
      const memoryInfo = (performance as any).memory;
      const memoryUsage = memoryInfo ? memoryInfo.usedJSHeapSize / 1024 / 1024 : 0;
      
      setDebugMetrics(prev => ({
        ...prev,
        memoryUsage,
        markerCount: missionMarkersRef.current.length,
        lastUpdate: Date.now()
      }));
    }, 1000);
    
    return () => clearInterval(interval);
  }, [showDebugPanel]);

  // Simulation mode for testing
  useEffect(() => {
    if (!simulationMode) return;
    
    let simulationInterval: NodeJS.Timeout;
    let angle = 0;
    const centerLat = 13.0827;
    const centerLng = 80.2707;
    const radius = 0.001; // Small radius for simulation
    
    simulationInterval = setInterval(() => {
      angle += 0.1;
      const simulatedPos = {
        lat: centerLat + Math.cos(angle) * radius,
        lng: centerLng + Math.sin(angle) * radius
      };
      const simulatedHeading = (angle * 180 / Math.PI) % 360;
      
      // Simulate rover movement
      // Note: In real usage, this would be replaced by actual rover data updates
      // You would call your rover position update function here
      
      console.log('Simulation:', { pos: simulatedPos, heading: simulatedHeading });
    }, 100);
    
    return () => {
      if (simulationInterval) clearInterval(simulationInterval);
    };
  }, [simulationMode]);

  useEffect(() => {
    const onFullScreenChange = () => setIsMapFullScreen(document.fullscreenElement === mapWrapperRef.current);
    document.addEventListener('fullscreenchange', onFullScreenChange);
    return () => document.removeEventListener('fullscreenchange', onFullScreenChange);
  }, []);

  useEffect(() => {
    invalidateAndFitBounds();
  }, [viewMode, isFullScreen, isMapFullScreen, invalidateAndFitBounds]);

  // Reset auto-fit state whenever the mission geometry changes
  useEffect(() => {
    hasFittedMissionRef.current = false;
  }, [missionKey]);

  // Manage map event listeners based on active tool
  useEffect(() => {
    if (!mapRef.current) return;
    const map = mapRef.current;
    
    map.off('click').off('mousedown').off('mousemove').off('mouseup').off('dblclick');
    map.getContainer().style.cursor = 'grab';

    const handleSimpleClick = (e: any) => onMapClick(e.latlng.lat, e.latlng.lng);

    const handleMouseDown = (e: any) => {
        isDrawingRef.current = true;
        setDrawnPoints([e.latlng]);
        setMousePos(e.latlng);
    };

    const handleMouseMove = (e: any) => {
        setMousePos(e.latlng);
        if (isDrawingRef.current) {
          if (activeTool === 'rectangle' || activeTool === 'circle' || activeTool === 'hexagon') {
             setDrawnPoints(prev => [prev[0], e.latlng]);
          }
        }
    };
    
    const handleMouseUp = (e: any) => {
        isDrawingRef.current = false;
        if (drawnPoints.length === 0) return;

        const endPoint = e.latlng;
        const startPoint = drawnPoints[0];
        const radius = calculateDistance(startPoint, endPoint);
        let waypoints: {lat: number, lng: number}[] = [];
        
        if (activeTool === 'rectangle') {
            const bounds = L.latLngBounds(startPoint, endPoint);
            waypoints = [
                bounds.getSouthWest(),
                bounds.getNorthWest(),
                bounds.getNorthEast(),
                bounds.getSouthEast(),
            ];
        } else if (activeTool === 'circle') {
            waypoints = generateCircleWaypoints(startPoint, radius);
        } else if (activeTool === 'hexagon') {
            const bearing = calculateBearing(startPoint, endPoint);
            waypoints = generateRegularPolygonWaypoints(startPoint, radius, 6, bearing);
        } else if (activeTool === 'geofence') {
            // Create new geofence zone
            const newGeofence = {
                id: `geofence_${Date.now()}`,
                center: startPoint,
                radius: Math.max(radius, 10), // Minimum 10m radius
                name: `Geofence ${geofenceZones.length + 1}`,
                type: 'exclude' as const,
                alertLevel: 'warning' as const,
                isActive: true
            };
            setGeofenceZones(prev => [...prev, newGeofence]);
            setShowGeofencePanel(true);
        }

        if (waypoints.length > 0) {
            onNewMissionDrawn(waypoints);
        }
        handleCancelTool();
    };

    const handlePointClick = (e: any) => {
      setDrawnPoints(prev => [...prev, e.latlng]);
    };
    
    const handleFinishWithDoubleClick = () => {
       if (activeTool === 'measure') {
         setActiveTool(null);
       } else {
         handleFinishDrawing();
       }
    };


    switch (activeTool) {
        case 'line':
        case 'polygon':
        case 'measure':
            map.on('click', handlePointClick);
            map.on('dblclick', handleFinishWithDoubleClick);
            map.getContainer().style.cursor = 'crosshair';
            break;
        case 'rectangle':
        case 'circle':
        case 'hexagon':
        case 'geofence':
            map.on('mousedown', handleMouseDown);
            map.on('mousemove', handleMouseMove);
            map.on('mouseup', handleMouseUp);
            map.getContainer().style.cursor = 'crosshair';
            break;
        default:
            map.on('click', handleSimpleClick);
            break;
    }
    
    return () => {
        map.off('click').off('mousedown').off('mousemove').off('mouseup').off('dblclick');
        map.getContainer().style.cursor = 'grab';
    };
}, [activeTool, onMapClick]);

  // Effect to manage drawing/measuring visuals
  useEffect(() => {
    if (!mapRef.current) return;
    if (!drawingLayerRef.current) drawingLayerRef.current = L.layerGroup().addTo(mapRef.current);
    drawingLayerRef.current.clearLayers();

    if (drawnPoints.length > 0) {
        if (['line', 'polygon', 'rectangle', 'circle', 'hexagon', 'geofence'].includes(activeTool || '')) {
            const drawOptions = { color: '#22c55e', weight: 3, dashArray: '5, 5' };
            if (activeTool === 'rectangle' && drawnPoints.length === 2) {
                L.rectangle(L.latLngBounds(drawnPoints[0], drawnPoints[1]), drawOptions).addTo(drawingLayerRef.current);
            } else if ((activeTool === 'circle' || activeTool === 'hexagon' || activeTool === 'geofence') && drawnPoints.length > 0 && mousePos) {
                const radius = calculateDistance(drawnPoints[0], mousePos);
                const color = activeTool === 'geofence' ? '#ff6b6b' : '#22c55e';
                L.circle(drawnPoints[0], { ...drawOptions, color, radius }).addTo(drawingLayerRef.current);
            } else {
                 drawnPoints.forEach(p => L.circleMarker(p, { radius: 4, color: '#22c55e', fillOpacity: 1 }).addTo(drawingLayerRef.current));
                 if (mousePos && drawnPoints.length > 0) {
                    const previewPoints = [...drawnPoints, mousePos];
                     L.polyline(previewPoints, drawOptions).addTo(drawingLayerRef.current);
                     if (activeTool === 'polygon') {
                        // Also draw a closing line for polygons
                        L.polyline([drawnPoints[drawnPoints.length - 1], drawnPoints[0]], drawOptions).addTo(drawingLayerRef.current);
                     }
                 }
            }
        } else if (activeTool === 'measure') {
            let totalDistance = 0;
            drawnPoints.forEach((p, i) => {
                L.circleMarker(p, { radius: 4, color: '#3b82f6', fillOpacity: 1 }).addTo(drawingLayerRef.current);
                if (i > 0) {
                    const dist = mapRef.current.distance(p, drawnPoints[i - 1]);
                    totalDistance += dist;
                    L.tooltip({ permanent: true, direction: 'center', className: 'measurement-tooltip' })
                      .setLatLng(L.latLngBounds(p, drawnPoints[i - 1]).getCenter())
                      .setContent(`${(dist / 1000).toFixed(2)} km`)
                      .addTo(drawingLayerRef.current);
                }
            });
            if (drawnPoints.length > 1) {
                L.polyline(drawnPoints, { color: '#3b82f6' }).addTo(drawingLayerRef.current);
                setMeasurementText(`Total: ${(totalDistance / 1000).toFixed(2)} km`);
            } else if (drawnPoints.length === 1 && mousePos) {
                const dist = mapRef.current.distance(drawnPoints[0], mousePos);
                setMeasurementText(`Dist: ${(dist/1000).toFixed(2)} km`);
                L.polyline([drawnPoints[0], mousePos], { color: '#3b82f6', dashArray: '5, 5' }).addTo(drawingLayerRef.current);
            } else {
                setMeasurementText('Click to add points');
            }
        }
    }
  }, [drawnPoints, activeTool, mousePos]);

  // Enhanced mission waypoints rendering with lazy loading
  useEffect(() => {
    if (!mapRef.current) return;
    
    const startTime = performance.now();
    
    if (!missionLayerRef.current) missionLayerRef.current = L.layerGroup().addTo(mapRef.current);
    missionLayerRef.current.clearLayers();
    missionMarkersRef.current = [];
    if (traveledPathLayerRef.current) traveledPathLayerRef.current.remove();
    traveledPathLayerRef.current = null;
    if (headingLineRef.current) headingLineRef.current.remove();
    headingLineRef.current = null;

  if (visibleWaypoints.length > 0 && layers.find(l => l.type === 'waypoints')?.visible) {
      // Use visible waypoints for performance
      const latLngs = visibleWaypoints.map((wp) => [wp.lat, wp.lng]);
      const isDraggable = viewMode === 'planning';
      
      // Batch marker creation for better performance
      const markers = [];
      
      visibleWaypoints.forEach((wp, index) => {
        const icon = getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex);
        const marker = L.marker([wp.lat, wp.lng], { 
          icon,
          draggable: isDraggable,
        })
          .bindTooltip(`<b>Waypoint ${wp.id}</b><br>${wp.command}<br>Lat: ${wp.lat.toFixed(6)}<br>Lng: ${wp.lng.toFixed(6)}<br>Alt: ${wp.alt}m`)
          .addTo(missionLayerRef.current);

        if (isDraggable) {
          marker.on('dragend', (event: any) => {
            const newPos = event.target.getLatLng();
            onUpdateWaypointPosition(wp.id, { lat: newPos.lat, lng: newPos.lng });
          });
          
          marker.on('contextmenu', (event: any) => {
            event.originalEvent.preventDefault();
            const containerRect = mapContainerRef.current?.getBoundingClientRect();
            if (containerRect) {
              setContextMenu({
                x: event.originalEvent.clientX - containerRect.left,
                y: event.originalEvent.clientY - containerRect.top,
                waypointId: wp.id
              });
            }
            setSelectedWaypointId(wp.id);
          });
          
          marker.on('click', () => {
            setSelectedWaypointId(wp.id);
          });
        }
        
        markers.push(marker);
      });
      
      missionMarkersRef.current = markers;
      
      if (layers.find(l => l.type === 'waypoints')?.visible) {
        // Only draw path if we have reasonable number of waypoints
        if (visibleWaypoints.length <= 100) {
          L.polyline(latLngs, { color: 'orange', weight: 2 }).addTo(missionLayerRef.current);
        } else {
          // For large datasets, draw simplified path
          const simplified = visibleWaypoints.filter((_, i) => i % Math.ceil(visibleWaypoints.length / 50) === 0);
          const simplifiedLatLngs = simplified.map(wp => [wp.lat, wp.lng]);
          L.polyline(simplifiedLatLngs, { color: 'orange', weight: 2, opacity: 0.7 }).addTo(missionLayerRef.current);
        }
      }
      
      // Auto-fit only once for a given mission
      if (!hasFittedMissionRef.current) {
        invalidateAndFitBounds();
        hasFittedMissionRef.current = true;
      }
    }
    
    // Performance tracking
    const renderTime = performance.now() - startTime;
    if (renderTime > 50) { // Log slow renders
      console.warn(`Slow waypoint render: ${renderTime.toFixed(2)}ms for ${visibleWaypoints.length} waypoints`);
    }
    
  }, [visibleWaypoints, viewMode, activeWaypointIndex, invalidateAndFitBounds, onUpdateWaypointPosition, layers]); // ✅ FIXED: Added layers to deps

  useEffect(() => {
    if (!mapRef.current || missionMarkersRef.current.length === 0) return;
    pathWaypoints.forEach((wp, index) => {
      if (missionMarkersRef.current[index]) {
        missionMarkersRef.current[index].setIcon(
          getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex)
        );
      }
    });
  }, [activeWaypointIndex, pathWaypoints]);

  // Rover marker updates are handled inside useMapTelemetry
  
  // Trail is drawn by useMapTelemetry into its dedicated layer group

  // Enhanced Geofencing visualization and monitoring
  useEffect(() => {
    if (!mapRef.current) return;
    
    // Initialize geofence layer
    if (!geofenceLayerRef.current) {
      geofenceLayerRef.current = L.layerGroup().addTo(mapRef.current);
    }
    
    // Clear and redraw geofence zones
    geofenceLayerRef.current.clearLayers();
    
    geofenceZones.forEach((zone) => {
      if (!zone.isActive) return;
      
      const color = zone.type === 'include' ? '#22c55e' : '#ff6b6b';
      const alertColors = {
        info: '#3b82f6',
        warning: '#f59e0b', 
        critical: '#ef4444'
      };
      
      const circle = L.circle([zone.center.lat, zone.center.lng], {
        radius: zone.radius,
        color: alertColors[zone.alertLevel],
        fillColor: color,
        fillOpacity: 0.1,
        weight: 3,
        dashArray: zone.type === 'exclude' ? '10, 5' : '5, 5'
      });
      
      circle.bindTooltip(`
        <div class="text-sm">
          <b>${zone.name}</b><br>
          Type: ${zone.type}<br>
          Radius: ${zone.radius.toFixed(0)}m<br>
          Alert: ${zone.alertLevel}
        </div>
      `, {
        permanent: false,
        direction: 'top'
      });
      
      // Add click handler for editing
      circle.on('click', () => {
        setEditingGeofence(zone.id);
        setShowGeofencePanel(true);
      });
      
      circle.addTo(geofenceLayerRef.current);
      
      // Add center marker for editing
      const centerMarker = L.circleMarker([zone.center.lat, zone.center.lng], {
        radius: 6,
        color: alertColors[zone.alertLevel],
        fillColor: '#fff',
        fillOpacity: 1,
        weight: 2
      });
      
      centerMarker.bindTooltip(zone.name, { direction: 'bottom', offset: [0, 10] });
      centerMarker.addTo(geofenceLayerRef.current);
    });
    
    // Enhanced rover geofence monitoring
    if (roverPosition) {
      const now = Date.now();
      
      geofenceZones.forEach((zone) => {
        if (!zone.isActive) return;
        
        const distance = calculateDistance(roverPosition, zone.center);
        const isInside = distance <= zone.radius;
        const shouldAlert = (zone.type === 'exclude' && isInside) || (zone.type === 'include' && !isInside);
        
        if (shouldAlert) {
          const alertId = `${zone.id}_${Math.floor(now / 5000)}`; // Group alerts by 5-second intervals
          const existingAlert = geofenceAlerts.find(alert => alert.id === alertId);
          
          if (!existingAlert) {
            const message = zone.type === 'exclude' 
              ? `Rover entered restricted zone: ${zone.name}` 
              : `Rover left safe zone: ${zone.name}`;
              
            setGeofenceAlerts(prev => [...prev.slice(-9), { // Keep last 10 alerts
              id: alertId,
              message,
              level: zone.alertLevel,
              timestamp: now
            }]);
            
            // Console warning for debugging
            console.warn(`Geofence Alert [${zone.alertLevel.toUpperCase()}]: ${message}`);
          }
        }
      });
      
      // Clean old alerts (older than 2 minutes)
      setGeofenceAlerts(prev => prev.filter(alert => now - alert.timestamp < 120000));
    }
  }, [geofenceZones, roverPosition]); // ✅ FIXED: Removed geofenceAlerts from deps

  // Keep rover in view without constantly re-fitting bounds (prevents zoom/pan oscillation)
  useEffect(() => {
    if (!mapRef.current || !roverPosition) return;
    const map = mapRef.current;
    const currentBounds = map.getBounds();
    const roverLatLng = L.latLng(roverPosition.lat, roverPosition.lng);
    // If rover drifts out of view (with a small margin), gently pan back
    if (!currentBounds.pad(-0.2).contains(roverLatLng)) {
      if (pathWaypoints.length > 0) {
        const missionBounds = L.latLngBounds(pathWaypoints.map((wp) => [wp.lat, wp.lng]));
        if (missionBounds.isValid()) {
          const combined = missionBounds.extend(roverLatLng);
          map.fitBounds(combined.pad(0.2), { animate: true, duration: 0.5, maxZoom: 26 });
        } else {
          map.panTo(roverLatLng, { animate: true, duration: 0.5 });
        }
      } else {
        map.panTo(roverLatLng, { animate: true, duration: 0.5 });
      }
    }
  }, [roverPosition, pathWaypoints]);
  
  // Remove heading line (blue) between rover and next waypoint per request
  useEffect(() => {
      if (!mapRef.current) return;
      if (headingLineRef.current) {
          headingLineRef.current.remove();
          headingLineRef.current = null;
      }
  }, [roverPosition, activeWaypointIndex, pathWaypoints]);

  const handleCenterOnUser = () => navigator.geolocation.getCurrentPosition(pos => {
    if (!mapRef.current) return;
    const latLng = { lat: pos.coords.latitude, lng: pos.coords.longitude };
    mapRef.current.panTo(latLng);
    if (userLocationMarkerRef.current) userLocationMarkerRef.current.setLatLng(latLng);
    else userLocationMarkerRef.current = L.marker(latLng).addTo(mapRef.current);
  }, () => alert('Could not get location.'));

  const handleToggleMapFullScreen = () => {
    if (!document.fullscreenElement) mapWrapperRef.current?.requestFullscreen();
    else document.exitFullscreen();
  };
  
  const handleToolToggle = (tool: Tool) => {
    setActiveTool(prev => {
        const newTool = prev === tool ? null : tool;
        setDrawnPoints([]);
        setMeasurementText(null);
        setMousePos(null);
        isDrawingRef.current = false;
        return newTool;
    });
  };

  const handleFinishDrawing = () => {
    if (drawnPoints.length > 1) {
      onNewMissionDrawn(drawnPoints);
    }
    handleCancelTool();
  };

  const handleCancelTool = () => {
    setActiveTool(null);
    setDrawnPoints([]);
    setMeasurementText(null);
    setMousePos(null);
    isDrawingRef.current = false;
  };

  const handleDeleteWaypoint = () => {
    if (contextMenu && onDeleteWaypoint) {
      onDeleteWaypoint(contextMenu.waypointId);
      setContextMenu(null);
      setSelectedWaypointId(null);
    }
  };

  const handleInsertWaypoint = () => {
    if (contextMenu && onInsertWaypoint && mapRef.current) {
      const map = mapRef.current;
      const center = map.getCenter();
      onInsertWaypoint(contextMenu.waypointId, { lat: center.lat, lng: center.lng });
      setContextMenu(null);
    }
  };

  // Close context menu on map click
  useEffect(() => {
    const handleClick = () => setContextMenu(null);
    window.addEventListener('click', handleClick);
    return () => window.removeEventListener('click', handleClick);
  }, []);

  const drawingTools = [
    { name: 'line', icon: LineIcon, title: 'Draw Path' },
    { name: 'rectangle', icon: RectangleIcon, title: 'Draw Rectangle' },
    { name: 'circle', icon: CircleIcon, title: 'Draw Circle' },
    { name: 'polygon', icon: PolygonIcon, title: 'Draw Polygon' },
    { name: 'hexagon', icon: HexagonIcon, title: 'Draw Hexagon' },
    { name: 'geofence', icon: GeofenceIcon, title: 'Create Geofence' },
  ];
  
  const isToolActive = activeTool && ['line', 'rectangle', 'circle', 'polygon', 'hexagon', 'measure', 'geofence'].includes(activeTool);

  return (
    <div ref={mapWrapperRef} className="relative w-full h-full rounded-lg overflow-hidden bg-gray-700 flex flex-col">
       <DrawingInstructions activeTool={activeTool} />
       {/* Keyboard Shortcuts Help Overlay */}
       {showKeyboardHelp && (
         <div
           className="absolute inset-0 z-[3000] bg-black/60 backdrop-blur-sm flex items-center justify-center p-4"
           role="dialog"
           aria-modal="true"
           aria-labelledby="kb-help-title"
           onClick={() => setShowKeyboardHelp(false)}
         >
           <div
             className="w-full max-w-2xl bg-gray-900 text-gray-100 rounded-xl border border-gray-700 shadow-2xl overflow-hidden"
             onClick={(e) => e.stopPropagation()}
           >
             <div className="flex items-center justify-between px-4 py-3 bg-gradient-to-r from-indigo-600 to-blue-700">
               <h2 id="kb-help-title" className="text-lg font-semibold">Keyboard Shortcuts</h2>
               <button
                 className="px-2 py-1 text-sm rounded-md bg-black/20 hover:bg-black/30 border border-white/20"
                 onClick={() => setShowKeyboardHelp(false)}
                 aria-label="Close"
               >
                 Esc
               </button>
             </div>
             <div className="p-4 grid grid-cols-1 md:grid-cols-2 gap-4">
               <div>
                 <h3 className="text-sm font-semibold text-gray-300 mb-2">Map Navigation</h3>
                 <ul className="space-y-1 text-sm">
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Zoom in</span><span className="font-mono">+</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Zoom out</span><span className="font-mono">-</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Pan up</span><span className="font-mono">Ctrl + ↑</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Pan down</span><span className="font-mono">Ctrl + ↓</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Pan left</span><span className="font-mono">Ctrl + ←</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Pan right</span><span className="font-mono">Ctrl + →</span></li>
                 </ul>
               </div>
               <div>
                 <h3 className="text-sm font-semibold text-gray-300 mb-2">Rover & View</h3>
                 <ul className="space-y-1 text-sm">
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Toggle fullscreen</span><span className="font-mono">Ctrl + F</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Center on rover</span><span className="font-mono">Ctrl + R</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Fit to mission</span><span className="font-mono">Ctrl + M</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Geofence panel</span><span className="font-mono">Ctrl + G</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Debug panel</span><span className="font-mono">Ctrl + D</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Simulation mode</span><span className="font-mono">Ctrl + Shift + S</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Open/Close this help</span><span className="font-mono">Shift + ?</span></li>
                   <li className="flex items-center justify-between gap-4"><span className="text-gray-400">Close help</span><span className="font-mono">Esc</span></li>
                 </ul>
               </div>
               <div className="md:col-span-2 mt-2">
                 <h3 className="text-sm font-semibold text-gray-300 mb-2">Drawing Tips</h3>
                 <ul className="space-y-1 text-sm text-gray-300 list-disc list-inside">
                   <li>Click to add points. Double‑click to finish line/polygon.</li>
                   <li>For rectangle/circle/hexagon, click-drag-release to define shape.</li>
                   <li>Use the bottom toolbar to switch tools or measure distance.</li>
                 </ul>
               </div>
             </div>
             <div className="px-4 py-3 border-t border-gray-800 flex items-center justify-between text-xs text-gray-400">
               <span>Pan step: 0.001° per key press</span>
               <span>Hint: You can also zoom with the mouse wheel</span>
             </div>
           </div>
         </div>
       )}
       
       {/* Geofence Management Panel */}
       {showGeofencePanel && (
         <div className="absolute top-2 left-2 z-[2500] w-80 bg-gray-900 bg-opacity-95 text-white rounded-lg border border-gray-700 shadow-xl overflow-hidden">
           <div className="flex items-center justify-between px-4 py-3 bg-gradient-to-r from-purple-600 to-pink-600">
             <h3 className="text-sm font-semibold">Geofence Management</h3>
             <button
               onClick={() => {setShowGeofencePanel(false); setEditingGeofence(null);}}
               className="text-white hover:bg-black/20 rounded p-1"
             >
               ✕
             </button>
           </div>
           
           <div className="p-4 max-h-96 overflow-y-auto">
             <button
               onClick={() => handleToolToggle('geofence')}
               className="w-full mb-4 px-3 py-2 bg-purple-600 hover:bg-purple-700 rounded-md text-sm font-medium"
             >
               + Create New Geofence
             </button>
             
             <div className="space-y-3">
               {geofenceZones.map((zone) => (
                 <div key={zone.id} className={`p-3 rounded-md border ${editingGeofence === zone.id ? 'border-purple-500 bg-purple-900/20' : 'border-gray-600 bg-gray-800/50'}`}>
                   <div className="flex items-center justify-between mb-2">
                     <input
                       type="text"
                       value={zone.name}
                       onChange={(e) => {
                         setGeofenceZones(prev => prev.map(z => 
                           z.id === zone.id ? {...z, name: e.target.value} : z
                         ));
                       }}
                       className="bg-transparent border-b border-gray-600 text-sm font-medium focus:border-purple-400 outline-none"
                     />
                     <div className="flex items-center gap-2">
                       <label className="flex items-center cursor-pointer">
                         <input
                           type="checkbox"
                           checked={zone.isActive}
                           onChange={(e) => {
                             setGeofenceZones(prev => prev.map(z => 
                               z.id === zone.id ? {...z, isActive: e.target.checked} : z
                             ));
                           }}
                           className="w-4 h-4 text-purple-600"
                         />
                         <span className="ml-1 text-xs">Active</span>
                       </label>
                       <button
                         onClick={() => {
                           setGeofenceZones(prev => prev.filter(z => z.id !== zone.id));
                           setEditingGeofence(null);
                         }}
                         className="text-red-400 hover:text-red-300 text-xs"
                       >
                         Delete
                       </button>
                     </div>
                   </div>
                   
                   <div className="grid grid-cols-2 gap-2 text-xs">
                     <div>
                       <label className="block text-gray-400 mb-1">Type</label>
                       <select
                         value={zone.type}
                         onChange={(e) => {
                           setGeofenceZones(prev => prev.map(z => 
                             z.id === zone.id ? {...z, type: e.target.value as 'include' | 'exclude'} : z
                           ));
                         }}
                         className="w-full bg-gray-700 border border-gray-600 rounded px-2 py-1"
                       >
                         <option value="exclude">Exclude Zone</option>
                         <option value="include">Safe Zone</option>
                       </select>
                     </div>
                     
                     <div>
                       <label className="block text-gray-400 mb-1">Alert Level</label>
                       <select
                         value={zone.alertLevel}
                         onChange={(e) => {
                           setGeofenceZones(prev => prev.map(z => 
                             z.id === zone.id ? {...z, alertLevel: e.target.value as 'info' | 'warning' | 'critical'} : z
                           ));
                         }}
                         className="w-full bg-gray-700 border border-gray-600 rounded px-2 py-1"
                       >
                         <option value="info">Info</option>
                         <option value="warning">Warning</option>
                         <option value="critical">Critical</option>
                       </select>
                     </div>
                   </div>
                   
                   <div className="mt-2">
                     <label className="block text-gray-400 text-xs mb-1">Radius (m)</label>
                     <input
                       type="number"
                       value={Math.round(zone.radius)}
                       onChange={(e) => {
                         const radius = Math.max(10, parseInt(e.target.value) || 10);
                         setGeofenceZones(prev => prev.map(z => 
                           z.id === zone.id ? {...z, radius} : z
                         ));
                       }}
                       className="w-full bg-gray-700 border border-gray-600 rounded px-2 py-1 text-xs"
                       min="10"
                       step="5"
                     />
                   </div>
                   
                   <div className="mt-2 text-xs text-gray-400">
                     Center: {zone.center.lat.toFixed(6)}, {zone.center.lng.toFixed(6)}
                   </div>
                 </div>
               ))}
               
               {geofenceZones.length === 0 && (
                 <div className="text-center text-gray-400 py-4 text-sm">
                   No geofences created yet.<br/>
                   Click "Create New Geofence" to start.
                 </div>
               )}
             </div>
           </div>
         </div>
       )}
       
       {/* Waypoint Context Menu */}
       {contextMenu && viewMode === 'planning' && (
         <div 
           className="absolute z-[2000] bg-gray-800 rounded-md shadow-lg border border-gray-600 py-1"
           style={{ left: `${contextMenu.x}px`, top: `${contextMenu.y}px` }}
           onClick={(e) => e.stopPropagation()}
         >
           {onDeleteWaypoint && (
             <button
               onClick={handleDeleteWaypoint}
               className="w-full px-4 py-2 text-left text-sm text-white hover:bg-red-600 transition-colors"
             >
               Delete Waypoint
             </button>
           )}
           {onInsertWaypoint && (
             <button
               onClick={handleInsertWaypoint}
               className="w-full px-4 py-2 text-left text-sm text-white hover:bg-green-600 transition-colors"
             >
               Insert Waypoint After
             </button>
           )}
         </div>
       )}
       
       {/* North/Heading compass (updates with rover heading; placed bottom-left) */}
       <div className="absolute bottom-2 left-2 z-[1600]">
         <div className="flex items-center justify-center w-10 h-10 rounded-full bg-gray-800 bg-opacity-70 text-white border border-gray-600 shadow">
           {(() => {
             let compassHeading = 0;
             if (typeof heading === 'number' && Number.isFinite(heading)) {
               compassHeading = heading;
             } else {
               const a = prevSampleRef.current;
               const b = lastSampleRef.current;
               if (a && b) {
                 compassHeading = calculateBearing({ lat: a.lat, lng: a.lng }, { lat: b.lat, lng: b.lng });
               }
             }
             return <NorthArrowIcon className="w-16 h-16" headingDeg={compassHeading} />;
           })()}
         </div>
       </div>

       {viewMode === 'planning' && (
         <div className="absolute bottom-4 left-1/2 -translate-x-1/2 z-[1000] flex items-end gap-2">
            <div className="flex items-center gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-1">
                {drawingTools.map(tool => {
                    const Icon = tool.icon;
                    return (
                        <button key={tool.name} onClick={() => handleToolToggle(tool.name as Tool)} className={`p-2 rounded-md ${activeTool === tool.name ? 'bg-green-600 text-white' : 'text-gray-300 hover:bg-gray-700'}`} title={tool.title}>
                            <Icon className="w-5 h-5" />
                        </button>
                    )
                })}
                <div className="border-l border-gray-600 h-6 mx-1"></div>
                <button onClick={() => handleToolToggle('measure')} className={`p-2 rounded-md ${activeTool === 'measure' ? 'bg-blue-600 text-white' : 'text-gray-300 hover:bg-gray-700'}`} title="Measure Distance">
                    <RulerIcon className="w-5 h-5" />
                </button>
            </div>
            
            {(isToolActive && drawnPoints.length > 0) && (
                <div className="flex flex-col gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-2">
                    {['line', 'polygon'].includes(activeTool || '') && (
                        <button onClick={handleFinishDrawing} className="w-full text-xs bg-green-500 hover:bg-green-600 text-white font-bold py-1 px-4 rounded">Finish</button>
                    )}
                    <button onClick={handleCancelTool} className="w-full text-xs bg-red-500 hover:bg-red-600 text-white font-bold py-1 px-4 rounded">Cancel</button>
                </div>
            )}
            
            {activeTool === 'measure' && (
                <div className="flex flex-col items-center gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-2">
                    {measurementText && <p className="text-xs text-center text-white font-mono whitespace-nowrap">{measurementText}</p>}
                    <button onClick={handleCancelTool} className="w-full text-xs bg-red-500 hover:bg-red-600 text-white font-bold py-1 px-4 rounded">Clear</button>
                </div>
            )}
        </div>
       )}
      
       <div className="absolute top-2 right-2 z-[1000] flex flex-col gap-2">
         <button onClick={handleToggleMapFullScreen} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title={isMapFullScreen ? "Exit Fullscreen" : "Enter Fullscreen"}><FullScreenToggleIcon isFullScreen={isMapFullScreen} className="w-5 h-5" /></button>
         <button onClick={handleCenterOnUser} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title="Center on me"><CrosshairIcon className="w-5 h-5" /></button>
         <button onClick={() => setShowGeofencePanel(!showGeofencePanel)} className={`p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90 ${showGeofencePanel ? 'bg-purple-600' : ''}`} title="Manage Geofences"><GeofenceIcon className="w-5 h-5" /></button>
         <button onClick={() => setShowDebugPanel(!showDebugPanel)} className={`p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90 ${showDebugPanel ? 'bg-green-600' : ''}`} title="Debug Panel"><DebugIcon className="w-5 h-5" /></button>
         {pathWaypoints.length > 0 && (<button onClick={invalidateAndFitBounds} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title="Auto Zoom & Pan to mission"><ExpandIcon className="w-5 h-5" /></button>)}
         
         {/* Geofence Alerts */}
         {geofenceAlerts.length > 0 && (
           <div className="w-64 space-y-2 mt-4">
             {geofenceAlerts.slice(-3).map((alert) => (
               <div
                 key={alert.id}
                 className={`p-3 rounded-lg border shadow-lg animate-pulse ${
                   alert.level === 'critical' ? 'bg-red-900 border-red-700 text-red-100' :
                   alert.level === 'warning' ? 'bg-yellow-900 border-yellow-700 text-yellow-100' :
                   'bg-blue-900 border-blue-700 text-blue-100'
                 }`}
               >
                 <div className="flex items-center justify-between">
                   <span className="text-xs font-semibold uppercase">{alert.level}</span>
                   <span className="text-xs opacity-75">
                     {Math.round((Date.now() - alert.timestamp) / 1000)}s ago
                   </span>
                 </div>
                 <p className="text-sm mt-1">{alert.message}</p>
               </div>
             ))}
           </div>
         )}
       </div>
       <div ref={mapContainerRef} className="w-full flex-1" aria-label="Interactive map for setting a waypoint" />
       
       {/* Debug Panel */}
       {showDebugPanel && (
         <div className="absolute bottom-2 right-2 z-[2500] w-80 bg-gray-900 bg-opacity-95 text-white rounded-lg border border-gray-700 shadow-xl overflow-hidden">
           <div className="flex items-center justify-between px-4 py-3 bg-gradient-to-r from-green-600 to-teal-600">
             <h3 className="text-sm font-semibold">Debug Panel</h3>
             <button
               onClick={() => setShowDebugPanel(false)}
               className="text-white hover:bg-black/20 rounded p-1"
             >
               ✕
             </button>
           </div>
           
           <div className="p-4 space-y-4 max-h-96 overflow-y-auto">
             <div>
               <h4 className="text-sm font-semibold text-gray-300 mb-2">Performance</h4>
               <div className="grid grid-cols-2 gap-2 text-xs">
                 <div className="bg-gray-800 p-2 rounded">
                   <div className="text-gray-400">Render Time</div>
                   <div className="text-green-400">{debugMetrics.renderTime.toFixed(1)}ms</div>
                 </div>
                 <div className="bg-gray-800 p-2 rounded">
                   <div className="text-gray-400">Markers</div>
                   <div className="text-blue-400">{debugMetrics.markerCount}</div>
                 </div>
                 <div className="bg-gray-800 p-2 rounded">
                   <div className="text-gray-400">Memory</div>
                   <div className="text-yellow-400">{debugMetrics.memoryUsage.toFixed(1)}MB</div>
                 </div>
                 <div className="bg-gray-800 p-2 rounded">
                   <div className="text-gray-400">Zoom Level</div>
                   <div className="text-purple-400">{currentZoom}</div>
                 </div>
               </div>
             </div>
             
             {roverPosition && (
               <div>
                 <h4 className="text-sm font-semibold text-gray-300 mb-2">Rover State</h4>
                 <div className="bg-gray-800 p-2 rounded text-xs space-y-1">
                   <div><span className="text-gray-400">Position:</span> {roverPosition.lat.toFixed(6)}, {roverPosition.lng.toFixed(6)}</div>
                   <div><span className="text-gray-400">Heading:</span> {heading?.toFixed(1)}°</div>
                   <div><span className="text-gray-400">Connected:</span> <span className={isConnectedToRover ? 'text-green-400' : 'text-red-400'}>{isConnectedToRover ? 'Yes' : 'No'}</span></div>
                 </div>
               </div>
             )}
             
             <div>
               <h4 className="text-sm font-semibold text-gray-300 mb-2">Map State</h4>
               <div className="bg-gray-800 p-2 rounded text-xs space-y-1">
                 <div><span className="text-gray-400">View Mode:</span> {viewMode}</div>
                 <div><span className="text-gray-400">Active Tool:</span> {activeTool || 'None'}</div>
                 <div><span className="text-gray-400">Waypoints:</span> {pathWaypoints.length} total, {visibleWaypoints.length} visible</div>
                 <div><span className="text-gray-400">Geofences:</span> {geofenceZones.length}</div>
               </div>
             </div>
             
             <div>
               <h4 className="text-sm font-semibold text-gray-300 mb-2">Testing</h4>
               <div className="space-y-2">
                 <label className="flex items-center cursor-pointer">
                   <input
                     type="checkbox"
                     checked={simulationMode}
                     onChange={(e) => setSimulationMode(e.target.checked)}
                     className="w-4 h-4 text-green-600 mr-2"
                   />
                   <span className="text-sm">Simulation Mode</span>
                 </label>
                 
                 <button
                   onClick={() => {
                     const testGeofence = {
                       id: `test_${Date.now()}`,
                       center: roverPosition || { lat: 13.0827, lng: 80.2707 },
                       radius: 100,
                       name: 'Test Geofence',
                       type: 'exclude' as const,
                       alertLevel: 'warning' as const,
                       isActive: true
                     };
                     setGeofenceZones(prev => [...prev, testGeofence]);
                   }}
                   className="w-full px-3 py-1 bg-blue-600 hover:bg-blue-700 rounded text-sm"
                 >
                   Add Test Geofence
                 </button>
                 
                 <button
                   onClick={() => {
                     console.clear();
                     setDebugMetrics(prev => ({ ...prev, renderTime: 0, lastUpdate: Date.now() }));
                   }}
                   className="w-full px-3 py-1 bg-gray-600 hover:bg-gray-700 rounded text-sm"
                 >
                   Clear Logs
                 </button>
               </div>
             </div>
             
             <div className="text-xs text-gray-400 border-t border-gray-600 pt-2">
               <div>Ctrl+D: Toggle Debug Panel</div>
               <div>Ctrl+G: Toggle Geofence Panel</div>
               <div>Ctrl+Shift+S: Toggle Simulation</div>
             </div>
           </div>
         </div>
       )}

        {/* Layer visibility toggles */}
        <div className="absolute top-2 left-2 z-[1200] bg-gray-800/80 text-white rounded-md border border-gray-700 p-2">
          <div className="text-xs font-semibold mb-1">Layers</div>
          <div className="space-y-1 text-xs">
            {layers.map((layer) => (
              <label key={layer.type} className="flex items-center gap-2 cursor-pointer">
                <input
                  type="checkbox"
                  checked={layer.visible}
                  onChange={(e) => {
                    const visible = e.target.checked;
                    setLayers((prev) => prev.map((l) => (l.type === layer.type ? { ...l, visible } : l)));
                    // Attach/detach layer groups from map
                    if (layer.type === 'vehicle' && vehicleLayerGroupRef.current && mapRef.current) {
                      if (visible) vehicleLayerGroupRef.current.addTo(mapRef.current);
                      else mapRef.current.removeLayer(vehicleLayerGroupRef.current);
                    }
                    if (layer.type === 'trail' && trailLayerGroupRef.current && mapRef.current) {
                      if (visible) trailLayerGroupRef.current.addTo(mapRef.current);
                      else mapRef.current.removeLayer(trailLayerGroupRef.current);
                    }
                    if (layer.type === 'geofence' && geofenceLayerRef.current && mapRef.current) {
                      if (visible) geofenceLayerRef.current.addTo(mapRef.current);
                      else mapRef.current.removeLayer(geofenceLayerRef.current);
                    }
                    if (layer.type === 'waypoints' && missionLayerRef.current && mapRef.current) {
                      if (visible) missionLayerRef.current.addTo(mapRef.current);
                      else mapRef.current.removeLayer(missionLayerRef.current);
                    }
                  }}
                />
                <span>{layer.name}</span>
              </label>
            ))}
          </div>
        </div>
    </div>
  );
};

export default MapView;
