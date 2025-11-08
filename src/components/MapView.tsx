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
import { HexagonIcon } from './icons/HexagonIcon';
import { generateCircleWaypoints, generateRegularPolygonWaypoints, calculateDistance, calculateBearing, calculateDestination } from '../utils/geo';
import DrawingInstructions from './DrawingInstructions';
import { useFrameTicker } from '../hooks/useFrameTicker';
import { useRover } from '../context/RoverContext';
import useMapTelemetry from '../hooks/useMapTelemetry';
import type { MapLayer } from '../types/map';
import { useDialog } from '../hooks/useDialog';
import GenericDialog from './GenericDialog';


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
  clearTrail?: () => void;
  activeDrawingTool?: string | null;
  telemetry?: {
    speed?: number;
    battery?: number;
    signalStrength?: number;
    altitude?: number;
    satellites?: number;
  };
};

type Tool = 'smart-dimension' | 'profile' | 'line' | 'rectangle' | 'circle' | 'hexagon' | null;

const MAP_COMMANDS = new Set<string>([
  'WAYPOINT',
  'TAKEOFF',
  'LAND',
  'RETURN_TO_LAUNCH',
  'LOITER_TIME',
  'LOITER_TURNS',
]);

// Rover icon using asset image
const ROVER_ICON_URL = new URL('../assets/rover-icon.png', import.meta.url).href;

const getWaypointIcon = (waypoint: Waypoint, index: number, total: number, activeId: number | null | undefined, isPartOfSelectedSegment: boolean = false): any => {
    const isActive = waypoint.id === activeId;
    const isStart = index === 0;
    const isEnd = index === total - 1 && total > 1;

    let fill = '#f97316'; // Default orange
    if (isStart) fill = '#16a34a';
    if (isEnd) fill = '#dc2626';
    if (isActive) fill = '#22c55e';
    if (isPartOfSelectedSegment) fill = '#ff6b35'; // Highlight color for selected segments

    const size = isActive ? 32 : (isPartOfSelectedSegment ? 28 : 24);

    const svgIcon = `
      <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="${size}" height="${size}" fill="${fill}" class="${isPartOfSelectedSegment ? 'drop-shadow-lg animate-pulse' : 'drop-shadow-lg'}">
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
  clearTrail,
  activeDrawingTool,
  telemetry,
}) => {
  const pathWaypoints = useMemo(
    () =>
      missionWaypoints.filter((wp) => {
        // Only consider navigation commands that should appear on the map
        if (!MAP_COMMANDS.has(wp.command)) return false;
        // Respect explicit flag to hide waypoints from the map (placeholders)
        if (wp.showOnMap === false) return false;
        if (!Number.isFinite(wp.lat) || !Number.isFinite(wp.lng)) return false;
        // Optional: ignore obvious placeholder coords (0,0)
        if (wp.lat === 0 && wp.lng === 0) return false;
        return true;
      }),
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
  const isDrawingRef = useRef(false);

  // Keep previous and last rover samples for interpolation
  const prevSampleRef = useRef<{ t: number, lat: number, lng: number, heading: number } | null>(null);
  const lastSampleRef = useRef<{ t: number, lat: number, lng: number, heading: number } | null>(null);
  const lastIconUpdateRef = useRef<number>(0);
  const lastPositionUpdateRef = useRef<number>(0);
  const { dialogState, showAlert } = useDialog();
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
  () => pathWaypoints.map((wp) => `${wp.lat.toFixed(7)},${wp.lng.toFixed(7)}`).join('|'),
    [pathWaypoints]
  );

  const [isMapFullScreen, setIsMapFullScreen] = useState(false);
  const [drawnPoints, setDrawnPoints] = useState<any[]>([]);
  const [mousePos, setMousePos] = useState<any>(null);
  const [rectanglePhase, setRectanglePhase] = useState<'idle' | 'first-click' | 'dragging'>('idle');
  const [selectedSegments, setSelectedSegments] = useState<Set<number>>(new Set());
  const [editingDimension, setEditingDimension] = useState<{type: 'distance' | 'angle', segmentIndex: number, value: number} | null>(null);
  const [measurementText, setMeasurementText] = useState<string | null>(null);
  const [selectedWaypointId, setSelectedWaypointId] = useState<number | null>(null);
  const [contextMenu, setContextMenu] = useState<{ x: number, y: number, waypointId: number } | null>(null);
  const [showKeyboardHelp, setShowKeyboardHelp] = useState(false);
  // Trail is now managed internally via useMapTelemetry to avoid React re-renders
  
  // Lazy loading state
  const [visibleMarkers, setVisibleMarkers] = useState<Set<number>>(new Set());
  const [mapBounds, setMapBounds] = useState<any>(null);
  const [currentZoom, setCurrentZoom] = useState(13);
  const lastBoundsUpdateRef = useRef<number>(0);

  // Layer visibility management
  const [layers, setLayers] = useState<MapLayer[]>([
    { name: 'Vehicle', type: 'vehicle', visible: true },
    { name: 'Trail', type: 'trail', visible: true },
    { name: 'Waypoints', type: 'waypoints', visible: true },
  ]);

  // Vehicle and trail updates handled via useMapTelemetry (non-react updates)
  const mapTelemetryControls = useMapTelemetry({
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

  const handleDimensionEdit = (segmentIndex: number, newValue: number, type: 'distance' | 'angle') => {
    if (type === 'distance') {
      // Adjust waypoint coordinates to maintain the new distance
      adjustSegmentDistance(segmentIndex, newValue * 1000); // Convert km to meters
    } else if (type === 'angle') {
      // Find the angle adjustment and apply it
      adjustSegmentAngle(segmentIndex, newValue);
    }
    setEditingDimension(null);
  };

  const adjustSegmentDistance = (segmentIndex: number, newDistance: number) => {
    if (segmentIndex >= pathWaypoints.length - 1) return;

    const startWp = pathWaypoints[segmentIndex];
    const endWp = pathWaypoints[segmentIndex + 1];
    const startPoint = L.latLng(startWp.lat, startWp.lng);
    const endPoint = L.latLng(endWp.lat, endWp.lng);

    const currentDistance = mapRef.current.distance(startPoint, endPoint);

    // Calculate new end point coordinates
    const bearing = calculateBearing(startPoint, endPoint);
    const newEndPoint = calculateDestination(startPoint, bearing, newDistance);

    // Update the waypoint
    onUpdateWaypointPosition(pathWaypoints[segmentIndex + 1].id, {
      lat: newEndPoint.lat,
      lng: newEndPoint.lng
    });

    // Update all subsequent waypoints if they exist
    if (segmentIndex + 2 < pathWaypoints.length) {
      const nextSegmentBearing = calculateBearing(
        newEndPoint,
        L.latLng(pathWaypoints[segmentIndex + 2].lat, pathWaypoints[segmentIndex + 2].lng)
      );

      for (let i = segmentIndex + 2; i < pathWaypoints.length; i++) {
        const prevWp = i === segmentIndex + 1 ? { lat: newEndPoint.lat, lng: newEndPoint.lng } : pathWaypoints[i - 1];
        const currentWp = pathWaypoints[i];
        const distanceToPrev = mapRef.current.distance(
          L.latLng(prevWp.lat, prevWp.lng),
          L.latLng(currentWp.lat, currentWp.lng)
        );

        const newPoint = calculateDestination(
          { lat: prevWp.lat, lng: prevWp.lng },
          nextSegmentBearing,
          distanceToPrev
        );

        onUpdateWaypointPosition(currentWp.id, newPoint);
      }
    }
  };

  const adjustSegmentAngle = (vertexIndex: number, newAngle: number) => {
    // This is more complex - we need to find the angle between two segments
    // and adjust the middle waypoint to achieve the new angle
    if (vertexIndex < 1 || vertexIndex >= pathWaypoints.length - 1) return;

    const prevWp = pathWaypoints[vertexIndex - 1];
    const currentWp = pathWaypoints[vertexIndex];
    const nextWp = pathWaypoints[vertexIndex + 1];

    const bearing1 = calculateBearing(
      L.latLng(prevWp.lat, prevWp.lng),
      L.latLng(currentWp.lat, currentWp.lng)
    );

    // Calculate the required bearing for the second segment
    const bearing2 = (bearing1 + newAngle) % 360;

    // Keep the distance to the next waypoint the same
    const distanceToNext = mapRef.current.distance(
      L.latLng(currentWp.lat, currentWp.lng),
      L.latLng(nextWp.lat, nextWp.lng)
    );

    const newNextPoint = calculateDestination(
      { lat: currentWp.lat, lng: currentWp.lng },
      bearing2,
      distanceToNext
    );

    // Update the next waypoint
    onUpdateWaypointPosition(nextWp.id, newNextPoint);
  };

  useEffect(() => {
    if (mapContainerRef.current && !mapRef.current) {
      // Use WGS84 Web Mercator (EPSG3857) for correct tile rendering
      const map = L.map(mapContainerRef.current, { crs: L.CRS.EPSG3857, zoomControl: true, maxZoom: 24 }).setView([13.0827, 80.2707], 16);
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
          maxZoom: 24
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

  useEffect(() => {
    const onFullScreenChange = () => setIsMapFullScreen(document.fullscreenElement === mapWrapperRef.current);
    document.addEventListener('fullscreenchange', onFullScreenChange);
    return () => document.removeEventListener('fullscreenchange', onFullScreenChange);
  }, []);

  useEffect(() => {
    // Auto-fit on view mode / fullscreen changes was removed per request.
    // Keeping this effect intentionally empty to avoid automatic zoom/pan.
    // Call `invalidateAndFitBounds()` manually via Ctrl+M or programmatic action if needed.
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
        if (activeDrawingTool === 'rectangle' && rectanglePhase === 'first-click') {
          // Show live preview for rectangle
          setRectanglePhase('dragging');
        } else if (isDrawingRef.current) {
          if (activeDrawingTool === 'rectangle' || activeDrawingTool === 'circle' || activeDrawingTool === 'hexagon') {
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
        
        if (activeDrawingTool === 'rectangle') {
            const bounds = L.latLngBounds(startPoint, endPoint);
            waypoints = [
                bounds.getSouthWest(),
                bounds.getNorthWest(),
                bounds.getNorthEast(),
                bounds.getSouthEast(),
            ];
        } else if (activeDrawingTool === 'circle') {
            waypoints = generateCircleWaypoints(startPoint, radius);
        } else if (activeDrawingTool === 'hexagon') {
            const bearing = calculateBearing(startPoint, endPoint);
            waypoints = generateRegularPolygonWaypoints(startPoint, radius, 6, bearing);
        }

        if (waypoints.length > 0) {
            onNewMissionDrawn(waypoints);
        }
        handleCancelTool();
    };

    const handleSegmentSelection = (e: any) => {
      const clickPoint = e.latlng;
      let closestSegment = -1;
      let minDistance = Infinity;

      // Find the closest segment to the click point
      pathWaypoints.forEach((wp, index) => {
        if (index < pathWaypoints.length - 1) {
          const nextWp = pathWaypoints[index + 1];
          const segmentStart = L.latLng(wp.lat, wp.lng);
          const segmentEnd = L.latLng(nextWp.lat, nextWp.lng);

          // Calculate distance from click point to line segment
          const distance = getDistanceToLineSegment(clickPoint, segmentStart, segmentEnd);
          if (distance < minDistance && distance < 50) { // 50 meter tolerance
            minDistance = distance;
            closestSegment = index;
          }
        }
      });

      if (closestSegment !== -1) {
        setSelectedSegments(prev => {
          const newSet = new Set(prev);
          if (newSet.has(closestSegment)) {
            newSet.delete(closestSegment);
          } else {
            // If Ctrl is not pressed, clear other selections
            if (!e.originalEvent.ctrlKey) {
              newSet.clear();
            }
            newSet.add(closestSegment);
          }
          return newSet;
        });
      }
    };

    // Helper function to calculate distance from point to line segment
    const getDistanceToLineSegment = (point: L.LatLng, lineStart: L.LatLng, lineEnd: L.LatLng): number => {
      const A = point.lat - lineStart.lat;
      const B = point.lng - lineStart.lng;
      const C = lineEnd.lat - lineStart.lat;
      const D = lineEnd.lng - lineStart.lng;

      const dot = A * C + B * D;
      const lenSq = C * C + D * D;

      let param = -1;
      if (lenSq !== 0) {
        param = dot / lenSq;
      }

      let xx, yy;
      if (param < 0) {
        xx = lineStart.lat;
        yy = lineStart.lng;
      } else if (param > 1) {
        xx = lineEnd.lat;
        yy = lineEnd.lng;
      } else {
        xx = lineStart.lat + param * C;
        yy = lineStart.lng + param * D;
      }

      const dx = point.lat - xx;
      const dy = point.lng - yy;
      return Math.sqrt(dx * dx + dy * dy) * 111320; // Convert degrees to meters (approximate)
    };

    const handlePointClick = (e: any) => {
      if (activeDrawingTool === 'rectangle') {
        if (rectanglePhase === 'idle') {
          // First click - set first corner
          setDrawnPoints([e.latlng]);
          setRectanglePhase('first-click');
        } else if (rectanglePhase === 'first-click') {
          // Second click - complete rectangle
          const firstPoint = drawnPoints[0];
          const secondPoint = e.latlng;
          const bounds = L.latLngBounds(firstPoint, secondPoint);
          const waypoints = [
            bounds.getSouthWest(),
            bounds.getNorthWest(),
            bounds.getNorthEast(),
            bounds.getSouthEast(),
          ];
          onNewMissionDrawn(waypoints);
          handleCancelTool();
          setRectanglePhase('idle');
        }
      } else if (activeDrawingTool === 'smart-dimension') {
        // Handle segment selection for smart dimension editing
        handleSegmentSelection(e);
      } else {
        setDrawnPoints(prev => [...prev, e.latlng]);
      }
    };
    
    const handleFinishWithDoubleClick = () => {
       if (activeDrawingTool === 'smart-dimension') {
         // Tool state is managed at App level, no need to set locally
       } else {
         handleFinishDrawing();
       }
    };


  switch (activeDrawingTool) {
    case 'line':
    case 'smart-dimension':
      map.on('click', handlePointClick);
      map.on('dblclick', handleFinishWithDoubleClick);
      map.getContainer().style.cursor = 'crosshair';
      break;
    case 'rectangle':
      map.on('click', handlePointClick);
      map.on('mousemove', handleMouseMove);
      map.getContainer().style.cursor = 'crosshair';
      break;
    case 'circle':
    case 'hexagon':
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
}, [activeDrawingTool, onMapClick]);

  // Effect to manage drawing/measuring visuals
  useEffect(() => {
    if (!mapRef.current) return;
    if (!drawingLayerRef.current) drawingLayerRef.current = L.layerGroup().addTo(mapRef.current);
    drawingLayerRef.current.clearLayers();

    if (drawnPoints.length > 0) {
        if (['line', 'polygon', 'rectangle', 'circle', 'hexagon'].includes(activeDrawingTool || '')) {
            const drawOptions = { color: '#22c55e', weight: 3, dashArray: '5, 5' };
            if (activeDrawingTool === 'rectangle') {
              if (rectanglePhase === 'dragging' && drawnPoints.length === 1 && mousePos) {
                // Live preview for rectangle during dragging
                L.rectangle(L.latLngBounds(drawnPoints[0], mousePos), drawOptions).addTo(drawingLayerRef.current);
              } else if (drawnPoints.length === 2) {
                // Completed rectangle preview
                L.rectangle(L.latLngBounds(drawnPoints[0], drawnPoints[1]), drawOptions).addTo(drawingLayerRef.current);
              }
            } else if ((activeDrawingTool === 'circle' || activeDrawingTool === 'hexagon') && drawnPoints.length > 0 && mousePos) {
                const radius = calculateDistance(drawnPoints[0], mousePos);
                L.circle(drawnPoints[0], { ...drawOptions, radius }).addTo(drawingLayerRef.current);
            } else {
                 drawnPoints.forEach(p => L.circleMarker(p, { radius: 4, color: '#22c55e', fillOpacity: 1 }).addTo(drawingLayerRef.current));
            }
            if (mousePos && drawnPoints.length > 0) {
               const previewPoints = [...drawnPoints, mousePos];
              L.polyline(previewPoints, drawOptions).addTo(drawingLayerRef.current);
            }
        } else if (activeDrawingTool === 'smart-dimension') {
            let totalDistance = 0;
            let angles: {point: any, angle: number, segmentIndices: [number, number]}[] = [];

            // Render existing path with segment selection highlighting
            pathWaypoints.forEach((wp, index) => {
                if (index < pathWaypoints.length - 1) {
                    const nextWp = pathWaypoints[index + 1];
                    const segmentStart = L.latLng(wp.lat, wp.lng);
                    const segmentEnd = L.latLng(nextWp.lat, nextWp.lng);
                    const dist = mapRef.current.distance(segmentStart, segmentEnd);
                    totalDistance += dist;

                    // Highlight selected segments
                    const isSelected = selectedSegments.has(index);
                    const segmentColor = isSelected ? '#ff6b35' : '#3b82f6';
                    const segmentWeight = isSelected ? 5 : 2;
                    const segmentOpacity = isSelected ? 1 : 0.7;
                    const segmentDashArray = isSelected ? null : '5, 5';

                    L.polyline([segmentStart, segmentEnd], {
                        color: segmentColor,
                        weight: segmentWeight,
                        opacity: segmentOpacity,
                        dashArray: segmentDashArray
                    }).addTo(drawingLayerRef.current);

                    // Add distance dimension for selected segments
                    if (isSelected) {
                        const centerPoint = L.latLngBounds(segmentStart, segmentEnd).getCenter();
                        L.tooltip({ permanent: true, direction: 'center', className: 'dimension-tooltip' })
                          .setLatLng(centerPoint)
                          .setContent(`<div class="dimension-box" style="background: rgba(255, 255, 255, 0.95); border: 2px solid #ff6b35; padding: 4px 6px; border-radius: 6px; font-size: 12px; color: #ff6b35; font-weight: bold; box-shadow: 0 2px 4px rgba(0,0,0,0.2);">${dist.toFixed(2)} m</div>`)
                          .addTo(drawingLayerRef.current);
                    }

                    // Calculate angle if we have 3+ points
                    if (index > 0 && index < pathWaypoints.length - 1) {
                        const prevWp = pathWaypoints[index - 1];
                        const currentWp = pathWaypoints[index];
                        const nextWp_check = pathWaypoints[index + 1];

                        const bearing1 = calculateBearing(L.latLng(prevWp.lat, prevWp.lng), L.latLng(currentWp.lat, currentWp.lng));
                        const bearing2 = calculateBearing(L.latLng(currentWp.lat, currentWp.lng), L.latLng(nextWp_check.lat, nextWp_check.lng));
                        let angle = Math.abs(bearing2 - bearing1);

                        // Normalize angle to 0-180 degrees
                        if (angle > 180) angle = 360 - angle;
                        if (angle > 90) angle = 180 - angle;

                        // Check if both adjacent segments are selected
                        const prevSegmentSelected = selectedSegments.has(index - 1);
                        const currentSegmentSelected = selectedSegments.has(index);

                        if (prevSegmentSelected && currentSegmentSelected) {
                            angles.push({
                                point: L.latLng(currentWp.lat, currentWp.lng),
                                angle: angle,
                                segmentIndices: [index - 1, index]
                            });
                        }
                    }
                }
            });

            // Draw angles for selected segment pairs
            angles.forEach(({point, angle}) => {
                L.tooltip({ permanent: true, direction: 'center', className: 'angle-tooltip' })
                  .setLatLng(point)
                  .setContent(`<div class="dimension-box" style="background: white; border: 1px solid #ff6b35; padding: 2px 4px; border-radius: 3px; font-size: 11px; color: #ff6b35; font-weight: bold;">${angle.toFixed(1)}°</div>`)
                  .addTo(drawingLayerRef.current);
            });

            if (pathWaypoints.length > 1) {
                setMeasurementText(`Selected: ${selectedSegments.size} segments | Total: ${totalDistance.toFixed(2)} m${angles.length > 0 ? ` | ${angles.length} angles` : ''}`);
            } else {
                setMeasurementText('Click on path segments to select for editing (Ctrl+click for multiple)');
            }
        }
    }
  }, [drawnPoints, activeDrawingTool, mousePos]);

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
        // Check if this waypoint is part of a selected segment
        const isPartOfSelectedSegment = (index > 0 && selectedSegments.has(index - 1)) || 
                                       (index < pathWaypoints.length - 1 && selectedSegments.has(index));
        
        const icon = getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex, isPartOfSelectedSegment);
        const marker = L.marker([wp.lat, wp.lng], { 
          icon,
          draggable: isDraggable,
        })
          .bindTooltip(`<b>Waypoint ${wp.id}</b><br>${wp.command}<br>Lat: ${wp.lat.toFixed(7)}<br>Lng: ${wp.lng.toFixed(7)}<br>Alt: ${wp.alt}m`)
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
      
      // Auto-fit on mission load has been disabled per user request.
      // If a manual fit is needed, press Ctrl+M (keyboard) or call `invalidateAndFitBounds()` programmatically.
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
        // Check if this waypoint is part of a selected segment
        const isPartOfSelectedSegment = (index > 0 && selectedSegments.has(index - 1)) || 
                                       (index < pathWaypoints.length - 1 && selectedSegments.has(index));
        
        missionMarkersRef.current[index].setIcon(
          getWaypointIcon(wp, index, pathWaypoints.length, activeWaypointIndex, isPartOfSelectedSegment)
        );
      }
    });
  }, [activeWaypointIndex, pathWaypoints, selectedSegments]);

  // Rover marker updates are handled inside useMapTelemetry
  
  // Trail is drawn by useMapTelemetry into its dedicated layer group

  // Keep rover in view without constantly re-fitting bounds (prevents zoom/pan oscillation)
  useEffect(() => {
    // Auto-pan / auto-zoom behavior disabled per user request.
    // Previously, the map would automatically pan or fit bounds when the rover
    // moved out of view. That behavior caused unexpected view jumps.
    // Intentionally no-op: keep the user's current map view stable.
    // If a manual center/fit is needed, user can press Ctrl+R (center) or Ctrl+M (fit bounds).
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
  }, async () => await showAlert('Location Error', 'Could not get location.'));

  const handleToggleMapFullScreen = () => {
    if (!document.fullscreenElement) mapWrapperRef.current?.requestFullscreen();
    else document.exitFullscreen();
  };

  const handleFinishDrawing = () => {
    if (drawnPoints.length > 1) {
      onNewMissionDrawn(drawnPoints);
    }
    handleCancelTool();
  };

  const handleCancelTool = () => {
    setDrawnPoints([]);
    setMeasurementText(null);
    setMousePos(null);
    setRectanglePhase('idle');
    setSelectedSegments(new Set());
    setEditingDimension(null);
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

  const isToolActive = activeDrawingTool && ['line', 'rectangle', 'circle', 'hexagon', 'smart-dimension'].includes(activeDrawingTool);

  return (
    <div ref={mapWrapperRef} className="relative w-full h-full rounded-lg overflow-hidden bg-gray-700 flex flex-col">
       <DrawingInstructions activeTool={activeDrawingTool as Tool} />
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
            {(isToolActive && drawnPoints.length > 0) && (
                <div className="flex flex-col gap-1 bg-gray-800 bg-opacity-80 backdrop-blur-sm rounded-lg p-2">
                    {['line', 'polygon'].includes(activeDrawingTool || '') && (
                        <button onClick={handleFinishDrawing} className="w-full text-xs bg-green-500 hover:bg-green-600 text-white font-bold py-1 px-4 rounded">Finish</button>
                    )}
                    <button onClick={handleCancelTool} className="w-full text-xs bg-red-500 hover:bg-red-600 text-white font-bold py-1 px-4 rounded">Cancel</button>
                </div>
            )}
            
            {activeDrawingTool === 'smart-dimension' && (
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
         {pathWaypoints.length > 0 && (<button onClick={invalidateAndFitBounds} className="p-2 bg-gray-800 bg-opacity-70 rounded-md text-white hover:bg-opacity-90" title="Auto Zoom & Pan to mission"><ExpandIcon className="w-5 h-5" /></button>)}
       </div>
       <div ref={mapContainerRef} className="w-full flex-1" aria-label="Interactive map for setting a waypoint" />

       {/* Dimension Editing Inputs */}
       {activeDrawingTool === 'smart-dimension' && selectedSegments.size > 0 && mapRef.current && (
         <div className="absolute inset-0 pointer-events-none z-[2000]">
           {Array.from(selectedSegments).map(segmentIndex => {
             if (segmentIndex >= pathWaypoints.length - 1) return null;

             const startWp = pathWaypoints[segmentIndex];
             const endWp = pathWaypoints[segmentIndex + 1];
             const centerPoint = L.latLngBounds(
               L.latLng(startWp.lat, startWp.lng),
               L.latLng(endWp.lat, endWp.lng)
             ).getCenter();

             const pixelPoint = mapRef.current.latLngToContainerPoint(centerPoint);

             return (
               <div
                 key={segmentIndex}
                 className="absolute pointer-events-auto"
                 style={{
                   left: pixelPoint.x - 40,
                   top: pixelPoint.y - 15,
                   transform: 'translate(-50%, -50%)'
                 }}
               >
                 <input
                   type="number"
                   step="0.01"
                   className="w-20 px-2 py-1 text-xs bg-orange-500 text-white border border-orange-600 rounded font-mono text-center"
                   defaultValue={(mapRef.current.distance(
                     L.latLng(startWp.lat, startWp.lng),
                     L.latLng(endWp.lat, endWp.lng)
                   ) / 1000).toFixed(2)}
                   onKeyDown={(e) => {
                     if (e.key === 'Enter') {
                       const newValue = parseFloat(e.currentTarget.value);
                       if (!isNaN(newValue) && newValue > 0) {
                         handleDimensionEdit(segmentIndex, newValue, 'distance');
                       }
                     } else if (e.key === 'Escape') {
                       setSelectedSegments(prev => {
                         const newSet = new Set(prev);
                         newSet.delete(segmentIndex);
                         return newSet;
                       });
                     }
                   }}
                   onBlur={(e) => {
                     const newValue = parseFloat(e.target.value);
                     if (!isNaN(newValue) && newValue > 0) {
                       handleDimensionEdit(segmentIndex, newValue, 'distance');
                     }
                   }}
                   autoFocus
                 />
               </div>
             );
           })}

           {/* Angle editing for adjacent selected segments */}
           {(() => {
             const angleEdits: React.JSX.Element[] = [];
             Array.from(selectedSegments).forEach(segmentIndex => {
               if (segmentIndex > 0 && selectedSegments.has(segmentIndex - 1)) {
                 // This segment and the previous one are both selected
                 const vertexIndex = segmentIndex;
                 const currentWp = pathWaypoints[vertexIndex];
                 const pixelPoint = mapRef.current.latLngToContainerPoint(L.latLng(currentWp.lat, currentWp.lng));

                 // Calculate current angle
                 const prevWp = pathWaypoints[vertexIndex - 1];
                 const nextWp = pathWaypoints[vertexIndex + 1];

                 const bearing1 = calculateBearing(
                   L.latLng(prevWp.lat, prevWp.lng),
                   L.latLng(currentWp.lat, currentWp.lng)
                 );
                 const bearing2 = calculateBearing(
                   L.latLng(currentWp.lat, currentWp.lng),
                   L.latLng(nextWp.lat, nextWp.lng)
                 );

                 let angle = Math.abs(bearing2 - bearing1);
                 if (angle > 180) angle = 360 - angle;
                 if (angle > 90) angle = 180 - angle;

                 angleEdits.push(
                   <div
                     key={`angle-${vertexIndex}`}
                     className="absolute pointer-events-auto"
                     style={{
                       left: pixelPoint.x - 30,
                       top: pixelPoint.y + 20,
                       transform: 'translate(-50%, -50%)'
                     }}
                   >
                     <input
                       type="number"
                       step="0.1"
                       className="w-16 px-2 py-1 text-xs bg-blue-500 text-white border border-blue-600 rounded font-mono text-center"
                       defaultValue={angle.toFixed(1)}
                       onKeyDown={(e) => {
                         if (e.key === 'Enter') {
                           const newValue = parseFloat(e.currentTarget.value);
                           if (!isNaN(newValue) && newValue >= 0 && newValue <= 180) {
                             handleDimensionEdit(vertexIndex, newValue, 'angle');
                           }
                         } else if (e.key === 'Escape') {
                           // Deselect both segments
                           setSelectedSegments(prev => {
                             const newSet = new Set(prev);
                             newSet.delete(segmentIndex);
                             newSet.delete(segmentIndex - 1);
                             return newSet;
                           });
                         }
                       }}
                       onBlur={(e) => {
                         const newValue = parseFloat(e.target.value);
                         if (!isNaN(newValue) && newValue >= 0 && newValue <= 180) {
                           handleDimensionEdit(vertexIndex, newValue, 'angle');
                         }
                       }}
                     />
                   </div>
                 );
               }
             });
             return angleEdits;
           })()}
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
      <GenericDialog
        isOpen={dialogState.isOpen}
        type={dialogState.type}
        title={dialogState.title}
        message={dialogState.message}
        onConfirm={dialogState.onConfirm}
        onCancel={dialogState.onCancel}
        confirmText={dialogState.confirmText}
        cancelText={dialogState.cancelText}
      />
    </div>
  );
};

export default MapView;
