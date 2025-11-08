import { useEffect, useRef } from 'react';
// Use global Leaflet instance to match MapView setup
declare var L: any;
import { useRover } from '../context/RoverContext';
import { createPathTrail } from '../utils/leaflet-helpers';
import RoverMarker, { createRoverMarker } from '../components/map/RoverMarker';
import { useSmoothedHeading } from './useSmoothedHeading';

interface UseMapTelemetryParams {
  mapRef: React.MutableRefObject<any>;
  vehicleLayerRef: React.MutableRefObject<any>;
  trailLayerRef: React.MutableRefObject<any>;
  throttleMs?: number; // default 250ms (4 Hz)
  maxTrailPoints?: number; // default 500
}

export function useMapTelemetry({
  mapRef,
  vehicleLayerRef, 
  trailLayerRef,
  throttleMs = 250,
  maxTrailPoints = 500,
}: UseMapTelemetryParams) {
  const { telemetry, roverPosition } = useRover();
  const targetHeading = telemetry.attitude?.yaw_deg ?? 0;
  const smoothedHeading = useSmoothedHeading(targetHeading);

  const markerRef = useRef<RoverMarker | null>(null);
  const trailRef = useRef<any | null>(null);
  const pointsRef = useRef<Array<[number, number]>>([]);
  const lastEmitRef = useRef<number>(0);
  const lastTooltipUpdateRef = useRef<number>(0);
  const previousTooltipRef = useRef<string>('');

  // Ensure layer groups exist on map
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    if (!vehicleLayerRef.current) {
      vehicleLayerRef.current = L.layerGroup().addTo(map);
    }
    if (!trailLayerRef.current) {
      trailLayerRef.current = L.layerGroup().addTo(map);
    }
  }, [mapRef, vehicleLayerRef, trailLayerRef]);

  // Initialize marker and trail ONCE when position first becomes available
  useEffect(() => {
    const map = mapRef.current;
    const vehicleLayer = vehicleLayerRef.current;
    const trailLayer = trailLayerRef.current;
    
    if (!map || !vehicleLayer || !trailLayer) {
      return;
    }

    // Only create marker if we don't have one AND position is available
    if (!markerRef.current && roverPosition) {
      const status: 'armed' | 'disarmed' | 'rtk' = telemetry.state?.armed
        ? 'armed'
        // Treat DGPS (4) as non-RTK; RTK Float (5) and RTK Fixed (6) are considered RTK
        : (telemetry.rtk?.fix_type ?? 0) >= 5
          ? 'rtk'
          : 'disarmed';
      
      const currentZoom = map.getZoom();
      
      markerRef.current = createRoverMarker(roverPosition.lat, roverPosition.lng, {
        heading: smoothedHeading,
        altitude: telemetry.global?.alt_rel ?? 0,
        status,
        zoomLevel: currentZoom,
        zIndexOffset: 1000,
      }).addTo(vehicleLayer);
      
      console.log('[useMapTelemetry] 🎯 Marker created with heading:', {
        heading: smoothedHeading.toFixed(1),
        zoom: currentZoom,
        position: [roverPosition.lat, roverPosition.lng]
      });
      
      // Create enhanced tooltip with more data
      const speed = telemetry.global?.vel ?? 0;
      const sats = telemetry.global?.satellites_visible ?? 0;
      const battery = telemetry.battery?.percentage ?? 0;
      const tt = `Position: ${roverPosition.lat.toFixed(7)}, ${roverPosition.lng.toFixed(7)}
Altitude: ${(telemetry.global?.alt_rel ?? 0).toFixed(1)} m
Heading: ${smoothedHeading.toFixed(1)}°
Speed: ${speed.toFixed(2)} m/s
Satellites: ${sats}
Battery: ${battery.toFixed(0)}%`;
      
      markerRef.current.bindTooltip(tt, { 
        permanent: false, 
        direction: 'top', 
        offset: [0, -10],
        className: 'rover-tooltip'
      });
      previousTooltipRef.current = tt;
      lastTooltipUpdateRef.current = performance.now();
      
      console.log('[useMapTelemetry] ✅ Rover marker created at:', { 
        lat: roverPosition.lat, 
        lng: roverPosition.lng,
        status 
      });
    }

    // Only create trail if we don't have one
    if (!trailRef.current && trailLayer) {
      trailRef.current = createPathTrail([], '#0ea5e9').addTo(trailLayer);
      console.log('[useMapTelemetry] ✅ Trail layer created and added to map');
    }

    // No cleanup needed here - marker and trail persist across position updates
    // Cleanup will happen when component unmounts (handled in separate effect)
  }, [mapRef, vehicleLayerRef, trailLayerRef, roverPosition, smoothedHeading, telemetry]);

  // Cleanup effect - runs only on component unmount
  useEffect(() => {
    return () => {
      console.log('[useMapTelemetry] 🧹 Component unmounting - cleaning up marker and trail');
      if (markerRef.current) {
        markerRef.current.remove();
        markerRef.current = null;
      }
      if (trailRef.current) {
        trailRef.current.remove();
        trailRef.current = null;
      }
      pointsRef.current = [];
    };
  }, []); // Empty dependency array = runs only on mount/unmount


  // Throttled, non-blocking updates using requestAnimationFrame
  useEffect(() => {
    const pos = roverPosition;
    const marker = markerRef.current;
    const trail = trailRef.current;

    // Bail early if no position yet
    if (!pos) {
      return;
    }

    // If marker/trail not ready yet, they'll be created in the init effect above
    if (!marker || !trail) {
      return;
    }

    const now = performance.now();
    const elapsed = now - lastEmitRef.current;

    const update = () => {
      // Update marker position
      marker.setLatLng(pos);

      // Update heading (use RoverMarker API)
      marker.setHeading(smoothedHeading);
      
      // Debug: Log heading updates occasionally
      if (Math.random() < 0.1) { // Log 10% of updates to avoid spam
        console.log('[useMapTelemetry] 🧭 Heading updated:', {
          smoothedHeading: smoothedHeading.toFixed(1),
          targetHeading: targetHeading.toFixed(1)
        });
      }

      // Update tooltip - throttle to every 1000ms (1 second) to reduce jitter
      const tooltipElapsed = now - lastTooltipUpdateRef.current;
      if (tooltipElapsed > 1000) {
        const status: 'armed' | 'disarmed' | 'rtk' = telemetry.state?.armed
          ? 'armed'
          : (telemetry.rtk?.fix_type ?? 0) >= 5
            ? 'rtk'
            : 'disarmed';
        
        const speed = telemetry.global?.vel ?? 0;
        const sats = telemetry.global?.satellites_visible ?? 0;
        const battery = telemetry.battery?.percentage ?? 0;
        
        const tt = `Position: ${pos.lat.toFixed(7)}, ${pos.lng.toFixed(7)}
Altitude: ${(telemetry.global?.alt_rel ?? 0).toFixed(1)} m
Heading: ${smoothedHeading.toFixed(1)}°
Speed: ${speed.toFixed(2)} m/s
Satellites: ${sats}
Battery: ${battery.toFixed(0)}%`;
        
        // Only update if tooltip content actually changed
        if (tt !== previousTooltipRef.current) {
          marker.setTooltipContent(tt);
          previousTooltipRef.current = tt;
          lastTooltipUpdateRef.current = now;
        }
        
        marker.updateStatus(status);
      }

      // Add to trail if position changed
      const lastPoint = pointsRef.current[pointsRef.current.length - 1];
      if (!lastPoint || lastPoint[0] !== pos.lat || lastPoint[1] !== pos.lng) {
        pointsRef.current.push([pos.lat, pos.lng]);
        if (pointsRef.current.length > maxTrailPoints) {
          pointsRef.current.shift();
        }
        trail.setLatLngs(pointsRef.current);
        console.log('[useMapTelemetry] 🛤️ Trail updated:', {
          totalPoints: pointsRef.current.length,
          newPoint: [pos.lat, pos.lng]
        });
      }
    };

    if (elapsed > throttleMs) {
      requestAnimationFrame(update);
      lastEmitRef.current = now;
    }
  }, [
    roverPosition, 
    telemetry.state?.armed, 
    telemetry.rtk?.fix_type, 
    telemetry.global?.alt_rel,
    telemetry.global?.vel,
    telemetry.global?.satellites_visible,
    telemetry.battery?.percentage,
    throttleMs, 
    maxTrailPoints,
    smoothedHeading,
    targetHeading,
  ]);

  // Expose methods for external control
  const clearTrail = () => {
    if (trailRef.current) {
      pointsRef.current = [];
      trailRef.current.setLatLngs([]);
      console.log('[useMapTelemetry] 🧹 Trail cleared');
    }
  };

  return {
    clearTrail,
  };
}

export default useMapTelemetry;
