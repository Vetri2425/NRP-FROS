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
        : (telemetry.rtk?.fix_type ?? 0) >= 4
          ? 'rtk'
          : 'disarmed';
      
      markerRef.current = createRoverMarker(roverPosition.lat, roverPosition.lng, {
        heading: smoothedHeading,
        altitude: telemetry.global?.alt_rel ?? 0,
        status,
        zIndexOffset: 1000,
      }).addTo(vehicleLayer);
      
      const tt = `Lat: ${roverPosition.lat.toFixed(6)}\nLon: ${roverPosition.lng.toFixed(6)}\nAlt: ${(telemetry.global?.alt_rel ?? 0).toFixed(1)} m\nHeading: ${(smoothedHeading).toFixed(1)}°`;
      markerRef.current.bindTooltip(tt, { permanent: false, direction: 'top', offset: [0, -10] });
      
      console.log('[useMapTelemetry] ✅ Rover marker created at:', { 
        lat: roverPosition.lat, 
        lng: roverPosition.lng,
        status 
      });
    }

    // Only create trail if we don't have one
    if (!trailRef.current) {
      trailRef.current = createPathTrail([], '#0ea5e9').addTo(trailLayer);
      console.log('[useMapTelemetry] ✅ Trail layer created');
    }

    // Cleanup ONLY when component unmounts or layers change
    return () => {
      if (markerRef.current) {
        markerRef.current.remove();
        markerRef.current = null;
      }
      if (trailRef.current) {
        trailRef.current.remove();
        trailRef.current = null;
      }
    };
  }, [mapRef, vehicleLayerRef, trailLayerRef, roverPosition]);


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

      // Update tooltip
      const status: 'armed' | 'disarmed' | 'rtk' = telemetry.state?.armed
        ? 'armed'
        : (telemetry.rtk?.fix_type ?? 0) >= 4
          ? 'rtk'
          : 'disarmed';
      const tt = `Lat: ${pos.lat.toFixed(6)}\nLon: ${pos.lng.toFixed(6)}\nAlt: ${(telemetry.global?.alt_rel ?? 0).toFixed(1)} m\nHeading: ${smoothedHeading.toFixed(1)}°`;
      marker.setTooltipContent(tt);
      marker.updateStatus(status);

      // Add to trail if position changed
      const lastPoint = pointsRef.current[pointsRef.current.length - 1];
      if (!lastPoint || lastPoint[0] !== pos.lat || lastPoint[1] !== pos.lng) {
        pointsRef.current.push([pos.lat, pos.lng]);
        if (pointsRef.current.length > maxTrailPoints) {
          pointsRef.current.shift();
        }
        trail.setLatLngs(pointsRef.current);
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
    throttleMs, 
    maxTrailPoints,
    smoothedHeading,
    targetHeading,
  ]);
}

export default useMapTelemetry;
