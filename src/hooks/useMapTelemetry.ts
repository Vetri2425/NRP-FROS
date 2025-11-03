import { useEffect, useRef } from 'react';
// Use global Leaflet instance to match MapView setup
declare var L: any;
import { useRover } from '../context/RoverContext';
import { createPathTrail } from '../utils/leaflet-helpers';
import RoverMarker, { createRoverMarker } from '../components/map/RoverMarker';

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

  // Initialize marker and trail
  useEffect(() => {
    const map = mapRef.current;
    const vehicleLayer = vehicleLayerRef.current;
    const trailLayer = trailLayerRef.current;
    if (!map || !vehicleLayer || !trailLayer) return;

    if (!markerRef.current && roverPosition) {
      const status: 'armed' | 'disarmed' | 'rtk' = telemetry.state?.armed
        ? 'armed'
        : (telemetry.rtk?.fix_type ?? 0) >= 4
          ? 'rtk'
          : 'disarmed';
      markerRef.current = createRoverMarker(roverPosition.lat, roverPosition.lng, {
        heading: 0,
        altitude: telemetry.global?.alt_rel ?? 0,
        status,
        zIndexOffset: 1000,
      }).addTo(vehicleLayer);
      // Initial tooltip
      const tt = `Lat: ${roverPosition.lat.toFixed(6)}\nLon: ${roverPosition.lng.toFixed(6)}\nAlt: ${(telemetry.global?.alt_rel ?? 0).toFixed(1)} m\nHeading: ${(0).toFixed(1)}°`;
      markerRef.current.bindTooltip(tt, { permanent: false, direction: 'top', offset: [0, -10] });
    }

    if (!trailRef.current) {
      trailRef.current = createPathTrail([], '#0ea5e9').addTo(trailLayer);
    }

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
  }, [mapRef, vehicleLayerRef, trailLayerRef, roverPosition, telemetry.state?.armed, telemetry.rtk?.fix_type, telemetry.global?.alt_rel]);


  // Throttled, non-blocking updates using requestAnimationFrame
  useEffect(() => {
    const pos = roverPosition;
    if (!pos) return;
    const marker = markerRef.current;
    const trail = trailRef.current;
    if (!marker || !trail) return;

    const now = performance.now();
    if (now - lastEmitRef.current < throttleMs) return;
    lastEmitRef.current = now;

    // Marker position
    marker.setLatLng([pos.lat, pos.lng]);

    // Heading (ATTITUDE heading may come elsewhere; compute fallback using velocity vector if needed)
    const heading = telemetry.attitude?.yaw_deg ?? undefined;
    if (typeof heading === 'number' && isFinite(heading)) {
      marker.setHeading(heading);
    }

    // Update status/altitude visuals
    const status: 'armed' | 'disarmed' | 'rtk' = telemetry.state?.armed
      ? 'armed'
      : (telemetry.rtk?.fix_type ?? 0) >= 4
        ? 'rtk'
        : 'disarmed';
    marker.setStatus(status);
    marker.setAltitude(telemetry.global?.alt_rel ?? 0);

    // Trail history with culling
    const last = pointsRef.current[pointsRef.current.length - 1];
    const p: [number, number] = [pos.lat, pos.lng];
    if (!last || Math.abs(last[0] - p[0]) > 1e-6 || Math.abs(last[1] - p[1]) > 1e-6) {
      pointsRef.current.push(p);
      if (pointsRef.current.length > maxTrailPoints) {
        pointsRef.current.splice(0, pointsRef.current.length - maxTrailPoints);
      }
      trail.setLatLngs(pointsRef.current);
    }

    // Update tooltip
    const currentAlt = telemetry.global?.alt_rel ?? 0;
    const currentHdg = telemetry.attitude?.yaw_deg ?? 0;
    const content = `Lat: ${pos.lat.toFixed(6)}\nLon: ${pos.lng.toFixed(6)}\nAlt: ${currentAlt.toFixed(1)} m\nHeading: ${currentHdg.toFixed(1)}°`;
    const tooltip = marker.getTooltip?.();
    if (tooltip) {
      tooltip.setContent(content);
    } else {
      marker.bindTooltip?.(content, { permanent: false, direction: 'top', offset: [0, -10] });
    }
  }, [roverPosition, telemetry, throttleMs]);
}

export default useMapTelemetry;
