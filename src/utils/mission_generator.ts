// File: src/utils/mission_generator.ts

import { Waypoint } from '../types';
import { calculateDistance, calculateBearing } from './geo';

/**
 * Utility: Convert meters to latitude/longitude offset.
 */
function metersToLatLng(lat: number, dx: number, dy: number): { lat: number; lng: number } {
  const R = 6378137; // Earth radius in meters
  const newLat = lat + (dy / R) * (180 / Math.PI);
  const newLng = lat + (dx / (R * Math.cos((Math.PI * lat) / 180))) * (180 / Math.PI);
  return { lat: newLat, lng: newLng };
}

/**
 * Utility: Convert offset in meters at given bearing and distance to lat/lng.
 */
function offsetLatLng(lat: number, lng: number, distanceM: number, bearingDeg: number): { lat: number; lng: number } {
  const R = 6378137; // Earth radius
  const brng = (bearingDeg * Math.PI) / 180;
  const dByR = distanceM / R;

  const lat1 = (lat * Math.PI) / 180;
  const lng1 = (lng * Math.PI) / 180;

  const lat2 = Math.asin(Math.sin(lat1) * Math.cos(dByR) + Math.cos(lat1) * Math.sin(dByR) * Math.cos(brng));
  const lng2 = lng1 + Math.atan2(Math.sin(brng) * Math.sin(dByR) * Math.cos(lat1), Math.cos(dByR) - Math.sin(lat1) * Math.sin(lat2));

  return { lat: (lat2 * 180) / Math.PI, lng: (lng2 * 180) / Math.PI };
}

/**
 * 1️⃣ Generate circular mission around a center point.
 * @param center { lat, lng } - Circle center
 * @param radiusM - radius in meters
 * @param numPoints - number of waypoints in circle
 * @param altitude - waypoint altitude (m)
 */
export function generateCircleMission(center: { lat: number; lng: number }, radiusM: number, numPoints: number, altitude = 30): Waypoint[] {
  const waypoints: Waypoint[] = [];
  for (let i = 0; i < numPoints; i++) {
    const angleDeg = (i / numPoints) * 360;
    const pt = offsetLatLng(center.lat, center.lng, radiusM, angleDeg);
    waypoints.push({
      id: i + 1,
  lat: parseFloat(Number(pt.lat).toFixed(7)),
  lng: parseFloat(Number(pt.lng).toFixed(7)),
      alt: altitude,
      frame: 3,
      command: 'WAYPOINT',
      current: i === 0 ? 1 : 0,
      autocontinue: 1,
      param1: 0,
      param2: 0,
      param3: 0,
      param4: 0,
      action: 'NONE'
    });
  }
  // Close the circle by repeating first point
  waypoints.push({ ...waypoints[0], id: numPoints + 1 });
  return waypoints;
}

/**
 * 2️⃣ Generate simple rectangular grid between two corners.
 * @param start - southwest corner
 * @param end - northeast corner
 * @param spacingM - distance between rows (m)
 * @param altitude - altitude (m)
 * @param angleDeg - heading of grid (deg)
 */
export function generateGridMission(start: { lat: number; lng: number }, end: { lat: number; lng: number }, spacingM = 5, altitude = 30, angleDeg = 0): Waypoint[] {
  const waypoints: Waypoint[] = [];
  const latSpan = end.lat - start.lat;
  const lngSpan = end.lng - start.lng;

  const numRows = Math.max(2, Math.floor(calculateDistance(start, { lat: end.lat, lng: start.lng }) / spacingM));
  const numCols = Math.max(2, Math.floor(calculateDistance(start, { lat: start.lat, lng: end.lng }) / spacingM));

  let id = 1;
  for (let row = 0; row < numRows; row++) {
    const rowLat = start.lat + (latSpan * row) / (numRows - 1);
    const rowWaypoints: Waypoint[] = [];
    for (let col = 0; col < numCols; col++) {
      const colLng = start.lng + (lngSpan * col) / (numCols - 1);
      rowWaypoints.push({
        id,
  lat: parseFloat(Number(rowLat).toFixed(7)),
  lng: parseFloat(Number(colLng).toFixed(7)),
        alt: altitude,
        frame: 3,
        command: 'WAYPOINT',
        current: id === 1 ? 1 : 0,
        autocontinue: 1,
        param1: 0,
        param2: 0,
        param3: 0,
        param4: 0,
        action: 'NONE'
      });
      id++;
    }
    if (row % 2 === 1) rowWaypoints.reverse(); // lawnmower pattern
    waypoints.push(...rowWaypoints);
  }

  return waypoints;
}

/**
 * 3️⃣ Generate polygon sweep pattern (for irregular area coverage).
 * @param polygon - array of lat/lng vertices (closed)
 * @param spacingM - spacing between sweep lines
 * @param altitude - altitude (m)
 * @param directionDeg - sweep direction (deg)
 */
export function generatePolygonSweep(polygon: { lat: number; lng: number }[], spacingM = 5, altitude = 30, directionDeg = 90): Waypoint[] {
  // --- Simplified algorithm (first version): create bounding box + parallel lines
  if (polygon.length < 3) throw new Error('Polygon must have at least 3 vertices');
  
  const bounds = {
    minLat: Math.min(...polygon.map(p => p.lat)),
    maxLat: Math.max(...polygon.map(p => p.lat)),
    minLng: Math.min(...polygon.map(p => p.lng)),
    maxLng: Math.max(...polygon.map(p => p.lng)),
  };

  const width = calculateDistance({ lat: bounds.minLat, lng: bounds.minLng }, { lat: bounds.minLat, lng: bounds.maxLng });
  const numLines = Math.max(2, Math.floor(width / spacingM));
  const waypoints: Waypoint[] = [];

  let id = 1;
  for (let i = 0; i < numLines; i++) {
    const offset = (i / (numLines - 1)) * (bounds.maxLng - bounds.minLng);
    const line: Waypoint[] = [];
    polygon.forEach((vertex, idx) => {
      const next = polygon[(idx + 1) % polygon.length];
      const midLng = vertex.lng + offset;
      const midLat = vertex.lat;
      line.push({
        id,
        lat: midLat,
        lng: midLng,
        alt: altitude,
        frame: 3,
        command: 'WAYPOINT',
        current: id === 1 ? 1 : 0,
        autocontinue: 1,
        param1: 0,
        param2: 0,
        param3: 0,
        param4: 0,
        action: 'NONE'
      });
      id++;
    });
    if (i % 2 === 1) line.reverse(); // alternate sweep
    waypoints.push(...line);
  }
  return waypoints;
}
