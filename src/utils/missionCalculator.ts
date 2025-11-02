import { Waypoint } from '../types';

/**
 * Calculate distance between two waypoints using Haversine formula
 */
export const calculateDistance = (
  wp1: { lat: number; lng: number },
  wp2: { lat: number; lng: number }
): number => {
  const R = 6371000; // Earth radius in meters
  const lat1 = (wp1.lat * Math.PI) / 180;
  const lat2 = (wp2.lat * Math.PI) / 180;
  const deltaLat = ((wp2.lat - wp1.lat) * Math.PI) / 180;
  const deltaLng = ((wp2.lng - wp1.lng) * Math.PI) / 180;

  const a =
    Math.sin(deltaLat / 2) * Math.sin(deltaLat / 2) +
    Math.cos(lat1) *
      Math.cos(lat2) *
      Math.sin(deltaLng / 2) *
      Math.sin(deltaLng / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
};

/**
 * Calculate total mission distance
 */
export const calculateMissionDistance = (waypoints: Waypoint[]): number => {
  if (waypoints.length < 2) return 0;

  let totalDistance = 0;
  for (let i = 1; i < waypoints.length; i++) {
    totalDistance += calculateDistance(waypoints[i - 1], waypoints[i]);
  }
  return totalDistance;
};

/**
 * Calculate estimated flight time
 */
export const calculateEstimatedFlightTime = (
  distance: number,
  roverSpeed: number = 1.0 // meters per second
): number => {
  if (roverSpeed <= 0) return 0;
  return distance / roverSpeed;
};

/**
 * Calculate mission bounds
 */
export const calculateMissionBounds = (waypoints: Waypoint[]) => {
  if (waypoints.length === 0) {
    return { north: 0, south: 0, east: 0, west: 0 };
  }

  let north = waypoints[0].lat;
  let south = waypoints[0].lat;
  let east = waypoints[0].lng;
  let west = waypoints[0].lng;

  for (const wp of waypoints) {
    north = Math.max(north, wp.lat);
    south = Math.min(south, wp.lat);
    east = Math.max(east, wp.lng);
    west = Math.min(west, wp.lng);
  }

  return { north, south, east, west };
};

/**
 * Get altitude range
 */
export const getAltitudeRange = (waypoints: Waypoint[]) => {
  if (waypoints.length === 0) {
    return { min: 0, max: 0 };
  }

  let min = waypoints[0].alt;
  let max = waypoints[0].alt;

  for (const wp of waypoints) {
    min = Math.min(min, wp.alt);
    max = Math.max(max, wp.alt);
  }

  return { min, max };
};

/**
 * Format time for display
 */
export const formatFlightTime = (seconds: number): string => {
  if (seconds < 60) {
    return `${Math.ceil(seconds)} second${Math.ceil(seconds) !== 1 ? 's' : ''}`;
  }

  if (seconds < 3600) {
    const minutes = Math.floor(seconds / 60);
    const secs = Math.ceil(seconds % 60);
    return `${minutes}m ${secs}s`;
  }

  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  return `${hours}h ${minutes}m`;
};

/**
 * Format distance for display
 */
export const formatDistance = (meters: number): string => {
  if (meters < 1000) {
    return `${Math.round(meters)} m`;
  }
  return `${(meters / 1000).toFixed(2)} km`;
};

/**
 * Get mission summary statistics
 */
export interface MissionStatistics {
  waypointCount: number;
  totalDistance: number;
  estimatedTime: number;
  altitudeRange: { min: number; max: number };
  boundingBox: { north: number; south: number; east: number; west: number };
}

export const calculateMissionStatistics = (
  waypoints: Waypoint[]
): MissionStatistics => {
  return {
    waypointCount: waypoints.length,
    totalDistance: calculateMissionDistance(waypoints),
    estimatedTime: calculateEstimatedFlightTime(
      calculateMissionDistance(waypoints),
      1.0 // default 1 m/s
    ),
    altitudeRange: getAltitudeRange(waypoints),
    boundingBox: calculateMissionBounds(waypoints)
  };
};
