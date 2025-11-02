import { describe, it, expect } from 'vitest';
import {
  calculateDistance,
  calculateMissionDistance,
  calculateEstimatedFlightTime,
  calculateMissionBounds,
  getAltitudeRange,
  formatFlightTime,
  formatDistance,
  calculateMissionStatistics
} from '../utils/missionCalculator';
import { Waypoint } from '../types';

describe('missionCalculator', () => {
  // Sample waypoints for testing
  const createWaypoint = (lat: number, lng: number, alt: number): Waypoint => ({
    id: 1,
    lat,
    lng,
    alt,
    command: 'WAYPOINT',
    frame: 0
  });

  describe('calculateDistance', () => {
    it('should calculate distance between two points (Haversine formula)', () => {
      // Distance from New York to Los Angeles (approx 3,944 km)
      const ny = { lat: 40.7128, lng: -74.0060 };
      const la = { lat: 34.0522, lng: -118.2437 };
      
      const distance = calculateDistance(ny, la);
      
      // Should be approximately 3,944,000 meters
      expect(distance).toBeGreaterThan(3900000);
      expect(distance).toBeLessThan(4000000);
    });

    it('should return 0 for same coordinates', () => {
      const point = { lat: 40.7128, lng: -74.0060 };
      const distance = calculateDistance(point, point);
      expect(distance).toBe(0);
    });

    it('should handle equator crossing', () => {
      const point1 = { lat: 10, lng: 0 };
      const point2 = { lat: -10, lng: 0 };
      
      const distance = calculateDistance(point1, point2);
      
      // Should be approximately 2,222 km (20 degrees at equator)
      expect(distance).toBeGreaterThan(2000000);
      expect(distance).toBeLessThan(2500000);
    });
  });

  describe('calculateMissionDistance', () => {
    it('should calculate total distance for multiple waypoints', () => {
      const waypoints: Waypoint[] = [
        createWaypoint(0, 0, 100),
        createWaypoint(0.01, 0, 100),
        createWaypoint(0.01, 0.01, 100),
        createWaypoint(0, 0.01, 100)
      ];

      const distance = calculateMissionDistance(waypoints);
      
      // Should be greater than 0
      expect(distance).toBeGreaterThan(0);
      
      // Rough square, total ~3.3 km
      expect(distance).toBeGreaterThan(3000);
      expect(distance).toBeLessThan(4000);
    });

    it('should return 0 for empty waypoints', () => {
      expect(calculateMissionDistance([])).toBe(0);
    });

    it('should return 0 for single waypoint', () => {
      const waypoints = [createWaypoint(0, 0, 100)];
      expect(calculateMissionDistance(waypoints)).toBe(0);
    });
  });

  describe('calculateEstimatedFlightTime', () => {
    it('should calculate flight time at default speed (1 m/s)', () => {
      const distance = 1100; // meters
      const time = calculateEstimatedFlightTime(distance);
      
      // 1100m / 1m/s = 1100 seconds
      expect(time).toBe(1100);
    });

    it('should calculate flight time at custom speed', () => {
      const distance = 1100; // meters
      const time = calculateEstimatedFlightTime(distance, 10);
      
      // 1100m / 10m/s = 110 seconds
      expect(time).toBe(110);
    });

    it('should return 0 for zero distance', () => {
      expect(calculateEstimatedFlightTime(0)).toBe(0);
    });

    it('should return 0 for invalid speed', () => {
      expect(calculateEstimatedFlightTime(1000, 0)).toBe(0);
      expect(calculateEstimatedFlightTime(1000, -5)).toBe(0);
    });
  });

  describe('calculateMissionBounds', () => {
    it('should calculate correct bounding box', () => {
      const waypoints: Waypoint[] = [
        createWaypoint(10, 20, 100),
        createWaypoint(15, 25, 100),
        createWaypoint(12, 22, 100)
      ];

      const bounds = calculateMissionBounds(waypoints);
      
      expect(bounds.south).toBe(10);
      expect(bounds.north).toBe(15);
      expect(bounds.west).toBe(20);
      expect(bounds.east).toBe(25);
    });

    it('should handle single waypoint', () => {
      const waypoints = [createWaypoint(10, 20, 100)];
      const bounds = calculateMissionBounds(waypoints);
      
      expect(bounds.south).toBe(10);
      expect(bounds.north).toBe(10);
      expect(bounds.west).toBe(20);
      expect(bounds.east).toBe(20);
    });

    it('should handle negative coordinates', () => {
      const waypoints: Waypoint[] = [
        createWaypoint(-10, -20, 100),
        createWaypoint(-15, -25, 100)
      ];

      const bounds = calculateMissionBounds(waypoints);
      
      expect(bounds.south).toBe(-15);
      expect(bounds.north).toBe(-10);
      expect(bounds.west).toBe(-25);
      expect(bounds.east).toBe(-20);
    });

    it('should return zero bounds for empty waypoints', () => {
      const bounds = calculateMissionBounds([]);
      expect(bounds).toEqual({ north: 0, south: 0, east: 0, west: 0 });
    });
  });

  describe('getAltitudeRange', () => {
    it('should calculate altitude range', () => {
      const waypoints: Waypoint[] = [
        createWaypoint(0, 0, 100),
        createWaypoint(0, 0, 250),
        createWaypoint(0, 0, 150)
      ];

      const range = getAltitudeRange(waypoints);
      
      expect(range.min).toBe(100);
      expect(range.max).toBe(250);
    });

    it('should handle single waypoint', () => {
      const waypoints = [createWaypoint(0, 0, 100)];
      const range = getAltitudeRange(waypoints);
      
      expect(range.min).toBe(100);
      expect(range.max).toBe(100);
    });

    it('should return zero range for empty waypoints', () => {
      expect(getAltitudeRange([])).toEqual({ min: 0, max: 0 });
    });
  });

  describe('formatFlightTime', () => {
    it('should format seconds correctly', () => {
      expect(formatFlightTime(45)).toBe('45 seconds');
    });

    it('should format single second', () => {
      expect(formatFlightTime(1)).toBe('1 second');
    });

    it('should format minutes and seconds', () => {
      expect(formatFlightTime(125)).toBe('2m 5s');
    });

    it('should format hours and minutes', () => {
      expect(formatFlightTime(3665)).toBe('1h 1m');
    });

    it('should handle zero', () => {
      expect(formatFlightTime(0)).toBe('0 seconds');
    });
  });

  describe('formatDistance', () => {
    it('should format meters for distances under 1 km', () => {
      expect(formatDistance(500)).toBe('500 m');
    });

    it('should format kilometers for distances over 1 km', () => {
      expect(formatDistance(1500)).toBe('1.50 km');
    });

    it('should format with 2 decimal places', () => {
      expect(formatDistance(12345)).toBe('12.35 km');
    });

    it('should handle zero', () => {
      expect(formatDistance(0)).toBe('0 m');
    });
  });

  describe('calculateMissionStatistics', () => {
    it('should calculate all statistics', () => {
      const waypoints: Waypoint[] = [
        createWaypoint(0, 0, 100),
        createWaypoint(0.01, 0, 150),
        createWaypoint(0.01, 0.01, 200)
      ];

      const stats = calculateMissionStatistics(waypoints);
      
      expect(stats.waypointCount).toBe(3);
      expect(stats.totalDistance).toBeGreaterThan(0);
      expect(stats.estimatedTime).toBeGreaterThan(0);
      expect(stats.altitudeRange).toEqual({ min: 100, max: 200 });
      expect(stats.boundingBox).not.toBeNull();
      expect(stats.boundingBox.south).toBe(0);
      expect(stats.boundingBox.north).toBe(0.01);
    });

    it('should handle empty waypoints', () => {
      const stats = calculateMissionStatistics([]);
      
      expect(stats.waypointCount).toBe(0);
      expect(stats.totalDistance).toBe(0);
      expect(stats.estimatedTime).toBe(0);
      expect(stats.altitudeRange).toEqual({ min: 0, max: 0 });
      expect(stats.boundingBox).toEqual({ north: 0, south: 0, east: 0, west: 0 });
    });
  });
});
