import { describe, it, expect } from 'vitest';
import {
  validateWaypoints,
  hasCriticalErrors,
  getCriticalErrors,
  getWarnings,
  formatValidationErrors,
  sanitizeWaypointsForUpload,
} from '../utils/waypointValidator';
import { Waypoint } from '../types';

describe('waypointValidator', () => {
  describe('validateWaypoints', () => {
    it('should pass validation for valid waypoints', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
        { id: 2, lat: 40.7129, lng: -74.007, alt: 60, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(errors).toHaveLength(0);
    });

    it('should detect invalid latitude (NaN)', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: NaN, lng: -74.006, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(errors).toHaveLength(1);
      expect(errors[0].code).toBe('INVALID_LAT');
      expect(errors[0].type).toBe('error');
    });

    it('should detect invalid longitude (NaN)', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: NaN, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(errors).toHaveLength(1);
      expect(errors[0].code).toBe('INVALID_LNG');
      expect(errors[0].type).toBe('error');
    });

    it('should detect latitude out of range', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 200, lng: -74.006, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const latErrors = errors.filter(e => e.code === 'LAT_OUT_OF_RANGE');
      expect(latErrors).toHaveLength(1);
      expect(latErrors[0].type).toBe('error');
    });

    it('should detect longitude out of range', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -200, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const lngErrors = errors.filter(e => e.code === 'LNG_OUT_OF_RANGE');
      expect(lngErrors).toHaveLength(1);
      expect(lngErrors[0].type).toBe('error');
    });

    it('should detect missing altitude', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: undefined as any, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(errors.some(e => e.code === 'MISSING_ALTITUDE')).toBe(true);
      expect(errors.find(e => e.code === 'MISSING_ALTITUDE')?.type).toBe('error');
    });

    it('should warn about negative altitude (not block upload)', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: -10, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const negAltWarnings = errors.filter(e => e.code === 'NEGATIVE_ALTITUDE');
      expect(negAltWarnings).toHaveLength(1);
      expect(negAltWarnings[0].type).toBe('warning');
      expect(negAltWarnings[0].message).toContain('Will be converted to positive');
    });

    it('should warn about unusually high altitude', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: 15000, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const altWarnings = errors.filter(e => e.code === 'ALTITUDE_UNUSUALLY_HIGH');
      expect(altWarnings).toHaveLength(1);
      expect(altWarnings[0].type).toBe('warning');
    });

    it('should warn about zero altitude', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: 0, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const zeroAltWarnings = errors.filter(e => e.code === 'ZERO_ALTITUDE');
      expect(zeroAltWarnings).toHaveLength(1);
      expect(zeroAltWarnings[0].type).toBe('warning');
    });

    it('should detect duplicate waypoints', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.712800, lng: -74.006000, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
        { id: 2, lat: 40.712800, lng: -74.006000, alt: 60, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const duplicates = errors.filter(e => e.code === 'DUPLICATE_WAYPOINT');
      expect(duplicates).toHaveLength(1);
      expect(duplicates[0].type).toBe('warning');
      expect(duplicates[0].waypointIndex).toBe(1); // Second occurrence
    });

    it('should warn about null island (0, 0)', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 0, lng: 0, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const nullIslandWarnings = errors.filter(e => e.code === 'NULL_ISLAND');
      expect(nullIslandWarnings).toHaveLength(1);
      expect(nullIslandWarnings[0].type).toBe('warning');
    });

    it('should detect multiple errors in a single waypoint', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 200, lng: -200, alt: -10, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(errors.length).toBeGreaterThan(2); // Multiple errors expected
      expect(errors.some(e => e.code === 'LAT_OUT_OF_RANGE')).toBe(true);
      expect(errors.some(e => e.code === 'LNG_OUT_OF_RANGE')).toBe(true);
      expect(errors.some(e => e.code === 'NEGATIVE_ALTITUDE')).toBe(true);
    });
  });

  describe('hasCriticalErrors', () => {
    it('should return true when critical errors exist', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 200, lng: -74.006, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(hasCriticalErrors(errors)).toBe(true);
    });

    it('should return false when only warnings exist', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: 15000, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      expect(hasCriticalErrors(errors)).toBe(false);
    });
  });

  describe('getCriticalErrors', () => {
    it('should filter only critical errors', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 200, lng: -74.006, alt: 15000, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const criticalErrors = getCriticalErrors(errors);
      
      expect(criticalErrors.every(e => e.type === 'error')).toBe(true);
      expect(criticalErrors.some(e => e.code === 'LAT_OUT_OF_RANGE')).toBe(true);
    });
  });

  describe('getWarnings', () => {
    it('should filter only warnings', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: 15000, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const warnings = getWarnings(errors);
      
      expect(warnings.every(e => e.type === 'warning')).toBe(true);
      expect(warnings.some(e => e.code === 'ALTITUDE_UNUSUALLY_HIGH')).toBe(true);
    });
  });

  describe('formatValidationErrors', () => {
    it('should format errors into human-readable string', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 200, lng: -74.006, alt: 50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const formatted = formatValidationErrors(errors);
      
      expect(formatted).toContain('error');
      expect(formatted).toContain('Waypoint 0');
    });

    it('should limit the number of errors shown', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 200, lng: -200, alt: -10, command: 'WAYPOINT', frame: 3, action: 'NONE' },
        { id: 2, lat: 200, lng: -200, alt: -10, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const errors = validateWaypoints(waypoints);
      const formatted = formatValidationErrors(errors, 2);
      
      expect(formatted).toBeTruthy();
      expect(formatted.split('\n').length).toBeLessThan(errors.length + 5);
    });
  });

  describe('sanitizeWaypointsForUpload', () => {
    it('should convert negative altitudes to positive', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: -50, command: 'WAYPOINT', frame: 3, action: 'NONE' },
        { id: 2, lat: 40.7138, lng: -74.007, alt: 100, command: 'WAYPOINT', frame: 3, action: 'NONE' },
        { id: 3, lat: 40.7148, lng: -74.008, alt: -25.5, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const sanitized = sanitizeWaypointsForUpload(waypoints);
      
      expect(sanitized[0].alt).toBe(50);
      expect(sanitized[1].alt).toBe(100);
      expect(sanitized[2].alt).toBe(25.5);
    });

    it('should not modify waypoints without negative altitudes', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: 100, command: 'WAYPOINT', frame: 3, action: 'NONE' },
        { id: 2, lat: 40.7138, lng: -74.007, alt: 200, command: 'WAYPOINT', frame: 3, action: 'NONE' },
      ];

      const sanitized = sanitizeWaypointsForUpload(waypoints);
      
      expect(sanitized).toEqual(waypoints);
    });

    it('should preserve all other waypoint properties', () => {
      const waypoints: Waypoint[] = [
        { id: 1, lat: 40.7128, lng: -74.006, alt: -50, command: 'LOITER', frame: 3, action: 'LAND', param1: 10 },
      ];

      const sanitized = sanitizeWaypointsForUpload(waypoints);
      
      expect(sanitized[0].id).toBe(1);
      expect(sanitized[0].lat).toBe(40.7128);
      expect(sanitized[0].lng).toBe(-74.006);
      expect(sanitized[0].command).toBe('LOITER');
      expect(sanitized[0].frame).toBe(3);
      expect(sanitized[0].action).toBe('LAND');
      expect(sanitized[0].param1).toBe(10);
      expect(sanitized[0].alt).toBe(50); // Converted to positive
    });
  });
});
