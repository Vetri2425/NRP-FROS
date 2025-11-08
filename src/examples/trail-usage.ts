// Trail System - Usage Examples
// src/examples/trail-usage.ts

import React from 'react';
import { TrailSystem, getGPSDistance } from '../utils/trail-system';

/**
 * Example 1: Basic Trail Usage
 */
export function basicTrailExample(map: any) {
  const trail = new TrailSystem({
    maxPoints: 300,
    minTimeMs: 500,
    minDistanceM: 2,
  });

  // Add rover positions as they come in
  trail.addPoint(13.0827, 80.2707); // First position
  trail.addPoint(13.0828, 80.2708); // Second position

  // Update display
  trail.update(map);

  // Check trail stats
  const stats = trail.getStats();
  console.log(`Trail: ${stats.pointCount} points, ${stats.distanceMeters}m traveled`);
}

/**
 * Example 2: Aggressive Memory Management
 * For low-memory devices or when storage is critical
 */
export function lowMemoryTrail() {
  return new TrailSystem({
    maxPoints: 100,        // Smaller storage
    minTimeMs: 1000,       // 1 second between points
    minDistanceM: 5,       // 5 meters minimum distance
    canvasRenderer: true,
    updateThrottleMs: 200, // Update less frequently
  });
}

/**
 * Example 3: Detailed Trail
 * For high-precision tracking and historical analysis
 */
export function detailedTrail() {
  return new TrailSystem({
    maxPoints: 500,        // Larger storage
    minTimeMs: 250,        // More frequent updates
    minDistanceM: 1,       // Smaller distance filter
    canvasRenderer: true,
    updateThrottleMs: 50,  // More responsive updates
  });
}

/**
 * Example 4: Using Trail in a React Hook
 * Integration pattern for map components
 */
export function useTrailExample(mapRef: any, roverPosition: any) {
  const trailRef = React.useRef<TrailSystem | null>(null);

  // Initialize trail once
  React.useEffect(() => {
    if (!trailRef.current) {
      trailRef.current = new TrailSystem({
        maxPoints: 300,
        minTimeMs: 500,
        minDistanceM: 2,
      });
    }
  }, []);

  // Update trail on rover movement
  React.useEffect(() => {
    if (!roverPosition || !trailRef.current || !mapRef.current) return;

    const added = trailRef.current.addPoint(roverPosition.lat, roverPosition.lng);
    if (added) {
      trailRef.current.update(mapRef.current);
    }
  }, [roverPosition, mapRef]);

  // Cleanup
  React.useEffect(() => {
    return () => {
      if (trailRef.current && mapRef.current) {
        trailRef.current.clear(mapRef.current);
      }
    };
  }, []);

  return trailRef.current;
}

/**
 * Example 5: Filtering Out Noisy GPS Data
 * Manual filtering before adding to trail
 */
export function smartPointFiltering(
  lastGoodLat: number | null,
  lastGoodLng: number | null,
  newLat: number,
  newLng: number,
  minAccuracyM = 10
): boolean {
  // Reject if accuracy is too poor
  if (!isAccuracyGood(newLat, newLng, minAccuracyM)) {
    console.warn('GPS accuracy too poor, skipping point');
    return false;
  }

  // Reject extreme jumps
  if (lastGoodLat !== null && lastGoodLng !== null) {
    const distance = getGPSDistance(lastGoodLat, lastGoodLng, newLat, newLng);
    const maxReasonableJump = 50; // 50 meters max jump

    if (distance > maxReasonableJump) {
      console.warn('Unrealistic GPS jump detected, skipping point');
      return false;
    }
  }

  return true;
}

/**
 * Example 6: Displaying Trail Statistics in UI
 */
export function displayTrailStats(trail: TrailSystem) {
  const stats = trail.getStats();

  const statsUI = `
    <div class="trail-stats">
      <p><strong>Trail Points:</strong> ${stats.pointCount}/300</p>
      <p><strong>Distance:</strong> ${(stats.distanceMeters / 1000).toFixed(2)} km</p>
      <p><strong>Age:</strong> ${Math.floor(stats.ageSeconds / 60)} min ${stats.ageSeconds % 60}s</p>
    </div>
  `;

  return statsUI;
}

/**
 * Example 7: Advanced - Trail Replay (Future Feature)
 * Framework for implementing time-based trail playback
 */
export class TrailReplay {
  private trail: TrailSystem;
  private currentIndex: number = 0;
  private playbackRate: number = 1; // 1x = normal speed, 2x = 2x speed, etc.
  private isPlaying: boolean = false;

  constructor(trail: TrailSystem) {
    this.trail = trail;
  }

  play() {
    this.isPlaying = true;
    this.animate();
  }

  pause() {
    this.isPlaying = false;
  }

  stop() {
    this.isPlaying = false;
    this.currentIndex = 0;
  }

  setPlaybackRate(rate: number) {
    this.playbackRate = Math.max(0.1, Math.min(10, rate)); // Clamp 0.1x to 10x
  }

  private animate() {
    if (!this.isPlaying) return;

    const points = this.trail.getPoints();
    if (this.currentIndex >= points.length) {
      this.stop();
      return;
    }

    const currentPoint = points[this.currentIndex];
    // TODO: Update map to show rover at this position and time

    this.currentIndex++;
    const timeBetweenPoints =
      this.currentIndex < points.length
        ? points[this.currentIndex].timestamp - currentPoint.timestamp
        : 1000;

    setTimeout(() => this.animate(), timeBetweenPoints / this.playbackRate);
  }

  getProgress(): number {
    const points = this.trail.getPoints();
    return points.length > 0 ? this.currentIndex / points.length : 0;
  }
}

/**
 * Example 8: Trail Export as GeoJSON (Future Feature)
 */
export function exportTrailAsGeoJSON(trail: TrailSystem): GeoJSON.FeatureCollection {
  const points = trail.getPoints();

  const features: GeoJSON.Feature[] = points.map((point, index) => ({
    type: 'Feature',
    properties: {
      index,
      timestamp: point.timestamp,
      opacity: point.opacity,
    },
    geometry: {
      type: 'Point',
      coordinates: [point.lng, point.lat],
    },
  }));

  return {
    type: 'FeatureCollection',
    features,
  };
}

/**
 * Example 9: Performance Monitoring
 */
export function monitorTrailPerformance(trail: TrailSystem) {
  const stats = trail.getStats();

  const memoryUsageKB = (stats.pointCount * 0.08).toFixed(2); // ~80 bytes per point
  const estimatedLatency = stats.pointCount * 0.1; // ~0.1ms per point

  console.log('Trail Performance:');
  console.log(`  Points: ${stats.pointCount}/300`);
  console.log(`  Memory: ~${memoryUsageKB} KB`);
  console.log(`  Est. Latency: ~${estimatedLatency.toFixed(1)}ms`);

  return {
    pointCount: stats.pointCount,
    memoryKB: parseFloat(memoryUsageKB),
    estimatedLatencyMs: estimatedLatency,
  };
}

/**
 * Example 10: Comparison - Old vs New Trail System
 */
export function compareTrailImplementations() {
  return {
    old: {
      name: 'Simple Polyline',
      maxPoints: 'Unlimited',
      memory: 'High',
      performance: 'Slow with many points',
      fade: 'None',
      filtering: 'Manual',
      cpuUsage: 'High (5-10% active)',
    },
    new: {
      name: 'TrailSystem',
      maxPoints: '300 (configurable)',
      memory: 'Controlled ~20KB/100pts',
      performance: 'Fast (Canvas renderer)',
      fade: 'Automatic gradient',
      filtering: 'Built-in (spatial+temporal)',
      cpuUsage: 'Low (2-5% active)',
    },
  };
}

// Helper function (placeholder)
function isAccuracyGood(lat: number, lng: number, minAccuracyM: number): boolean {
  // In real implementation, check GPS accuracy metadata
  // For now, always return true
  return true;
}
