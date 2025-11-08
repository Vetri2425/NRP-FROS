// Use global Leaflet instance to stay consistent with current project setup
declare var L: any;
import type { RoverStatus } from '../types/map';

// Compute a color based on rover status
export function statusToColor(status?: RoverStatus): string {
  switch (status) {
    case 'armed':
      return '#ef4444'; // red
    case 'rtk':
      return '#3b82f6'; // blue
    case 'disarmed':
    default:
      return '#22c55e'; // green
  }
}

// Map altitude to a visual scale factor (clamped)
export function altitudeToScale(alt?: number): number {
  if (typeof alt !== 'number' || !isFinite(alt)) return 1;
  // Clamp between 0m and 100m relative, scale 0.9 - 1.6
  const clamped = Math.max(0, Math.min(100, alt));
  return 0.9 + (clamped / 100) * 0.7;
}

// Map zoom level to a size scale factor
export function zoomToScale(zoom?: number): number {
  if (typeof zoom !== 'number' || !isFinite(zoom)) return 1;
  // At zoom 22 (max zoom in), scale = 1 (full 150px)
  // At zoom 10 (min zoom out), scale is much smaller
  // Linear scale: (zoom - 10) / (22 - 10)
  const clamped = Math.max(10, Math.min(22, zoom ?? 13));
  return (clamped - 10) / 12;
}

// Create an image-based icon with rotation driven by CSS transform
export function createRotatableIcon(options: {
  headingDeg: number;
  color: string;
  scale?: number;
  zoomScale?: number;
  tooltip?: string;
}): any {
  const { headingDeg, scale = 1, zoomScale = 1 } = options;
  const size = Math.round(150 * scale * zoomScale);
  const half = Math.round(size / 2);

  const html = `<img src="/assets/rover-icon.png" style="width:${size}px;height:${size}px;display:block;margin:0;padding:0;transform:rotate(${headingDeg}deg);will-change:transform;transition:transform 80ms linear;" alt="Rover" />`;

  return L.divIcon({
    html,
    className: 'bg-transparent border-0',
    iconSize: [size, size],
    iconAnchor: [half, half],
  });
}

export function updateMarkerRotation(marker: any, headingDeg: number): boolean {
  const el = marker.getElement();
  if (!el) {
    console.warn('[updateMarkerRotation] No element found for marker - not yet in DOM');
    return false;
  }
  // Find the img element directly (since we're using plain img without wrapper)
  const img = el.querySelector('img') as HTMLImageElement | null;
  if (img) {
    img.style.transform = `rotate(${headingDeg}deg)`;
    img.style.willChange = 'transform';
    img.style.transition = 'transform 80ms linear';
    console.log('[updateMarkerRotation] ✅ Applied rotation:', headingDeg.toFixed(1) + '°');
    return true;
  } else {
    console.warn('[updateMarkerRotation] No img element found in marker');
    return false;
  }
}

// Create a polyline for trail and return with managed points (culling handled externally)
export function createPathTrail(points: any[], color = '#0ea5e9'): any {
  return L.polyline(points, { color, weight: 3, opacity: 0.9 });
}

export function createGeofenceLayer(polygon: any[] | any, options?: any): any {
  if (Array.isArray(polygon)) {
    return L.polygon(polygon, { color: '#ff6b6b', weight: 2, fillOpacity: 0.1, ...(options || {}) });
  }
  return L.rectangle(polygon, { color: '#ff6b6b', weight: 2, fillOpacity: 0.1, ...(options || {}) });
}

export function createWaypointLayer(waypoints: Array<{ lat: number; lng: number }>, options?: any): any {
  const group = L.layerGroup();
  waypoints.forEach((w) => {
    L.circleMarker([w.lat, w.lng], { radius: 4, color: '#f97316', fillOpacity: 1, ...(options || {}) }).addTo(group);
  });
  return group;
}
