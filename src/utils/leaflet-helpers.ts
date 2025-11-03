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

// Create an SVG-based icon with rotation driven by CSS transform
export function createRotatableIcon(options: {
  headingDeg: number;
  color: string;
  scale?: number;
  tooltip?: string;
}): any {
  const { headingDeg, color, scale = 1 } = options;
  const size = Math.round(28 * scale);
  const half = Math.round(size / 2);

  const svg = `
  <svg viewBox="0 0 64 64" xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" style="display:block">
    <g>
      <rect x="12" y="20" width="40" height="24" rx="6" ry="6" fill="${color}" stroke="#111" stroke-width="2" />
      <rect x="18" y="26" width="12" height="5" rx="1" fill="#111" opacity="0.85" />
      <rect x="34" y="26" width="12" height="5" rx="1" fill="#111" opacity="0.85" />
      <path d="M 32 6 L 40 18 L 24 18 Z" fill="#ffeb3b" stroke="#f59e0b" stroke-width="1" />
      <circle cx="32" cy="32" r="2" fill="#fff" opacity="0.7" />
    </g>
  </svg>`;

  const html = `<div class="rover-rotatable" style="transform: rotate(${headingDeg}deg); will-change: transform; transition: transform 80ms linear;">${svg}</div>`;

  return L.divIcon({
    html,
    className: 'bg-transparent border-0 rover-icon',
    iconSize: [size, size],
    iconAnchor: [half, half],
  });
}

export function updateMarkerRotation(marker: any, headingDeg: number): void {
  const el = marker.getElement();
  if (!el) return;
  const rot = el.querySelector('.rover-rotatable') as HTMLElement | null;
  if (rot) {
    rot.style.transform = `rotate(${headingDeg}deg)`;
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
