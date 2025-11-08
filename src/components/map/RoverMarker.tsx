/**
 * Ensure Leaflet types are globally available.
 * This declaration resolves the `L` namespace issue.
 */
declare const L: typeof import('leaflet');

import { altitudeToScale, zoomToScale, createRotatableIcon, statusToColor, updateMarkerRotation } from '../../utils/leaflet-helpers';
import type { RoverStatus } from '../../types/map';

export interface RoverMarkerOptions extends Record<string, any> {
  heading?: number;
  altitude?: number;
  status?: RoverStatus;
  zoomLevel?: number;
  // Directional indicators (degrees, 0-360)
  courseOverGround?: number; // BLACK line - actual movement direction
  navigationBearing?: number; // GREEN line - direction to waypoint
  targetBearing?: number; // ORANGE line - MAVLink target bearing
  showDirectionalIndicators?: boolean; // Toggle for bearing lines
}

export class RoverMarker extends (L as any).Marker {
  private _heading: number;
  private _altitude?: number;
  private _status?: RoverStatus;
  private _zoomLevel?: number;
  private _courseOverGround?: number;
  private _navigationBearing?: number;
  private _targetBearing?: number;
  private _showDirectionalIndicators: boolean;
  private _bearingLinesLayer?: any; // Leaflet layer group for bearing lines

  constructor(latlng: any, options?: RoverMarkerOptions) {
    const heading = options?.heading ?? 0;
    const status = options?.status;
    const altitude = options?.altitude;
    const zoomLevel = options?.zoomLevel;
    const icon = createRotatableIcon({
      headingDeg: heading,
      color: statusToColor(status),
      scale: altitudeToScale(altitude),
      zoomScale: zoomToScale(zoomLevel),
    });
    super(latlng, { ...(options || {}), icon });
    this._heading = heading;
    this._status = status;
    this._altitude = altitude;
    this._zoomLevel = zoomLevel;
    this._courseOverGround = options?.courseOverGround;
    this._navigationBearing = options?.navigationBearing;
    this._targetBearing = options?.targetBearing;
    this._showDirectionalIndicators = options?.showDirectionalIndicators ?? false;
  }

  setHeading(deg: number): this {
    this._heading = ((deg % 360) + 360) % 360;
    
    // Try fast path: direct DOM rotation without recreating icon
    const success = updateMarkerRotation(this, this._heading);
    
    // Fallback: if DOM manipulation failed (marker not yet in DOM),
    // recreate the icon with the new heading
    if (!success) {
      const icon = createRotatableIcon({
        headingDeg: this._heading,
        color: statusToColor(this._status),
        scale: altitudeToScale(this._altitude),
        zoomScale: zoomToScale(this._zoomLevel),
      });
      this.setIcon(icon);
    }
    
    return this;
  }

  // Back-compat alias for code that used leaflet-rotatedmarker API
  setRotationAngle(deg: number): this {
    return this.setHeading(deg);
  }

  setAltitude(alt?: number): this {
    this._altitude = alt;
    return this._refreshIcon();
  }

  setStatus(status?: RoverStatus): this {
    this._status = status;
    return this._refreshIcon();
  }

  setZoomLevel(zoom?: number): this {
    this._zoomLevel = zoom;
    return this._refreshIcon();
  }

  setCourseOverGround(deg?: number): this {
    this._courseOverGround = deg;
    this._updateBearingLines();
    return this;
  }

  setNavigationBearing(deg?: number): this {
    this._navigationBearing = deg;
    this._updateBearingLines();
    return this;
  }

  setTargetBearing(deg?: number): this {
    this._targetBearing = deg;
    this._updateBearingLines();
    return this;
  }

  setShowDirectionalIndicators(show: boolean): this {
    this._showDirectionalIndicators = show;
    this._updateBearingLines();
    return this;
  }

  /**
   * Update bearing lines overlay
   * RED = compass heading, GREEN = navigation bearing, 
   * BLACK = course over ground, ORANGE = target bearing
   */
  private _updateBearingLines(): void {
    if (!this._showDirectionalIndicators) {
      this._removeBearingLines();
      return;
    }

    // Remove old lines
    this._removeBearingLines();

    const map = (this as any)._map;
    if (!map) return;

    const latlng = this.getLatLng();
    const lineLength = 0.0002; // ~22 meters at equator

    // Create layer group for all bearing lines
    this._bearingLinesLayer = L.layerGroup();

    // Helper to create bearing line
    const createBearingLine = (bearing: number | undefined, color: string, weight: number = 2) => {
      if (bearing === undefined || !Number.isFinite(bearing)) return;
      
      const bearingRad = (bearing * Math.PI) / 180;
      const endLat = latlng.lat + lineLength * Math.cos(bearingRad);
      const endLng = latlng.lng + lineLength * Math.sin(bearingRad) / Math.cos(latlng.lat * Math.PI / 180);
      
      return L.polyline([[latlng.lat, latlng.lng], [endLat, endLng]], {
        color,
        weight,
        opacity: 0.8,
        dashArray: color === '#000000' ? '5, 5' : undefined, // Dashed line for COG
      });
    };

    // Add bearing lines in order (back to front for proper z-order)
    const targetLine = createBearingLine(this._targetBearing, '#ff6600', 3); // ORANGE
    const cogLine = createBearingLine(this._courseOverGround, '#000000', 2); // BLACK (dashed)
    const navLine = createBearingLine(this._navigationBearing, '#00ff00', 3); // GREEN
    const headingLine = createBearingLine(this._heading, '#ff0000', 3); // RED

    if (targetLine) this._bearingLinesLayer.addLayer(targetLine);
    if (cogLine) this._bearingLinesLayer.addLayer(cogLine);
    if (navLine) this._bearingLinesLayer.addLayer(navLine);
    if (headingLine) this._bearingLinesLayer.addLayer(headingLine);

    this._bearingLinesLayer.addTo(map);
  }

  private _removeBearingLines(): void {
    if (this._bearingLinesLayer) {
      const map = (this as any)._map;
      if (map) {
        map.removeLayer(this._bearingLinesLayer);
      }
      this._bearingLinesLayer = null;
    }
  }

  private _refreshIcon(): this {
    const icon = createRotatableIcon({
      headingDeg: this._heading,
      color: statusToColor(this._status),
      scale: altitudeToScale(this._altitude),
      zoomScale: zoomToScale(this._zoomLevel),
    });
    (this as any).setIcon(icon);
    return this;
  }

  setLatLng(latlng: L.LatLngExpression): this {
    super.setLatLng(latlng);
    // Update bearing lines position
    this._updateBearingLines();
    return this;
  }

  // Override onAdd to ensure bearing lines are added when marker is added
  onAdd(map: any): this {
    super.onAdd(map);
    this._updateBearingLines();
    return this;
  }

  // Override onRemove to clean up bearing lines
  onRemove(map: any): this {
    this._removeBearingLines();
    super.onRemove(map);
    return this;
  }
}

export function createRoverMarker(
  lat: number,
  lon: number,
  options?: RoverMarkerOptions,
): RoverMarker {
  return new RoverMarker([lat, lon], options);
}

export default RoverMarker;
