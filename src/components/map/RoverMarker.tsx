/**
 * Ensure Leaflet types are globally available.
 * This declaration resolves the `L` namespace issue.
 */
declare const L: typeof import('leaflet');

import { altitudeToScale, createRotatableIcon, statusToColor, updateMarkerRotation } from '../../utils/leaflet-helpers';
import type { RoverStatus } from '../../types/map';

export interface RoverMarkerOptions extends Record<string, any> {
  heading?: number;
  altitude?: number;
  status?: RoverStatus;
}

export class RoverMarker extends (L as any).Marker {
  private _heading: number;
  private _altitude?: number;
  private _status?: RoverStatus;

  constructor(latlng: any, options?: RoverMarkerOptions) {
    const heading = options?.heading ?? 0;
    const status = options?.status;
    const altitude = options?.altitude;
    const icon = createRotatableIcon({
      headingDeg: heading,
      color: statusToColor(status),
      scale: altitudeToScale(altitude),
    });
    super(latlng, { ...(options || {}), icon });
    this._heading = heading;
    this._status = status;
    this._altitude = altitude;
  }

  setHeading(deg: number): this {
    this._heading = ((deg % 360) + 360) % 360;
    // Fast path: direct DOM rotation without recreating icon
    updateMarkerRotation(this, this._heading);
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

  private _refreshIcon(): this {
    const icon = createRotatableIcon({
      headingDeg: this._heading,
      color: statusToColor(this._status),
      scale: altitudeToScale(this._altitude),
    });
    (this as any).setIcon(icon);
    return this;
  }

  setLatLng(latlng: L.LatLngExpression): this {
    super.setLatLng(latlng);
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
