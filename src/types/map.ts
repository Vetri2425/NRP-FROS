// Map-related types for Leaflet layers and rover marker config

export type RoverStatus = 'armed' | 'disarmed' | 'rtk';

export interface RoverMarkerConfig {
  lat: number;
  lon: number;
  heading: number; // degrees 0-360
  altitude?: number; // meters (relative)
  status?: RoverStatus;
}

export type MapLayerType = 'vehicle' | 'trail' | 'waypoints';

export interface MapLayer {
  name: string;
  type: MapLayerType;
  visible: boolean;
}

export interface MapBounds {
  north: number;
  south: number;
  east: number;
  west: number;
}

export interface MapState {
  roverMarker?: RoverMarkerConfig | null;
  pathTrail: Array<{ lat: number; lon: number; t: number }>;
  layers: MapLayer[];
  bounds?: MapBounds;
}
