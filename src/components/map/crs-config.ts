import L from 'leaflet';
import 'proj4';
import 'proj4leaflet';

// Define the proj4 string for WGS 84 / UTM zone 44N, which covers Tamil Nadu / Chennai
const utm44nProj4 = '+proj=utm +zone=44 +datum=WGS84 +units=m +no_defs';

// Define the CRS using the proj4 definition
export const crsUtm44n = new L.Proj.CRS('EPSG:32644', utm44nProj4, {
  resolutions: [
    8192, 4096, 2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0.5,
  ],
  origin: [0, 0],
});

// Define the map center for Egmore, Chennai in WGS84 coordinates
export const egmoreLatLng: L.LatLngTuple = [13.0837, 80.2637];

// Define map options for this CRS
export const mapOptions = {
  crs: crsUtm44n,
  center: egmoreLatLng,
  zoom: 12,
  minZoom: 5,
  maxZoom: 14,
};
