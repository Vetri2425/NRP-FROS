import { Waypoint } from "../types";

const EARTH_RADIUS_METERS = 6371000;

/**
 * Calculates the distance between two lat/lng points using the Haversine formula.
 * @param p1 - The first point { lat, lng }.
 * @param p2 - The second point { lat, lng }.
 * @returns The distance in meters.
 */
export const calculateDistance = (p1: { lat: number, lng: number }, p2: { lat: number, lng: number }): number => {
    const dLat = (p2.lat - p1.lat) * Math.PI / 180;
    const dLng = (p2.lng - p1.lng) * Math.PI / 180;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(p1.lat * Math.PI / 180) * Math.cos(p2.lat * Math.PI / 180) *
              Math.sin(dLng / 2) * Math.sin(dLng / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return EARTH_RADIUS_METERS * c;
};

/**
 * Calculates the destination point given a starting point, bearing, and distance.
 * @param start - The starting point { lat, lng }.
 * @param bearing - The bearing in degrees (0-360).
 * @param distance - The distance to travel in meters.
 * @returns The destination point { lat, lng }.
 */
export const calculateDestination = (start: { lat: number, lng: number }, bearing: number, distance: number): { lat: number, lng: number } => {
    const lat1 = start.lat * Math.PI / 180;
    const lon1 = start.lng * Math.PI / 180;
    const brng = bearing * Math.PI / 180;
    const lat2 = Math.asin(Math.sin(lat1) * Math.cos(distance / EARTH_RADIUS_METERS) +
                           Math.cos(lat1) * Math.sin(distance / EARTH_RADIUS_METERS) * Math.cos(brng));
    const lon2 = lon1 + Math.atan2(Math.sin(brng) * Math.sin(distance / EARTH_RADIUS_METERS) * Math.cos(lat1),
                                  Math.cos(distance / EARTH_RADIUS_METERS) - Math.sin(lat1) * Math.sin(lat2));
    return {
        lat: lat2 * 180 / Math.PI,
        lng: lon2 * 180 / Math.PI
    };
};

/**
 * Calculates the bearing between two points.
 * @param p1 - The first point { lat, lng }.
 * @param p2 - The second point { lat, lng }.
 * @returns The bearing in degrees (0-360).
 */
export const calculateBearing = (p1: { lat: number, lng: number }, p2: { lat: number, lng: number }): number => {
    const lat1 = p1.lat * Math.PI / 180;
    const lon1 = p1.lng * Math.PI / 180;
    const lat2 = p2.lat * Math.PI / 180;
    const lon2 = p2.lng * Math.PI / 180;
    const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
    const bearing = Math.atan2(y, x) * 180 / Math.PI;
    return (bearing + 360) % 360; // Normalize to 0-360
};

/**
 * Iterates through a mission and calculates the distance from the previous waypoint for each point.
 * Stores the result in the 'param4' property.
 * @param waypoints - An array of Waypoint objects.
 * @returns A new array of Waypoint objects with 'param4' populated with distances.
 */
export const calculateDistancesForMission = (waypoints: Waypoint[]): Waypoint[] => {
    return waypoints.map((wp, index, arr) => {
        if (index === 0) {
            return { ...wp, param4: 0 };
        }
        const prevWp = arr[index - 1];
        const distance = calculateDistance({ lat: prevWp.lat, lng: prevWp.lng }, { lat: wp.lat, lng: wp.lng });
        return { ...wp, param4: distance };
    });
};

/**
 * Generates an array of waypoint coordinates around the circumference of a circle.
 * @param center - The center point { lat, lng }.
 * @param radius - The radius in meters.
 * @param numPoints - The number of waypoints to generate.
 * @returns An array of { lat, lng } points.
 */
export const generateCircleWaypoints = (center: { lat: number, lng: number }, radius: number, numPoints: number = 12): { lat: number, lng: number }[] => {
    const points = [];
    for (let i = 0; i < numPoints; i++) {
        const angle = (i / numPoints) * 360;
        points.push(calculateDestination(center, angle, radius));
    }
    return points;
};

/**
 * Generates an array of waypoint coordinates for the vertices of a regular polygon.
 * @param center - The center point { lat, lng }.
 * @param radius - The distance from the center to each vertex in meters.
 * @param numSides - The number of sides for the polygon (e.g., 6 for a hexagon).
 * @param startBearing - The bearing from the center to the first vertex in degrees.
 * @returns An array of { lat, lng } points.
 */
export const generateRegularPolygonWaypoints = (center: { lat: number, lng: number }, radius: number, numSides: number, startBearing: number = 0): { lat: number, lng: number }[] => {
    const points = [];
    const angleIncrement = 360 / numSides;
    for (let i = 0; i < numSides; i++) {
        const bearing = (startBearing + i * angleIncrement) % 360;
        points.push(calculateDestination(center, bearing, radius));
    }
    return points;
};
