
import { Waypoint } from '../types';

export type ParsedWaypoint = Omit<Waypoint, 'id' | 'command'> & { command?: string };

export const parseMissionFile = async (file: File): Promise<ParsedWaypoint[]> => {
  const extension = file.name.split('.').pop()?.toLowerCase();
  const text = await file.text();

  switch (extension) {
    case 'waypoint':
      return parseWaypointFile(text);
    case 'csv':
      return parseCsvFile(text);
    case 'dxf':
      console.warn("DXF parsing is not yet supported.");
      alert("DXF file parsing is not yet supported. Please use .waypoint or .csv files.");
      return [];
    default:
      throw new Error(`Unsupported file type: .${extension}. Please use .waypoint, .csv, or .dxf.`);
  }
};

const parseWaypointFile = (text: string): ParsedWaypoint[] => {
    return text.split(/[\r\n]+/)
        .map(line => line.trim())
        .filter(line => line.length > 0 && !line.startsWith('#'))
        .map(line => {
            const [lat, lng, alt] = line.split(/[,\s\t]+/).map(Number);
            if (!isNaN(lat) && !isNaN(lng)) {
                return { lat, lng, alt: isNaN(alt) ? 50 : alt, frame: 3 };
            }
            return null;
        }).filter((wp): wp is ParsedWaypoint => wp !== null);
};

const parseCsvFile = (text: string): ParsedWaypoint[] => {
    const lines = text.split(/[\r\n]+/).map(l => l.trim()).filter(Boolean);
    if (lines.length < 2) return [];

    const header = lines[0].split(',').map(h => h.trim().toLowerCase().replace(/"/g, ''));
    const latIndex = header.findIndex(h => h.includes('lat'));
    const lngIndex = header.findIndex(h => h.includes('lon'));
    const altIndex = header.findIndex(h => h.includes('alt'));

    if (latIndex === -1 || lngIndex === -1) {
        throw new Error("CSV must contain columns with 'lat' and 'lon' in the header.");
    }
    
    return lines.slice(1).map(line => {
        const values = line.split(',');
        const lat = parseFloat(values[latIndex]);
        const lng = parseFloat(values[lngIndex]);
        const alt = altIndex > -1 ? parseFloat(values[altIndex]) : 50;
        
        if (!isNaN(lat) && !isNaN(lng)) {
            return { lat, lng, alt: isNaN(alt) ? 50 : alt, frame: 3 };
        }
        return null;
    }).filter((wp): wp is ParsedWaypoint => wp !== null);
};

export const toQGCWPL110 = (waypoints: Waypoint[]): string => {
  let fileContent = 'QGC WPL 110\n';
  if (waypoints.length > 0) {
      // Home position line
      fileContent += `0\t1\t0\t16\t0\t0\t0\t0\t${waypoints[0].lat}\t${waypoints[0].lng}\t0\t1\n`;
  }

  waypoints.forEach((wp, index) => {
    const commandMap: { [key: string]: number } = {
      'WAYPOINT': 16,
      'TAKEOFF': 22,
      'LAND': 21,
      'LOITER_TURNS': 18
    };
    const commandId = commandMap[wp.command] || 16;
    const frame = wp.frame || 3; // Default to Global Relative Alt

    const line = [
      index + 1,
      0, // Current
      frame,
      commandId,
      wp.param1 || 0,
      wp.param2 || 0,
      wp.param3 || 0,
      wp.param4 || 0,
      wp.lat,
      wp.lng,
      wp.alt,
      1 // Autocontinue
    ].join('\t');
    fileContent += line + '\n';
  });

  return fileContent;
};
