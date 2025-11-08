import { Waypoint } from '../types';
import { parseQGCWaypoints } from './waypoint_parser';

export type ParsedWaypoint = Omit<Waypoint, 'id' | 'command'> & { command?: string };

export const parseMissionFile = async (file: File): Promise<ParsedWaypoint[]> => {
  const ext = file.name.split('.').pop()?.toLowerCase() || '';
  const text = await file.text();

  // Handle Mission Planner / QGroundControl files (.waypoint or QGC header)
  if (ext === 'waypoint' || /QGC WPL 110/i.test(text)) {
    try {
      const qgc = parseQGCWaypoints(text);
      // Filter invalid coords first, then map to this module's ParsedWaypoint shape
      return qgc
        .filter((wp) => !isNaN(wp.lat) && !isNaN(wp.lng))
        .map<ParsedWaypoint>((wp) => ({
          lat: wp.lat,
          lng: wp.lng,
          alt: isNaN(wp.alt) ? 50 : wp.alt,
          frame: wp.frame ?? 3,
          current: (wp as any).current ?? 0,
          autocontinue: (wp as any).autocontinue ?? 1,
          param1: wp.param1,
          param2: wp.param2,
          param3: wp.param3,
          param4: wp.param4,
          action: 'NONE',
          command: wp.command, // optional in our type; UI will default if missing
        }));
    } catch (e) {
      // Fallback to simple lat,lng,alt per line parser if header not present
      return parseWaypointFile(text);
    }
  }

  if (ext === 'csv') {
    return parseCsvFile(text);
  }

  if (ext === 'dxf') {
    console.warn('DXF parsing is not yet supported.');
    alert('DXF file parsing is not yet supported. Please use .waypoint or .csv files.');
    return [];
  }

  throw new Error(`Unsupported file format: .${ext}`);
};

const parseWaypointFile = (text: string): ParsedWaypoint[] => {
    return text.split(/[\r\n]+/)
        .map(line => line.trim())
        .filter(line => line.length > 0 && !line.startsWith('#'))
        .map((line): ParsedWaypoint | null => {
            const [lat, lng, alt] = line.split(/[,\s\t]+/).map(Number);
            if (!isNaN(lat) && !isNaN(lng)) {
                return { lat, lng, alt: isNaN(alt) ? 50 : alt, frame: 3, action: 'NONE' };
            }
            return null;
        }).filter((wp): wp is ParsedWaypoint => wp !== null);
};

/* const parseCsvFileLegacy = (text: string): ParsedWaypoint[] => {
    const lines = text.split(/[\r\n]+/).map(l => l.trim()).filter(Boolean);
    if (lines.length < 2) return [];

    const header = lines[0].split(',').map(h => h.trim().toLowerCase().replace(/"/g, ''));
    const latIndex = header.findIndex(h => h.includes('lat'));
    const lngIndex = header.findIndex(h => h.includes('lon'));
    const altIndex = header.findIndex(h => h.includes('alt'));

    if (latIndex === -1 || lngIndex === -1) {
        throw new Error("CSV must contain columns with 'lat' and 'lon' in the header.");
    }
    
    return lines.slice(1).map((line): ParsedWaypoint | null => {
        const values = line.split(',');
        const lat = parseFloat(values[latIndex]);
        const lng = parseFloat(values[lngIndex]);
        const alt = altIndex > -1 ? parseFloat(values[altIndex]) : 50;
        
        if (!isNaN(lat) && !isNaN(lng)) {
            return { lat, lng, alt: isNaN(alt) ? 50 : alt, frame: 3, action: 'NONE' };
        }
        return null;
    }).filter((wp): wp is ParsedWaypoint => wp !== null);
}; */

// Enhanced CSV parser: supports optional headers and common delimiters
const parseCsvFile = (text: string): ParsedWaypoint[] => {
    const stripBOM = (s: string) => s.replace(/^\uFEFF/, '');
    const cleanCell = (s: string) => stripBOM(s).trim().replace(/^"|"$/g, '');

    const detectDelimiter = (line: string): ',' | ';' | '\t' => {
        const counts: Record<string, number> = {
            ',': (line.match(/,/g) || []).length,
            ';': (line.match(/;/g) || []).length,
            '\t': (line.match(/\t/g) || []).length,
        };
        const best = Object.entries(counts).sort((a, b) => b[1] - a[1])[0]?.[0] as ',' | ';' | '\t' | undefined;
        return best || ',';
    };

    const normalized = stripBOM(text);
    const lines = normalized.split(/\r?\n/).map(l => l.trim()).filter(Boolean);
    if (lines.length === 0) return [];

    const firstLine = lines[0];
    const delim = detectDelimiter(firstLine);
    const firstCells = firstLine.split(delim).map(c => cleanCell(c).toLowerCase());

    const hasHeader = firstCells.some(c => /lat|latitude/.test(c)) && firstCells.some(c => /lon|lng|long|longitude/.test(c));

    const headerCells = hasHeader ? firstCells : [];
    const findIndex = (candidates: string[]): number => {
        if (!hasHeader) return -1;
        return headerCells.findIndex(h => candidates.some(c => h.includes(c)));
    };

    let latIndex = findIndex(['lat', 'latitude']);
    let lngIndex = findIndex(['lon', 'lng', 'long', 'longitude']);
    let altIndex = findIndex(['alt', 'altitude', 'height', 'agl']);

    if (!hasHeader) {
        latIndex = 0;
        lngIndex = 1;
        altIndex = 2;
    }

    if (hasHeader && (latIndex === -1 || lngIndex === -1)) {
        throw new Error("CSV must include latitude and longitude columns (e.g., 'lat' and 'lon'/'lng').");
    }

    const startIndex = hasHeader ? 1 : 0;

    const waypoints = lines.slice(startIndex).map((line): ParsedWaypoint | null => {
        if (!line) return null;
        const cells = line.split(delim).map(cleanCell);
        const lat = parseFloat(cells[latIndex]);
        const lng = parseFloat(cells[lngIndex]);
        const altRaw = cells[altIndex];
        const alt = altIndex > -1 && altRaw !== undefined ? parseFloat(altRaw) : 50;

        if (!isNaN(lat) && !isNaN(lng)) {
            return { lat, lng, alt: isNaN(alt) ? 50 : alt, frame: 3, action: 'NONE' };
        }
        return null;
    }).filter((wp): wp is ParsedWaypoint => wp !== null);

    return waypoints;
};

export const toQGCWPL110 = (waypoints: Waypoint[]): string => {
  const COMMAND_IDS: Record<string, number> = {
    WAYPOINT: 16,
    LOITER_UNLIM: 17,
    LOITER_TURNS: 18,
    LOITER_TIME: 19,
    RETURN_TO_LAUNCH: 20,
    LAND: 21,
    TAKEOFF: 22,
    CONDITION_DELAY: 112,
    DO_CHANGE_SPEED: 178,
    DO_SET_HOME: 183,
    DO_SET_SERVO: 184,
    SPLINE_WAYPOINT: 82,
  };

  const resolveCommandId = (cmd: string): number => {
    if (!cmd) return 16;
    if (COMMAND_IDS[cmd]) return COMMAND_IDS[cmd];
    // Accept CMD_###
    const match = cmd.match(/^CMD_(\d+)$/i);
    if (match) return parseInt(match[1], 10) || 16;
    return 16;
  };

  let fileContent = 'QGC WPL 110\n';
  if (waypoints.length > 0) {
    // Home position line (index 0)
    const homeLat = Number(waypoints[0].lat).toFixed(7);
    const homeLng = Number(waypoints[0].lng).toFixed(7);
    fileContent += `0\t1\t0\t16\t0\t0\t0\t0\t${homeLat}\t${homeLng}\t0\t1\n`;
  }

  waypoints.forEach((wp, index) => {
    const frame = typeof wp.frame === 'number' ? wp.frame : 3;
    const commandId = resolveCommandId(wp.command);
    const current = typeof wp.current === 'number' ? wp.current : (index === 0 ? 1 : 0);
    const autocont = typeof wp.autocontinue === 'number' ? wp.autocontinue : 1;

    const line = [
      index + 1,
      current,
      frame,
      commandId,
      wp.param1 ?? 0,
      wp.param2 ?? 0,
      wp.param3 ?? 0,
      wp.param4 ?? 0,
      Number(wp.lat).toFixed(7),
      Number(wp.lng).toFixed(7),
      wp.alt,
      autocont,
    ].join('\t');
    fileContent += line + '\n';
  });

  return fileContent;
};
