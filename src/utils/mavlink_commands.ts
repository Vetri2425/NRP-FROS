// File: src/utils/mavlink_commands.ts
// MAVLink command definitions for Mission Planner compatibility

export const MAV_CMD = {
  // Navigation commands (Rover-specific only)
  NAV_WAYPOINT: 16,
  // Removed: NAV_LOITER_UNLIM, NAV_LOITER_TURNS, NAV_LOITER_TIME (aircraft-specific)
  NAV_RETURN_TO_LAUNCH: 20,
  // Removed: NAV_LAND, NAV_TAKEOFF (aircraft-specific)
  NAV_LOITER_TO_ALT: 31, // Keep for potential future use
  NAV_SPLINE_WAYPOINT: 82,
  
  // DO (immediate) commands
  DO_JUMP: 177,
  DO_CHANGE_SPEED: 178,
  DO_SET_HOME: 179,
  DO_SET_SERVO: 183,
  DO_SET_RELAY: 181,
  DO_REPEAT_RELAY: 182,
  DO_REPEAT_SERVO: 184,
  DO_DIGICAM_CONTROL: 203,
  DO_MOUNT_CONTROL: 205,
  DO_SET_CAM_TRIGG_DIST: 206,
  
  // CONDITION commands
  CONDITION_DELAY: 112,
  CONDITION_DISTANCE: 114,
  CONDITION_YAW: 115,
} as const;

export type MAVCommand = typeof MAV_CMD[keyof typeof MAV_CMD];

export interface CommandDefinition {
  id: number;
  name: string;
  description: string;
  category: 'nav' | 'do' | 'condition';
  params: {
    param1?: string;
    param2?: string;
    param3?: string;
    param4?: string;
  };
}

export const COMMAND_DEFINITIONS: Record<number, CommandDefinition> = {
  [MAV_CMD.NAV_WAYPOINT]: {
    id: 16,
    name: 'WAYPOINT',
    description: 'Navigate to waypoint',
    category: 'nav',
    params: {
      param1: 'Hold (s)',
      // Removed: Accept radius, Pass radius, Yaw angle (keep only hold time)
    },
  },
  // Removed: LOITER_UNLIM, LOITER_TURNS, LOITER_TIME (aircraft-specific)
  [MAV_CMD.NAV_RETURN_TO_LAUNCH]: {
    id: 20,
    name: 'RTL',
    description: 'Return to launch point',
    category: 'nav',
    params: {},
  },
  // Removed: LAND, TAKEOFF (aircraft-specific)
  [MAV_CMD.NAV_SPLINE_WAYPOINT]: {
    id: 82,
    name: 'SPLINE_WP',
    description: 'Spline waypoint (smooth curves)',
    category: 'nav',
    params: {
      param1: 'Hold (s)',
      param2: 'Accept radius (m)',
      param3: 'Pass radius (m)',
      param4: 'Yaw (deg)',
    },
  },
  [MAV_CMD.DO_JUMP]: {
    id: 177,
    name: 'DO_JUMP',
    description: 'Jump to waypoint',
    category: 'do',
    params: {
      param1: 'Target WP',
      param2: 'Repeat count',
    },
  },
  [MAV_CMD.DO_CHANGE_SPEED]: {
    id: 178,
    name: 'DO_CHANGE_SPEED',
    description: 'Change speed',
    category: 'do',
    params: {
      param1: 'Speed type (0=airspeed, 1=groundspeed)',
      param2: 'Speed (m/s)',
      param3: 'Throttle (%)',
    },
  },
  [MAV_CMD.DO_SET_SERVO]: {
    id: 183,
    name: 'DO_SET_SERVO',
    description: 'Set servo PWM',
    category: 'do',
    params: {
      param1: 'Servo number',
      param2: 'PWM value',
    },
  },
  [MAV_CMD.CONDITION_DELAY]: {
    id: 112,
    name: 'CONDITION_DELAY',
    description: 'Delay next waypoint',
    category: 'condition',
    params: {
      param1: 'Delay (s)',
    },
  },
  [MAV_CMD.CONDITION_YAW]: {
    id: 115,
    name: 'CONDITION_YAW',
    description: 'Set yaw angle',
    category: 'condition',
    params: {
      param1: 'Angle (deg)',
      param2: 'Angular speed (deg/s)',
      param3: 'Direction (-1=CCW, 1=CW)',
      param4: 'Relative (0=absolute, 1=relative)',
    },
  },
};

/**
 * Get command definition by ID or name
 */
export function getCommandDefinition(idOrName: number | string): CommandDefinition | undefined {
  if (typeof idOrName === 'number') {
    return COMMAND_DEFINITIONS[idOrName];
  }
  return Object.values(COMMAND_DEFINITIONS).find(cmd => cmd.name === idOrName);
}

/**
 * Get all navigation commands
 */
export function getNavigationCommands(): CommandDefinition[] {
  return Object.values(COMMAND_DEFINITIONS).filter(cmd => cmd.category === 'nav');
}

/**
 * Get all DO commands
 */
export function getDoCommands(): CommandDefinition[] {
  return Object.values(COMMAND_DEFINITIONS).filter(cmd => cmd.category === 'do');
}

/**
 * Get all CONDITION commands
 */
export function getConditionCommands(): CommandDefinition[] {
  return Object.values(COMMAND_DEFINITIONS).filter(cmd => cmd.category === 'condition');
}

/**
 * Convert command name to ID
 */
export function commandNameToId(name: string): number {
  const def = getCommandDefinition(name);
  return def ? def.id : MAV_CMD.NAV_WAYPOINT;
}

/**
 * Convert command ID to name
 */
export function commandIdToName(id: number): string {
  const def = COMMAND_DEFINITIONS[id];
  return def ? def.name : 'WAYPOINT';
}
