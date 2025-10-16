// src/types/servo.ts
export type ServoMode = 'MARK_AT_WAYPOINT' | 'CONTINUOUS_LINE' | 'INTERVAL_SPRAY';

export type ServoConfigBase = {
  mode: ServoMode;
  servoNumber: number;
  pwmOn: number;
  pwmOff: number;
  /** from the table selection; for CONTINUOUS_LINE & INTERVAL_SPRAY choose a range (min..max) */
  selectedIds?: number[];
};

export type MarkAtWaypointConfig = ServoConfigBase & {
  mode: 'MARK_AT_WAYPOINT';
  sprayDuration: number; // seconds
};

export type ContinuousLineConfig = ServoConfigBase & {
  mode: 'CONTINUOUS_LINE';
};

export type IntervalSprayConfig = ServoConfigBase & {
  mode: 'INTERVAL_SPRAY';
  distanceOnMeters: number;
  distanceOffMeters: number;
  startWp: number;
  endWp: number;
};

export type ServoConfig =
  | MarkAtWaypointConfig
  | ContinuousLineConfig
  | IntervalSprayConfig;
