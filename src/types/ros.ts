export interface TelemetryState {
  armed: boolean;
  mode: string;
  system_status: string;
  heartbeat_ts: number;
}

export interface TelemetryGlobal {
  lat: number;
  lon: number;
  alt_rel: number;
  vel: number;
  satellites_visible: number;
  // Velocity components (m/s) for interpolation and course calculation
  vx?: number; // East velocity
  vy?: number; // North velocity
  vz?: number; // Down velocity (negative = climbing)
  // Navigation bearings (degrees, 0-360)
  course_over_ground?: number; // Actual movement direction
  navigation_bearing?: number; // Direction to target waypoint
  target_bearing?: number; // MAVLink target bearing
}

export interface TelemetryBattery {
  voltage: number;
  current: number;
  percentage: number;
}

export interface TelemetryRtk {
  fix_type: number;
  baseline_age: number;
  base_linked: boolean;
}

export interface TelemetryMission {
  total_wp: number;
  current_wp: number;
  status: string;
  progress_pct: number;
}

export interface NetworkData {
  connection_type: 'wifi' | 'ethernet' | 'none';
  wifi_signal_strength: number;
  wifi_rssi: number;
  interface: string;
  wifi_connected: boolean;
  lora_connected: boolean;
}

export interface ServoStatus {
  servo_id: number;
  active: boolean;
  last_command_ts: number;
  pwm_values?: number[]; // Live PWM values from /mavros/rc/out
  servo1_pwm?: number;
  servo2_pwm?: number;
  servo3_pwm?: number;
  servo4_pwm?: number;
  servo5_pwm?: number;
  servo6_pwm?: number;
  servo7_pwm?: number;
  servo8_pwm?: number;
  servo9_pwm?: number;
  servo10_pwm?: number;
  servo11_pwm?: number;
  servo12_pwm?: number;
  servo13_pwm?: number;
  servo14_pwm?: number;
  servo15_pwm?: number;
  servo16_pwm?: number;
}

export interface TelemetryEnvelope {
  state?: TelemetryState;
  global?: TelemetryGlobal;
  battery?: TelemetryBattery;
  rtk?: TelemetryRtk;
  // Optional horizontal / vertical RMS and IMU status supplied by backend
  hrms?: number;
  vrms?: number;
  imu_status?: string;
  mission?: TelemetryMission;
  servo?: ServoStatus;
  network?: NetworkData;
  attitude?: TelemetryAttitude;
  timestamp?: number;
}

export interface ServiceResponse {
  success: boolean;
  message?: string;
  [key: string]: unknown;
}

export interface TelemetryAttitude {
  yaw_deg: number; // Yaw in degrees
  pitch_deg?: number; // Optional pitch in degrees
  roll_deg?: number; // Optional roll in degrees
}

export interface RoverTelemetry {
  state: TelemetryState;
  global: TelemetryGlobal;
  battery: TelemetryBattery;
  rtk: TelemetryRtk;
  // Optional accuracy metrics and IMU status
  hrms?: number;
  vrms?: number;
  imu_status?: string;
  mission: TelemetryMission;
  servo: ServoStatus;
  network: NetworkData;
  lastMessageTs: number | null;
  attitude?: TelemetryAttitude; // Added attitude property
}

// Mission controller status from ROS (Jetson mission controller)
export interface MissionStatus {
  timestamp: string;
  event_type: 'mission_started' | 'mission_stopped' | 'mission_paused' | 
              'mission_resumed' | 'waypoint_reached' | 'waypoint_timeout' | 
              'mission_completed' | 'mission_error' | 'spray_started' | 
              'spray_completed' | 'waiting_for_next';
  message: string;
  current_waypoint: number;
  total_waypoints: number;
  mission_active: boolean;
  mission_paused: boolean;
  auto_mode: boolean;
  gps: {
    lat: number | null;
    lon: number | null;
  };
}

// Servo/sprayer configuration (backend API format)
export interface BackendServoConfig {
  mode: 'interval' | 'continuous' | 'wpmark';
  servo_channel?: number;
  servo_pwm_start?: number;
  servo_pwm_stop?: number;
  spray_duration?: number;
  delay_before_spray?: number;
  delay_after_spray?: number;
  interval_distance?: number;
  gps_timeout?: number;
  auto_mode?: boolean;
}

// Mission controller configuration (full config from backend)
export interface MissionControllerConfig {
  sprayer_parameters: {
    servo_channel: number;
    servo_pwm_start: number;
    servo_pwm_stop: number;
    spray_duration: number;
    delay_before_spray: number;
    delay_after_spray: number;
    gps_timeout: number;
    auto_mode: boolean;
  };
  mission_parameters: {
    waypoint_reach_threshold: number;
    max_retry_attempts: number;
    retry_delay: number;
    status_update_interval: number;
  };
  safety_parameters: {
    max_speed: number;
    emergency_stop_enabled: boolean;
    low_battery_threshold: number;
    connection_timeout: number;
  };
}
