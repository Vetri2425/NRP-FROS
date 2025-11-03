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
  mission?: TelemetryMission;
  servo?: ServoStatus;
  network?: NetworkData;
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
  mission: TelemetryMission;
  servo: ServoStatus;
  network: NetworkData;
  lastMessageTs: number | null;
  attitude?: TelemetryAttitude; // Added attitude property
}
