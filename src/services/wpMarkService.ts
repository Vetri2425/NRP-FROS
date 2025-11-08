// src/services/wpMarkService.ts

import { BACKEND_URL } from '../config';

export interface WPMarkConfig {
  delay_before_start: number;
  pwm_start: number;
  delay_before_stop: number;
  pwm_stop: number;
  delay_after_stop: number;
  servo_number: number;
}

export interface WPMarkResponse {
  success: boolean;
  message?: string;
  error?: string;
  config?: WPMarkConfig;
  mission_info?: {
    total_waypoints: number;
    estimated_duration_minutes: number;
    started_at: string;
  };
  stats?: {
    waypoints_completed: number;
    total_waypoints: number;
    duration_seconds: number;
  };
}

export interface WPMarkStatus {
  running: boolean;
  current_waypoint: number;
  total_waypoints: number;
  current_phase: 'idle' | 'initializing' | 'navigating' | 'waiting_arrival' | 'delay_before_start' | 'spraying' | 'delay_after_stop' | 'completed' | 'error';
  config: WPMarkConfig;
  uptime_seconds: number;
  last_action: string;
}

/**
 * Start WP_MARK mission with configuration
 * 
 * @param config - Mission configuration parameters
 * @returns Promise with mission start response
 * 
 * @example
 * ```typescript
 * const config = {
 *   delay_before_start: 2.0,
 *   pwm_start: 1500,
 *   delay_before_stop: 5.0,
 *   pwm_stop: 1000,
 *   delay_after_stop: 1.0
 * };
 * 
 * const response = await startWPMarkMission(config);
 * if (response.success) {
 *   console.log('Mission started!', response.mission_info);
 * }
 * ```
 */
export async function startWPMarkMission(config: WPMarkConfig): Promise<WPMarkResponse> {
  try {
    const response = await fetch(`${BACKEND_URL}/wp_mark/start`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(config),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || `HTTP ${response.status}: Failed to start mission`);
    }

    return data;
  } catch (error) {
    console.error('Failed to start WP_MARK mission:', error);
    throw error;
  }
}

/**
 * Stop currently running WP_MARK mission
 * 
 * @returns Promise with mission stop response including statistics
 * 
 * @example
 * ```typescript
 * const response = await stopWPMarkMission();
 * if (response.success) {
 *   console.log('Mission stopped:', response.stats);
 * }
 * ```
 */
export async function stopWPMarkMission(): Promise<WPMarkResponse> {
  try {
    const response = await fetch(`${BACKEND_URL}/wp_mark/stop`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || `HTTP ${response.status}: Failed to stop mission`);
    }

    return data;
  } catch (error) {
    console.error('Failed to stop WP_MARK mission:', error);
    throw error;
  }
}

/**
 * Get current WP_MARK mission status
 * 
 * @returns Promise with current mission status
 * 
 * @example
 * ```typescript
 * const status = await getWPMarkStatus();
 * console.log(`Mission: ${status.running ? 'Running' : 'Stopped'}`);
 * console.log(`Progress: ${status.current_waypoint}/${status.total_waypoints}`);
 * console.log(`Phase: ${status.current_phase}`);
 * ```
 */
export async function getWPMarkStatus(): Promise<WPMarkStatus> {
  try {
    const response = await fetch(`${BACKEND_URL}/wp_mark/status`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || `HTTP ${response.status}: Failed to get status`);
    }

    return data;
  } catch (error) {
    console.error('Failed to get WP_MARK status:', error);
    throw error;
  }
}

/**
 * Poll mission status at regular intervals
 * 
 * @param onUpdate - Callback function called with each status update
 * @param intervalMs - Polling interval in milliseconds (default: 2000ms)
 * @returns Cleanup function to stop polling
 * 
 * @example
 * ```typescript
 * // Start polling
 * const stopPolling = pollWPMarkStatus((status) => {
 *   console.log('Status update:', status);
 *   setMissionStatus(status);
 * }, 2000);
 * 
 * // Later, stop polling
 * stopPolling();
 * ```
 */
export function pollWPMarkStatus(
  onUpdate: (status: WPMarkStatus) => void,
  intervalMs: number = 2000
): () => void {
  const intervalId = setInterval(async () => {
    try {
      const status = await getWPMarkStatus();
      onUpdate(status);
    } catch (error) {
      console.error('Status polling error:', error);
    }
  }, intervalMs);

  // Return cleanup function
  return () => clearInterval(intervalId);
}

/**
 * Validate WP_MARK configuration parameters
 * 
 * @param config - Configuration to validate
 * @returns Validation result with error message if invalid
 * 
 * @example
 * ```typescript
 * const validation = validateWPMarkConfig(config);
 * if (!validation.valid) {
 *   alert(validation.error);
 * }
 * ```
 */
export function validateWPMarkConfig(config: WPMarkConfig): { valid: boolean; error?: string } {
  // Check delay_before_start
  if (config.delay_before_start < 0 || config.delay_before_start > 60) {
    return { valid: false, error: 'Delay before start must be between 0-60 seconds' };
  }

  // Check delay_before_stop
  if (config.delay_before_stop < 0 || config.delay_before_stop > 60) {
    return { valid: false, error: 'Delay before stop must be between 0-60 seconds' };
  }

  // Check delay_after_stop
  if (config.delay_after_stop < 0 || config.delay_after_stop > 60) {
    return { valid: false, error: 'Delay after stop must be between 0-60 seconds' };
  }

  // Check pwm_start
  if (config.pwm_start < 100 || config.pwm_start > 2000) {
    return { valid: false, error: 'PWM Start must be between 100-2000' };
  }

  // Check pwm_stop
  if (config.pwm_stop < 100 || config.pwm_stop > 2000) {
    return { valid: false, error: 'PWM Stop must be between 100-2000' };
  }

  return { valid: true };
}

/**
 * Save WP_MARK configuration to local storage
 * 
 * @param config - Configuration to save
 * @returns Promise that resolves when config is saved
 * 
 * @example
 * ```typescript
 * const config = {
 *   delay_before_start: 2.0,
 *   pwm_start: 1500,
 *   delay_before_stop: 5.0,
 *   pwm_stop: 1000,
 *   delay_after_stop: 1.0,
 *   servo_number: 10
 * };
 * 
 * await saveWPMarkConfig(config);
 * ```
 */
export async function saveWPMarkConfig(config: WPMarkConfig): Promise<void> {
  try {
    localStorage.setItem('wp_mark_config', JSON.stringify(config));
  } catch (error) {
    console.error('Failed to save WP_MARK config:', error);
    throw new Error('Failed to save configuration');
  }
}

/**
 * Load WP_MARK configuration from local storage
 * 
 * @returns Promise with loaded configuration or null if not found
 * 
 * @example
 * ```typescript
 * const config = await loadWPMarkConfig();
 * if (config) {
 *   console.log('Loaded config:', config);
 * } else {
 *   console.log('No saved config found');
 * }
 * ```
 */
export async function loadWPMarkConfig(): Promise<WPMarkConfig | null> {
  try {
    const stored = localStorage.getItem('wp_mark_config');
    if (!stored) {
      return null;
    }
    return JSON.parse(stored);
  } catch (error) {
    console.error('Failed to load WP_MARK config:', error);
    return null;
  }
}

/**
 * Start WP_MARK mission using saved configuration
 * 
 * @returns Promise with mission start response
 * 
 * @example
 * ```typescript
 * const response = await startWPMarkMissionFromSavedConfig();
 * if (response.success) {
 *   console.log('Mission started!', response.mission_info);
 * } else {
 *   console.log('Failed to start mission:', response.error);
 * }
 * ```
 */
export async function startWPMarkMissionFromSavedConfig(): Promise<WPMarkResponse> {
  try {
    const config = await loadWPMarkConfig();
    if (!config) {
      throw new Error('No saved configuration found. Please configure WP_MARK parameters first.');
    }

    return await startWPMarkMission(config);
  } catch (error) {
    console.error('Failed to start WP_MARK mission from saved config:', error);
    throw error;
  }
}
