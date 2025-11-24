import React, { useState, useEffect } from 'react';
import { toast } from 'react-toastify';
import { BACKEND_URL } from '../../config';

interface ServoConfig {
  servo_channel: number | string;
  servo_pwm_on: number | string;
  servo_pwm_off: number | string;
  servo_delay_before: number | string;
  servo_spray_duration: number | string;
  servo_delay_after: number | string;
  servo_enabled: boolean;
}

const ServoConfigPanel: React.FC = () => {
  const [config, setConfig] = useState<ServoConfig>({
    servo_channel: '',
    servo_pwm_on: '',
    servo_pwm_off: '',
    servo_delay_before: '',
    servo_spray_duration: '',
    servo_delay_after: '',
    servo_enabled: true
  });

  const [loading, setLoading] = useState(false);
  const [fetching, setFetching] = useState(true);

  // Load current config on mount
  useEffect(() => {
    const fetchConfig = async () => {
      try {
        const response = await fetch(`${BACKEND_URL}/api/mission/servo_config`);
        if (!response.ok) {
          throw new Error('Failed to fetch servo config');
        }
        const result = await response.json();
        if (result.success && result.message) {
          setConfig(result.message);
          toast.success('Servo configuration loaded');
        }
      } catch (err) {
        console.error('Failed to load servo config:', err);
        toast.error('Failed to load servo configuration');
      } finally {
        setFetching(false);
      }
    };

    fetchConfig();
  }, []);

  const handleUpdate = async () => {
    setLoading(true);
    try {
      // Convert string values to numbers for API
      const payload = {
        servo_channel: typeof config.servo_channel === 'string' ? parseInt(config.servo_channel) || 0 : config.servo_channel,
        servo_pwm_on: typeof config.servo_pwm_on === 'string' ? parseInt(config.servo_pwm_on) || 0 : config.servo_pwm_on,
        servo_pwm_off: typeof config.servo_pwm_off === 'string' ? parseInt(config.servo_pwm_off) || 0 : config.servo_pwm_off,
        servo_delay_before: typeof config.servo_delay_before === 'string' ? parseFloat(config.servo_delay_before) || 0 : config.servo_delay_before,
        servo_spray_duration: typeof config.servo_spray_duration === 'string' ? parseFloat(config.servo_spray_duration) || 0 : config.servo_spray_duration,
        servo_delay_after: typeof config.servo_delay_after === 'string' ? parseFloat(config.servo_delay_after) || 0 : config.servo_delay_after,
        servo_enabled: config.servo_enabled
      };
      
      const response = await fetch(`${BACKEND_URL}/api/mission/servo_config`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        throw new Error('Failed to update servo config');
      }

      const result = await response.json();
      if (result.success) {
        toast.success(`Servo config updated: ${result.updated_params?.join(', ') || result.message || 'All parameters'}`);
      } else {
        toast.error(result.message || 'Failed to update configuration');
      }
    } catch (err) {
      console.error('Failed to update servo config:', err);
      toast.error('Failed to update servo configuration');
    } finally {
      setLoading(false);
    }
  };

  const handleReset = () => {
    setConfig({
      servo_channel: '',
      servo_pwm_on: '',
      servo_pwm_off: '',
      servo_delay_before: '',
      servo_spray_duration: '',
      servo_delay_after: '',
      servo_enabled: true
    });
    toast.info('Configuration reset to empty');
  };

  if (fetching) {
    return (
      <div className="flex items-center justify-center p-6">
        <div className="text-gray-400">Loading servo configuration...</div>
      </div>
    );
  }

  return (
    <div className="p-6 bg-slate-900 text-white rounded-xl shadow-xl max-w-4xl">
      <h2 className="text-2xl font-bold mb-2 flex items-center gap-2">
        ⚙️ Servo Configuration
      </h2>
      <p className="text-gray-400 mb-6 text-sm">
        Configure servo parameters for mission spray control. Changes take effect immediately for future waypoints.
      </p>

      <div className="space-y-4">
        {/* Servo Channel and PWM Settings */}
        <div className="bg-slate-800 p-4 rounded-lg">
          <h3 className="text-lg font-semibold mb-3 text-blue-400">Servo Hardware Settings</h3>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            <div>
              <label className="block text-sm text-gray-300 mb-2">
                Servo Channel
              </label>
              <input
                type="number"
                value={config.servo_channel}
                onChange={(e) => setConfig({ ...config, servo_channel: e.target.value })}
                className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2 text-white focus:outline-none focus:ring-2 focus:ring-blue-500"
                min="0"
                max="16"
              />
              <p className="text-xs text-gray-500 mt-1">Channel: 1-16</p>
            </div>

            <div>
              <label className="block text-sm text-gray-300 mb-2">
                PWM ON/START (µs)
              </label>
              <input
                type="number"
                value={config.servo_pwm_on}
                onChange={(e) => setConfig({ ...config, servo_pwm_on: e.target.value })}
                className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2 text-white focus:outline-none focus:ring-2 focus:ring-green-500"
                min="0"
                max="2500"
              />
              <p className="text-xs text-gray-500 mt-1">Start spraying</p>
            </div>

            <div>
              <label className="block text-sm text-gray-300 mb-2">
                PWM OFF/STOP (µs)
              </label>
              <input
                type="number"
                value={config.servo_pwm_off}
                onChange={(e) => setConfig({ ...config, servo_pwm_off: e.target.value })}
                className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2 text-white focus:outline-none focus:ring-2 focus:ring-red-500"
                min="0"
                max="2500"
              />
              <p className="text-xs text-gray-500 mt-1">Stop spraying</p>
            </div>
          </div>
        </div>

        {/* Timing Configuration */}
        <div className="bg-slate-800 p-4 rounded-lg">
          <h3 className="text-lg font-semibold mb-3 text-amber-400">⏱ Timing Configuration</h3>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            <div>
              <label className="block text-sm text-gray-300 mb-2">
                Delay Before Spray (s)
              </label>
              <input
                type="number"
                step="0.1"
                value={config.servo_delay_before}
                onChange={(e) => setConfig({ ...config, servo_delay_before: e.target.value })}
                className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2 text-white focus:outline-none focus:ring-2 focus:ring-amber-500"
                min="0"
              />
              <p className="text-xs text-gray-500 mt-1">Wait before starting</p>
            </div>

            <div>
              <label className="block text-sm text-gray-300 mb-2">
                Spray Duration (s)
              </label>
              <input
                type="number"
                step="0.1"
                value={config.servo_spray_duration}
                onChange={(e) => setConfig({ ...config, servo_spray_duration: e.target.value })}
                className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2 text-white focus:outline-none focus:ring-2 focus:ring-cyan-500"
                min="0"
              />
              <p className="text-xs text-gray-500 mt-1">Active spray time</p>
            </div>

            <div>
              <label className="block text-sm text-gray-300 mb-2">
                Delay After Spray (s)
              </label>
              <input
                type="number"
                step="0.1"
                value={config.servo_delay_after}
                onChange={(e) => setConfig({ ...config, servo_delay_after: e.target.value })}
                className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2 text-white focus:outline-none focus:ring-2 focus:ring-purple-500"
                min="0"
              />
              <p className="text-xs text-gray-500 mt-1">Wait after stopping</p>
            </div>
          </div>
        </div>

        {/* Action Buttons */}
        <div className="flex gap-3 pt-4">
          <button
            onClick={handleUpdate}
            disabled={loading}
            className={`flex-1 py-3 rounded-lg font-semibold transition ${
              loading
                ? 'bg-gray-600 cursor-not-allowed'
                : 'bg-green-600 hover:bg-green-700'
            }`}
          >
            {loading ? 'Updating...' : '✓ Update Configuration'}
          </button>
          <button
            onClick={handleReset}
            disabled={loading}
            className="px-6 py-3 bg-gray-700 hover:bg-gray-600 rounded-lg font-semibold transition"
          >
            Reset to Defaults
          </button>
        </div>
      </div>
    </div>
  );
};

export default ServoConfigPanel;
