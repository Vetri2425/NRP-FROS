import React, { useState } from 'react';
import { saveWPMarkConfig, validateWPMarkConfig, WPMarkConfig } from '../services/wpMarkService';

interface WPMarkDialogProps {
  isOpen: boolean;
  onClose: () => void;
}

interface WPMarkParams {
  delayBeforeStart: number;
  pwmStart: number;
  delayBeforeStop: number;
  pwmStop: number;
  delayAfterStop: number;
  servoNumber: number;
}

const WPMarkDialog: React.FC<WPMarkDialogProps> = ({ isOpen, onClose }) => {
  const [params, setParams] = useState<WPMarkParams>({
    delayBeforeStart: 2.0,
    pwmStart: 1500,
    delayBeforeStop: 5.0,
    pwmStop: 1000,
    delayAfterStop: 1.0,
    servoNumber: 10,
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (field: keyof WPMarkParams, value: string) => {
    setParams((prev) => ({
      ...prev,
      [field]: parseFloat(value) || 0,
    }));
  };

  const handleSaveConfig = async () => {
    // Validate parameters using service function
    const validation = validateWPMarkConfig({
      delay_before_start: params.delayBeforeStart,
      pwm_start: params.pwmStart,
      delay_before_stop: params.delayBeforeStop,
      pwm_stop: params.pwmStop,
      delay_after_stop: params.delayAfterStop,
      servo_number: params.servoNumber,
    });

    if (!validation.valid) {
      setError(validation.error || 'Invalid parameters');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const config: WPMarkConfig = {
        delay_before_start: params.delayBeforeStart,
        pwm_start: params.pwmStart,
        delay_before_stop: params.delayBeforeStop,
        pwm_stop: params.pwmStop,
        delay_after_stop: params.delayAfterStop,
        servo_number: params.servoNumber,
      };

      await saveWPMarkConfig(config);

      alert(
        `✅ Configuration saved successfully!\n\n` +
        `Servo: ${params.servoNumber}\n` +
        `Delay Before Start: ${params.delayBeforeStart}s\n` +
        `PWM Start: ${params.pwmStart}\n` +
        `Delay Before Stop: ${params.delayBeforeStop}s\n` +
        `PWM Stop: ${params.pwmStop}\n` +
        `Delay After Stop: ${params.delayAfterStop}s`
      );
      onClose();
    } catch (err: any) {
      setError(err.message || 'Failed to save configuration');
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[9999]">
      <div className="bg-slate-800 rounded-lg shadow-2xl w-full max-w-md max-h-[450px] overflow-y-auto p-6 border border-slate-700">
        {/* Header */}
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-xl font-bold text-white">🎯 WP_MARK Mission Setup</h2>
          <button
            onClick={onClose}
            className="text-gray-400 hover:text-white text-2xl"
            disabled={isLoading}
          >
            ×
          </button>
        </div>

        {/* Parameters Form */}
        <div className="space-y-4">
          {/* Servo Number */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              0️⃣ Servo Number
            </label>
            <input
              type="number"
              min="1"
              max="99"
              value={params.servoNumber}
              onChange={(e) => handleChange('servoNumber', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">Servo output channel number (1-99)</p>
          </div>

          {/* Delay Before Start */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              1️⃣ Delay Before Start Servo (seconds)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="60"
              value={params.delayBeforeStart}
              onChange={(e) => handleChange('delayBeforeStart', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">Wait time after reaching waypoint</p>
          </div>

          {/* PWM Start */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              2️⃣ Servo Start PWM
            </label>
            <input
              type="number"
              min="100"
              max="2000"
              value={params.pwmStart}
              onChange={(e) => handleChange('pwmStart', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">PWM value to activate servo (100-2000)</p>
          </div>

          {/* Delay Before Stop */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              3️⃣ Delay Before Stop PWM (seconds)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="60"
              value={params.delayBeforeStop}
              onChange={(e) => handleChange('delayBeforeStop', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">How long servo stays active</p>
          </div>

          {/* PWM Stop */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              4️⃣ Servo Stop PWM
            </label>
            <input
              type="number"
              min="100"
              max="2000"
              value={params.pwmStop}
              onChange={(e) => handleChange('pwmStop', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">PWM value to deactivate servo</p>
          </div>

          {/* Delay After Stop */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-1">
              5️⃣ Delay After Stop Servo (seconds)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="60"
              value={params.delayAfterStop}
              onChange={(e) => handleChange('delayAfterStop', e.target.value)}
              className="w-full px-3 py-2 bg-slate-900 border border-slate-600 rounded text-white focus:outline-none focus:border-orange-500"
              disabled={isLoading}
            />
            <p className="text-xs text-slate-500 mt-1">Wait before moving to next waypoint</p>
          </div>
        </div>

        {/* Error Message */}
        {error && (
          <div className="mt-4 p-3 bg-red-900/30 border border-red-500 rounded text-red-200 text-sm">
            ⚠️ {error}
          </div>
        )}

        {/* Mission Sequence Preview */}
        <div className="mt-4 p-3 bg-blue-900/20 border border-blue-700/50 rounded">
          <h4 className="text-xs font-semibold text-blue-300 mb-2">📋 Mission Sequence:</h4>
          <ol className="text-xs text-blue-200 space-y-1">
            <li>1. Navigate to waypoint</li>
            <li>2. Wait {params.delayBeforeStart}s</li>
            <li>3. Set servo PWM to {params.pwmStart}</li>
            <li>4. Wait {params.delayBeforeStop}s</li>
            <li>5. Set servo PWM to {params.pwmStop}</li>
            <li>6. Wait {params.delayAfterStop}s</li>
            <li>7. Repeat for next waypoint</li>
          </ol>
        </div>

        {/* Action Buttons */}
        <div className="flex gap-3 mt-6">
          <button
            onClick={onClose}
            disabled={isLoading}
            className="flex-1 px-4 py-2 bg-gray-600 hover:bg-gray-700 text-white rounded font-medium disabled:opacity-50"
          >
            Cancel
          </button>
          <button
            onClick={handleSaveConfig}
            disabled={isLoading}
            className="flex-1 px-4 py-2 bg-orange-600 hover:bg-orange-700 text-white rounded font-medium disabled:opacity-50"
          >
            {isLoading ? '⏳ Saving...' : '� Save Configs'}
          </button>
        </div>
      </div>
    </div>
  );
};

export default WPMarkDialog;
