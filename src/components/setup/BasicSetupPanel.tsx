import React, { useState } from "react";

const BasicSetupPanel: React.FC = () => {
  const [brakeEnabled, setBrakeEnabled] = useState(false);
  const [sensorEnabled, setSensorEnabled] = useState(false);
  const [sprayEnabled, setSprayEnabled] = useState(false);
  const [autoTuning, setAutoTuning] = useState(false);

  const toggle = (setter: React.Dispatch<React.SetStateAction<boolean>>) => {
    setter((prev) => !prev);
  };

  return (
    <div className="p-6 bg-slate-900 text-white rounded-xl shadow-xl max-w-4xl mb-6">
      <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
        🧰 Basic Setup Panel
      </h2>
      <p className="text-gray-400 mb-6 text-sm">
        Use this panel to enable or disable key rover systems before starting
        operations. These settings will later connect to backend control logic.
      </p>

      <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
        {/* Brake Enable */}
        <button
          onClick={() => toggle(setBrakeEnabled)}
          className={`py-3 rounded-lg font-semibold transition ${
            brakeEnabled
              ? "bg-green-600 hover:bg-green-700"
              : "bg-gray-700 hover:bg-gray-600"
          }`}
        >
          {brakeEnabled ? "🟢 Brake Enabled" : "⚪ Brake Disabled"}
        </button>

        {/* Sensor Enable */}
        <button
          onClick={() => toggle(setSensorEnabled)}
          className={`py-3 rounded-lg font-semibold transition ${
            sensorEnabled
              ? "bg-blue-600 hover:bg-blue-700"
              : "bg-gray-700 hover:bg-gray-600"
          }`}
        >
          {sensorEnabled ? "🟢 Sensor Enabled" : "⚪ Sensor Disabled"}
        </button>

        {/* Spray Enable */}
        <button
          onClick={() => toggle(setSprayEnabled)}
          className={`py-3 rounded-lg font-semibold transition ${
            sprayEnabled
              ? "bg-orange-600 hover:bg-orange-700"
              : "bg-gray-700 hover:bg-gray-600"
          }`}
        >
          {sprayEnabled ? "🟢 Spray Enabled" : "⚪ Spray Disabled"}
        </button>

        {/* Auto Tuning */}
        <button
          onClick={() => toggle(setAutoTuning)}
          className={`py-3 rounded-lg font-semibold transition ${
            autoTuning
              ? "bg-purple-600 hover:bg-purple-700"
              : "bg-gray-700 hover:bg-gray-600"
          }`}
        >
          {autoTuning ? "🟢 Auto Tuning Active" : "⚪ Auto Tuning Off"}
        </button>
      </div>
    </div>
  );
};

export default BasicSetupPanel;
