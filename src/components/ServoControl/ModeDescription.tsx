// src/components/ServoControl/ModeDescription.tsx
import React from "react";

type Props = {
  selectedMode: string;
};

export default function ModeDescription({ selectedMode }: Props) {
  if (selectedMode !== "wpmark") return null;

  return (
    <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg shadow mt-4">
      <h3 className="font-semibold mb-3 text-slate-200">WP Mark Mode Guide</h3>
      
      <div className="space-y-4">
        <div>
          <h4 className="text-sm font-semibold text-green-400 mb-1">🎯 ACTIVE Mode (Recommended)</h4>
          <p className="text-xs text-slate-300">
            Script controls waypoint navigation. Reads mission waypoints and systematically 
            navigates to each one, performing spray sequence with industrial safety checks.
          </p>
          <div className="text-xs text-slate-400 mt-1">
            <strong>Sequence:</strong> Set WP → Wait for proximity → Delay → Spray → Delay → Next WP
          </div>
        </div>

        <div>
          <h4 className="text-sm font-semibold text-yellow-400 mb-1">🔄 PASSIVE Mode</h4>
          <p className="text-xs text-slate-300">
            Waits for MISSION_ITEM_REACHED messages. Traditional approach with higher 
            failure rate due to message dependency.
          </p>
        </div>

        <div className="bg-blue-900/20 border border-blue-700/50 rounded p-3">
          <h4 className="text-sm font-semibold text-blue-300 mb-2">🛡 Safety Features</h4>
          <ul className="text-xs text-blue-200 space-y-1">
            <li>• GPS quality verification (RTK required)</li>
            <li>• Velocity monitoring (stops when rover stops)</li>
            <li>• Mission state validation (AUTO + armed)</li>
            <li>• Emergency stop capability</li>
            <li>• Complete audit logging</li>
          </ul>
        </div>
      </div>
    </div>
  );
}
