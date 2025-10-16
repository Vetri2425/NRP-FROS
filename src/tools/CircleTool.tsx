// File: src/tools/CircleTool.tsx

import React, { useState } from "react";
import { generateCircleMission } from "../utils/mission_generator";
import { Waypoint } from "../types";

type CircleToolProps = {
  onGenerate: (waypoints: Waypoint[]) => void;
  onClose: () => void;
};

const CircleTool: React.FC<CircleToolProps> = ({ onGenerate, onClose }) => {
  const [centerLat, setCenterLat] = useState(13.0764);
  const [centerLng, setCenterLng] = useState(80.1559);
  const [radius, setRadius] = useState(30);
  const [points, setPoints] = useState(12);
  const [altitude, setAltitude] = useState(30);

  const handleGenerate = () => {
    if (points < 3) return alert("At least 3 waypoints required");
    const waypoints = generateCircleMission(
      { lat: centerLat, lng: centerLng },
      radius,
      points,
      altitude
    );
    onGenerate(waypoints);
    onClose();
  };

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm flex items-center justify-center z-[2000]">
      <div className="bg-slate-800 rounded-xl shadow-lg p-6 w-[400px] text-white">
        <h2 className="text-lg font-bold mb-4 text-center">🌀 Generate Circle Mission</h2>

        <div className="space-y-3">
          <div>
            <label className="block text-sm mb-1 text-gray-300">Center Latitude</label>
            <input
              type="number"
              value={centerLat}
              onChange={(e) => setCenterLat(parseFloat(e.target.value))}
              className="w-full p-2 rounded bg-slate-700 text-white"
            />
          </div>

          <div>
            <label className="block text-sm mb-1 text-gray-300">Center Longitude</label>
            <input
              type="number"
              value={centerLng}
              onChange={(e) => setCenterLng(parseFloat(e.target.value))}
              className="w-full p-2 rounded bg-slate-700 text-white"
            />
          </div>

          <div className="grid grid-cols-2 gap-3">
            <div>
              <label className="block text-sm mb-1 text-gray-300">Radius (m)</label>
              <input
                type="number"
                value={radius}
                onChange={(e) => setRadius(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-700 text-white"
              />
            </div>
            <div>
              <label className="block text-sm mb-1 text-gray-300">Points</label>
              <input
                type="number"
                value={points}
                onChange={(e) => setPoints(parseInt(e.target.value))}
                className="w-full p-2 rounded bg-slate-700 text-white"
              />
            </div>
          </div>

          <div>
            <label className="block text-sm mb-1 text-gray-300">Altitude (m)</label>
            <input
              type="number"
              value={altitude}
              onChange={(e) => setAltitude(parseFloat(e.target.value))}
              className="w-full p-2 rounded bg-slate-700 text-white"
            />
          </div>
        </div>

        <div className="flex justify-end gap-3 mt-6">
          <button
            onClick={onClose}
            className="px-4 py-2 rounded-md bg-gray-600 hover:bg-gray-700 transition"
          >
            Cancel
          </button>
          <button
            onClick={handleGenerate}
            className="px-4 py-2 rounded-md bg-green-600 hover:bg-green-700 transition font-semibold"
          >
            Generate
          </button>
        </div>
      </div>
    </div>
  );
};

export default CircleTool;
