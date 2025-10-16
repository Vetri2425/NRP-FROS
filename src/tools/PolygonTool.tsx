// File: src/tools/PolygonTool.tsx

import React, { useState } from "react";
import { generatePolygonSweep } from "../utils/mission_generator";
import { Waypoint } from "../types";

type PolygonToolProps = {
  onGenerate: (waypoints: Waypoint[]) => void;
  onClose: () => void;
};

const PolygonTool: React.FC<PolygonToolProps> = ({ onGenerate, onClose }) => {
  const [polygonText, setPolygonText] = useState(
    "13.0764,80.1559\n13.0764,80.1569\n13.0754,80.1569\n13.0754,80.1559"
  );
  const [spacing, setSpacing] = useState(5);
  const [altitude, setAltitude] = useState(30);
  const [direction, setDirection] = useState(90);

  const handleGenerate = () => {
    const points = polygonText
      .split("\n")
      .map((line) => line.trim())
      .filter(Boolean)
      .map((line) => {
        const [latStr, lngStr] = line.split(/[,\s]+/);
        return { lat: parseFloat(latStr), lng: parseFloat(lngStr) };
      });

    if (points.length < 3) {
      alert("A polygon must have at least 3 vertices.");
      return;
    }

    try {
      const waypoints = generatePolygonSweep(points, spacing, altitude, direction);
      onGenerate(waypoints);
      onClose();
    } catch (err) {
      console.error("Polygon generation error:", err);
      alert("Error generating polygon mission. Check coordinates or spacing.");
    }
  };

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 backdrop-blur-sm flex items-center justify-center z-[2000]">
      <div className="bg-slate-800 rounded-xl shadow-lg p-6 w-[480px] text-white">
        <h2 className="text-lg font-bold mb-4 text-center">🗺️ Generate Polygon Mission</h2>

        <div className="space-y-3">
          <div>
            <label className="block text-sm mb-1 text-gray-300">
              Polygon Coordinates (lat,lng per line)
            </label>
            <textarea
              rows={5}
              value={polygonText}
              onChange={(e) => setPolygonText(e.target.value)}
              className="w-full p-2 rounded bg-slate-700 text-white font-mono text-sm"
            />
            <p className="text-xs text-gray-400 mt-1">
              Example:  
              <br />
              13.0764, 80.1559  
              <br />
              13.0764, 80.1569  
              <br />
              13.0754, 80.1569  
              <br />
              13.0754, 80.1559
            </p>
          </div>

          <div className="grid grid-cols-3 gap-3">
            <div>
              <label className="block text-sm mb-1 text-gray-300">Spacing (m)</label>
              <input
                type="number"
                value={spacing}
                onChange={(e) => setSpacing(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-700 text-white"
              />
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
            <div>
              <label className="block text-sm mb-1 text-gray-300">Direction (°)</label>
              <input
                type="number"
                value={direction}
                onChange={(e) => setDirection(parseFloat(e.target.value))}
                className="w-full p-2 rounded bg-slate-700 text-white"
              />
            </div>
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

export default PolygonTool;
