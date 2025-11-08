import React from 'react';
import { ChevronDownIcon } from '../icons/ChevronDownIcon';

const MapSelector: React.FC = () => {
  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden mb-2">
      <header className="bg-indigo-700 px-3 py-2 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className="text-indigo-300">🗺️</span>
          <span className="font-semibold text-white tracking-wide text-sm">Map Settings</span>
        </div>
        <span className="text-xs text-indigo-100 uppercase">Grid & Display</span>
      </header>

      <div className="p-2">
        {/* Grid picker */}
        <div className="flex items-center justify-between text-white">
          <span className="text-xs">Grid</span>
          <div className="flex items-center space-x-1 text-xs">
            <select className="bg-slate-700 text-white px-1.5 py-0.5 rounded text-xs">
              <option>GoogleHybridMap</option>
              <option>OpenStreetMap</option>
              <option>GoogleSatellite</option>
            </select>
            <ChevronDownIcon className="w-3 h-3" />
          </div>
        </div>

        <div className="text-xs text-green-400">Status: loaded tiles</div>
      </div>
    </div>
  );
};

export default MapSelector;