import React from 'react';
import { useRover } from '../context/RoverContext';
import { TelemetryRtk } from '../types/ros';

// GPS fix type mapping according to GPSRAW message standard:
// 0-1: No Fix, 2: 2D Fix, 3: 3D Fix, 4: DGPS, 5: RTK Float, 6: RTK Fixed
const FIX_LABELS: Record<number, string> = {
  0: 'No Fix',
  1: 'No Fix',
  2: '2D Fix',
  3: '3D Fix',
  4: 'DGPS',
  5: 'RTK Float',
  6: 'RTK Fixed',
};

const RTKPanel_Backup: React.FC = () => {
  const {
    telemetry: { rtk, global },
  } = useRover();

  // Print RTK fix type for 5 seconds when component mounts
  React.useEffect(() => {
    const interval = setInterval(() => {
      console.log('[RTKPanel - 5sec Debug] RTK fix_type from backend:', rtk.fix_type);
    }, 1000);

    // Stop after 5 seconds
    const timeout = setTimeout(() => {
      clearInterval(interval);
      console.log('[RTKPanel] 5-second debug logging stopped');
    }, 5000);

    return () => {
      clearInterval(interval);
      clearTimeout(timeout);
    };
  }, []);

  // Debug logging for RTK fix type (only when it changes)
  React.useEffect(() => {
    console.log('[RTKPanel] RTK fix_type changed:', {
      fix_type: rtk.fix_type,
      base_linked: rtk.base_linked,
      baseline_age: rtk.baseline_age,
      satellites_visible: global.satellites_visible,
    });
  }, [rtk.fix_type, rtk.base_linked, rtk.baseline_age, global.satellites_visible]);

  const fixLabel = FIX_LABELS[rtk.fix_type] ?? `Fix ${rtk.fix_type}`;
  
  // Fix type color and animation logic
  const getFixTypeStyle = () => {
    if (rtk.fix_type === 6) {
      // RTK Fixed - Solid glowing green
      return 'bg-green-500 shadow-lg shadow-green-500/50';
    } else if (rtk.fix_type === 5) {
      // RTK Float - Blinking green
      return 'bg-green-500 animate-pulse';
    } else if (rtk.fix_type >= 2 && rtk.fix_type <= 4) {
      // DGPS/3D/2D - Orange
      return 'bg-orange-500';
    } else {
      // No Fix - Red
      return 'bg-red-500';
    }
  };

  // Satellite count health indicator
  // 15+ satellites = 100% (excellent)
  // Below 15 = 60-70% range (acceptable to poor)
  const getSatelliteStyle = () => {
    const satCount = global.satellites_visible;
    
    if (satCount >= 15) {
      // 15+ satellites - Green (100% health)
      return 'bg-green-500 shadow-lg shadow-green-500/50';
    } else if (satCount >= 10) {
      // 10-14 satellites - Orange (70% health range)
      return 'bg-orange-500';
    } else {
      // Below 10 satellites - Red (60% or below)
      return 'bg-red-500';
    }
  };

  // Base station is considered "linked" if we have DGPS or better (fix_type >= 4)
  // This means RTK corrections are being received, even if not fully locked
  const isBaseLinked = rtk.base_linked || rtk.fix_type >= 4;

  return (
    <div className="bg-[#111827] rounded-lg p-4 flex flex-col gap-3">
      <header className="flex items-center justify-between">
        <h3 className="text-white font-semibold uppercase tracking-wide text-sm">RTK Status</h3>
      </header>

      {/* Status Row - Two Sections Side by Side */}
      <div className="grid grid-cols-2 gap-4">
        {/* RTK Fix Type Section */}
        <div className="flex items-center gap-3">
          <div className="flex-shrink-0">
            {/* Rectangle box with circle indicator (Britannia logo style) */}
            <div className={`${getFixTypeStyle()} rounded-md p-1.5 flex items-center justify-center relative transition-all duration-300 w-16`}>
              {/* Small solid circle inside */}
              <div className={`${getFixTypeStyle()} w-2 h-2 rounded-full absolute top-0.5 right-0.5`}></div>
              <span className="text-white font-bold text-xs">{fixLabel}</span>
            </div>
          </div>
          <div className="flex-1 min-w-0">
            <p className="text-xs uppercase text-slate-400">RTK Fix Type</p>
          </div>
        </div>

        {/* Satellite Count Section */}
        <div className="flex items-center gap-3">
          <div className="flex-shrink-0">
            {/* Rectangle box with circle indicator */}
            <div className={`${getSatelliteStyle()} rounded-md p-1.5 flex items-center justify-center relative transition-all duration-300 w-16`}>
              {/* Small solid circle inside */}
              <div className={`${getSatelliteStyle()} w-2 h-2 rounded-full absolute top-0.5 right-0.5`}></div>
              <span className="text-white font-bold text-lg">{global.satellites_visible}</span>
            </div>
          </div>
          <div className="flex-1 min-w-0">
            <p className="text-xs uppercase text-slate-400">Satellites</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RTKPanel_Backup;
