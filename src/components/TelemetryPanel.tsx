import React from 'react';
import { useRover } from '../context/RoverContext';

const formatTimestamp = (ts: number | null): string => {
  if (!ts) {
    return '—';
  }
  const date = new Date(ts);
  return Number.isNaN(date.getTime()) ? '—' : date.toLocaleTimeString();
};

const safeFixed = (value: number, digits: number): string => {
  if (!Number.isFinite(value)) {
    return '—';
  }
  return value.toFixed(digits);
};

const TelemetryPanel: React.FC = () => {
  const { telemetry, connectionState, roverPosition } = useRover();
  const { state, global, battery, mission, rtk, lastMessageTs } = telemetry;

  // Log when specific RTK values change (not on every render)
  React.useEffect(() => {
    console.log('[DATA FLOW] 📊 TelemetryPanel - RTK data updated:', {
      rtk_fix_type: rtk.fix_type,
      rtk_base_linked: rtk.base_linked,
      satellites: global.satellites_visible,
    });
  }, [rtk.fix_type, rtk.base_linked, global.satellites_visible]);

  const armed = state.armed ? 'Yes' : 'No';
  const rtkLabel = (() => {
    switch (rtk.fix_type) {
      // 0: No GPS, 1: No Fix, 2: 2D Fix, 3: 3D Fix, 4: DGPS, 5: RTK Float, 6: RTK Fixed
      case 0:
        return 'No GPS';
      case 1:
        return 'No Fix';
      case 2:
        return '2D Fix';
      case 3:
        return '3D Fix';
      case 4:
        return 'DGPS';
      case 5:
        return 'RTK Float';
      case 6:
        return 'RTK Fixed';
      default:
        return 'No Fix';
    }
  })();

  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden flex flex-col">
      <header className="bg-indigo-700 px-3 py-2 flex items-center justify-between">
        <span className="font-semibold text-white tracking-wide text-sm">Telemetry</span>
        <span className="text-xs text-indigo-100 uppercase">{connectionState}</span>
      </header>

      <div className="p-3 flex flex-col gap-2.5 text-xs text-slate-200">
        <section className="grid grid-cols-2 gap-2">
          <div>
            <p className="text-xs uppercase text-slate-400">Mode</p>
            <p className="font-semibold">{state.mode || 'UNKNOWN'}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Armed</p>
            <p className={`font-semibold ${state.armed ? 'text-green-400' : 'text-red-400'}`}>
              {armed}
            </p>
          </div>
          <div className="col-span-2">
            <p className="text-xs uppercase text-slate-400">Heartbeat</p>
            <p className="font-semibold font-mono text-xs">{formatTimestamp(state.heartbeat_ts)}</p>
          </div>
        </section>

        <section className="grid grid-cols-2 gap-2">
          <div>
            <p className="text-xs uppercase text-slate-400">Latitude</p>
            <p className="font-mono text-xs">{safeFixed(global.lat, 7)}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Longitude</p>
            <p className="font-mono text-xs">{safeFixed(global.lon, 7)}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Altitude (rel)</p>
            <p className="font-semibold text-xs">{safeFixed(global.alt_rel, 1)} m</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Ground Speed</p>
            <p className="font-semibold text-xs">{safeFixed(global.vel, 2)} m/s</p>
          </div>
        </section>

        <section className="grid grid-cols-2 gap-2">
          <div>
            <p className="text-xs uppercase text-slate-400">Mission</p>
            <p className="font-semibold text-xs">{mission.status}</p>
          </div>
          <div>
            <p className="text-xs uppercase text-slate-400">Progress</p>
            <p className="font-semibold text-xs">
              {safeFixed(mission.progress_pct, 1)}% ({mission.current_wp}/{mission.total_wp})
            </p>
          </div>
        </section>

        {/* RTK and Satellites info removed; now shown in RTKPanel */}

        <footer className="text-xs text-slate-500 pt-1 border-t border-slate-700">
          <div className="truncate">Last: {formatTimestamp(lastMessageTs)}</div>
        </footer>
      </div>
    </div>
  );
};

export default TelemetryPanel;
