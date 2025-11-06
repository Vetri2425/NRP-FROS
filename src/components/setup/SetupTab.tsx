import React, { useCallback, useMemo, useState } from "react";
import RTKInjectorPanel from "./RTKInjectorPanel.tsx";
import RTKSerialPanel from './RTKSerialPanel.tsx';
import RTKNetworkPanel from './RTKNetworkPanel.tsx';
import BaseMapPanel from './BaseMapPanel.tsx';
import BaseStatusPanel from './BaseStatusPanel.tsx';
import { useRover } from '../../context/RoverContext';
import SetupConsole, { SetupLogEntry, SetupLogLevel } from "./SetupConsole"; // kept for log model, not shown by default

type RTKMode = 'NTRIP' | 'Serial' | 'TCP' | 'UDP';

const SetupTab: React.FC = () => {
  const [logs, setLogs] = useState<SetupLogEntry[]>([]);
  const maxLog = 1500;
  const [mode, setMode] = useState<RTKMode>('NTRIP');

  const { telemetry } = useRover();

  const pushLog = useCallback(
    (level: SetupLogLevel, msg: string, data?: unknown) => {
      setLogs((prev) => {
        const next = [...prev, { t: Date.now(), level, msg, data }];
        return next.length > maxLog ? next.slice(-maxLog) : next;
      });
    },
    []
  );

  const onClear = useCallback(() => setLogs([]), []);

  const rtkOnLog = useMemo(() => ({
    info: (msg: string, data?: unknown) => pushLog('info', msg, data),
    warn: (msg: string, data?: unknown) => pushLog('warn', msg, data),
    error: (msg: string, data?: unknown) => pushLog('error', msg, data),
    success: (msg: string, data?: unknown) => pushLog('success', msg, data),
    debug: (msg: string, data?: unknown) => pushLog('debug', msg, data),
  }), [pushLog]);

  return (
    <div className="w-full h-full overflow-y-auto custom-scrollbar p-3 text-white bg-slate-900 space-y-3">
      <div>
        <h1 className="text-xl font-bold mb-3">📡 RTK Setup</h1>
        <p className="text-gray-300 text-sm mb-4">Configure RTK correction streams and related features here.</p>

        <div className="flex items-center gap-3 mb-4">
          <label className="text-sm text-slate-300">Mode</label>
          <select value={mode} onChange={(e) => setMode(e.target.value as RTKMode)} className="bg-slate-800 text-white px-2 py-1 rounded border border-slate-700">
            <option value="NTRIP">NTRIP</option>
            <option value="Serial">Serial</option>
            <option value="TCP">TCP</option>
            <option value="UDP">UDP</option>
          </select>
        </div>

        {/* Mode-specific area */}
        <div className="mb-4">
          {mode === 'NTRIP' && <RTKInjectorPanel onLog={rtkOnLog} />}
          {mode === 'Serial' && <RTKSerialPanel onLog={rtkOnLog} />}
          {(mode === 'TCP' || mode === 'UDP') && (
            <RTKNetworkPanel mode={mode} onLog={rtkOnLog} />
          )}
        </div>

        {/* Two-panel status area: map + base/status */}
        <div className="grid grid-cols-2 gap-3">
          <div>
            <BaseMapPanel basePosition={telemetry.global && telemetry.global.lat !== 0 && telemetry.global.lon !== 0 ? { lat: telemetry.global.lat, lng: telemetry.global.lon } : null} />
          </div>
          <div>
            <BaseStatusPanel telemetry={telemetry} rtkMode={mode} />
          </div>
        </div>
      </div>

      {/* Hidden log console for development; kept for now but not shown prominently */}
      <div className="hidden">
        <SetupConsole entries={logs} onClear={onClear} />
      </div>
    </div>
  );
};

export default SetupTab;
