import React from 'react';
import type { RoverTelemetry } from '../../types/ros';

const mapFixTypeToLabel = (fixType: number | undefined) => {
  switch (fixType) {
    // 0: No GPS, 1: No Fix, 2: 2D Fix, 3: 3D Fix (GPS), 4: DGPS, 5: RTK Float, 6: RTK Fixed
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
};

interface Props {
  telemetry: RoverTelemetry;
  rtkMode: string;
}

const BaseStatusPanel: React.FC<Props> = ({ telemetry, rtkMode }) => {
  const lat = telemetry.global?.lat ?? 0;
  const lon = telemetry.global?.lon ?? 0;
  const alt = telemetry.global?.alt_rel ?? 0;
  const sats = telemetry.global?.satellites_visible ?? 0;
  const hrms = (telemetry as any).hrms ?? null;
  const vrms = (telemetry as any).vrms ?? null;
  const signal = telemetry.network?.wifi_signal_strength ?? telemetry.network?.wifi_rssi ?? null;
  const baselineAge = telemetry.rtk?.baseline_age ?? null;
  const fixLabel = mapFixTypeToLabel(telemetry.rtk?.fix_type);
  const lastTs = telemetry.lastMessageTs ? new Date(telemetry.lastMessageTs).toLocaleTimeString() : '—';

  return (
    <div className="bg-[#0b1220] rounded-lg p-3 border border-slate-700">
      <div className="flex items-center justify-between mb-2">
        <div className="text-sm font-semibold">Base Station Status</div>
        <div className="text-xs text-slate-400">Mode: <span className="uppercase">{rtkMode}</span></div>
      </div>

      <div className="grid grid-cols-2 gap-2 text-xs text-slate-300">
        <div>
          <p className="text-slate-400 text-[10px] uppercase">Latitude</p>
          <div className="font-mono">{lat !== 0 ? lat.toFixed(7) : '—'}</div>
        </div>
        <div>
          <p className="text-slate-400 text-[10px] uppercase">Longitude</p>
          <div className="font-mono">{lon !== 0 ? lon.toFixed(7) : '—'}</div>
        </div>

        <div>
          <p className="text-slate-400 text-[10px] uppercase">Altitude (rel)</p>
          <div>{alt !== 0 ? `${alt.toFixed(1)} m` : '—'}</div>
        </div>
        <div>
          <p className="text-slate-400 text-[10px] uppercase">Satellites</p>
          <div>{sats}</div>
        </div>

        <div>
          <p className="text-slate-400 text-[10px] uppercase">Signal</p>
          <div>{signal != null ? signal : '—'}</div>
        </div>
        <div>
          <p className="text-slate-400 text-[10px] uppercase">Last update</p>
          <div>{lastTs}</div>
        </div>

        <div>
          <p className="text-slate-400 text-[10px] uppercase">RTK Status</p>
          <div>{fixLabel}</div>
        </div>
        <div>
          <p className="text-slate-400 text-[10px] uppercase">Baseline Age</p>
          <div>{baselineAge != null ? `${baselineAge}s` : '—'}</div>
        </div>

        <div>
          <p className="text-slate-400 text-[10px] uppercase">HRMS</p>
          <div>{hrms != null ? `${Number(hrms).toFixed(2)} m` : '—'}</div>
        </div>
        <div>
          <p className="text-slate-400 text-[10px] uppercase">VRMS</p>
          <div>{vrms != null ? `${Number(vrms).toFixed(2)} m` : '—'}</div>
        </div>

        <div className="col-span-2">
          <p className="text-slate-400 text-[10px] uppercase">Input / Output Data Rate</p>
          <div className="text-slate-300 text-xs">N/A</div>
        </div>
      </div>
    </div>
  );
};

export default BaseStatusPanel;
