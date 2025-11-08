import React, { useEffect, useState } from 'react';

interface Props {
  onLog?: {
    info?: (msg: string, data?: unknown) => void;
    warn?: (msg: string, data?: unknown) => void;
    error?: (msg: string, data?: unknown) => void;
    success?: (msg: string, data?: unknown) => void;
    debug?: (msg: string, data?: unknown) => void;
  };
}

const defaultBaudRates = [9600, 19200, 38400, 57600, 115200, 230400];

const RTKSerialPanel: React.FC<Props> = ({ onLog }) => {
  const [ports, setPorts] = useState<string[]>([]);
  const [selectedPort, setSelectedPort] = useState<string | null>(null);
  const [baud, setBaud] = useState<number>(115200);
  const [status, setStatus] = useState<string>('idle');
  const [message, setMessage] = useState<string | null>(null);

  useEffect(() => {
    // Try to fetch available serial ports from backend; fall back to common names
    const fetchPorts = async () => {
      try {
        const res = await fetch('/api/serial/ports');
        if (res.ok) {
          const json = await res.json();
          if (Array.isArray(json.ports)) {
            setPorts(json.ports);
            if (json.ports.length > 0) setSelectedPort(json.ports[0]);
            onLog?.info?.('Fetched serial ports from backend', json.ports);
            return;
          }
        }
      } catch (e) {
        // ignore
      }
      // fallback
      const fallback = ['ttyACM0', 'ttyUSB0', 'ttyTHS0'];
      setPorts(fallback);
      setSelectedPort(fallback[0]);
      onLog?.warn?.('Using fallback serial port list', fallback);
    };
    fetchPorts();
  }, [onLog]);

  const handleConnect = async () => {
    if (!selectedPort) return;
    setStatus('connecting');
    setMessage(null);
    try {
      const resp = await fetch('/api/rtk/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'serial', port: selectedPort, baud }),
      });
      const j = await resp.json().catch(() => ({}));
      if (resp.ok && (j.success || j.connected)) {
        setStatus('connected');
        setMessage(j.message ?? 'Connected');
        onLog?.success?.('Serial RTK connected', { port: selectedPort, baud });
        return;
      }
      setStatus('error');
      setMessage(j.message ?? 'Failed to connect');
      onLog?.error?.('Serial RTK connect failed', j);
    } catch (err) {
      setStatus('error');
      setMessage(String(err));
      onLog?.error?.('Serial RTK connect exception', err);
    }
  };

  const handleDisconnect = async () => {
    setStatus('disconnecting');
    try {
      const resp = await fetch('/api/rtk/disconnect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'serial' }),
      });
      const j = await resp.json().catch(() => ({}));
      setStatus('idle');
      setMessage(j.message ?? 'Disconnected');
      onLog?.info?.('Serial RTK disconnected', j);
    } catch (err) {
      setStatus('error');
      setMessage(String(err));
      onLog?.error?.('Serial RTK disconnect exception', err);
    }
  };

  return (
    <div className="bg-[#0b1220] rounded-lg p-3 border border-slate-700">
      <div className="flex items-center justify-between mb-2">
        <div className="text-sm font-semibold">Serial RTK</div>
        <div className="text-xs text-slate-400">Status: <span className={status === 'connected' ? 'text-emerald-300' : status === 'connecting' ? 'text-yellow-300' : 'text-slate-400'}>{status}</span></div>
      </div>

      <div className="grid grid-cols-2 gap-2">
        <div>
          <label className="text-xs text-slate-300">Port</label>
          <select className="w-full bg-slate-800 px-2 py-1 rounded text-white" value={selectedPort ?? ''} onChange={(e) => setSelectedPort(e.target.value)}>
            {ports.map((p) => <option key={p} value={p}>{p}</option>)}
          </select>
        </div>
        <div>
          <label className="text-xs text-slate-300">Baud</label>
          <select className="w-full bg-slate-800 px-2 py-1 rounded text-white" value={baud} onChange={(e) => setBaud(Number(e.target.value))}>
            {defaultBaudRates.map((b) => <option key={b} value={b}>{b}</option>)}
          </select>
        </div>
      </div>

      <div className="mt-3 flex items-center gap-2">
        <button onClick={handleConnect} className="px-3 py-1 rounded bg-indigo-600 hover:bg-indigo-500">Connect</button>
        <button onClick={handleDisconnect} className="px-3 py-1 rounded bg-slate-700 hover:bg-slate-600">Disconnect</button>
        {message && <div className="text-xs text-slate-300 ml-2">{message}</div>}
      </div>
    </div>
  );
};

export default RTKSerialPanel;
