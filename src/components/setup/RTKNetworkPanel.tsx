import React, { useState } from 'react';

interface Props {
  mode: 'TCP' | 'UDP';
  onLog?: {
    info?: (msg: string, data?: unknown) => void;
    warn?: (msg: string, data?: unknown) => void;
    error?: (msg: string, data?: unknown) => void;
    success?: (msg: string, data?: unknown) => void;
    debug?: (msg: string, data?: unknown) => void;
  };
}

const RTKNetworkPanel: React.FC<Props> = ({ mode, onLog }) => {
  const [host, setHost] = useState<string>('');
  const [port, setPort] = useState<number>(2101);
  const [username, setUsername] = useState<string>('');
  const [password, setPassword] = useState<string>('');
  const [status, setStatus] = useState<string>('idle');
  const [message, setMessage] = useState<string | null>(null);

  const handleConnect = async () => {
    setStatus('connecting');
    setMessage(null);
    try {
      const resp = await fetch('/api/rtk/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: mode.toLowerCase(), host, port, username, password }),
      });
      const j = await resp.json().catch(() => ({}));
      if (resp.ok && (j.success || j.connected)) {
        setStatus('connected');
        setMessage(j.message ?? 'Connected');
        onLog?.success?.('RTK network connected', { mode, host, port });
        return;
      }
      setStatus('error');
      setMessage(j.message ?? 'Failed to connect');
      onLog?.error?.('RTK network connect failed', j);
    } catch (err) {
      setStatus('error');
      setMessage(String(err));
      onLog?.error?.('RTK network connect exception', err);
    }
  };

  const handleDisconnect = async () => {
    setStatus('disconnecting');
    try {
      const resp = await fetch('/api/rtk/disconnect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: mode.toLowerCase() }),
      });
      const j = await resp.json().catch(() => ({}));
      setStatus('idle');
      setMessage(j.message ?? 'Disconnected');
      onLog?.info?.('RTK network disconnected', j);
    } catch (err) {
      setStatus('error');
      setMessage(String(err));
      onLog?.error?.('RTK network disconnect exception', err);
    }
  };

  return (
    <div className="bg-[#0b1220] rounded-lg p-3 border border-slate-700">
      <div className="flex items-center justify-between mb-2">
        <div className="text-sm font-semibold">{mode} RTK</div>
        <div className="text-xs text-slate-400">Status: <span className={status === 'connected' ? 'text-emerald-300' : status === 'connecting' ? 'text-yellow-300' : 'text-slate-400'}>{status}</span></div>
      </div>

      <div className="grid grid-cols-2 gap-2">
        <div>
          <label className="text-xs text-slate-300">Host</label>
          <input className="w-full bg-slate-800 px-2 py-1 rounded text-white" value={host} onChange={(e) => setHost(e.target.value)} />
        </div>
        <div>
          <label className="text-xs text-slate-300">Port</label>
          <input type="number" className="w-full bg-slate-800 px-2 py-1 rounded text-white" value={port} onChange={(e) => setPort(Number(e.target.value))} />
        </div>
        {mode === 'TCP' && (
          <>
            <div>
              <label className="text-xs text-slate-300">Username</label>
              <input className="w-full bg-slate-800 px-2 py-1 rounded text-white" value={username} onChange={(e) => setUsername(e.target.value)} />
            </div>
            <div>
              <label className="text-xs text-slate-300">Password</label>
              <input type="password" className="w-full bg-slate-800 px-2 py-1 rounded text-white" value={password} onChange={(e) => setPassword(e.target.value)} />
            </div>
          </>
        )}
      </div>

      <div className="mt-3 flex items-center gap-2">
        <button onClick={handleConnect} className="px-3 py-1 rounded bg-indigo-600 hover:bg-indigo-500">Connect</button>
        <button onClick={handleDisconnect} className="px-3 py-1 rounded bg-slate-700 hover:bg-slate-600">Disconnect</button>
        {message && <div className="text-xs text-slate-300 ml-2">{message}</div>}
      </div>
    </div>
  );
};

export default RTKNetworkPanel;
