// src/components/ServoControl/ModeSelector.tsx
import React from "react";
import { useDialog } from '../../hooks/useDialog';
import GenericDialog from '../GenericDialog';

export default function ModeSelector({ selectedMode, setSelectedMode, JETSON_API, status, onRefresh }: any) {
  const [loading, setLoading] = React.useState<null | 'start' | 'stop'>(null);
  const { dialogState, showConfirm, showAlert } = useDialog();

  const startMode = async () => {
    try {
      setLoading('start');
      // Check if another mode is running first
      const statusRes = await fetch(`${JETSON_API}/status`);
      const st = await statusRes.json().catch(() => ({} as any));
      const running = Object.entries(st || {}).find(([m, info]: any) => info?.running);
      let replace = false;
      if (running && running[0] !== selectedMode) {
        const confirmed = await showConfirm('Mode Conflict', `Mode "${running[0]}" is currently RUNNING.\nDo you want to stop it and switch to "${selectedMode}"?`);
        if (!confirmed) return;
        replace = true;
      }
      const res = await fetch(`${JETSON_API}/run?mode=${encodeURIComponent(selectedMode)}${replace ? '&replace=1' : ''}`);
      const data = await res.json().catch(() => ({}));
      if (res.status === 409) {
        // Backend refused due to another running mode. Ask again.
        const cur = (data as any)?.current_mode;
        const confirmed = await showConfirm('Mode Conflict', `Mode "${cur}" is RUNNING. Switch to "${selectedMode}"?`);
        if (!confirmed) return;
        const res2 = await fetch(`${JETSON_API}/run?mode=${encodeURIComponent(selectedMode)}&replace=1`);
        const data2 = await res2.json().catch(() => ({}));
        if (!res2.ok) throw new Error((data2 as any).error || 'Failed to start');
        await showAlert('Mode Started', (data2 as any).status || `Started ${selectedMode} mode!`);
      } else {
        if (!res.ok) throw new Error((data as any).error || 'Failed to start');
        await showAlert('Mode Started', (data as any).status || `Started ${selectedMode} mode!`);
      }
    } catch (e: any) {
      await showAlert('Mode Start Failed', e?.message || `Failed to start ${selectedMode}`);
    } finally {
      setLoading(null);
      onRefresh && onRefresh();
    }
  };
  const stopMode = async () => {
    try {
      setLoading('stop');
      const res = await fetch(`${JETSON_API}/stop?mode=${encodeURIComponent(selectedMode)}`);
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(data.error || 'Failed to stop');
      await showAlert('Mode Stopped', data.status || `Stopped ${selectedMode} mode!`);
    } catch (e: any) {
      await showAlert('Mode Stop Failed', e?.message || `Failed to stop ${selectedMode}`);
    } finally {
      setLoading(null);
      onRefresh && onRefresh();
    }
  };

  return (
    <>
      <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg shadow">
      <h3 className="font-semibold mb-2 text-slate-200">Select Script Mode</h3>
      <select
        value={selectedMode}
        onChange={(e) => setSelectedMode(e.target.value)}
        className="bg-slate-900 border border-slate-700 p-2 rounded w-full text-white"
      >
        <option value="wpmark">WP Mark (Active)</option>
        <option value="continuous">Continuous Line</option>
        <option value="interval">Interval Spray</option>
      </select>

      <div className="flex gap-2 mt-3 items-center">
        <button
          onClick={startMode}
          disabled={loading !== null}
          className={`px-3 py-2 rounded text-white ${loading === 'start' ? 'bg-green-700 opacity-70' : 'bg-green-600 hover:bg-green-700'}`}
        >
          {loading === 'start' ? 'Starting…' : 'Start'}
        </button>
        <button
          onClick={stopMode}
          disabled={loading !== null}
          className={`px-3 py-2 rounded text-white ${loading === 'stop' ? 'bg-red-700 opacity-70' : 'bg-red-600 hover:bg-red-700'}`}
        >
          {loading === 'stop' ? 'Stopping…' : 'Stop'}
        </button>
        <div className="ml-auto text-xs text-slate-400">
          {status && status[selectedMode]?.running ? (
            <span className="text-green-400">{selectedMode} is RUNNING</span>
          ) : (
            <span className="text-slate-500">{selectedMode} is stopped</span>
          )}
        </div>
      </div>
      </div>
    <GenericDialog
      isOpen={dialogState.isOpen}
      type={dialogState.type}
      title={dialogState.title}
      message={dialogState.message}
      onConfirm={dialogState.onConfirm}
      onCancel={dialogState.onCancel}
      confirmText={dialogState.confirmText}
      cancelText={dialogState.cancelText}
    />
    </>
  );
}