// src/components/ServoControl/LogViewer.tsx
import React, { useEffect } from "react";

export default function LogViewer({ status, JETSON_API, logText, setLogText }: any) {
  const fetchLogs = async () => {
    try {
      // pick the first running mode
      const runningMode = Object.keys(status).find((m) => status[m]?.running);
      if (!runningMode) return;
      const logPath = status[runningMode].log;
      if (!logPath) return;
      const res = await fetch(`${JETSON_API}/log?path=${encodeURIComponent(logPath)}`);
      if (!res.ok) return;
      const data = await res.json();
      if (data.log) setLogText(data.log);
    } catch (_) {
      // ignore
    }
  };

  useEffect(() => {
    const timer = setInterval(fetchLogs, 2000);
    return () => clearInterval(timer);
  }, [status]);

  return (
    <div className="bg-slate-900 border border-slate-700 p-3 rounded-lg h-64 flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <h3 className="font-semibold text-slate-200 text-sm">Logs</h3>
        <button
          onClick={() => setLogText("")}
          className="px-2 py-1 text-xs bg-red-600 hover:bg-red-700 text-white rounded transition-colors"
          title="Clear console"
        >
          Clear
        </button>
      </div>
      <div className="flex-1 overflow-y-auto custom-scrollbar">
        <pre className="whitespace-pre-wrap text-green-300 text-xs">{logText || "No logs yet..."}</pre>
      </div>
    </div>
  );
}
