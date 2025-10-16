// src/components/ServoControl/StatusPanel.tsx
import React from "react";

export default function StatusPanel({ status }: any) {
  return (
    <div className="bg-slate-800 border border-slate-700 p-4 rounded-lg shadow mb-4">
      <h3 className="font-semibold mb-2 text-slate-200">Running Status</h3>
      {Object.keys(status).length === 0 ? (
        <p className="text-slate-400">Loading...</p>
      ) : (
        Object.entries(status as any).map(([mode, info]: any) => (
          <div key={mode} className="flex justify-between border-b border-slate-700 py-1 text-slate-200">
            <span>{String(mode).toUpperCase()}</span>
            <span className={info?.running ? "text-green-400" : "text-slate-500"}>
              {info?.running ? "RUNNING ✅" : "Stopped ❌"}
            </span>
          </div>
        ))
      )}
    </div>
  );
}
