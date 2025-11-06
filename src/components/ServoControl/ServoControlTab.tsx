// src/components/ServoControl/ServoControlTab.tsx
import React, { useEffect, useState } from "react";
import ModeSelector from "./ModeSelector";
import ConfigEditor from "./ConfigEditor";
import ModeDescription from "./ModeDescription";
import StatusPanel from "./StatusPanel";
import LogViewer from "./LogViewer";
import ReportPanel from "./ReportPanel";
import { BACKEND_URL } from "../../config";

export default function ServoControlTab() {
  const [status, setStatus] = useState<any>({});
  const [selectedMode, setSelectedMode] = useState<string>("wpmark");
  const [logText, setLogText] = useState<string>("");

  // Use the same backend URL from config (reads from .env or uses current hostname)
  const JETSON_API = `${BACKEND_URL}/servo`;

  // Helper to fetch status
  const refreshStatus = async () => {
    try {
      const res = await fetch(`${JETSON_API}/status`);
      if (!res.ok) {
        // Backend not available - silently fail
        return;
      }
      const data = await res.json();
      setStatus(data);
    } catch (err) {
      // Backend not running - this is expected if servo backend is not started
      // Silently fail to avoid console spam
    }
  };

  // Fetch status every 2s
  useEffect(() => {
    refreshStatus();
    const interval = setInterval(refreshStatus, 2000);
    return () => clearInterval(interval);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); // Empty dependency array since JETSON_API is constant

  return (
    <div className="w-full h-full overflow-y-auto custom-scrollbar p-3 grid grid-cols-2 gap-3 text-white">
      <div className="flex flex-col gap-3">
        <h2 className="text-lg font-bold text-orange-400">Servo Control Center</h2>
        <ModeSelector
          selectedMode={selectedMode}
          setSelectedMode={setSelectedMode}
          JETSON_API={JETSON_API}
          status={status}
          onRefresh={refreshStatus}
        />
        <ConfigEditor
          selectedMode={selectedMode}
          JETSON_API={JETSON_API}
          status={status}
          onRefresh={refreshStatus}
        />
        <ModeDescription selectedMode={selectedMode} />
      </div>

      <div className="flex flex-col gap-3">
        <StatusPanel status={status} />
        <LogViewer logText={logText} setLogText={setLogText} JETSON_API={JETSON_API} status={status} />
        <ReportPanel status={status} />
      </div>
    </div>
  );
}
