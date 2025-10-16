// src/components/setup/RTKInjectorPanel.tsx
import React from "react";
import { useRtk } from "../../context/RtkContext";

const RTKInjectorPanel: React.FC = () => {
  const {
    isConnected,
    isConnecting,
    status,
    bytes,
    logs,
    caster,
    setCaster,
    startStream,
    stopStream,
    clearLogs,
  } = useRtk();

  // 🟢 CONNECT Button Handler
  const handleConnect = () => {
    if (!caster.host || !caster.mountpoint) {
      alert("Please fill in host and mountpoint!");
      return;
    }
    startStream();
  };

  // 🔴 DISCONNECT Button Handler
  const handleDisconnect = () => {
    stopStream();
  };

  // 💾 Download logs as text file
  const handleDownloadLogs = () => {
    const blob = new Blob([logs.join("\n")], { type: "text/plain" });
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = `rtk_log_${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;
    link.click();
    URL.revokeObjectURL(link.href);
  };

  // 🧹 Clear console
  const handleClearConsole = () => {
    clearLogs();
  };

  // Handle input changes
  const onInput = (e: React.ChangeEvent<HTMLInputElement>) => {
    setCaster({ ...caster, [e.target.name]: e.target.value } as any);
  };

  // Determine status color
  const getStatusColor = () => {
    if (isConnected) {
      return 'text-green-400';
    }
    if (isConnecting) {
      return 'text-yellow-400 animate-pulse';
    }
    if (status === 'Error') {
      return 'text-red-400';
    }
    return 'text-gray-400';
  };

  // Get status icon
  const getStatusIcon = () => {
    if (isConnected) return '✅';
    if (isConnecting) return '⏳';
    if (status === 'Error') return '❌';
    return '⚪';
  };

  return (
    <div className="p-6 bg-slate-900 text-white rounded-xl shadow-xl max-w-4xl">
      <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
        🛰️ RTK Correction Bridge
      </h2>
      
      <p className="text-gray-400 mb-6 text-sm">
        Connect to an NTRIP caster to receive real-time GPS corrections for centimeter-level accuracy.
      </p>

      {/* Connection Settings Form */}
      <div className="grid grid-cols-2 gap-3 mb-4">
        <div>
          <label className="block text-xs text-gray-400 mb-1">Caster Host</label>
          <input
            name="host"
            value={caster.host}
            onChange={onInput}
            placeholder="caster.emlid.com"
            disabled={isConnected || isConnecting}
            className="w-full p-2 text-black rounded focus:ring-2 focus:ring-blue-500 disabled:bg-gray-700 disabled:text-gray-400"
          />
        </div>
        
        <div>
          <label className="block text-xs text-gray-400 mb-1">Port</label>
          <input
            name="port"
            value={caster.port}
            onChange={onInput}
            placeholder="2101"
            disabled={isConnected || isConnecting}
            className="w-full p-2 text-black rounded focus:ring-2 focus:ring-blue-500 disabled:bg-gray-700 disabled:text-gray-400"
          />
        </div>
        
        <div>
          <label className="block text-xs text-gray-400 mb-1">Mountpoint</label>
          <input
            name="mountpoint"
            value={caster.mountpoint}
            onChange={onInput}
            placeholder="RTCM3"
            disabled={isConnected || isConnecting}
            className="w-full p-2 text-black rounded focus:ring-2 focus:ring-blue-500 disabled:bg-gray-700 disabled:text-gray-400"
          />
        </div>
        
        <div>
          <label className="block text-xs text-gray-400 mb-1">Username</label>
          <input
            name="user"
            value={caster.user}
            onChange={onInput}
            placeholder="username"
            disabled={isConnected || isConnecting}
            className="w-full p-2 text-black rounded focus:ring-2 focus:ring-blue-500 disabled:bg-gray-700 disabled:text-gray-400"
          />
        </div>
        
        <div className="col-span-2">
          <label className="block text-xs text-gray-400 mb-1">Password</label>
          <input
            name="password"
            type="password"
            value={caster.password}
            onChange={onInput}
            placeholder="password"
            disabled={isConnected || isConnecting}
            className="w-full p-2 text-black rounded focus:ring-2 focus:ring-blue-500 disabled:bg-gray-700 disabled:text-gray-400"
          />
        </div>
      </div>

      {/* Control Buttons */}
      <div className="flex gap-2 flex-wrap mb-4">
        <button
          onClick={handleConnect}
          className="bg-green-600 hover:bg-green-700 disabled:bg-gray-600 disabled:cursor-not-allowed px-6 py-2 rounded font-semibold transition"
          title={isConnected ? "Already connected" : isConnecting ? "Connecting..." : "Start RTK stream"}
        >
          {isConnecting ? '⏳ Connecting...' : isConnected ? '✅ Connected' : '🟢 Start'}
        </button>
        
        <button
          onClick={handleDisconnect}
          className="bg-red-600 hover:bg-red-700 disabled:bg-gray-600 disabled:cursor-not-allowed px-6 py-2 rounded font-semibold transition"
          title={isConnected || isConnecting ? "Stop RTK stream" : "Not connected"}
        >
          🔴 Stop
        </button>
        
        <button
          onClick={handleDownloadLogs}
          className="bg-blue-600 hover:bg-blue-700 px-6 py-2 rounded font-semibold transition"
          title="Download console logs as text file"
        >
          💾 Download Logs
        </button>
        
        <button
          onClick={handleClearConsole}
          className="bg-yellow-600 hover:bg-yellow-700 px-6 py-2 rounded font-semibold transition"
          title="Clear console window"
        >
          🧹 Clear Console
        </button>
      </div>

      {/* Status Display */}
      <div className="grid grid-cols-2 gap-4 mb-4 p-4 bg-slate-800 rounded">
        <div>
          <p className="text-xs text-gray-400 mb-1">Connection Status</p>
          <p className={`text-lg font-bold ${getStatusColor()} flex items-center gap-2`}>
            <span>{getStatusIcon()}</span>
            <span>{status}</span>
          </p>
        </div>
        <div>
          <p className="text-xs text-gray-400 mb-1">Data Forwarded to Rover</p>
          <p className="text-lg font-bold text-blue-400">
            {bytes < 1024 
              ? `${bytes} bytes`
              : bytes < 1024 * 1024
              ? `${(bytes / 1024).toFixed(2)} KB`
              : `${(bytes / (1024 * 1024)).toFixed(2)} MB`
            }
          </p>
        </div>
      </div>

      {/* Console Log */}
      <div className="bg-black rounded p-3 h-72 overflow-y-auto font-mono text-xs border border-gray-700">
        {logs.length === 0 ? (
          <p className="text-gray-500 italic">Console logs will appear here...</p>
        ) : (
          logs.map((log, idx) => (
            <div 
              key={idx} 
              className={`mb-1 ${
                log.includes('❌') ? 'text-red-400' :
                log.includes('✅') ? 'text-green-400' :
                log.includes('⚠️') ? 'text-yellow-400' :
                log.includes('📡') ? 'text-blue-400' :
                'text-gray-300'
              }`}
            >
              {log}
            </div>
          ))
        )}
      </div>

      {/* Help Section */}
      <div className="mt-4 p-3 bg-blue-900 bg-opacity-30 rounded text-sm border border-blue-700">
        <p className="font-semibold mb-2 text-blue-300">💡 Quick Tips:</p>
        <ul className="list-disc list-inside space-y-1 text-gray-300 text-xs">
          <li><strong>Step 1:</strong> Make sure your rover is connected first (see header status)</li>
          <li><strong>Step 2:</strong> Get mountpoint credentials from your NTRIP provider</li>
          <li><strong>Step 3:</strong> Fill in all fields above and click "Start"</li>
          <li><strong>RTK Fix:</strong> Typically takes 30-60 seconds to achieve after connection</li>
          <li><strong>Free Casters:</strong> Try UNAVCO, Emlid Caster, or local government services</li>
          <li><strong>Troubleshooting:</strong> Check logs below for detailed error messages</li>
        </ul>
      </div>

      {/* Connection Requirements Warning */}
      {!isConnected && !isConnecting && (
        <div className="mt-3 p-3 bg-orange-900 bg-opacity-30 rounded text-sm border border-orange-700">
          <p className="font-semibold text-orange-300 mb-1">⚠️ Before Starting RTK:</p>
          <p className="text-gray-300 text-xs">
            Ensure your rover is connected and you have valid NTRIP credentials. 
            The rover must be outdoors with clear sky view for RTK to work.
          </p>
        </div>
      )}

      {/* Success Indicator */}
      {isConnected && (
        <div className="mt-3 p-3 bg-green-900 bg-opacity-30 rounded text-sm border border-green-700 animate-pulse">
          <p className="font-semibold text-green-300 mb-1">✅ RTK Stream Active!</p>
          <p className="text-gray-300 text-xs">
            Correction data is being forwarded to your rover. 
            Check your rover's GPS status for "RTK Float" or "RTK Fixed".
          </p>
        </div>
      )}
    </div>
  );
};

export default RTKInjectorPanel;
