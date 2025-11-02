import React from 'react';
import { useRover } from '../context/RoverContext';
import { Wifi, WifiOff, Radio, Battery, Cable } from 'lucide-react';

interface TooltipProps {
  text: string;
  children: React.ReactNode;
}

const Tooltip: React.FC<TooltipProps> = ({ text, children }) => {
  return (
    <div className="relative group">
      {children}
      <div className="absolute bottom-full left-1/2 -translate-x-1/2 mb-2 px-2 py-1 bg-gray-900 text-white text-xs rounded opacity-0 group-hover:opacity-100 transition-opacity pointer-events-none whitespace-nowrap z-10">
        {text}
        <div className="absolute top-full left-1/2 -translate-x-1/2 -mt-1 border-4 border-transparent border-t-gray-900"></div>
      </div>
    </div>
  );
};

const StatusBar: React.FC = () => {
  const {
    telemetry,
    connectionState,
    roverPosition,
  } = useRover();

  const { battery, network } = telemetry;

  // Log only when connection state changes
  React.useEffect(() => {
    console.log('[DATA FLOW] 📶 StatusBar - Connection state changed:', connectionState);
  }, [connectionState]);

  // Network Signal (Jetson) - based on real network data
  const getNetworkStatus = () => {
    // Safe defaults
    const connectionType = network?.connection_type || 'none';
    const wifiSignalStrength = network?.wifi_signal_strength || 0;
    const wifiConnected = network?.wifi_connected || false;
    
    // Ethernet connection
    if (connectionType === 'ethernet') {
      return {
        icon: Cable,
        color: 'text-green-400',
        label: 'Jetson: Ethernet - Excellent',
      };
    }
    
    // WiFi connection
    if (connectionType === 'wifi' && wifiConnected) {
      if (wifiSignalStrength >= 4) {
        return {
          icon: Wifi,
          color: 'text-green-400',
          label: 'Jetson: WiFi - Excellent',
        };
      } else if (wifiSignalStrength === 3) {
        return {
          icon: Wifi,
          color: 'text-green-400',
          label: 'Jetson: WiFi - Good',
        };
      } else if (wifiSignalStrength === 2) {
        return {
          icon: Wifi,
          color: 'text-yellow-400',
          label: 'Jetson: WiFi - Fair',
        };
      } else if (wifiSignalStrength === 1) {
        return {
          icon: Wifi,
          color: 'text-red-400',
          label: 'Jetson: WiFi - Weak',
        };
      }
    }
    
    // Disconnected or no connection
    return {
      icon: WifiOff,
      color: 'text-gray-500',
      label: 'Jetson: Disconnected',
    };
  };

  // LoRa Status - using real backend data
  const getLoRaStatus = () => {
    const isConnected = network?.lora_connected || false;
    return {
      connected: isConnected,
      color: isConnected ? 'text-green-400' : 'text-gray-500',
      tooltip: isConnected ? 'LoRa: Connected' : 'LoRa: Disconnected',
    };
  };

  // Battery Status
  const getBatteryStatus = () => {
    const percentage = battery.percentage;
    let color = 'text-green-400';
    if (percentage < 30) {
      color = 'text-red-400';
    } else if (percentage < 60) {
      color = 'text-yellow-400';
    }
    return {
      percentage,
      color,
      tooltip: `Battery: ${percentage.toFixed(0)}%`,
    };
  };

  const networkStatus = getNetworkStatus();
  const loraStatus = getLoRaStatus();
  const batteryStatus = getBatteryStatus();

  return (
    <>
      <style>{`
        .status-bar-panel {
          background: #111827;
          border-radius: 8px;
          overflow: hidden;
          width: 100%;
          position: relative;
          top: -10%;
        }

        .status-bar-header {
          background: #4f46e5;
          padding: 8px 12px;
          display: flex;
          align-items: center;
          border-radius: 8px 8px 0 0;
        }

        .status-bar-title {
          color: white;
          font-size: 14px;
          font-weight: 600;
          letter-spacing: 0.025em;
        }

        .status-bar-content {
          padding: 14px 16px;
          border-radius: 0 0 8px 8px;
        }
      `}</style>
      
      <div className="status-bar-panel">
        {/* Header */}
        <div className="status-bar-header">
          <span className="status-bar-title">SYSTEM STATUS</span>
        </div>

        {/* Content */}
        <div className="status-bar-content">
          <div className="flex items-center gap-5">
            {/* Network Signal (Jetson) */}
            <Tooltip text={networkStatus.label}>
              <div className={`${networkStatus.color} transition-colors`}>
                <networkStatus.icon size={20} strokeWidth={2} />
              </div>
            </Tooltip>

            {/* LoRa */}
            <Tooltip text={loraStatus.tooltip}>
              <div className={`${loraStatus.color} transition-colors`}>
                <Radio size={20} strokeWidth={2} />
              </div>
            </Tooltip>

            {/* Battery with Percentage Inside */}
            <Tooltip text={batteryStatus.tooltip}>
              <div className="relative">
                <Battery 
                  size={28} 
                  strokeWidth={2}
                  className={`${batteryStatus.color} transition-colors`}
                  fill={batteryStatus.percentage > 20 ? 'currentColor' : 'none'}
                />
                <span className={`absolute inset-0 flex items-center justify-center text-[9px] font-bold ${batteryStatus.percentage > 20 ? 'text-gray-900' : batteryStatus.color} transition-colors`}>
                  {batteryStatus.percentage.toFixed(0)}
                </span>
              </div>
            </Tooltip>
          </div>
        </div>
      </div>
    </>
  );
};

export default StatusBar;
