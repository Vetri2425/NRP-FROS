import React from 'react';
import { useRover } from '../context/RoverContext';
import { TelemetryRtk, TelemetryGlobal } from '../types/ros';

const getRTKStatus = (fixType: number) => {
  const statusMap: Record<number, { label: string; description: string; boxClass: string; icon: string }> = {
    0: { label: 'No Fix', description: 'No Signal', boxClass: 'rtk-nofix', icon: '✕' },
    1: { label: 'No Fix', description: 'No Signal', boxClass: 'rtk-nofix', icon: '✕' },
    2: { label: '2D Fix', description: '2 Dimensions', boxClass: 'rtk-dgps', icon: '◐' },
    3: { label: '3D Fix', description: '3 Dimensions', boxClass: 'rtk-dgps', icon: '◑' },
    4: { label: 'DGPS', description: 'Corrections Active', boxClass: 'rtk-dgps', icon: '◑' },
    5: { label: 'RTK Float', description: 'Converging...', boxClass: 'rtk-float', icon: '◐' },
    6: { label: 'RTK Fixed', description: 'Connected & Locked', boxClass: 'rtk-fixed', icon: '✓' },
  };
  return statusMap[fixType] || { label: `Fix ${fixType}`, description: 'Unknown', boxClass: 'rtk-nofix', icon: '?' };
};

const getSatelliteStatus = (satelliteCount: number) => {
  // Realistic satellite count scale (GPS + GLONASS + Galileo + BeiDou = max ~32 satellites)
  // 24+ satellites = 100% (Excellent - multi-constellation coverage)
  // 20-23 satellites = 80-99% (Very Good)
  // 15-19 satellites = 60-79% (Good - adequate for RTK)
  // 10-14 satellites = 40-59% (Fair)
  // 6-9 satellites = 20-39% (Poor)
  // <6 satellites = <20% (Critical)
  
  const maxSatellites = 24; // Realistic maximum for excellent coverage
  const percentage = Math.min((satelliteCount / maxSatellites) * 100, 100);

  let boxClass = 'sat-red';
  let description = '';

  if (satelliteCount >= 24) {
    boxClass = 'sat-full-green';
    description = 'Excellent';
  } else if (satelliteCount >= 20) {
    boxClass = 'sat-medium-green';
    description = 'Very Good';
  } else if (satelliteCount >= 15) {
    boxClass = 'sat-light-green';
    description = 'Good';
  } else if (satelliteCount >= 10) {
    boxClass = 'sat-orange';
    description = 'Fair';
  } else if (satelliteCount >= 6) {
    boxClass = 'sat-light-orange';
    description = 'Poor';
  } else {
    boxClass = 'sat-red';
    description = 'Critical';
  }

  return {
    boxClass,
    description,
    percentage: Math.round(percentage),
  };
};

const RTKPanel: React.FC = () => {
  const {
    telemetry: { rtk, global },
    roverPosition,
  } = useRover();

  // Log only when RTK fix type changes (the most important metric)
  React.useEffect(() => {
    console.log('[DATA FLOW] 📡 RTKPanel - Fix type changed:', {
      rtk_fix_type: rtk.fix_type,
      satellites: global.satellites_visible,
    });
  }, [rtk.fix_type]);

  const rtkStatus = getRTKStatus(rtk.fix_type);
  const satelliteStatus = getSatelliteStatus(global.satellites_visible);

  return (
    <>
      <style>{`
        .status-panel {
          background: #111827;
          border-radius: 8px;
          overflow: hidden;
          width: 100%;
          font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        }

        .panel-header {
          background: #4f46e5;
          padding: 8px 12px;
          display: flex;
          align-items: center;
          justify-content: space-between;
        }

        .panel-title {
          color: white;
          font-size: 14px;
          font-weight: 600;
          letter-spacing: 0.025em;
        }

        .panel-subtitle {
          color: #c7d2fe;
          font-size: 12px;
          text-transform: uppercase;
        }

        .panel-content {
          padding: 12px;
        }

        .status-section {
          margin-bottom: 14px;
        }

        .status-section:last-child {
          margin-bottom: 0;
        }

        .section-label {
          color: #94a3b8;
          font-size: 12px;
          text-transform: uppercase;
          letter-spacing: 0.025em;
          margin-bottom: 8px;
          display: block;
        }

        .status-box {
          border-radius: 3px;
          padding: 8px;
          display: flex;
          align-items: center;
          justify-content: space-between;
          transition: all 0.3s ease;
        }

        .status-content {
          display: flex;
          flex-direction: column;
          gap: 4px;
        }

        .status-value {
          font-size: 14px;
          font-weight: 600;
          letter-spacing: -0.25px;
        }

        .status-description {
          font-size: 12px;
          opacity: 0.8;
        }

        .status-icon {
          font-size: 16px;
        }

        /* RTK Fixed - Solid Glowing Green */
        .rtk-fixed {
          background: linear-gradient(135deg, rgba(34, 197, 94, 0.2), rgba(34, 197, 94, 0.1));
          border: 1px solid #22c55e;
          color: #22c55e;
          box-shadow: 0 0 10px rgba(34, 197, 94, 0.5), inset 0 0 10px rgba(34, 197, 94, 0.1);
          animation: glow-green 2s ease-in-out infinite;
        }

        @keyframes glow-green {
          0%, 100% {
            box-shadow: 0 0 10px rgba(34, 197, 94, 0.5), inset 0 0 10px rgba(34, 197, 94, 0.1);
          }
          50% {
            box-shadow: 0 0 15px rgba(34, 197, 94, 0.8), inset 0 0 15px rgba(34, 197, 94, 0.2);
          }
        }

        /* RTK Float - Blinking Green */
        .rtk-float {
          background: linear-gradient(135deg, rgba(34, 197, 94, 0.15), rgba(34, 197, 94, 0.05));
          border: 1px solid #22c55e;
          color: #22c55e;
          animation: blink-green 1.5s ease-in-out infinite;
        }

        @keyframes blink-green {
          0%, 100% {
            opacity: 1;
            box-shadow: 0 0 5px rgba(34, 197, 94, 0.3);
          }
          50% {
            opacity: 0.5;
            box-shadow: 0 0 2.5px rgba(34, 197, 94, 0.1);
          }
        }

        /* RTK DGPS/3D/2D - Orange */
        .rtk-dgps {
          background: linear-gradient(135deg, rgba(251, 146, 60, 0.15), rgba(251, 146, 60, 0.05));
          border: 1px solid #fb923c;
          color: #fb923c;
        }

        /* RTK No Fix - Red */
        .rtk-nofix {
          background: linear-gradient(135deg, rgba(239, 68, 68, 0.2), rgba(239, 68, 68, 0.1));
          border: 1px solid #ef4444;
          color: #ef4444;
        }

        /* Satellite - Full Green (90%+) */
        .sat-full-green {
          background: linear-gradient(135deg, rgba(34, 197, 94, 0.2), rgba(34, 197, 94, 0.1));
          border: 1px solid #22c55e;
          color: #22c55e;
        }

        /* Satellite - Medium Green (70-90%) */
        .sat-medium-green {
          background: linear-gradient(135deg, rgba(74, 222, 128, 0.15), rgba(74, 222, 128, 0.05));
          border: 1px solid #4ade80;
          color: #4ade80;
        }

        /* Satellite - Light Green (60-70%) */
        .sat-light-green {
          background: linear-gradient(135deg, rgba(134, 239, 172, 0.15), rgba(134, 239, 172, 0.05));
          border: 1px solid #86efac;
          color: #86efac;
        }

        /* Satellite - Orange (40-60%) */
        .sat-orange {
          background: linear-gradient(135deg, rgba(251, 146, 60, 0.15), rgba(251, 146, 60, 0.05));
          border: 1px solid #fb923c;
          color: #fb923c;
        }

        /* Satellite - Light Orange (20-40%) */
        .sat-light-orange {
          background: linear-gradient(135deg, rgba(253, 186, 116, 0.15), rgba(253, 186, 116, 0.05));
          border: 1px solid #fdba74;
          color: #fdba74;
        }

        /* Satellite - Red (<20%) */
        .sat-red {
          background: linear-gradient(135deg, rgba(239, 68, 68, 0.2), rgba(239, 68, 68, 0.1));
          border: 1px solid #ef4444;
          color: #ef4444;
        }
      `}</style>

      <div className="status-panel">
        <header className="panel-header">
          <span className="panel-title">RTK Status</span>
          <span className="panel-subtitle">GPS Control</span>
        </header>

        <div className="panel-content">
          {/* RTK Fix Type Section */}
          <div className="status-section">
            <span className="section-label">RTK Fix Type</span>
            <div className={`status-box ${rtkStatus.boxClass}`}>
              <div className="status-content">
                <div className="status-value">{rtkStatus.label}</div>
                <div className="status-description">{rtkStatus.description}</div>
              </div>
              <div className="status-icon">{rtkStatus.icon}</div>
            </div>
          </div>

          {/* Satellite Count Section */}
          <div className="status-section">
            <span className="section-label">Satellite Count</span>
            <div className={`status-box ${satelliteStatus.boxClass}`}>
              <div className="status-content">
                <div className="status-value">{global.satellites_visible}</div>
                <div className="status-description">{satelliteStatus.description} ({satelliteStatus.percentage}%)</div>
              </div>
              <div className="status-icon">📡</div>
            </div>
          </div>
        </div>
      </div>
    </>
  );
};

export default RTKPanel;
