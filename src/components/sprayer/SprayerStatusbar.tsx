import React from 'react';
import { Waypoint, RoverData } from '../../types';
// You can add sprayer-specific icons here if needed

export type SprayerStatusbarProps = {
  missionName: string | null;
  waypoints: Waypoint[];
  liveRoverData: RoverData;
};

const SprayerStatusbar: React.FC<SprayerStatusbarProps> = ({ missionName, waypoints, liveRoverData }) => {
  // You can customize this logic for sprayer-specific status
  // For now, reuse the same logic as LiveStatusbar
  const {
    completedWaypointIds = [],
    activeWaypointIndex,
    distanceToNext,
    battery,
    rtk_status,
    imu_status,
    satellites_visible,
    fix_type
  } = liveRoverData;

  const activeWaypointId =
    activeWaypointIndex !== null && activeWaypointIndex !== undefined
      ? activeWaypointIndex + 1
      : null;

  const lastCompletedWpId = completedWaypointIds.length > 0 ? completedWaypointIds[completedWaypointIds.length - 1] : null;
  const lastCompletedWp = waypoints.find(wp => wp.id === lastCompletedWpId);
  const nextWp = waypoints.find(wp => wp.id === activeWaypointId);

  let deltaElevation = 0;
  if (lastCompletedWp) {
    const prevIndex = waypoints.findIndex(wp => wp.id === lastCompletedWp.id) - 1;
    if (prevIndex >= 0) {
      deltaElevation = lastCompletedWp.alt - waypoints[prevIndex].alt;
    }
  }

  const batteryPercentage = battery ? Math.round(battery) : 0;
  const batteryStatus = batteryPercentage > 20 ? `OK ${batteryPercentage}%` : `LOW ${batteryPercentage}%`;

  // Format HRMS and VRMS
  const hrmsValue = typeof liveRoverData.hrms === 'number' ? liveRoverData.hrms.toFixed(3) : liveRoverData.hrms;
  const vrmsValue = typeof liveRoverData.vrms === 'number' ? liveRoverData.vrms.toFixed(3) : liveRoverData.vrms;

  return (
    <div className="h-full text-white">
      {/* Status Panel Only */}
      <div className="bg-[#111827] rounded-lg p-2 flex flex-col">
        <div className="border-b border-gray-700 pb-2 mb-3">
          <h3 className="font-bold text-gray-400 text-center">Sprayer Status</h3>
        </div>
        <div className="space-y-2">
          <div className="flex justify-between items-center text-xs">
            <span className="text-sm">Battery</span>
            <span className="font-mono text-sm">{batteryStatus}</span>
          </div>
          <div className="flex justify-between items-center text-xs">
            <span className="text-sm">GPS/RTK</span>
            <span className="font-mono text-sm">{rtk_status || 'No Fix'}</span>
          </div>
          <div className="flex justify-between items-center text-xs">
            <span className="text-sm">Satellites</span>
            <span className="font-mono text-sm">{satellites_visible ?? 0}</span>
          </div>
          <div className="flex justify-between items-center text-xs">
            <span className="text-sm">HRMS</span>
            <span className="font-mono text-sm">{hrmsValue} m</span>
          </div>
          <div className="flex justify-between items-center text-xs">
            <span className="text-sm">VRMS</span>
            <span className="font-mono text-sm">{vrmsValue} m</span>
          </div>
          <div className="flex justify-between items-center text-xs">
            <span className="text-sm">IMU</span>
            <span className="font-mono text-sm">{imu_status || 'ALIGNED'}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SprayerStatusbar;
