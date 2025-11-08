
import React from 'react';
import { Waypoint, RoverData } from '../../types';
import { BatteryIcon } from '../icons/BatteryIcon';
import { RtkIcon } from '../icons/RtkIcon';
import { ImuIcon } from '../icons/ImuIcon';
import { WPMarkStatus } from '../../services/wpMarkService';

type LiveStatusbarProps = {
  missionName: string | null;
  waypoints: Waypoint[];
  liveRoverData: RoverData;
  wpMarkStatus?: WPMarkStatus | null;
};

const StatusItem: React.FC<{ label: string; value: string | number; unit?: string; status?: string }> = ({ label, value, unit, status }) => {
  let statusClass = '';
  if (status === 'ALIGNED' || status?.includes('RTK Fixed')) {
    statusClass = 'bg-green-600 text-white';
  } else if (status?.includes('OK') && !status.includes('LOW')) {
    statusClass = 'bg-green-600 text-white';
  } else if (status?.includes('LOW')) {
    statusClass = 'bg-red-600 text-white';
  } else if (status?.includes('RTK Float') || status?.includes('DGPS')) {
    statusClass = 'bg-yellow-600 text-white';
  } else if (status) {
    statusClass = 'bg-blue-500 text-white';
  }

  return (
    <div className="flex justify-between items-center text-xs">
      <div className="flex items-center gap-2 text-gray-300">
        {label === 'Battery' && <BatteryIcon level={typeof value === 'number' ? value : undefined} className="w-4 h-4" />}
        {label === 'GPS' && <RtkIcon className="w-4 h-4 text-blue-400" />}
        {label === 'IMU' && <ImuIcon className="w-4 h-4 text-yellow-400" />}
        <span className="text-sm">{label}</span>
      </div>
      <div className="font-mono flex items-center gap-2 text-sm">
        {value}
        {unit && <span className="text-gray-400">{unit}</span>}
        {status && <span className={`text-xs font-bold px-2 py-0.5 rounded-full ${statusClass}`}>{status}</span>}
      </div>
    </div>
  );
};

const InfoItem: React.FC<{ label: string; children: React.ReactNode }> = ({ label, children }) => (
  <div className="text-center">
    <p className="text-xs text-gray-400">{label}</p>
    <div className="mt-1">{children}</div>
  </div>
);

const LiveStatusbar: React.FC<LiveStatusbarProps> = ({ missionName, waypoints, liveRoverData, wpMarkStatus }) => {
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

  // 🔍 DEBUG: Log what LiveStatusbar receives
  React.useEffect(() => {
    console.log('[LiveStatusbar] Received liveRoverData:', {
      rtk_status,
      fix_type,
      satellites_visible,
      battery,
      'has_position': !!liveRoverData.position
    });
  }, [rtk_status, fix_type, satellites_visible, battery, liveRoverData.position]);

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
  
  // Determine IMU status
  const imuStatusText = imu_status && imu_status !== 'UNKNOWN' ? imu_status : 'ALIGNED';
  
  // Satellites
  const satellitesCount = satellites_visible ?? 0;

  // 🔍 DEBUG: Log final display values
  console.log('[LiveStatusbar] Display values:', {
    'GPS/RTK status': rtk_status || 'No Fix',
    'Satellites': satellitesCount,
    'Battery': batteryStatus
  });

  return (
    <div className="h-full grid grid-cols-3 gap-3 text-white">
      {/* Panel 1: Status */}
      <div className="bg-[#111827] rounded-lg p-2 flex flex-col">
        <div className="border-b border-gray-700 pb-2 mb-3">
          <h3 className="font-bold text-gray-400 text-center">Status</h3>
        </div>
        <div className="space-y-2">
            <StatusItem label="Battery" value="" status={batteryStatus} />
            <StatusItem label="GPS/RTK" value="" status={rtk_status || 'No Fix'} />
            <StatusItem label="Satellites" value={satellitesCount} />
            <StatusItem label="HRMS" value={hrmsValue} unit="m" />
            <StatusItem label="VRMS" value={vrmsValue} unit="m" />
            <StatusItem label="IMU" value="" status={imuStatusText} />
            
            {/* WP_MARK Status */}
            {wpMarkStatus && wpMarkStatus.running && (
              <>
                <div className="border-t border-gray-600 pt-2 mt-2">
                  <h4 className="text-xs font-semibold text-orange-400 mb-1">🎯 WP_MARK Mission</h4>
                </div>
                <StatusItem label="Phase" value="" status={wpMarkStatus.current_phase.toUpperCase()} />
                <StatusItem label="Progress" value={`${wpMarkStatus.current_waypoint}/${wpMarkStatus.total_waypoints}`} />
                <StatusItem label="Uptime" value={wpMarkStatus.uptime_seconds.toFixed(1)} unit="s" />
                <div className="text-xs text-gray-400 mt-1 truncate" title={wpMarkStatus.last_action}>
                  {wpMarkStatus.last_action}
                </div>
              </>
            )}
        </div>
      </div>

      {/* Panel 2: Mission Grid */}
      <div className="bg-[#111827] rounded-lg p-2 flex flex-col justify-center items-center text-center">
        <h3 className="text-lg font-bold text-gray-300 truncate w-full" title={missionName || 'No Mission'}>{missionName || 'No Mission Loaded'}</h3>
        <p className="text-gray-400">{waypoints.length} points in current mission</p>
    <p className="text-3xl font-light mt-2">
      Distance: {Number.isFinite(distanceToNext) ? distanceToNext.toFixed(2) : '---'} <span className="text-xl text-gray-400">m</span>
    </p>
      </div>

      {/* Panel 3: Info */}
      <div className="bg-[#111827] rounded-lg p-2 flex items-center justify-around">
        <InfoItem label="Marked Point">
            <p className="text-xl font-bold bg-gray-600 px-3 py-1 rounded-md">{lastCompletedWp ? `p${lastCompletedWp.id}` : '---'}</p>
            {lastCompletedWp ? (
              <>
                <p className="text-xs text-gray-400 mt-1">Lat: {lastCompletedWp.lat.toFixed(7)}</p>
                <p className="text-xs text-gray-400">Lng: {lastCompletedWp.lng.toFixed(7)}</p>
                <p className="text-sm font-bold text-white bg-blue-600 px-2 py-1 mt-1 rounded-md">{lastCompletedWp.command || 'WAYPOINT'}</p>
              </>
            ) : (
              <p className="text-sm font-bold text-white bg-gray-500 px-2 py-1 mt-2 rounded-md">N/A</p>
            )}
        </InfoItem>
        <InfoItem label="Delta Elevation">
             <p className={`text-2xl font-bold p-2 rounded-md ${deltaElevation > 0 ? 'text-green-400' : deltaElevation < 0 ? 'text-red-400' : 'text-gray-400'}`}>
                {deltaElevation !== 0 ? (<>{deltaElevation >= 0 ? '+' : ''}{deltaElevation.toFixed(2)}' f {deltaElevation >= 0 ? '↑' : '↓'}</>) : ('---')}
             </p>
        </InfoItem>
        <InfoItem label="Next Point">
            <p className="text-xl font-bold bg-orange-500 px-3 py-1 rounded-md">{nextWp ? `p${nextWp.id}` : '---'}</p>
            {nextWp ? (
              <>
                <p className="text-xs text-gray-400 mt-1">Lat: {nextWp.lat.toFixed(7)}</p>
                <p className="text-xs text-gray-400">Lng: {nextWp.lng.toFixed(7)}</p>
                <p className="text-sm font-bold text-white bg-blue-600 px-2 py-1 mt-1 rounded-md">{nextWp.command || 'WAYPOINT'}</p>
              </>
            ) : (
              <p className="text-sm font-bold text-white bg-gray-500 px-2 py-1 mt-2 rounded-md">N/A</p>
            )}
        </InfoItem>
      </div>
    </div>
  );
};

export default LiveStatusbar;
