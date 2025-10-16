
import React from 'react';
import MissionControls from './MissionControls';
import StatusPanel from './StatusPanel';
import { MissionFileInfo, Waypoint } from '../types';
import { RoverData } from '../hooks/useRoverConnection';
import LogsPanel from './LogsPanel';
import { useMissionLogs } from '../hooks/useMissionLogs';

type LeftSidebarProps = {
  onMissionUpload: (waypoints: Waypoint[], info: MissionFileInfo) => void;
  onUploadInitiated: () => void;
  onClearMission: () => void;
  roverData: RoverData;
  isConnected: boolean;
  onChangeMode: (mode: string) => void;
  onArmDisarm: () => void;
  missionLogs: ReturnType<typeof useMissionLogs>['missionLogs'];
  missionFileInfo: MissionFileInfo | null;
};

const LeftSidebar: React.FC<LeftSidebarProps> = ({ 
  onMissionUpload, 
  onUploadInitiated, 
  onClearMission, 
  roverData,
  isConnected,
  onChangeMode,
  onArmDisarm,
  missionLogs,
  missionFileInfo,
}) => {
  return (
    <aside className="w-1/4 max-w-xs flex flex-col gap-4">
      <MissionControls 
        onMissionUpload={onMissionUpload} 
        onUploadInitiated={onUploadInitiated} 
        onClearMission={onClearMission}
        roverMode={roverData.mode}
        roverStatus={roverData.status}
        isConnected={isConnected}
        onChangeMode={onChangeMode}
        onArmDisarm={onArmDisarm}
        currentMissionInfo={missionFileInfo}
      />
      <StatusPanel 
        roverData={roverData}
        isConnected={isConnected}
      />
      <LogsPanel missionLogs={missionLogs} />
    </aside>
  );
};

export default LeftSidebar;
