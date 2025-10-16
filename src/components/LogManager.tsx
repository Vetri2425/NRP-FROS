
import React, { useState } from 'react';
import { MissionLog } from '../types';
import LogPreviewModal from './LogPreviewModal';
import { exportLogsToCSV } from '../utils/logExporter';

interface Props {
  logs: MissionLog[];
}

const LogManager: React.FC<Props> = ({ logs }) => {
  const [selectedLog, setSelectedLog] = useState<MissionLog | null>(null);

  const getStatusColor = (status: MissionLog['status']) => {
    switch (status) {
      case 'Completed': return 'text-green-400';
      case 'In Progress': return 'text-yellow-400 animate-pulse';
      case 'Incomplete': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  return (
    <div className="text-sm h-full flex flex-col">
      {logs.length === 0 ? (
        <div className="flex-1 flex items-center justify-center text-gray-500 h-full">
            No mission logs to display.
        </div>
      ) : (
        <ul className="space-y-2">
          {logs.map((log) => (
            <li key={log.id} className="bg-[#1F2937] p-2 rounded-md flex justify-between items-center">
              <div className="flex-1 min-w-0">
                <p className="text-gray-200 font-semibold truncate" title={log.name}>{log.name}</p>
                <div className="text-xs text-gray-400 flex items-center gap-2">
                  <span className={getStatusColor(log.status)}>{log.status}</span>
                  <span>–</span>
                  <span>{log.timestamp.toLocaleTimeString()}</span>
                </div>
              </div>
              <div className="space-x-2 flex-shrink-0 ml-2">
                <button 
                    onClick={() => setSelectedLog(log)} 
                    className="px-2 py-1 text-xs bg-blue-600 text-white rounded-md hover:bg-blue-700 transition-colors disabled:bg-gray-600 disabled:cursor-not-allowed"
                    disabled={log.entries.length === 0}
                    aria-label={`Preview log for ${log.name}`}
                >
                  Preview
                </button>
                <button 
                    onClick={() => exportLogsToCSV(log.entries)} 
                    className="px-2 py-1 text-xs bg-green-600 text-white rounded-md hover:bg-green-700 transition-colors disabled:bg-gray-600 disabled:cursor-not-allowed"
                    disabled={log.entries.length === 0}
                    aria-label={`Download log for ${log.name}`}
                >
                  Download
                </button>
              </div>
            </li>
          ))}
        </ul>
      )}

      {selectedLog && (
        <LogPreviewModal log={selectedLog} onClose={() => setSelectedLog(null)} />
      )}
    </div>
  );
};

export default LogManager;
