import React, { useState, useEffect, useRef } from 'react';
import { LogsIcon } from '../icons/LogsIcon';
import { LogEntry } from '../../types';

type MissionLogsProps = {
    logEntries: LogEntry[];
    onDownload: () => void;
    onClear: () => void;
};

type MissionStatus = 'idle' | 'started' | 'active' | 'hold' | 'completed' | 'failed';

const MissionLogs: React.FC<MissionLogsProps> = ({ logEntries, onDownload, onClear }) => {
  const headers = ['#', 'Time', 'WP', 'Lat', 'Lng', 'Status', 'Spray', 'Remark'];
  const tableEndRef = useRef<HTMLDivElement>(null);
  const [autoScroll, setAutoScroll] = useState(true);
  const prevLogLengthRef = useRef(0);
  
  // Filters
  const [statusFilter, setStatusFilter] = useState<string>('all');
  const [waypointFilter, setWaypointFilter] = useState<string>('all');
  const [remarkFilter, setRemarkFilter] = useState<string>('all');
  const [timeFilter, setTimeFilter] = useState<string>('all');

  const missionEvents = logEntries;
  const hasLogs = missionEvents.length > 0;

  // Determine mission status
  const getMissionStatus = (): MissionStatus => {
    if (!hasLogs) return 'idle';
    
    const lastEntry = missionEvents[missionEvents.length - 1];
    const eventLower = lastEntry.event.toLowerCase();
    
    if (eventLower.includes('fail') || eventLower.includes('error')) {
      return 'failed';
    }
    if (eventLower.includes('complete') || eventLower.includes('finished')) {
      return 'completed';
    }
    if (eventLower.includes('hold') || eventLower.includes('pause')) {
      return 'hold';
    }
    if (eventLower.includes('start')) {
      return 'started';
    }
    if (hasLogs) {
      return 'active';
    }
    return 'idle';
  };

  const missionStatus = getMissionStatus();

  // Get status badge
  const getStatusBadge = () => {
    const statusConfig = {
      idle: { label: '⚪ Idle', color: '#6b7280', bg: '#374151' },
      started: { label: '🟡 Mission Started', color: '#fbbf24', bg: '#78350f' },
      active: { label: '🟢 Active', color: '#22c55e', bg: '#14532d' },
      hold: { label: '🟠 On Hold', color: '#f97316', bg: '#7c2d12' },
      completed: { label: '🔵 Completed', color: '#60a5fa', bg: '#1e3a8a' },
      failed: { label: '🔴 Failed', color: '#ef4444', bg: '#7f1d1d' },
    };
    return statusConfig[missionStatus];
  };

  const statusBadge = getStatusBadge();

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    if (autoScroll && missionEvents.length > prevLogLengthRef.current) {
      tableEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
    prevLogLengthRef.current = missionEvents.length;
  }, [missionEvents.length, autoScroll]);

  // Get unique values for filters
  const uniqueWaypoints = Array.from(new Set(missionEvents.map(e => e.waypointId).filter(w => w != null)));
  const uniqueStatuses = ['all', 'completed', 'error', 'stop', 'skip'];
  const uniqueRemarks = ['all', 'good', 'ng', 'skip'];
  
  // Generate time filter options (every 3 minutes)
  const getTimeFilterOptions = () => {
    const options = ['all'];
    if (!hasLogs) return options;
    
    const firstTime = new Date(missionEvents[0].timestamp);
    const lastTime = new Date(missionEvents[missionEvents.length - 1].timestamp);
    const diffMinutes = Math.ceil((lastTime.getTime() - firstTime.getTime()) / (1000 * 60));
    
    for (let i = 0; i <= diffMinutes; i += 3) {
      const time = new Date(firstTime.getTime() + i * 60 * 1000);
      options.push(time.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }));
    }
    return options;
  };

  // Filter logs
  const filteredLogs = missionEvents.filter(entry => {
    // Status filter
    if (statusFilter !== 'all') {
      const status = (entry.status || '').toLowerCase();
      if (!status.includes(statusFilter)) return false;
    }
    
    // Waypoint filter
    if (waypointFilter !== 'all') {
      if (entry.waypointId?.toString() !== waypointFilter) return false;
    }
    
    // Remark filter
    if (remarkFilter !== 'all') {
      const event = entry.event.toLowerCase();
      if (!event.includes(remarkFilter)) return false;
    }
    
    // Time filter
    if (timeFilter !== 'all') {
      const entryTime = new Date(entry.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
      if (entryTime !== timeFilter) return false;
    }
    
    return true;
  });

  // Enhanced CSV export
  const handleDownloadCSV = () => {
    const csvRows = [];
    
    // Title and metadata
    csvRows.push('MISSION LOG REPORT');
    csvRows.push(`Generated: ${new Date().toLocaleString()}`);
    csvRows.push(`Mission Status: ${statusBadge.label}`);
    csvRows.push(`Total Entries: ${missionEvents.length}`);
    csvRows.push('');
    
    // Headers
    const headerRow = headers.join(',');
    csvRows.push(headerRow);
    
    // Data rows
    missionEvents.forEach((entry, index) => {
      const timestamp = new Date(entry.timestamp).toLocaleTimeString();
      const waypoint = entry.waypointId ?? '-';
      const lat = entry.lat != null ? entry.lat.toFixed(6) : '-';
      const lng = entry.lng != null ? entry.lng.toFixed(6) : '-';
      const status = entry.status ? entry.status.toUpperCase() : '-';
      const spray = entry.servoAction ? entry.servoAction.toUpperCase() : '-';
      const remark = `"${entry.event.replace(/"/g, '""')}"`;
      
      const row = [index + 1, timestamp, waypoint, lat, lng, status, spray, remark].join(',');
      csvRows.push(row);
    });
    
    const csvContent = csvRows.join('\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `mission-log-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  return (
    <>
      <style>{`
        .excel-logs-panel {
          background: #ffffff;
          border: 2px solid #d1d5db;
          border-radius: 8px;
          overflow: hidden;
          height: 100%;
          display: flex;
          flex-direction: column;
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        }

        .excel-logs-header {
          background: linear-gradient(180deg, #2563eb 0%, #1d4ed8 100%);
          padding: 10px 16px;
          display: flex;
          align-items: center;
          justify-content: space-between;
          border-bottom: 2px solid #1e40af;
        }

        .excel-logs-title {
          color: white;
          font-size: 14px;
          font-weight: 700;
          letter-spacing: 0.025em;
          display: flex;
          align-items: center;
          gap: 10px;
        }

        .mission-status-badge {
          padding: 4px 12px;
          border-radius: 16px;
          font-size: 11px;
          font-weight: 700;
          letter-spacing: 0.5px;
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
        }

        .excel-logs-actions {
          display: flex;
          gap: 10px;
          align-items: center;
        }

        .filter-bar {
          background: #f3f4f6;
          padding: 8px 16px;
          display: flex;
          gap: 12px;
          align-items: center;
          border-bottom: 1px solid #d1d5db;
          flex-wrap: wrap;
        }

        .filter-group {
          display: flex;
          align-items: center;
          gap: 6px;
        }

        .filter-label {
          font-size: 11px;
          font-weight: 600;
          color: #4b5563;
          text-transform: uppercase;
        }

        .filter-select {
          padding: 4px 8px;
          border: 1px solid #d1d5db;
          border-radius: 4px;
          font-size: 12px;
          background: white;
          color: #1f2937;
          cursor: pointer;
          min-width: 100px;
        }

        .filter-select:focus {
          outline: none;
          border-color: #2563eb;
          box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.1);
        }

        .excel-logs-content {
          flex: 1;
          overflow: hidden;
          display: flex;
          flex-direction: column;
          background: #ffffff;
        }

        .excel-logs-table-container {
          flex: 1;
          overflow: auto;
          position: relative;
        }

        .excel-logs-table {
          width: 100%;
          border-collapse: collapse;
          font-size: 12px;
          color: #1f2937;
        }

        .excel-logs-table thead {
          position: sticky;
          top: 0;
          z-index: 10;
          background: linear-gradient(180deg, #e5e7eb 0%, #d1d5db 100%);
        }

        .excel-logs-table thead th {
          padding: 10px 12px;
          text-align: left;
          font-size: 11px;
          color: #374151;
          text-transform: uppercase;
          letter-spacing: 0.05em;
          font-weight: 700;
          border: 1px solid #9ca3af;
          background: #e5e7eb;
        }

        .excel-logs-table tbody tr {
          border: 1px solid #e5e7eb;
          transition: background-color 0.15s;
        }

        .excel-logs-table tbody tr:hover {
          background: #f0f9ff;
        }

        .excel-logs-table tbody tr:nth-child(even) {
          background: #f9fafb;
        }

        .excel-logs-table tbody tr:nth-child(even):hover {
          background: #f0f9ff;
        }

        .excel-logs-table tbody td {
          padding: 8px 12px;
          border: 1px solid #e5e7eb;
          white-space: nowrap;
        }

        .excel-logs-table tbody td.remark-cell {
          max-width: 300px;
          overflow: hidden;
          text-overflow: ellipsis;
          white-space: nowrap;
        }

        .cell-index {
          background: #f3f4f6;
          font-weight: 600;
          color: #6b7280;
          text-align: center;
          width: 50px;
        }

        .cell-time {
          font-family: 'Courier New', monospace;
          font-size: 11px;
          color: #4b5563;
        }

        .cell-wp {
          text-align: center;
          font-weight: 600;
          color: #2563eb;
        }

        .cell-coord {
          font-family: 'Courier New', monospace;
          font-size: 11px;
          color: #059669;
        }

        .cell-status {
          font-weight: 600;
        }

        .status-completed {
          color: #059669;
          background: #d1fae5;
          padding: 2px 6px;
          border-radius: 4px;
        }

        .status-error {
          color: #dc2626;
          background: #fee2e2;
          padding: 2px 6px;
          border-radius: 4px;
        }

        .status-active {
          color: #2563eb;
          background: #dbeafe;
          padding: 2px 6px;
          border-radius: 4px;
        }

        .status-pending {
          color: #d97706;
          background: #fed7aa;
          padding: 2px 6px;
          border-radius: 4px;
        }

        .cell-spray {
          font-weight: 600;
          color: #7c3aed;
        }

        .btn-icon {
          padding: 6px 10px;
          border-radius: 6px;
          font-size: 12px;
          font-weight: 600;
          transition: all 0.2s;
          border: none;
          cursor: pointer;
          display: flex;
          align-items: center;
          gap: 6px;
        }

        .btn-download {
          background: #059669;
          color: white;
        }

        .btn-download:hover:not(:disabled) {
          background: #047857;
          transform: translateY(-1px);
          box-shadow: 0 4px 6px rgba(5, 150, 105, 0.3);
        }

        .btn-download:disabled {
          background: #9ca3af;
          cursor: not-allowed;
          opacity: 0.6;
        }

        .btn-trash {
          background: transparent;
          color: white;
          padding: 4px;
          font-size: 16px;
          border-radius: 4px;
        }

        .btn-trash:hover:not(:disabled) {
          background: rgba(255, 255, 255, 0.2);
        }

        .btn-trash:disabled {
          opacity: 0.4;
          cursor: not-allowed;
        }

        .logs-count-badge {
          background: rgba(255, 255, 255, 0.2);
          color: white;
          padding: 3px 10px;
          border-radius: 12px;
          font-size: 11px;
          font-weight: 700;
        }

        .auto-scroll-toggle {
          display: flex;
          align-items: center;
          gap: 6px;
          font-size: 11px;
          color: white;
          cursor: pointer;
          user-select: none;
        }

        .auto-scroll-toggle input {
          cursor: pointer;
        }

        .empty-state {
          text-align: center;
          padding: 48px 24px;
          color: #6b7280;
        }

        .empty-state-icon {
          font-size: 48px;
          margin-bottom: 12px;
        }

        .empty-state-text {
          font-size: 14px;
          color: #9ca3af;
        }

        /* Scrollbar styling */
        .excel-logs-table-container::-webkit-scrollbar {
          width: 8px;
          height: 8px;
        }

        .excel-logs-table-container::-webkit-scrollbar-track {
          background: #f3f4f6;
        }

        .excel-logs-table-container::-webkit-scrollbar-thumb {
          background: #d1d5db;
          border-radius: 4px;
        }

        .excel-logs-table-container::-webkit-scrollbar-thumb:hover {
          background: #9ca3af;
        }
      `}</style>

      <div className="excel-logs-panel">
        {/* Header */}
        <div className="excel-logs-header">
          <div className="excel-logs-title">
            <LogsIcon className="w-5 h-5" />
            <span>MISSION LOGS</span>
            {hasLogs && <span className="logs-count-badge">{missionEvents.length}</span>}
            <span className="mission-status-badge" style={{ backgroundColor: statusBadge.bg, color: statusBadge.color }}>
              {statusBadge.label}
            </span>
          </div>
          <div className="excel-logs-actions">
            <label className="auto-scroll-toggle">
              <input 
                type="checkbox" 
                checked={autoScroll} 
                onChange={(e) => setAutoScroll(e.target.checked)}
              />
              Auto-scroll
            </label>
            <button
              onClick={handleDownloadCSV}
              disabled={!hasLogs}
              className="btn-icon btn-download"
              title="Download professional CSV report"
            >
              📥 Export CSV
            </button>
            <button
              onClick={onClear}
              disabled={!hasLogs}
              className="btn-icon btn-trash"
              title="Clear all logs"
            >
              🗑️
            </button>
          </div>
        </div>

        {/* Filter Bar */}
        <div className="filter-bar">
          <div className="filter-group">
            <span className="filter-label">Status:</span>
            <select 
              className="filter-select" 
              value={statusFilter} 
              onChange={(e) => setStatusFilter(e.target.value)}
            >
              {uniqueStatuses.map(status => (
                <option key={status} value={status}>{status.charAt(0).toUpperCase() + status.slice(1)}</option>
              ))}
            </select>
          </div>

          <div className="filter-group">
            <span className="filter-label">Waypoint:</span>
            <select 
              className="filter-select" 
              value={waypointFilter} 
              onChange={(e) => setWaypointFilter(e.target.value)}
            >
              <option value="all">All</option>
              {uniqueWaypoints.map(wp => (
                <option key={wp} value={wp?.toString()}>{`WP ${wp}`}</option>
              ))}
            </select>
          </div>

          <div className="filter-group">
            <span className="filter-label">Remark:</span>
            <select 
              className="filter-select" 
              value={remarkFilter} 
              onChange={(e) => setRemarkFilter(e.target.value)}
            >
              {uniqueRemarks.map(remark => (
                <option key={remark} value={remark}>{remark.charAt(0).toUpperCase() + remark.slice(1)}</option>
              ))}
            </select>
          </div>

          <div className="filter-group">
            <span className="filter-label">Time (3min):</span>
            <select 
              className="filter-select" 
              value={timeFilter} 
              onChange={(e) => setTimeFilter(e.target.value)}
            >
              {getTimeFilterOptions().map(time => (
                <option key={time} value={time}>{time === 'all' ? 'All Times' : time}</option>
              ))}
            </select>
          </div>

          <div style={{ marginLeft: 'auto', fontSize: '11px', color: '#6b7280', fontWeight: 600 }}>
            Showing {filteredLogs.length} of {missionEvents.length} entries
          </div>
        </div>

        {/* Content */}
        <div className="excel-logs-content">
          <div className="excel-logs-table-container">
            {filteredLogs.length === 0 ? (
              <div className="empty-state">
                <div className="empty-state-icon">📋</div>
                <div className="empty-state-text">
                  {hasLogs ? 'No logs match the selected filters' : 'No mission logs to display. Start a mission to see logs here.'}
                </div>
              </div>
            ) : (
              <table className="excel-logs-table">
                <thead>
                  <tr>
                    {headers.map(header => (
                      <th key={header}>{header}</th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {filteredLogs.map((entry, index) => {
                    const timestamp = new Date(entry.timestamp).toLocaleTimeString();
                    const waypoint = entry.waypointId ?? '-';
                    const lat = entry.lat != null ? entry.lat.toFixed(6) : '-';
                    const lng = entry.lng != null ? entry.lng.toFixed(6) : '-';
                    const status = entry.status ? entry.status.toUpperCase() : '-';
                    const spray = entry.servoAction ? entry.servoAction.toUpperCase() : '-';
                    const remark = entry.event;
                    
                    // Determine status class
                    let statusClass = '';
                    if (status.includes('COMPLETE') || status.includes('SUCCESS')) {
                      statusClass = 'status-completed';
                    } else if (status.includes('FAIL') || status.includes('ERROR')) {
                      statusClass = 'status-error';
                    } else if (status.includes('ACTIVE') || status.includes('RUNNING')) {
                      statusClass = 'status-active';
                    } else if (status.includes('PENDING') || status.includes('WAIT')) {
                      statusClass = 'status-pending';
                    }
                    
                    return (
                      <tr key={`${entry.timestamp}-${index}`}>
                        <td className="cell-index">{index + 1}</td>
                        <td className="cell-time">{timestamp}</td>
                        <td className="cell-wp">{waypoint}</td>
                        <td className="cell-coord">{lat}</td>
                        <td className="cell-coord">{lng}</td>
                        <td className="cell-status">
                          <span className={statusClass}>{status}</span>
                        </td>
                        <td className="cell-spray">{spray}</td>
                        <td className="remark-cell" title={remark}>{remark}</td>
                      </tr>
                    );
                  })}
                </tbody>
              </table>
            )}
            <div ref={tableEndRef} />
          </div>
        </div>
      </div>
    </>
  );
};

export default MissionLogs;
