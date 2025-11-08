
import { LogEntry } from '../types';

export const exportLogsToCSV = (logEntries: LogEntry[]) => {
  if (logEntries.length === 0) {
    alert("No mission logs available to export.");
    return;
  }

  const headers = ['timestamp', 'latitude', 'longitude', 'waypoint', 'status', 'servo_action', 'event'];

  const csvRows = [
    headers.join(','),
    ...logEntries.map(entry => {
      const safeEvent = entry.event.replace(/"/g, '""');
      const lat = entry.lat != null ? entry.lat.toFixed(8) : '';
      const lng = entry.lng != null ? entry.lng.toFixed(8) : '';
      const waypoint = entry.waypointId ?? '';
      const status = entry.status ?? '';
      const servo = entry.servoAction ?? '';
      return [
        entry.timestamp,
        lat,
        lng,
        waypoint,
        status,
        servo,
        `"${safeEvent}"`,
      ].join(',');
    })
  ];

  const csvContent = csvRows.join('\n');
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const link = document.createElement('a');

  if ((navigator as any).msSaveBlob) { // IE 10+
    (navigator as any).msSaveBlob(blob, 'mission_log.csv');
  } else {
    link.href = URL.createObjectURL(blob);
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    link.download = `mission_log_${timestamp}.csv`;

    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  }
};
