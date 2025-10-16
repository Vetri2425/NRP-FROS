export interface LogEntry {
  timestamp: string;
  lat: number;
  lng: number;
  event: string;
}

export const exportLogsToCSV = (logEntries: LogEntry[]) => {
  if (logEntries.length === 0) {
    alert("No simulation logs available to export.");
    return;
  }

  const headers = ['timestamp', 'latitude', 'longitude', 'event'];
  
  const csvRows = [
    headers.join(','),
    ...logEntries.map(entry => {
      const row = [
        entry.timestamp,
        entry.lat.toFixed(8),
        entry.lng.toFixed(8),
        `"${entry.event.replace(/"/g, '""')}"` // Escape double quotes within the event string
      ];
      return row.join(',');
    })
  ];

  const csvContent = csvRows.join('\n');
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const link = document.createElement('a');

  // FIX: Cast navigator to 'any' to handle the non-standard 'msSaveBlob' property for legacy IE support.
  if ((navigator as any).msSaveBlob) { // IE 10+
    (navigator as any).msSaveBlob(blob, 'rover_simulation_log.csv');
  } else {
    link.href = URL.createObjectURL(blob);
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    link.download = `rover_simulation_log_${timestamp}.csv`;
    
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  }
};