import React, { useEffect, useState } from 'react';
import { Waypoint } from '../../types';
import { useRover } from '../../context/RoverContext';
import { BACKEND_URL } from '../../config';
import * as XLSX from 'xlsx';
import jsPDF from 'jspdf';
import autoTable from 'jspdf-autotable';
import ExcelJS from 'exceljs';

export type SprayerMissionLogsProps = {
  waypoints: Waypoint[];
};

type WpStatus = {
  reached?: boolean; // whether waypoint was reached
  marked?: boolean; // whether waypoint was marked/sprayed
  status?: 'completed' | 'loading' | 'skipped' | 'reached' | 'marked'; // combined status
  timestamp?: string; // actual timestamp when marked
  pile?: string | number; // pile identifier
  rowNo?: string | number; // row number
  remark?: string; // remarks/notes
};

const SprayerMissionLogs: React.FC<SprayerMissionLogsProps> = ({ waypoints }) => {
  const { onMissionEvent } = useRover();
  const [statusMap, setStatusMap] = useState<Record<number, WpStatus>>({});
  const [missionMode, setMissionMode] = useState<string | null>(null);
  const [isLogModalOpen, setIsLogModalOpen] = useState(false);
  const [exportFormat, setExportFormat] = useState<'excel' | 'pdf'>('excel');
  const [isDownloading, setIsDownloading] = useState(false);
  const [downloadError, setDownloadError] = useState<string | null>(null);

  // Fetch initial mission mode from backend on mount
  useEffect(() => {
    let mounted = true;
    (async () => {
      try {
        const res = await fetch(`${BACKEND_URL.replace(/\/$/, '')}/api/mission/mode`);
        if (!mounted) return;
        if (!res.ok) return;
        const body = await res.json().catch(() => null);
        if (body && (body.mode === 'auto' || body.mode === 'manual')) {
          setMissionMode(body.mode);
        } else if (typeof body === 'string' && (body === 'auto' || body === 'manual')) {
          setMissionMode(body as 'auto' | 'manual');
        }
      } catch (err) {
        console.debug('Could not fetch current mission mode:', err);
      }
    })();
    return () => { mounted = false; };
  }, []);

  useEffect(() => {
    const handler = (ev: any) => {
      // Normalize wrapper shape
      const payload = ev && ev.type === 'mission_status' && ev.data ? ev.data : ev;
      console.log('Mission Event Received:', payload);

      // Update mission mode if present in payload (mission_mode is part of base payload)
      const mode = payload?.mission_mode ?? payload?.missionMode ?? null;
      if (mode) {
        setMissionMode(String(mode).toLowerCase());
      }

      const waypointId = payload?.waypoint_id ?? payload?.current_waypoint ?? payload?.waypointId;
      if (!waypointId) return;

      setStatusMap((prev) => {
        const copy: Record<number, WpStatus> = { ...prev };
        if (payload.event_type === 'waypoint_reached') {
          console.log('Raw timestamp from backend:', payload.timestamp, 'Type:', typeof payload.timestamp);
          copy[waypointId] = {
            ...(copy[waypointId] || {}),
            reached: true,
            status: 'reached',
            pile: payload.pile ?? copy[waypointId]?.pile,
            rowNo: payload.rowNo ?? payload.row_no ?? copy[waypointId]?.rowNo,
          };
        }
        if (payload.event_type === 'waypoint_marked') {
          const timestamp = payload.timestamp;
          // Extract time directly from ISO string without Date conversion to avoid timezone issues
          // Backend format: "2025-11-20T16:36:49.297846"
          const displayTime = timestamp && typeof timestamp === 'string'
            ? timestamp.split('T')[1]?.split('.')[0] || '-'
            : '-';
          
          const markingStatus = payload.marking_status ?? payload.markingStatus ?? payload.sprayed ?? 'completed';
          copy[waypointId] = {
            ...(copy[waypointId] || {}),
            marked: true,
            status: markingStatus === 'completed' ? 'completed' : markingStatus === 'skipped' ? 'skipped' : 'marked',
            timestamp: displayTime,
            pile: payload.pile ?? copy[waypointId]?.pile,
            rowNo: payload.rowNo ?? payload.row_no ?? copy[waypointId]?.rowNo,
            remark: payload.remark ?? copy[waypointId]?.remark,
          };
          console.log('Backend timestamp:', timestamp, '→ Display time:', displayTime);
        }

        console.log('Status Map Updated:', copy);
        return copy;
      });
    };

    onMissionEvent(handler);
    // Cleanup: unregister the handler
    return onMissionEvent(handler);
  }, [onMissionEvent]);

  const handleDownload = async () => {
    setIsDownloading(true);
    setDownloadError(null);

    try {
      const data = waypoints.map((waypoint, idx) => {
        const wpStatus = statusMap[waypoint.id];
        const pile = wpStatus?.pile ?? Math.floor(idx / 10) + 1;
        const rowNo = wpStatus?.rowNo ?? (idx % 10) + 1;
        
        let statusDisplay = '-';
        if (wpStatus?.status === 'completed') {
          statusDisplay = 'Completed';
        } else if (wpStatus?.marked) {
          statusDisplay = 'Marked';
        } else if (wpStatus?.reached) {
          statusDisplay = 'Reached';
        } else if (wpStatus?.status === 'loading') {
          statusDisplay = 'Loading';
        } else if (wpStatus?.status === 'skipped') {
          statusDisplay = 'Skipped';
        }
        
        return {
          'S/N': idx + 1,
          'PILE': pile,
          'ROW No': rowNo,
          'Latitude': parseFloat(waypoint.lat.toFixed(7)),
          'Longitude': parseFloat(waypoint.lng.toFixed(7)),
          'Altitude': parseFloat(waypoint.alt.toFixed(2)),
          'Status': statusDisplay,
          'Timestamp': wpStatus?.timestamp ?? '-',
          'Remark': wpStatus?.remark ?? '-',
        };
      });

      if (exportFormat === 'excel') {
      // Calculate mission statistics
      const totalPoints = waypoints.length;
      const completedPoints = Object.values(statusMap).filter(s => s.status === 'completed' || s.marked).length;
      const errorPoints = Object.values(statusMap).filter(s => s.status === 'skipped').length;
      const successRate = totalPoints > 0 ? ((completedPoints / totalPoints) * 100).toFixed(1) : '0.0';
      
      // Calculate mission timing
      const timestamps = Object.values(statusMap)
        .map(s => s.timestamp)
        .filter(t => t && t !== '-');
      let missionDuration = 'N/A';
      if (timestamps.length >= 2) {
        const sortedTimes = timestamps.sort();
        const startTime = sortedTimes[0];
        const endTime = sortedTimes[sortedTimes.length - 1];
        missionDuration = `${startTime} - ${endTime}`;
      }
      
      // Find error locations
      const errorLocations: string[] = [];
      waypoints.forEach((wp, idx) => {
        const wpStatus = statusMap[wp.id];
        if (wpStatus?.status === 'skipped') {
          const pile = wpStatus?.pile ?? Math.floor(idx / 10) + 1;
          const rowNo = wpStatus?.rowNo ?? (idx % 10) + 1;
          errorLocations.push(`Pile ${pile}, Row ${rowNo} (WP #${idx + 1})`);
        }
      });

      const workbook = new ExcelJS.Workbook();
      const worksheet = workbook.addWorksheet('Mission Report');
      
      let currentRow = 1;
      
      // Header - Mission Report Title
      worksheet.mergeCells(`A${currentRow}:I${currentRow + 1}`);
      const titleCell = worksheet.getCell(`A${currentRow}`);
      titleCell.value = 'SPRAYER MISSION REPORT';
      titleCell.font = { size: 20, bold: true, color: { argb: 'FFFFFFFF' } };
      titleCell.fill = {
        type: 'pattern',
        pattern: 'solid',
        fgColor: { argb: 'FF2D3748' }
      };
      titleCell.alignment = { horizontal: 'center', vertical: 'middle' };
      worksheet.getRow(currentRow).height = 25;
      worksheet.getRow(currentRow + 1).height = 25;
      currentRow += 2;
      
      // Date and Mode
      currentRow++;
      worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
      const dateCell = worksheet.getCell(`A${currentRow}`);
      const dateStr = new Date().toLocaleString('en-US', {
        year: 'numeric',
        month: 'short',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
        hour12: true
      });
      dateCell.value = `Generated: ${dateStr} | Mode: ${missionMode?.toUpperCase() ?? 'UNKNOWN'}`;
      dateCell.font = { size: 10, color: { argb: 'FF6B7280' } };
      dateCell.alignment = { horizontal: 'center', vertical: 'middle' };
      worksheet.getRow(currentRow).height = 18;
      currentRow += 2;
      
      // Mission Summary Header
      worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
      const summaryHeaderCell = worksheet.getCell(`A${currentRow}`);
      summaryHeaderCell.value = 'MISSION SUMMARY';
      summaryHeaderCell.font = { size: 14, bold: true, color: { argb: 'FF1F2937' } };
      summaryHeaderCell.fill = {
        type: 'pattern',
        pattern: 'solid',
        fgColor: { argb: 'FFF9FAFB' }
      };
      summaryHeaderCell.alignment = { horizontal: 'center', vertical: 'middle' };
      summaryHeaderCell.border = {
        top: { style: 'medium' },
        left: { style: 'medium' },
        right: { style: 'medium' },
        bottom: { style: 'thin' }
      };
      worksheet.getRow(currentRow).height = 22;
      currentRow++;
      
      // Summary Content
      const summaryData = [
        { label: 'Total Waypoints:', value: totalPoints.toString(), color: 'FF000000' },
        { label: 'Completed:', value: completedPoints.toString(), color: 'FF10B981' },
        { label: 'Error Count:', value: errorPoints.toString(), color: 'FFEF4444' },
        { label: 'Success Rate:', value: `${successRate}%`, color: 'FF10B981' },
        { label: 'Mission Timing:', value: missionDuration, color: 'FF000000' },
      ];
      
      summaryData.forEach((item, idx) => {
        // Label in columns A-D
        worksheet.mergeCells(`A${currentRow}:D${currentRow}`);
        const labelCell = worksheet.getCell(`A${currentRow}`);
        labelCell.value = item.label;
        labelCell.font = { bold: true, size: 10 };
        labelCell.alignment = { horizontal: 'left', vertical: 'middle' };
        labelCell.fill = {
          type: 'pattern',
          pattern: 'solid',
          fgColor: { argb: 'FFF9FAFB' }
        };
        
        // Apply borders to all cells in label merge
        for (let col = 1; col <= 4; col++) {
          const cell = worksheet.getCell(currentRow, col);
          cell.border = {
            left: col === 1 ? { style: 'medium' } : { style: 'thin' },
            right: col === 4 ? { style: 'thin' } : undefined,
            top: { style: 'thin' },
            bottom: idx === summaryData.length - 1 ? { style: 'medium' } : { style: 'thin' }
          };
          cell.fill = {
            type: 'pattern',
            pattern: 'solid',
            fgColor: { argb: 'FFF9FAFB' }
          };
        }
        
        // Value in columns E-I
        worksheet.mergeCells(`E${currentRow}:I${currentRow}`);
        const valueCell = worksheet.getCell(`E${currentRow}`);
        valueCell.value = item.value;
        valueCell.font = { size: 10, color: { argb: item.color }, bold: item.color !== 'FF000000' };
        valueCell.alignment = { horizontal: 'left', vertical: 'middle' };
        valueCell.fill = {
          type: 'pattern',
          pattern: 'solid',
          fgColor: { argb: 'FFF9FAFB' }
        };
        
        // Apply borders to all cells in value merge
        for (let col = 5; col <= 9; col++) {
          const cell = worksheet.getCell(currentRow, col);
          cell.border = {
            left: col === 5 ? { style: 'thin' } : undefined,
            right: col === 9 ? { style: 'medium' } : { style: 'thin' },
            top: { style: 'thin' },
            bottom: idx === summaryData.length - 1 ? { style: 'medium' } : { style: 'thin' }
          };
          cell.fill = {
            type: 'pattern',
            pattern: 'solid',
            fgColor: { argb: 'FFF9FAFB' }
          };
        }
        
        worksheet.getRow(currentRow).height = 20;
        currentRow++;
      });
      
      // Error Locations
      if (errorLocations.length > 0) {
        currentRow++;
        worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
        const errorHeaderCell = worksheet.getCell(`A${currentRow}`);
        errorHeaderCell.value = 'Error Locations:';
        errorHeaderCell.font = { bold: true, size: 10, color: { argb: 'FFEF4444' } };
        errorHeaderCell.alignment = { horizontal: 'left', vertical: 'middle' };
        worksheet.getRow(currentRow).height = 16;
        currentRow++;
        
        errorLocations.forEach(loc => {
          worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
          const errorCell = worksheet.getCell(`A${currentRow}`);
          errorCell.value = `  • ${loc}`;
          errorCell.font = { size: 9, color: { argb: 'FF6B7280' } };
          errorCell.alignment = { horizontal: 'left', vertical: 'middle' };
          worksheet.getRow(currentRow).height = 15;
          currentRow++;
        });
      }
      
      // Watermark row
      currentRow += 2;
      worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
      const watermarkCell = worksheet.getCell(`A${currentRow}`);
      watermarkCell.value = 'Way 2 Mark';
      watermarkCell.font = { size: 36, bold: true, color: { argb: '30808080' } };
      watermarkCell.alignment = { horizontal: 'center', vertical: 'middle' };
      worksheet.getRow(currentRow).height = 50;
      currentRow += 2;
      
      // Waypoints Table Header
      worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
      const tableHeaderCell = worksheet.getCell(`A${currentRow}`);
      tableHeaderCell.value = 'MISSION WAYPOINTS LOG';
      tableHeaderCell.font = { size: 12, bold: true, color: { argb: 'FF1F2937' } };
      tableHeaderCell.alignment = { horizontal: 'center', vertical: 'middle' };
      worksheet.getRow(currentRow).height = 20;
      currentRow += 2;
      
      // Table column headers
      const headerRow = worksheet.getRow(currentRow);
      const headers = ['S/N', 'PILE', 'ROW No', 'Latitude', 'Longitude', 'Altitude', 'Status', 'Timestamp', 'Remark'];
      const columnWidths = [8, 8, 10, 12, 12, 10, 12, 15, 20];
      
      headers.forEach((header, idx) => {
        const cell = headerRow.getCell(idx + 1);
        cell.value = header;
        cell.fill = {
          type: 'pattern',
          pattern: 'solid',
          fgColor: { argb: 'FF2D3748' }
        };
        cell.font = { bold: true, color: { argb: 'FFFFFFFF' } };
        cell.border = {
          top: { style: 'thin' },
          left: { style: 'thin' },
          bottom: { style: 'thin' },
          right: { style: 'thin' }
        };
        cell.alignment = { horizontal: 'center', vertical: 'middle' };
        worksheet.getColumn(idx + 1).width = columnWidths[idx];
      });
      headerRow.height = 20;
      currentRow++;
      
      // Add data rows
      data.forEach((row, index) => {
        const dataRow = worksheet.getRow(currentRow);
        const values = [
          row['S/N'],
          row.PILE,
          row['ROW No'],
          row.Latitude,
          row.Longitude,
          row.Altitude,
          row.Status,
          row.Timestamp,
          row.Remark
        ];
        
        values.forEach((value, colIdx) => {
          const cell = dataRow.getCell(colIdx + 1);
          cell.value = value;
          
          const fillColor = index % 2 === 0 ? 'FFF9FAFB' : 'FFFFFFFF';
          cell.fill = {
            type: 'pattern',
            pattern: 'solid',
            fgColor: { argb: fillColor }
          };
          cell.border = {
            top: { style: 'thin' },
            left: { style: 'thin' },
            bottom: { style: 'thin' },
            right: { style: 'thin' }
          };
          cell.alignment = { horizontal: 'center', vertical: 'middle' };
          
          // Status column special styling
          if (colIdx === 6) {
            if (row.Status === 'Completed' || row.Status === 'Marked') {
              cell.fill = {
                type: 'pattern',
                pattern: 'solid',
                fgColor: { argb: 'FF10B981' }
              };
              cell.font = { color: { argb: 'FFFFFFFF' }, bold: true };
            } else if (row.Status === 'Reached') {
              cell.fill = {
                type: 'pattern',
                pattern: 'solid',
                fgColor: { argb: 'FFF59E0B' }
              };
              cell.font = { color: { argb: 'FF000000' } };
            } else if (row.Status === 'Skipped') {
              cell.fill = {
                type: 'pattern',
                pattern: 'solid',
                fgColor: { argb: 'FFEF4444' }
              };
              cell.font = { color: { argb: 'FFFFFFFF' } };
            }
          }
        });
        
        currentRow++;
      });
      
      // Footer
      currentRow += 2;
      worksheet.mergeCells(`A${currentRow}:I${currentRow}`);
      const footerCell = worksheet.getCell(`A${currentRow}`);
      footerCell.value = `NFROS - Sprayer Mission Log | Total: ${totalPoints} | Completed: ${completedPoints} | Success: ${successRate}%`;
      footerCell.font = { size: 9, color: { argb: 'FF6B7280' } };
      footerCell.alignment = { horizontal: 'center', vertical: 'middle' };
      
      // Generate and download
      const buffer = await workbook.xlsx.writeBuffer();
      const blob = new Blob([buffer], { type: 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet' });
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `sprayer_mission_${new Date().toISOString().split('T')[0]}.xlsx`;
      a.click();
      window.URL.revokeObjectURL(url);
      
      // Success - close modal after brief delay
      setTimeout(() => {
        setIsLogModalOpen(false);
        setIsDownloading(false);
      }, 500);
    } else if (exportFormat === 'pdf') {
      const doc = new jsPDF();
      
      // Calculate mission statistics
      const totalPoints = waypoints.length;
      const completedPoints = Object.values(statusMap).filter(s => s.status === 'completed' || s.marked).length;
      const errorPoints = Object.values(statusMap).filter(s => s.status === 'skipped').length;
      const successRate = totalPoints > 0 ? ((completedPoints / totalPoints) * 100).toFixed(1) : '0.0';
      
      // Calculate mission timing
      const timestamps = Object.values(statusMap)
        .map(s => s.timestamp)
        .filter(t => t && t !== '-');
      let missionDuration = 'N/A';
      if (timestamps.length >= 2) {
        const sortedTimes = timestamps.sort();
        const startTime = sortedTimes[0];
        const endTime = sortedTimes[sortedTimes.length - 1];
        missionDuration = `${startTime} - ${endTime}`;
      }
      
      // Find error locations
      const errorLocations: string[] = [];
      waypoints.forEach((wp, idx) => {
        const wpStatus = statusMap[wp.id];
        if (wpStatus?.status === 'skipped') {
          const pile = wpStatus?.pile ?? Math.floor(idx / 10) + 1;
          const rowNo = wpStatus?.rowNo ?? (idx % 10) + 1;
          errorLocations.push(`Pile ${pile}, Row ${rowNo} (WP #${idx + 1})`);
        }
      });
      
      const pageWidth = doc.internal.pageSize.getWidth();
      const pageHeight = doc.internal.pageSize.getHeight();
      let currentY = 15;
      
      // Header - Mission Report Title
      doc.setFillColor(45, 55, 72); // Dark gray background
      doc.rect(0, 0, pageWidth, 35, 'F');
      doc.setTextColor(255, 255, 255);
      doc.setFontSize(20);
      doc.setFont('helvetica', 'bold');
      doc.text('WAY 2 MARK - MISSION REPORT', pageWidth / 2, 15, { align: 'center' });
      
      doc.setFontSize(10);
      doc.setFont('helvetica', 'normal');
      const dateStr = new Date().toLocaleString('en-US', {
        year: 'numeric',
        month: 'short',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
        hour12: true
      });
      doc.text(`Generated: ${dateStr}`, pageWidth / 2, 25, { align: 'center' });
      doc.text(`Mode: ${missionMode?.toUpperCase() ?? 'UNKNOWN'}`, pageWidth / 2, 30, { align: 'center' });
      
      currentY = 45;
      
      // Mission Summary Box
      doc.setTextColor(0, 0, 0);
      doc.setFillColor(249, 250, 251); // Light gray background
      doc.roundedRect(10, currentY, pageWidth - 20, 55, 3, 3, 'F');
      doc.setDrawColor(209, 213, 219);
      doc.roundedRect(10, currentY, pageWidth - 20, 55, 3, 3, 'S');
      
      currentY += 8;
      doc.setFontSize(14);
      doc.setFont('helvetica', 'bold');
      doc.setTextColor(31, 41, 55);
      doc.text('Mission Summary', 15, currentY);
      
      currentY += 8;
      doc.setFontSize(10);
      doc.setFont('helvetica', 'normal');
      
      // Left column
      const leftColX = 15;
      const rightColX = pageWidth / 2 + 5;
      
      doc.setFont('helvetica', 'bold');
      doc.text('Total Waypoints:', leftColX, currentY);
      doc.setFont('helvetica', 'normal');
      doc.text(`${totalPoints}`, leftColX + 45, currentY);
      
      currentY += 7;
      doc.setFont('helvetica', 'bold');
      doc.text('Completed:', leftColX, currentY);
      doc.setFont('helvetica', 'normal');
      doc.setTextColor(16, 185, 129); // Green
      doc.text(`${completedPoints}`, leftColX + 45, currentY);
      doc.setTextColor(0, 0, 0);
      
      currentY += 7;
      doc.setFont('helvetica', 'bold');
      doc.text('Error Count:', leftColX, currentY);
      doc.setFont('helvetica', 'normal');
      doc.setTextColor(239, 68, 68); // Red
      doc.text(`${errorPoints}`, leftColX + 45, currentY);
      doc.setTextColor(0, 0, 0);
      
      // Right column
      currentY -= 14; // Reset to top of left column
      doc.setFont('helvetica', 'bold');
      doc.text('Success Rate:', rightColX, currentY);
      doc.setFont('helvetica', 'normal');
      doc.setTextColor(16, 185, 129);
      doc.text(`${successRate}%`, rightColX + 35, currentY);
      doc.setTextColor(0, 0, 0);
      
      currentY += 7;
      doc.setFont('helvetica', 'bold');
      doc.text('Mission Timing:', rightColX, currentY);
      doc.setFont('helvetica', 'normal');
      doc.text(missionDuration, rightColX + 35, currentY);
      
      currentY += 7;
      if (errorLocations.length > 0) {
        doc.setFont('helvetica', 'bold');
        doc.text('Error Locations:', rightColX, currentY);
        doc.setFont('helvetica', 'normal');
        doc.setFontSize(8);
        currentY += 5;
        const maxErrors = 2; // Show first 2 errors
        errorLocations.slice(0, maxErrors).forEach((loc, idx) => {
          doc.text(loc, rightColX, currentY);
          currentY += 4;
        });
        if (errorLocations.length > maxErrors) {
          doc.text(`... and ${errorLocations.length - maxErrors} more`, rightColX, currentY);
        }
        doc.setFontSize(10);
      }
      
      currentY = 110; // Position for table
      
      // Waypoints Table
      doc.setFontSize(12);
      doc.setFont('helvetica', 'bold');
      doc.setTextColor(31, 41, 55);
      doc.text('Mission Waypoints Log', 14, currentY);
      currentY += 5;
      
      autoTable(doc, {
        startY: currentY,
        head: [['S/N', 'PILE', 'ROW', 'Latitude', 'Longitude', 'Alt', 'Status', 'Time', 'Remark']],
        body: data.map(row => [
          row['S/N'],
          row.PILE,
          row['ROW No'],
          row.Latitude,
          row.Longitude,
          row.Altitude,
          row.Status,
          row.Timestamp,
          row.Remark
        ]),
        theme: 'grid',
        headStyles: {
          fillColor: [45, 55, 72],
          textColor: [255, 255, 255],
          fontStyle: 'bold',
          fontSize: 9,
          halign: 'center',
        },
        bodyStyles: {
          fontSize: 8,
          halign: 'center',
        },
        columnStyles: {
          0: { cellWidth: 12 },
          1: { cellWidth: 15 },
          2: { cellWidth: 15 },
          3: { cellWidth: 23 },
          4: { cellWidth: 23 },
          5: { cellWidth: 15 },
          6: { cellWidth: 20 },
          7: { cellWidth: 22 },
          8: { cellWidth: 35 },
        },
        alternateRowStyles: {
          fillColor: [249, 250, 251],
        },
        didParseCell: function (data: any) {
          // Color code status cells
          if (data.column.index === 6 && data.section === 'body') {
            const status = data.cell.raw;
            if (status === 'Completed' || status === 'Marked') {
              data.cell.styles.fillColor = [16, 185, 129]; // Green
              data.cell.styles.textColor = [255, 255, 255];
              data.cell.styles.fontStyle = 'bold';
            } else if (status === 'Reached') {
              data.cell.styles.fillColor = [251, 191, 36]; // Amber
              data.cell.styles.textColor = [0, 0, 0];
            } else if (status === 'Skipped') {
              data.cell.styles.fillColor = [239, 68, 68]; // Red
              data.cell.styles.textColor = [255, 255, 255];
            }
          }
        },
        margin: { top: 10, left: 10, right: 10 },
        didDrawPage: function (data: any) {
          // Watermark - "Way 2 Mark"
          doc.saveGraphicsState();
          doc.setGState(new (doc as any).GState({ opacity: 0.1 }));
          doc.setTextColor(150, 150, 150);
          doc.setFontSize(60);
          doc.setFont('helvetica', 'bold');
          
          // Center the watermark with rotation
          const watermarkText = 'Way 2 Mark';
          const centerX = pageWidth / 2;
          const centerY = pageHeight / 2;
          
          doc.text(watermarkText, centerX, centerY, {
            align: 'center',
            angle: 45,
          });
          doc.restoreGraphicsState();
          
          // Footer on every page
          const pageCount = doc.getNumberOfPages();
          const currentPage = (doc as any).getCurrentPageInfo().pageNumber;
          
          doc.setFontSize(8);
          doc.setTextColor(107, 114, 128);
          doc.setFont('helvetica', 'normal');
          
          // Left footer
          doc.text('NFROS - Sprayer Mission Log', 10, pageHeight - 10);
          
          // Center footer
          doc.text(`Total Points: ${totalPoints} | Completed: ${completedPoints} | Success: ${successRate}%`, 
            pageWidth / 2, pageHeight - 10, { align: 'center' });
          
          // Right footer - page number
          doc.text(`Page ${currentPage} of ${pageCount}`, pageWidth - 10, pageHeight - 10, { align: 'right' });
        },
      });
      
      doc.save(`sprayer_mission_${new Date().toISOString().split('T')[0]}.pdf`);
      
      // Success - close modal after brief delay
      setTimeout(() => {
        setIsLogModalOpen(false);
        setIsDownloading(false);
      }, 500);
    }
    } catch (error) {
      console.error('Download error:', error);
      setDownloadError(error instanceof Error ? error.message : 'Failed to generate file. Please try again.');
      setIsDownloading(false);
    }
  };

  return (
    <div className="bg-[#111827] rounded-lg p-2 h-full flex flex-col">
      <div className="flex items-center justify-between mb-2">
        <h3 className="font-bold text-gray-400 text-left flex-shrink-0">Mission Waypoints</h3>
        <div className="flex items-center gap-2">
          <button
            onClick={() => setIsLogModalOpen(true)}
            className="px-3 py-1 bg-blue-600 text-white rounded text-xs font-semibold hover:bg-blue-700"
          >
            Log
          </button>
          <span
            role="status"
            aria-label={`mission-mode-${missionMode ?? 'unknown'}`}
            className={`px-2 py-1 rounded text-xs font-semibold cursor-default ${
              missionMode === 'auto'
                ? 'bg-green-600 text-white'
                : missionMode === 'manual'
                ? 'bg-amber-400 text-black'
                : 'bg-gray-600 text-white'
            }`}
          >
            {(missionMode ? String(missionMode).toUpperCase() : 'UNKNOWN')}
          </span>
        </div>
      </div>
      <div className="flex-1 overflow-hidden">
        <div className="h-full overflow-y-auto">
          <table className="w-full text-xs text-left border-collapse">
            <thead className="bg-gray-800 sticky top-0">
              <tr>
                <th className="px-2 py-1">S/N</th>
                <th className="px-2 py-1">PILE</th>
                <th className="px-2 py-1">ROW No</th>
                <th className="px-2 py-1">Latitude</th>
                <th className="px-2 py-1">Longitude</th>
                <th className="px-2 py-1">Altitude</th>
                <th className="px-2 py-1">Status</th>
                <th className="px-2 py-1">Timestamp</th>
                <th className="px-2 py-1">Remark</th>
              </tr>
            </thead>
            <tbody>
              {waypoints.length === 0 ? (
                <tr>
                  <td colSpan={9} className="text-center py-4 text-gray-500">No waypoints loaded</td>
                </tr>
              ) : (
                waypoints.map((waypoint, idx) => {
                  const wpStatus = statusMap[waypoint.id];
                  const pile = wpStatus?.pile ?? Math.floor(idx / 10) + 1;
                  const rowNo = wpStatus?.rowNo ?? (idx % 10) + 1;
                  
                  // Status rendering logic
                  let statusDisplay = '-';
                  let statusColor = 'text-gray-400';
                  
                  if (wpStatus?.status === 'completed') {
                    statusDisplay = '✓✓';
                    statusColor = 'text-green-500 font-bold';
                  } else if (wpStatus?.marked) {
                    statusDisplay = '✓✓';
                    statusColor = 'text-blue-500 font-bold';
                  } else if (wpStatus?.reached) {
                    statusDisplay = '✓';
                    statusColor = 'text-green-400';
                  } else if (wpStatus?.status === 'loading') {
                    statusDisplay = '⟳';
                    statusColor = 'text-yellow-400 animate-spin inline-block';
                  } else if (wpStatus?.status === 'skipped') {
                    statusDisplay = '⊗';
                    statusColor = 'text-gray-500';
                  }
                  
                  return (
                    <tr key={waypoint.id || idx} className={idx % 2 === 0 ? 'bg-gray-900' : 'bg-gray-800'}>
                      <td className="px-2 py-1">{idx + 1}</td>
                      <td className="px-2 py-1">{pile}</td>
                      <td className="px-2 py-1">{rowNo}</td>
                      <td className="px-2 py-1">{waypoint.lat.toFixed(7)}</td>
                      <td className="px-2 py-1">{waypoint.lng.toFixed(7)}</td>
                      <td className="px-2 py-1">{waypoint.alt.toFixed(2)}</td>
                      <td className={`px-2 py-1 ${statusColor}`}>{statusDisplay}</td>
                      <td className="px-2 py-1 text-xs">{wpStatus?.timestamp ?? '-'}</td>
                      <td className="px-2 py-1 text-xs text-gray-400">{wpStatus?.remark ?? '-'}</td>
                    </tr>
                  );
                })
              )}
            </tbody>
          </table>
        </div>
      </div>

      {/* Log Modal */}
      {isLogModalOpen && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-[10000]">
          <div className="bg-gray-800 rounded-lg p-6 max-w-6xl w-full max-h-[90vh] overflow-hidden flex flex-col">
            <div className="flex justify-between items-center mb-4">
              <h3 className="text-lg font-bold text-white">Mission Waypoints Log</h3>
              <button
                onClick={() => setIsLogModalOpen(false)}
                className="text-gray-400 hover:text-white text-xl"
              >
                ×
              </button>
            </div>
            <div className="flex-1 overflow-hidden">
              <div className="h-full overflow-y-auto">
                <table className="w-full text-xs text-left border-collapse">
                  <thead className="bg-gray-700 sticky top-0">
                    <tr>
                      <th className="px-2 py-1">S/N</th>
                      <th className="px-2 py-1">PILE</th>
                      <th className="px-2 py-1">ROW No</th>
                      <th className="px-2 py-1">Latitude</th>
                      <th className="px-2 py-1">Longitude</th>
                      <th className="px-2 py-1">Altitude</th>
                      <th className="px-2 py-1">Status</th>
                      <th className="px-2 py-1">Timestamp</th>
                      <th className="px-2 py-1">Remark</th>
                    </tr>
                  </thead>
                  <tbody>
                    {waypoints.length === 0 ? (
                      <tr>
                        <td colSpan={9} className="text-center py-4 text-gray-500">No waypoints loaded</td>
                      </tr>
                    ) : (
                      waypoints.map((waypoint, idx) => {
                        const wpStatus = statusMap[waypoint.id];
                        const pile = wpStatus?.pile ?? Math.floor(idx / 10) + 1;
                        const rowNo = wpStatus?.rowNo ?? (idx % 10) + 1;
                        
                        // Status rendering logic
                        let statusDisplay = '-';
                        let statusColor = 'text-gray-400';
                        
                        if (wpStatus?.status === 'completed') {
                          statusDisplay = '✓✓';
                          statusColor = 'text-green-500 font-bold';
                        } else if (wpStatus?.marked) {
                          statusDisplay = '✓✓';
                          statusColor = 'text-blue-500 font-bold';
                        } else if (wpStatus?.reached) {
                          statusDisplay = '✓';
                          statusColor = 'text-green-400';
                        } else if (wpStatus?.status === 'loading') {
                          statusDisplay = '⟳';
                          statusColor = 'text-yellow-400 animate-spin inline-block';
                        } else if (wpStatus?.status === 'skipped') {
                          statusDisplay = '⊗';
                          statusColor = 'text-gray-500';
                        }
                        
                        return (
                          <tr key={waypoint.id || idx} className={idx % 2 === 0 ? 'bg-gray-900' : 'bg-gray-800'}>
                            <td className="px-2 py-1">{idx + 1}</td>
                            <td className="px-2 py-1">{pile}</td>
                            <td className="px-2 py-1">{rowNo}</td>
                            <td className="px-2 py-1">{waypoint.lat.toFixed(7)}</td>
                            <td className="px-2 py-1">{waypoint.lng.toFixed(7)}</td>
                            <td className="px-2 py-1">{waypoint.alt.toFixed(2)}</td>
                            <td className={`px-2 py-1 ${statusColor}`}>{statusDisplay}</td>
                            <td className="px-2 py-1 text-xs">{wpStatus?.timestamp ?? '-'}</td>
                            <td className="px-2 py-1 text-xs text-gray-400">{wpStatus?.remark ?? '-'}</td>
                          </tr>
                        );
                      })
                    )}
                  </tbody>
                </table>
              </div>
            </div>
            {downloadError && (
              <div className="mt-4 p-3 bg-red-900/50 border border-red-600 rounded text-red-200 text-sm">
                <div className="flex items-center gap-2">
                  <span className="text-red-400">⚠</span>
                  <span>{downloadError}</span>
                </div>
              </div>
            )}
            <div className="mt-4 flex items-center justify-between">
              <div className="flex items-center gap-4">
                <span className="text-white text-sm">File Format:</span>
                <label className="flex items-center gap-2 text-white">
                  <input
                    type="radio"
                    name="format"
                    value="excel"
                    checked={exportFormat === 'excel'}
                    onChange={() => setExportFormat('excel')}
                    disabled={isDownloading}
                  />
                  Excel
                </label>
                <label className="flex items-center gap-2 text-white">
                  <input
                    type="radio"
                    name="format"
                    value="pdf"
                    checked={exportFormat === 'pdf'}
                    onChange={() => setExportFormat('pdf')}
                    disabled={isDownloading}
                  />
                  PDF
                </label>
              </div>
              <button
                onClick={handleDownload}
                disabled={isDownloading}
                className={`px-4 py-2 rounded font-semibold transition-colors ${
                  isDownloading
                    ? 'bg-gray-600 text-gray-300 cursor-not-allowed'
                    : 'bg-green-600 text-white hover:bg-green-700'
                }`}
              >
                {isDownloading ? (
                  <span className="flex items-center gap-2">
                    <span className="animate-spin">⟳</span>
                    Generating...
                  </span>
                ) : (
                  'Download'
                )}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default SprayerMissionLogs;
