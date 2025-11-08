import React, { useRef } from 'react';
import { toast } from 'react-toastify';
import { MissionFileInfo, Waypoint } from '../../types';
import { parseMissionFile, ParsedWaypoint } from '../../utils/missionParser';
import { validateWaypoints, hasCriticalErrors, getCriticalErrors, getWarnings, formatValidationErrors } from '../../utils/waypointValidator';

type FileSectionProps = {
  onUpload: (waypoints: Waypoint[], info: MissionFileInfo) => void;
  onExport: () => void;
  onUploadInitiated: () => void;
  missionWaypoints: Waypoint[];
  currentFile: MissionFileInfo | null;
  onFileProcessed: (fileInfo: MissionFileInfo) => void;
};

const FileSection: React.FC<FileSectionProps> = ({
  onUpload,
  onExport,
  onUploadInitiated,
  missionWaypoints,
  currentFile,
  onFileProcessed,
}) => {
  const fileInputRef = useRef<HTMLInputElement>(null);

  const validateFile = (file: File): boolean => {
    const validExtensions = ['waypoint', 'waypoints', 'csv', 'dxf', 'json', 'kml'];
    const ext = file.name.split('.').pop()?.toLowerCase() || '';
    if (!validExtensions.includes(ext)) {
      toast.error(`Unsupported file format: .${ext}. Allowed: ${validExtensions.join(', ')}`);
      return false;
    }
    if (file.size === 0) {
      toast.error('File is empty. Please upload a valid mission file.');
      return false;
    }
    return true;
  };

  const processFiles = async (files: FileList | null) => {
    if (files && files.length > 0) {
      const file = files[0];
      if (!validateFile(file)) return;

      const ext = file.name.split('.').pop()?.toLowerCase() || '';

      try {
        const parsedWaypoints: ParsedWaypoint[] = await parseMissionFile(file);
        const mission: Waypoint[] = parsedWaypoints.map((wp, index) => ({
          ...wp,
          id: index + 1,
          command: wp.command || 'WAYPOINT',
        }));

        // Validate waypoints
        const validationErrors = validateWaypoints(mission);
        const criticalErrors = getCriticalErrors(validationErrors);
        const warnings = getWarnings(validationErrors);

        // Block upload if critical errors found
        if (hasCriticalErrors(validationErrors)) {
          const errorMessage = formatValidationErrors(criticalErrors, 3);
          toast.error(`Validation failed:\n${errorMessage}`);
          console.error('Critical validation errors:', criticalErrors);
          return;
        }

        // Show warnings but allow upload
        if (warnings.length > 0) {
          const warningMessage = formatValidationErrors(warnings, 3);
          toast.warning(`${warnings.length} warning(s) found:\n${warningMessage}`);
          console.warn('Validation warnings:', warnings);
        }

        const info: MissionFileInfo = {
          name: file.name,
          size: file.size,
          type: file.type || ext || 'unknown',
          uploadedAt: new Date().toISOString(),
          waypointCount: mission.length,
          source: 'file',
        };
        onFileProcessed(info);
        onUpload(mission, info);
        toast.success(`Successfully loaded ${mission.length} waypoints from ${file.name}`);
      } catch (error) {
        console.error('Error parsing mission file:', error);
        toast.error(`Failed to parse ${file.name}: ${(error as Error).message}`);
      }
    }
  };

  const handleUploadClick = () => {
    onUploadInitiated();
    fileInputRef.current?.click();
  };

  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden mb-2">
      <header className="bg-indigo-700 px-3 py-2 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className="text-indigo-300">📁</span>
          <span className="font-semibold text-white tracking-wide text-sm">Mission Files</span>
        </div>
        <span className="text-xs text-indigo-100 uppercase">Upload & Export</span>
      </header>

      <div className="p-2">
        {/* Hidden file input */}
        <input
          ref={fileInputRef}
          type="file"
          className="hidden"
          onChange={(e) => processFiles(e.target.files)}
          accept=".waypoint,.waypoints,.csv,.dxf,.json,.kml"
        />

        {/* Mission file actions */}
        <div className="space-y-1.5">
          <button
            onClick={handleUploadClick}
            className="w-full bg-green-600 hover:bg-green-700 text-white px-3 py-1.5 rounded-md transition-all duration-200 text-xs font-medium"
          >
            📁 Upload Mission
          </button>

          <button
            onClick={onExport}
            disabled={missionWaypoints.length === 0}
            className={`w-full px-3 py-1.5 rounded-md transition-all duration-200 text-xs font-medium ${
              missionWaypoints.length > 0
                ? 'bg-blue-600 hover:bg-blue-700 text-white'
                : 'bg-gray-500 text-gray-300 cursor-not-allowed'
            }`}
          >
            💾 Export Mission
          </button>
        </div>
      </div>
    </div>
  );
};

export default FileSection;