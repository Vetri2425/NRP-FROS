import React from 'react';

type Tool = 'smart-dimension' | 'profile' | 'line' | 'rectangle' | 'circle' | 'polygon' | 'hexagon' | 'geofence' | null;

type DrawingInstructionsProps = {
  activeTool: Tool;
};

const DrawingInstructions: React.FC<DrawingInstructionsProps> = ({ activeTool }) => {
  let instruction = '';

  switch (activeTool) {
    case 'line':
    case 'polygon':
    case 'smart-dimension':
      instruction = 'Click on the map to add points. Double-click the last point to finish.';
      break;
    case 'rectangle':
      instruction = 'Click first corner, then click second corner to complete rectangle.';
      break;
    case 'geofence':
      instruction = 'Click and drag to create a geofence zone. Name it after creation.';
      break;
    default:
      return null;
  }

  return (
    <div className="absolute top-2 left-1/2 -translate-x-1/2 z-[1000] bg-black bg-opacity-60 text-white text-sm font-semibold px-4 py-2 rounded-md pointer-events-none">
      <p>{instruction}</p>
    </div>
  );
};

export default DrawingInstructions;