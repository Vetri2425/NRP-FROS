import React from 'react';

type Tool = 'measure' | 'profile' | 'line' | 'rectangle' | 'circle' | 'polygon' | 'hexagon' | 'geofence' | null;

type DrawingInstructionsProps = {
  activeTool: Tool;
};

const DrawingInstructions: React.FC<DrawingInstructionsProps> = ({ activeTool }) => {
  let instruction = '';

  switch (activeTool) {
    case 'line':
    case 'polygon':
    case 'measure':
      instruction = 'Click on the map to add points. Double-click the last point to finish.';
      break;
    case 'rectangle':
    case 'circle':
    case 'hexagon':
      instruction = 'Click and drag on the map to draw the shape.';
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