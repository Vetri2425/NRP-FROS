import React from 'react';
import { LineIcon } from '../icons/LineIcon';
import { RectangleIcon } from '../icons/RectangleIcon';
import { CircleIcon } from '../icons/CircleIcon';
import { HexagonIcon } from '../icons/HexagonIcon';
import { RulerIcon } from '../icons/RulerIcon';

type ToolsProps = {
  onShowSurveyGridTool: () => void;
  onShowCircleTool: () => void;
  onDrawingToolSelect?: (tool: string | null) => void;
  activeDrawingTool?: string | null;
};

const Tools: React.FC<ToolsProps> = ({
  onShowSurveyGridTool,
  onShowCircleTool,
  onDrawingToolSelect,
  activeDrawingTool,
}) => {
  const drawingTools = [
    { name: 'line', icon: LineIcon, title: 'Draw Path' },
    { name: 'rectangle', icon: RectangleIcon, title: 'Draw Rectangle' },
    { name: 'circle', icon: CircleIcon, title: 'Draw Circle' },
    { name: 'hexagon', icon: HexagonIcon, title: 'Draw Hexagon' },
  ];
  return (
    <div className="bg-[#111827] rounded-lg overflow-hidden mb-2">
      <header className="bg-indigo-700 px-3 py-2 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className="text-indigo-300">✏️</span>
          <span className="font-semibold text-white tracking-wide text-sm">Tools</span>
        </div>
      </header>

      <div className="p-2">
        <div className="flex items-center gap-1">
          {drawingTools.map(tool => {
            const Icon = tool.icon;
            return (
              <button
                key={tool.name}
                onClick={() => onDrawingToolSelect && onDrawingToolSelect(activeDrawingTool === tool.name ? null : tool.name)}
                className={`p-1.5 rounded-md text-xs ${
                  activeDrawingTool === tool.name
                    ? 'bg-green-600 text-white'
                    : 'text-gray-300 hover:bg-gray-700'
                }`}
                title={tool.title}
              >
                <Icon className="w-4 h-4" />
              </button>
            );
          })}
          <button
            onClick={() => onDrawingToolSelect && onDrawingToolSelect(activeDrawingTool === 'smart-dimension' ? null : 'smart-dimension')}
            className={`p-1.5 rounded-md text-xs ${
              activeDrawingTool === 'smart-dimension'
                ? 'bg-blue-600 text-white'
                : 'text-gray-300 hover:bg-gray-700'
            }`}
            title="Smart Dimension"
          >
            <RulerIcon className="w-4 h-4" />
          </button>
          <div className="border-l border-gray-600 h-6 mx-1"></div>
          <button
            onClick={onShowCircleTool}
            className="p-1.5 rounded-md text-xs text-gray-300 hover:bg-gray-700"
            title="Auto Circle"
          >
            🌀
          </button>
          <button
            onClick={onShowSurveyGridTool}
            className="p-1.5 rounded-md text-xs text-gray-300 hover:bg-gray-700"
            title="Grid"
          >
            📐
          </button>
        </div>
      </div>
    </div>
  );
};

export default Tools;