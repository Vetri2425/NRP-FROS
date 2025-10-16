
import React, { useEffect, useRef } from 'react';
import { Waypoint } from '../../types';
import { CheckCircleIcon } from '../icons/CheckCircleIcon';

type WaypointStatusListProps = {
  waypoints: Waypoint[];
  activeWaypointId: number | null;
  completedWaypointIds: number[];
};

const WaypointStatusList: React.FC<WaypointStatusListProps> = ({
  waypoints,
  activeWaypointId,
  completedWaypointIds,
}) => {
  const activeItemRef = useRef<HTMLTableRowElement>(null);

  useEffect(() => {
    if (activeItemRef.current) {
      activeItemRef.current.scrollIntoView({
        behavior: 'smooth',
        block: 'center',
      });
    }
  }, [activeWaypointId]);

  return (
    <div className="h-full flex flex-col">
      <div className="bg-gray-700 p-2 flex-shrink-0">
        <table className="w-full text-left text-xs">
          <thead>
            <tr className="text-gray-400">
              <th className="px-2 py-1 font-semibold">Name</th>
              <th className="px-2 py-1 font-semibold">Status</th>
              <th className="px-2 py-1 font-semibold text-right">Δ Elevation</th>
            </tr>
          </thead>
        </table>
      </div>
      <div className="flex-1 overflow-y-auto">
        <table className="w-full text-left text-sm">
          <tbody className="divide-y divide-gray-700">
            {waypoints.map((wp, index) => {
              const prevWp = waypoints[index - 1];
              const deltaAlt = prevWp ? wp.alt - prevWp.alt : null;
              const isCompleted = (completedWaypointIds || []).includes(wp.id);
              const isActive = wp.id === activeWaypointId;

              const rowClass = isActive
                ? 'bg-orange-500/50'
                : isCompleted
                ? 'bg-transparent'
                : 'bg-transparent text-gray-400';

              return (
                <tr
                  key={wp.id}
                  ref={isActive ? activeItemRef : null}
                  className={`${rowClass} transition-colors duration-300`}
                >
                  <td className="px-2 py-2 font-mono">{`p${wp.id}`}</td>
                  <td className="px-2 py-2">
                    {isCompleted && <CheckCircleIcon className="w-5 h-5 text-green-500" />}
                    {!isCompleted && <span>-</span>}
                  </td>
                  <td className="px-2 py-2 font-mono text-right">
                    {isCompleted && deltaAlt !== null && (
                      <span
                        className={`font-bold px-2 py-0.5 rounded ${
                          deltaAlt >= 0 ? 'bg-green-500/80 text-white' : 'bg-red-500/80 text-white'
                        }`}
                      >
                        {deltaAlt >= 0 ? '+' : ''}
                        {deltaAlt.toFixed(2)}' f {deltaAlt >= 0 ? '↑' : '↓'}
                      </span>
                    )}
                     {!isCompleted && <span>-</span>}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default WaypointStatusList;
