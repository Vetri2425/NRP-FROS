
import React from 'react';
import { PlayIcon } from '../icons/PlayIcon';
import { PauseIcon } from '../icons/PauseIcon';

type LiveControlsProps = {
    isConnected: boolean;
};

const LiveControls: React.FC<LiveControlsProps> = ({ isConnected: _isConnected }) => {
    const commonClass =
        'w-full flex-1 text-white font-bold text-xl rounded-lg flex items-center justify-center gap-3 transition-colors hover:bg-opacity-80';

    return (
        <div className="h-full flex flex-col gap-4 relative">
            <button
                className={`${commonClass} bg-orange-500`}
                title="Skip to next waypoint"
            >
                <PlayIcon className="w-6 h-6 rotate-90" /> SKIP
            </button>
            <button
                className={`${commonClass} bg-blue-600`}
                title="Go back to previous waypoint"
            >
                <span className="transform -scale-x-100 inline-block">
                    <PlayIcon className="w-6 h-6 -rotate-90" />
                </span>
                 GO BACK
            </button>
            <button
                className={`${commonClass} bg-blue-600`}
                title="Pause mission"
            >
                <PauseIcon className="w-6 h-6" /> PAUSE
            </button>
            <button
                className={`${commonClass} bg-red-600`}
                title="Stop mission"
            >
                <div className="w-5 h-5 bg-white" /> STOP
            </button>
        </div>
    );
};

export default LiveControls;
