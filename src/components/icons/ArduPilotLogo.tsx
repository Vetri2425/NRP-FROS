import React from 'react';

type IconProps = {
  className?: string;
};

export const ArduPilotLogo: React.FC<IconProps> = ({ className }) => (
  <svg className={className} viewBox="0 0 200 40" xmlns="http://www.w3.org/2000/svg" preserveAspectRatio="xMinYMid meet">
    <style>
      {`.ardupilot-text { font-family: 'Orbitron', sans-serif; font-weight: 700; font-size: 32px; fill: #E5E7EB; letter-spacing: -1px; }`}
    </style>
    <text x="0" y="30" className="ardupilot-text">ARDUPILOT</text>
  </svg>
);