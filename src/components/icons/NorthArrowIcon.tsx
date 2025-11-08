import React from 'react';

type IconProps = {
  className?: string;
  headingDeg?: number; // 0-360, 0 = North
};

// Compass with rotating needle indicating heading (relative to North)
export const NorthArrowIcon: React.FC<IconProps> = ({ className, headingDeg = 0 }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    className={className}
    aria-label={`Heading ${Math.round(((headingDeg % 360) + 360) % 360)} degrees`}
  >
    {/* Outer circle */}
    <circle cx="12" cy="12" r="10" fill="none" stroke="currentColor" strokeWidth="2" />

    {/* Needle group rotates around center (12,12) */}
    <g transform={`rotate(${headingDeg} 12 12)`}>
      {/* North needle */}
      <path d="M12 4l3 8-3-2-3 2 3-8z" fill="currentColor" />
      {/* South tail (faded) */}
      <path d="M12 20l-3-8 3 2 3-2-3 8z" fill="currentColor" opacity="0.35" />
    </g>

    {/* N label stays upright */}
    <text x="12" y="7" textAnchor="middle" fontSize="6" fontFamily="sans-serif" fill="currentColor">N</text>
  </svg>
);
