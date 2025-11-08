import React, { useEffect, useRef } from 'react';

interface Props {
  basePosition: { lat: number; lng: number } | null;
}

const BaseMapPanel: React.FC<Props> = ({ basePosition }) => {
  const mapRef = useRef<any>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const markerRef = useRef<any>(null);

  useEffect(() => {
    const L = (window as any).L;
    if (!L || !containerRef.current) return;

    // initialize map only once
    if (!mapRef.current) {
      mapRef.current = L.map(containerRef.current, {
        crs: L.CRS.EPSG4326,
        center: [0, 0],
        zoom: 2,
        attributionControl: false,
        zoomControl: false,
        maxZoom: 24,
      });
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 24,
      }).addTo(mapRef.current);
    }

    if (basePosition) {
      const latlng = [basePosition.lat, basePosition.lng];
      mapRef.current.setView(latlng, 14);
      if (!markerRef.current) {
        markerRef.current = L.marker(latlng).addTo(mapRef.current);
      } else {
        markerRef.current.setLatLng(latlng);
      }
    }
  }, [basePosition]);

  return (
    <div className="bg-[#0b1220] rounded-lg p-3 border border-slate-700 h-64">
      <div className="flex items-center justify-between mb-2">
        <div className="text-sm font-semibold">Base Map</div>
        <div className="text-xs text-slate-400">Base marker only</div>
      </div>
      <div ref={containerRef} className="w-full h-48 rounded bg-slate-800" />
      {!basePosition && <div className="text-xs text-slate-400 mt-2">No base position available</div>}
      {basePosition && !(window as any).L && (
        <div className="text-xs text-slate-300 mt-2">Base: {basePosition.lat.toFixed(6)}, {basePosition.lng.toFixed(6)}</div>
      )}
    </div>
  );
};

export default BaseMapPanel;
