# Rover Position Update Logic

This document outlines the logic and code snippets used to ensure the rover's position is accurately displayed and updated in real-time on the map view.

## 5-Step Logic for Updating Rover Position

1. **Initialize Map and Layers**:
   - Use Leaflet to create the map instance and add overlays for markers, routes, and paths.
   - Example:
     ```tsx
     mapRef.current = L.map(mapContainerRef.current, { zoomControl: true, maxZoom: 22 }).setView([13.0827, 80.2707], 13);
     missionLayerRef.current = L.layerGroup().addTo(mapRef.current);
     ```

2. **Render Rover Marker**:
   - Create a custom SVG icon for the rover and use `L.divIcon` to render it.
   - Example:
     ```tsx
     const roverIconHTML = `<div style="width:32px;height:32px;display:flex;align-items:center;justify-content:center;transform:rotate(${angle}deg);">${ROVER_SVG}</div>`;
     const roverIcon = L.divIcon({ html: roverIconHTML, className: 'bg-transparent border-0', iconSize: [32, 32], iconAnchor: [16, 16] });
     roverMarkerRef.current = L.marker([roverPosition.lat, roverPosition.lng], { icon: roverIcon }).addTo(mapRef.current);
     ```

3. **Update Rover Position**:
   - Use `setLatLng` to update the rover's marker position dynamically.
   - Example:
     ```tsx
     roverMarkerRef.current.setLatLng([roverPosition.lat, roverPosition.lng]);
     ```

4. **Smooth Motion and Heading**:
   - Interpolate between previous and current positions for smooth transitions.
   - Rotate the rover icon based on its heading using CSS transforms.
   - Example:
     ```tsx
     const lerp = (x, y, t) => x + (y - x) * t;
     const lat = lerp(a.lat, b.lat, t);
     const lng = lerp(a.lng, b.lng, t);
     roverMarkerRef.current.setLatLng([lat, lng]);
     ```

5. **Keep Rover in View**:
   - Automatically pan or fit the map bounds to ensure the rover stays visible.
   - Example:
     ```tsx
     map.panTo([roverPosition.lat, roverPosition.lng], { animate: true });
     ```

## Relevant Code Snippets

### Map Initialization
```tsx
useEffect(() => {
  if (mapContainerRef.current && !mapRef.current) {
    const map = L.map(mapContainerRef.current, { zoomControl: true, maxZoom: 22 }).setView([13.0827, 80.2707], 13);
    mapRef.current = map;
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);
  }
}, []);
```

### Rover Marker Rendering
```tsx
useEffect(() => {
  if (!mapRef.current) return;
  if (roverPosition) {
    const angle = heading || 0;
    const roverIconHTML = `<div style="width:32px;height:32px;display:flex;align-items:center;justify-content:center;transform:rotate(${angle}deg);">${ROVER_SVG}</div>`;
    const roverIcon = L.divIcon({ html: roverIconHTML, className: 'bg-transparent border-0', iconSize: [32, 32], iconAnchor: [16, 16] });
    if (!roverMarkerRef.current) {
      roverMarkerRef.current = L.marker([roverPosition.lat, roverPosition.lng], { icon: roverIcon, zIndexOffset: 1000 }).addTo(mapRef.current);
    } else {
      roverMarkerRef.current.setLatLng([roverPosition.lat, roverPosition.lng]);
      roverMarkerRef.current.setIcon(roverIcon);
    }
  }
}, [roverPosition, heading]);
```

### Smooth Motion
```tsx
useEffect(() => {
  if (!mapRef.current || !roverMarkerRef.current) return;
  const a = prevSampleRef.current;
  const b = lastSampleRef.current;
  if (!b) return;
  const now = performance.now();
  let lat = b.lat, lng = b.lng, hdg = b.heading;
  if (a) {
    const dt = Math.max(0.0001, b.t - a.t);
    const t = Math.max(0, Math.min(1, (now - a.t) / dt));
    const lerp = (x, y, t) => x + (y - x) * t;
    lat = lerp(a.lat, b.lat, t);
    lng = lerp(a.lng, b.lng, t);
    const d = ((((b.heading - a.heading) + 540) % 360) - 180);
    hdg = a.heading + d * t;
  }
  roverMarkerRef.current.setLatLng([lat, lng]);
}, [frameNow]);
```

### Keep Rover in View
```tsx
useEffect(() => {
  if (!mapRef.current || !roverPosition) return;
  const map = mapRef.current;
  const currentBounds = map.getBounds();
  const roverLatLng = L.latLng(roverPosition.lat, roverPosition.lng);
  if (!currentBounds.pad(-0.2).contains(roverLatLng)) {
    map.panTo(roverLatLng, { animate: true, duration: 0.5 });
  }
}, [roverPosition]);
```

---

This document provides a clear understanding of how the rover's position is managed and updated in real-time on the map view.