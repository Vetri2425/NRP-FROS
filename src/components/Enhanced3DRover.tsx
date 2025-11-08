import { useEffect, useRef, useMemo } from 'react';
import * as THREE from 'three';

interface Enhanced3DRoverProps {
  width?: number;
  height?: number;
  heading?: number;
  position?: { lat: number; lng: number };
  telemetry?: {
    speed?: number;
    battery?: number;
    signalStrength?: number;
    altitude?: number;
    satellites?: number;
  };
  autoRotate?: boolean;
  interactive?: boolean;
  performanceMode?: 'high' | 'medium' | 'low';
  size?: 'small' | 'medium' | 'large';
  zoomLevel?: number; // Map zoom level for LOD
  enableFallback?: boolean; // Enable 2D fallback for low-end devices
}

const Enhanced3DRover: React.FC<Enhanced3DRoverProps> = ({
  width = 32,
  height = 32,
  heading = 0,
  telemetry = {},
  autoRotate = false,
  interactive = false,
  performanceMode = 'medium',
  size = 'small',
  zoomLevel = 13,
  enableFallback = true
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const roverRef = useRef<THREE.Group | null>(null);
  const materialsRef = useRef<{
    chassis?: THREE.MeshStandardMaterial;
    antenna?: THREE.MeshStandardMaterial;
    antennaGlow?: THREE.MeshStandardMaterial;
  }>({});
  
  // Performance detection
  const deviceCapability = useMemo(() => {
    // Check device memory (if available)
    const memory = (navigator as any).deviceMemory;
    const hardwareConcurrency = navigator.hardwareConcurrency || 4;
    
    // Check for mobile devices
    const isMobile = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
    
    // Performance scoring
    let score = 0;
    if (memory) score += memory > 4 ? 2 : memory > 2 ? 1 : 0;
    score += hardwareConcurrency > 4 ? 2 : hardwareConcurrency > 2 ? 1 : 0;
    score += isMobile ? 0 : 1;
    
    if (score >= 4) return 'high';
    if (score >= 2) return 'medium';
    return 'low';
  }, []);

  // LOD system based on zoom level
  const levelOfDetail = useMemo(() => {
    if (zoomLevel < 10) return 'far';      // Very low detail
    if (zoomLevel < 15) return 'medium';   // Medium detail
    return 'near';                         // Full detail
  }, [zoomLevel]);

  // Determine effective performance mode
  const effectivePerformanceMode = useMemo(() => {
    if (levelOfDetail === 'far') return 'low';
    if (deviceCapability === 'low') return 'low';
    if (levelOfDetail === 'medium' && deviceCapability === 'medium') return 'medium';
    return performanceMode;
  }, [levelOfDetail, deviceCapability, performanceMode]);

  // Check if we should use 2D fallback
  const shouldUseFallback = useMemo(() => {
    return enableFallback && (
      effectivePerformanceMode === 'low' ||
      deviceCapability === 'low' ||
      levelOfDetail === 'far'
    );
  }, [enableFallback, effectivePerformanceMode, deviceCapability, levelOfDetail]);

  // Mouse interaction refs
  const rotationRef = useRef({ x: -0.3, y: 0 });
  const isDraggingRef = useRef(false);
  const previousMouseRef = useRef({ x: 0, y: 0 });

  // Performance settings based on effective mode
  const performanceConfig = useMemo(() => {
    const configs = {
      high: { antialias: true, shadowMapSize: 2048, segments: 32, knobbyDetail: 3 },
      medium: { antialias: true, shadowMapSize: 1024, segments: 16, knobbyDetail: 2 },
      low: { antialias: false, shadowMapSize: 512, segments: 8, knobbyDetail: 1 }
    };
    return configs[effectivePerformanceMode];
  }, [effectivePerformanceMode]);

  // Size configurations
  const sizeConfig = useMemo(() => {
    const configs = {
      small: { scale: 0.81, cameraDistance: 8 },    // 0.54 * 1.5 = 0.81
      medium: { scale: 1.35, cameraDistance: 12 },  // 0.9 * 1.5 = 1.35
      large: { scale: 2.7, cameraDistance: 16 }     // 1.8 * 1.5 = 2.7
    };
    return configs[size];
  }, [size]);

  // Battery level to color mapping
  const batteryColor = useMemo(() => {
    const battery = telemetry.battery || 100;
    if (battery > 75) return 0xFFFF00; // Yellow (was Green)
    if (battery > 50) return 0xFFFF00; // Yellow
    if (battery > 25) return 0xFF8000; // Orange
    return 0xFF0000; // Red
  }, [telemetry.battery]);

  // Signal strength to glow intensity
  const signalIntensity = useMemo(() => {
    const signal = telemetry.signalStrength || 0;
    return Math.max(0.2, signal / 100);
  }, [telemetry.signalStrength]);

  // 2D Fallback Component
  const Fallback2DRover = () => {
    const batteryColor = telemetry.battery && telemetry.battery > 75 ? '#00FF00' : 
                        telemetry.battery && telemetry.battery > 50 ? '#FFFF00' :
                        telemetry.battery && telemetry.battery > 25 ? '#FF8000' : '#FF0000';
    
    return (
      <div 
        style={{ 
          width, 
          height, 
          display: 'flex', 
          alignItems: 'center', 
          justifyContent: 'center',
          transform: `rotate(${heading || 0}deg)`
        }}
      >
        <svg viewBox="0 0 64 64" width={width} height={height}>
          {/* Main body with battery color */}
          <rect x="6" y="14" width="52" height="36" rx="6" ry="6" 
                fill={batteryColor} stroke="#8b0000" strokeWidth="2" />
          {/* Windshield */}
          <rect x="12" y="20" width="40" height="16" rx="2" ry="2" 
                fill="#111" opacity="0.9" />
          {/* Wheels */}
          <rect x="14" y="38" width="10" height="6" rx="1" fill="#222" />
          <rect x="40" y="38" width="10" height="6" rx="1" fill="#222" />
          {/* Front indicator arrow */}
          <path d="M 32 6 L 38 14 L 26 14 Z" 
                fill="#ffeb3b" stroke="#f57f17" strokeWidth="1" />
          {/* Signal strength indicator */}
          {telemetry.signalStrength && telemetry.signalStrength > 0 && (
            <circle cx="32" cy="32" r="3" 
                    fill="#FFFF00" 
                    opacity={telemetry.signalStrength / 100} />
          )}
        </svg>
      </div>
    );
  };

  // Return fallback if performance requires it
  if (shouldUseFallback) {
    return <Fallback2DRover />;
  }

  useEffect(() => {
    if (!containerRef.current) return;

    // Scene setup
    const scene = new THREE.Scene();
    scene.background = null; // Transparent background for map overlay
    sceneRef.current = scene;

    // Camera setup
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    camera.position.set(0, 4, sizeConfig.cameraDistance);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    // Renderer setup
    const renderer = new THREE.WebGLRenderer({ 
      antialias: performanceConfig.antialias,
      alpha: true,
      premultipliedAlpha: false
    });
    renderer.setSize(width, height);
      renderer.shadowMap.enabled = effectivePerformanceMode !== 'low';
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setClearColor(0x000000, 0); // Transparent
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(2, 4, 2);
    if (effectivePerformanceMode !== 'low') {
      directionalLight.castShadow = true;
      directionalLight.shadow.mapSize.width = performanceConfig.shadowMapSize;
      directionalLight.shadow.mapSize.height = performanceConfig.shadowMapSize;
    }
    scene.add(directionalLight);

    // Rover group
    const rover = new THREE.Group();
    rover.scale.setScalar(sizeConfig.scale);
    roverRef.current = rover;

    // Materials with telemetry integration
    const chassisMaterial = new THREE.MeshStandardMaterial({
      color: 0xFFFF00, // Default to yellow (replacing green)
      metalness: 0.3,
      roughness: 0.4
    });
    materialsRef.current.chassis = chassisMaterial;

    const antennaMaterial = new THREE.MeshStandardMaterial({
      color: 0xFFFFFF,
      metalness: 0.3,
      roughness: 0.2
    });
    materialsRef.current.antenna = antennaMaterial;

    // Antenna glow material for signal strength
    const antennaGlowMaterial = new THREE.MeshStandardMaterial({
      color: 0x00FF00,
      emissive: 0x004400,
      metalness: 0.1,
      roughness: 0.8,
      transparent: true,
      opacity: 0.7
    });
    materialsRef.current.antennaGlow = antennaGlowMaterial;

    // Yellow Chassis - main body (6x3 ratio)
    const chassisGeometry = new THREE.BoxGeometry(3.0, 0.6, 6.0);
    const chassis = new THREE.Mesh(chassisGeometry, chassisMaterial);
    chassis.position.y = 1;
    if (effectivePerformanceMode !== 'low') {
      chassis.castShadow = true;
      chassis.receiveShadow = true;
    }
    rover.add(chassis);

    // Chassis top detail
    const topGeometry = new THREE.BoxGeometry(2.2, 0.3, 5.0);
    const topDeck = new THREE.Mesh(topGeometry, chassisMaterial);
    topDeck.position.y = 1.45;
    if (effectivePerformanceMode !== 'low') topDeck.castShadow = true;
    rover.add(topDeck);

    // Wheel positions
    const wheelPositions = [
      { x: 2.0, z: 2.5 },   // Front right
      { x: -2.0, z: 2.5 },  // Front left
      { x: 2.0, z: -2.5 },  // Back right
      { x: -2.0, z: -2.5 }  // Back left
    ];

    // Create wheels with performance-aware detail
    const tireMaterial = new THREE.MeshStandardMaterial({
      color: 0x2a2a2a,
      roughness: 0.95
    });

    const rimMaterial = new THREE.MeshStandardMaterial({
      color: 0x1a1a1a,
      metalness: 0.7,
      roughness: 0.3
    });

    wheelPositions.forEach(pos => {
      // Main tire body (width decreased by 20%)
      const tireGeometry = new THREE.CylinderGeometry(0.65, 0.65, 0.96, performanceConfig.segments);
      const tire = new THREE.Mesh(tireGeometry, tireMaterial);
      tire.rotation.z = Math.PI / 2;
      tire.position.set(pos.x, 0.65, pos.z);
      if (effectivePerformanceMode !== 'low') tire.castShadow = true;
      rover.add(tire);

      // Rim center
  const rimCenterGeometry = new THREE.CylinderGeometry(0.45, 0.45, 1.0, performanceConfig.segments);
      const rimCenter = new THREE.Mesh(rimCenterGeometry, rimMaterial);
      rimCenter.rotation.z = Math.PI / 2;
      rimCenter.position.set(pos.x, 0.65, pos.z);
      if (effectivePerformanceMode !== 'low') rimCenter.castShadow = true;
      rover.add(rimCenter);

      // Simplified treads based on performance
      if (effectivePerformanceMode !== 'low') {
        const knobbyCounts = performanceConfig.knobbyDetail === 3 ? 
          [{ radius: 0.65, count: 16, size: 0.12 }] : 
          [{ radius: 0.65, count: 8, size: 0.15 }];

        knobbyCounts.forEach(row => {
          for (let i = 0; i < row.count; i++) {
            const knobGeometry = new THREE.BoxGeometry(0.25, row.size, row.size);
            const knob = new THREE.Mesh(knobGeometry, tireMaterial);
            const angle = (i / row.count) * Math.PI * 2;
            const offsetX = Math.cos(angle) * row.radius;
            const offsetY = Math.sin(angle) * row.radius;

            knob.position.set(
              pos.x + offsetX * Math.sign(pos.x) * 0.1,
              0.65 + offsetY,
              pos.z
            );
            knob.rotation.z = Math.PI / 2;
            knob.rotation.x = angle;
            rover.add(knob);
          }
        });
      }
    });

    // GPS Antennas with signal strength visualization
    const blackMaterial = new THREE.MeshStandardMaterial({
      color: 0x1a1a1a,
      metalness: 0.6,
      roughness: 0.3
    });

  // Front antenna (increased size by 20%)
  const antennaBaseGeometry = new THREE.CylinderGeometry(0.36, 0.36, 0.18, performanceConfig.segments);
  const frontAntennaBase = new THREE.Mesh(antennaBaseGeometry, blackMaterial);
  frontAntennaBase.position.set(0, 1.85, 2.2);
    if (effectivePerformanceMode !== 'low') frontAntennaBase.castShadow = true;
    rover.add(frontAntennaBase);
  const antennaDomeGeometry = new THREE.SphereGeometry(0.42, performanceConfig.segments, performanceConfig.segments, 0, Math.PI * 2, 0, Math.PI / 2);
  const frontAntennaDome = new THREE.Mesh(antennaDomeGeometry, antennaMaterial);
  frontAntennaDome.position.set(0, 1.95, 2.2);
    if (effectivePerformanceMode !== 'low') frontAntennaDome.castShadow = true;
    rover.add(frontAntennaDome);

    // Signal strength glow effect (shared geometry)
    let glowGeometry: THREE.SphereGeometry | null = null;
    if (effectivePerformanceMode === 'high') {
      glowGeometry = new THREE.SphereGeometry(0.4, 16, 16);
      const frontGlow = new THREE.Mesh(glowGeometry, antennaGlowMaterial);
      frontGlow.position.set(0, 1.9, 2.2);
      rover.add(frontGlow);
    }

  const antennaPostGeometry = new THREE.CylinderGeometry(0.096, 0.096, 0.36, 8);
    const frontPost = new THREE.Mesh(antennaPostGeometry, blackMaterial);
    frontPost.position.set(0, 1.675, 2.2);
    if (effectivePerformanceMode !== 'low') frontPost.castShadow = true;
    rover.add(frontPost);

    // Back antenna
  const backAntennaBase = new THREE.Mesh(antennaBaseGeometry, blackMaterial);
  backAntennaBase.position.set(0, 1.85, -2.2);
    if (effectivePerformanceMode !== 'low') backAntennaBase.castShadow = true;
    rover.add(backAntennaBase);

  const backAntennaDome = new THREE.Mesh(antennaDomeGeometry, antennaMaterial);
  backAntennaDome.position.set(0, 1.95, -2.2);
    if (effectivePerformanceMode !== 'low') backAntennaDome.castShadow = true;
    rover.add(backAntennaDome);

    // Back antenna glow
    if (effectivePerformanceMode === 'high' && glowGeometry) {
      const backGlow = new THREE.Mesh(glowGeometry, antennaGlowMaterial);
      backGlow.position.set(0, 1.9, -2.2);
      rover.add(backGlow);
    }

    const backPost = new THREE.Mesh(antennaPostGeometry, blackMaterial);
    backPost.position.set(0, 1.675, -2.2);
    if (effectivePerformanceMode !== 'low') backPost.castShadow = true;
    rover.add(backPost);

    scene.add(rover);

    // Mouse interaction (only if interactive)
    let handleMouseDown: (e: MouseEvent) => void;
    let handleMouseMove: (e: MouseEvent) => void;
    let handleMouseUp: () => void;
    let handleWheel: (e: WheelEvent) => void;

    if (interactive) {
      handleMouseDown = (e: MouseEvent) => {
        isDraggingRef.current = true;
        previousMouseRef.current = { x: e.clientX, y: e.clientY };
      };

      handleMouseMove = (e: MouseEvent) => {
        if (isDraggingRef.current) {
          const deltaX = e.clientX - previousMouseRef.current.x;
          const deltaY = e.clientY - previousMouseRef.current.y;

          rotationRef.current.y += deltaX * 0.01;
          rotationRef.current.x += deltaY * 0.01;

          rotationRef.current.x = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, rotationRef.current.x));

          previousMouseRef.current = { x: e.clientX, y: e.clientY };
        }
      };

      handleMouseUp = () => {
        isDraggingRef.current = false;
      };

      handleWheel = (e: WheelEvent) => {
        e.preventDefault();
        camera.position.z += e.deltaY * 0.01;
        camera.position.z = Math.max(3, Math.min(25, camera.position.z));
      };

      renderer.domElement.addEventListener('mousedown', handleMouseDown);
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      renderer.domElement.addEventListener('wheel', handleWheel, { passive: false });
    }

    // Animation loop
    let animationId: number;
    const animate = () => {
      animationId = requestAnimationFrame(animate);

      if (autoRotate && !isDraggingRef.current) {
        rotationRef.current.y += 0.005;
      }

      // Apply rotation (either from interaction or heading)
      if (interactive) {
        rover.rotation.y = rotationRef.current.y;
        rover.rotation.x = rotationRef.current.x;
      } else {
        // Use heading for map display
        rover.rotation.y = (heading || 0) * Math.PI / 180;
      }

      // Subtle floating animation
      rover.position.y = Math.sin(Date.now() * 0.001) * 0.05;

      renderer.render(scene, camera);
    };

    animate();

    // Cleanup
    return () => {
      cancelAnimationFrame(animationId);
      
      if (interactive) {
        renderer.domElement.removeEventListener('mousedown', handleMouseDown);
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
        renderer.domElement.removeEventListener('wheel', handleWheel);
      }
      
      if (containerRef.current && renderer.domElement) {
        containerRef.current.removeChild(renderer.domElement);
      }
      renderer.dispose();
    };
  }, [width, height, autoRotate, interactive, effectivePerformanceMode, sizeConfig, shouldUseFallback]);

  // Update materials based on telemetry
  useEffect(() => {
    if (materialsRef.current.chassis) {
      // Update chassis color based on battery level
      // We intentionally bias toward yellow at high charge (r=1,g~batteryHealth)
      const batteryHealth = (telemetry.battery || 100) / 100;
      const r = 1; // keep red component present so low battery appears red/orange
      const g = Math.max(0, Math.min(1, batteryHealth));
      materialsRef.current.chassis.color.setRGB(r, g, 0);
    }

    if (materialsRef.current.antennaGlow) {
      // Update antenna glow based on signal strength
      materialsRef.current.antennaGlow.opacity = signalIntensity;
      materialsRef.current.antennaGlow.emissiveIntensity = signalIntensity;
    }
  }, [telemetry.battery, signalIntensity]);

  return (
    <div 
      ref={containerRef} 
      style={{ 
        width, 
        height, 
        cursor: interactive ? 'grab' : 'default',
        pointerEvents: interactive ? 'auto' : 'none'
      }} 
    />
  );
};

export default Enhanced3DRover;