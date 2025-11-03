# ArduPilot Mission Planner - Tech Stack Analysis

---

## Executive Summary

**Mission Planner** is a comprehensive, open-source ground control station (GCS) developed by Michael Oborne in **C# .NET** for the ArduPilot autopilot ecosystem. It provides a complete solution for UAV mission planning, real-time telemetry monitoring, firmware management, and post-flight log analysis for drones, planes, rovers, and submarines.

---

## Frontend Architecture

### Core UI Framework
- **Windows Forms (.NET Framework)**
  - Primary desktop UI implementation
  - Cross-platform capability via Mono (Linux/Mac support with limitations)
  - Android version available on Google Play Store

### Map & Visualization Libraries
- **GMap.NET (Custom fork)**
  - Windows Forms map control integration
  - Tile-based mapping system with caching
  - Multi-provider support (Google Maps, Bing Maps, OpenStreetMap)
  - Custom WMS (Web Map Service) layer support
  - Spatial reference handling and projection support

- **SkiaSharp (Graphic Rendering)**
  - Cross-platform 2D graphics library
  - High-performance drawing and rendering
  - Used for HUD (Heads-Up Display) visualization
  - Scale-dependent rendering for waypoint markers

- **OpenStreetMap Integration**
  - Nominatim API integration for geocoding
  - Real-time map tile downloading
  - GeoTIFF support for custom map data
  - SQLite-based tile caching (GMDB format)

### Additional Frontend Components
- **Transitions Library** - UI animation and transitions
- **Theme Manager** - Custom theming and skinning system
- **SharpGL** - OpenGL wrapper for 3D visualization (where applicable)
- **Joystick Integration** - Real-time input handling for RC controllers

---

## Backend/Core Logic Architecture

### Communication & Protocol Handler
- **MAVLink Interface** (Custom C# Implementation)
  - MAVLink v1 and v2 protocol support
  - Async/await pattern for non-blocking operations
  - Heartbeat detection and connection management
  - Packet subscription and event-driven architecture
  - MAVLink 2 signing support for secure communications
  
- **Serial Communication**
  - USB/Serial port connectivity to flight controllers
  - Baud rate configuration (typically 115,200)
  - TCP/UDP support for remote connections
  - Network proxy support for remote telemetry

### Data Processing & Mission Management
- **Mission Command Parser**
  - MAV_CMD processing and execution
  - Navigation (NAV_*) commands
  - Conditional (CONDITION_*) commands
  - Action (DO_*) commands
  - Waypoint management (MISSION_ITEM_INT protocol)
  - Geofence and Rally point handling

- **Telemetry Processing**
  - Real-time vehicle state tracking
  - GPS/RTK positioning (multiple receiver support: u-blox, Septentrio, etc.)
  - IMU/Sensor data aggregation
  - Battery monitoring and telemetry streaming
  - ADS-B (Automatic Dependent Surveillance-Broadcast) integration

### Logging & Data Storage
- **Log Analysis Engine**
  - Dataflash log parsing and analysis
  - Telemetry log (TLog) processing
  - Post-flight statistics and diagnostics
  - CSV export functionality

- **Data Serialization**
  - **Newtonsoft.Json (JSON.NET)** - JSON data serialization/deserialization
  - XML configuration files for autopilot parameters
  - Binary log format support

- **Caching & Storage**
  - **SQLite** - Map tile caching (GMDB database format)
  - Local file system for logs and configuration
  - Settings persistence via XML files

### Vehicle Control & Automation
- **ArduPilot Integration**
  - Parameter download/upload
  - Firmware flashing and installation
  - SITL (Software-in-the-Loop) simulator support
  - Flight mode management
  - Guided mode control

- **Vehicle-Specific Support**
  - Copter (MultiRotor) configurations
  - Plane (Fixed-wing) flight plans
  - Rover (Ground vehicle) operations
  - Submarine depth control
  - Antenna tracker support
  - VTOL/QuadPlane hybrid vehicles

### Advanced Features
- **Real-Time Control**
  - Joystick/Controller input mapping
  - RC channel passthrough
  - Gimbal control and payload management
  - Servo output configuration

- **Safety & Diagnostics**
  - Pre-flight checklist validation
  - Parameter sanity checking
  - Geofence violation alerts
  - Battery failsafe monitoring
  - Audio warning system (text-to-speech)

---

## Libraries & Dependencies

### Core .NET Libraries
| Library | Purpose |
|---------|---------|
| System.Windows.Forms | Desktop UI framework |
| System.Net.Sockets | Network communication |
| System.IO | File I/O operations |
| System.Threading.Tasks | Async/await pattern support |
| System.Reflection | Dynamic type introspection |
| System.Runtime.InteropServices | Native DLL interoperability |
| System.Management | Windows system management |
| System.Globalization | Localization support |

### Third-Party Libraries
| Library | Purpose |
|---------|---------|
| log4net | Logging framework |
| Newtonsoft.Json (JSON.NET) | JSON serialization |
| GMap.NET | Map control and tile management |
| SkiaSharp | Graphics rendering |
| Transitions | UI animations |
| Microsoft.Diagnostics.Runtime | Runtime diagnostics |
| GDAL | Geospatial data support |
| JetBrains.Profiler | Performance profiling |

### Integrated Components (ExtLibs)
- **MAVLink C# Library** - Auto-generated protocol bindings
- **GMap.NET.WindowsForms** - Forms-based map control
- **GMap.NET.Core** - Core mapping functionality
- **SimpleExample** - Minimal example project for integration

---

## Communication Protocols

### Primary Protocols

**MAVLink 2.0**
- Binary wire protocol for vehicle communication
- Heartbeat mechanism for connection detection
- Message queuing with CRC validation
- Supports extended missions and mission item int protocol
- GPS coordinate transmission (latitude/longitude/altitude)
- Telemetry data streaming at configurable rates

**MAVLink 1.0** (Legacy)
- Backward compatibility support
- Simplified message format
- Still widely supported but deprecated

### Network Communication

- **Serial Protocol** - Direct USB/UART connections (most common)
- **TCP** - Network-based telemetry links with acknowledgment
- **UDP** - Low-latency remote telemetry connections
- **MAVLink 2 Signing** - Encrypted passkey authentication for secure links

### Specialized Protocols

- **RTCM** - Real-Time Kinematic GPS correction data
- **ADS-B** - Aircraft position tracking (AVR, Beast formats)
- **NTRIP** - Network Real-Time TCP Protocol for RTK corrections
- **FTP** - File transfer protocol for drone file systems
- **DroneCAN/UAVCAN** - CAN bus protocol for peripheral communication

---

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Mission Planner UI                           │
│            (Windows Forms / Multi-Platform)                     │
├─────────────────────────────────────────────────────────────────┤
│  Flight Planner │ Setup │ Simulation │ Flight Data │ Log Viewer │
└───────────────────┬─────────────────────────────────────────────┘
                    │
┌───────────────────▼─────────────────────────────────────────────┐
│         MAVLink Interface Layer (C# Protocol Handler)           │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Heartbeat Detection │ Mission Mgmt │ Telemetry Process  │   │
│  └──────────────────────────────────────────────────────────┘   │
└───────────────────┬─────────────────────────────────────────────┘
                    │
┌───────────────────▼─────────────────────────────────────────────┐
│         Communication Layer (Serial/TCP/UDP)                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Serial Port │ TCP Socket │ UDP Connection │ Proxy Links  │   │
│  └──────────────────────────────────────────────────────────┘   │
└───────────────────┬─────────────────────────────────────────────┘
                    │
        ┌───────────┴───────────┬──────────────┐
        │                       │              │
        ▼                       ▼              ▼
    [Flight Controller]  [Telemetry Modem]  [GCS Network]
    (Pixhawk/CubeOrange) (3DR Radio/RFD900) (Remote Ops)
```

---

## Data Storage & Persistence

### Local Storage
- **Configuration Files** - XML-based parameter definitions
- **Mission Files** - QGC mission format (.mission)
- **Rally Points** - Saved safety locations (.ral)
- **Geofence Data** - Zone definitions and boundaries
- **Settings** - User preferences and calibration data

### Log Files
- **Dataflash Logs** - Binary flight logs from vehicle SD card
- **Telemetry Logs (TLogs)** - Real-time telemetry stream recordings
- **Application Logs** - Mission Planner debug/info logs (log4net)

### Cache Storage
- **SQLite GMDB Database** - Map tile cache
- **Terrain Data** - Elevation models for flight planning
- **SRTM Data** - Shuttle Radar Topography Mission data

---

## Platform Support

| Platform | Status | Notes |
|----------|--------|-------|
| Windows | ✅ Full Support | Primary platform with complete functionality |
| Linux | ⚠️ Partial (via Mono) | Not officially supported but runs via Mono |
| macOS | ⚠️ Experimental | Limited support, Boot Camp/Parallels recommended |
| iOS | ⚠️ Experimental | Limited functionality |
| Android | ✅ Full Support | Available on Google Play Store |

---

## Key Features Implementation

### Mission Planning
- Point-and-click waypoint entry
- Geofence creation and editing
- Rally point management
- Custom WMS map overlay support
- Terrain analysis and flight path visualization

### Real-Time Monitoring
- Live telemetry display (GPS, altitude, speed, heading)
- HUD (Heads-Up Display) rendering
- Parameter adjustment during flight
- MAVLink message inspector for debugging
- Custom audio alerts based on vehicle telemetry

### Firmware Management
- One-click firmware upload to autopilots
- Supported boards: Pixhawk, CubeOrange, and many others
- Full parameter list download and configuration
- Board detection and compatibility checking

### Post-Flight Analysis
- Log download from vehicle
- Mission replay with telemetry overlay
- Flight statistics calculation
- Performance metrics extraction
- Video linking with flight data (where applicable)

### Simulation & Testing
- SITL integration with ArduPilot simulator
- Parameter validation before upload
- Pre-flight checklist execution
- System health diagnostics

---

## Performance & Optimization

- **Async Architecture** - Non-blocking telemetry processing
- **Tile Caching** - SQLite-based map tile persistence
- **Message Subscription** - Event-driven packet handling
- **Configurable Update Rates** - Adjustable telemetry refresh frequency
- **Threading** - Background threads for communication and file I/O

---

## Integration Points

### External Systems
- **ArduPilot Firmware** - Flight control software ecosystem
- **Google Maps/Bing Maps APIs** - Online mapping services
- **OpenStreetMap** - Open-source mapping data
- **RTK GPS Services** - NTRIP correction providers
- **ADS-B Networks** - Aircraft tracking feeds
- **SITL Simulator** - Software-in-the-loop testing framework

### Developer Extensibility
- **Plugin Architecture** - Load custom mission commands
- **Configuration Pages** - Hardware configuration interfaces
- **MAVLink Message Handler** - Subscribe to custom message types
- **Theme System** - Custom UI theming
- **Open Source** - GPL v3 licensed codebase

---

## Security Features

- **MAVLink 2 Signing** - Passkey-based message authentication
- **Parameter Validation** - Sanity checking before vehicle execution
- **Geofencing** - Autonomous flight boundary enforcement
- **Failsafe Systems** - RTL (Return-to-Launch) on signal loss
- **GPS Spoofing Detection** - EKF-based anomaly detection

---

## Development & Build Environment

- **Primary IDE** - Microsoft Visual Studio (recommended)
- **Language** - C# (.NET Framework 4.6+)
- **Build System** - Visual Studio solution files
- **Version Control** - Git with submodule dependencies
- **CI/CD** - GitHub Actions integration
- **License** - GPL v3

---

## Future-Ready Architecture

- **VTOL Support** - QuadPlane and tilt-rotor platforms
- **Swarm Autonomy** - Multi-vehicle coordination
- **AI Integration** - Autonomous navigation planning
- **Cloud Connectivity** - Remote operations via internet
- **BVLOS Support** - Beyond Visual Line of Sight operations

---

## Conclusion

Mission Planner represents a mature, feature-rich ground control station built on modern C# architecture. Its modular design, extensive protocol support, and active community development make it the de facto standard for ArduPilot operations. The tech stack emphasizes reliability, extensibility, and comprehensive vehicle control across multiple platforms and vehicle types.

**Repository**: https://github.com/ArduPilot/MissionPlanner  
**License**: GPL v3  
**Developer**: Michael Oborne (Primary) + Open Source Community  
**Latest Version**: Actively maintained (as of October 2025)
