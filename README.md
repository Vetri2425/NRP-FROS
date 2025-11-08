# NRP ROS V2 - Frontend Only

<div align="center">
<img width="1200" height="475" alt="GHBanner" src="https://github.com/user-attachments/assets/0aa67016-6eaf-458a-adb2-6e31a0763ed6" />
</div>

A React + Vite frontend application for rover mission planning and control.

## Prerequisites

- Node.js (v16 or higher)
- npm or yarn

## Installation

1. Clone the repository:
```bash
git clone https://github.com/Vetri2425/NRP_ROS_V2.git
cd NRP_ROS_V2
```

2. Install dependencies:
```bash
npm install
```

3. (Optional) Configure environment variables:
   - Copy `.env` to `.env.local` if you need custom configuration
   - Set `GEMINI_API_KEY` if using AI features
   - Set `VITE_JETSON_BACKEND_URL` to point to your backend server

## Running the Application

Start the development server:

```bash
npm run dev
```

The application will be available at `http://localhost:3000`

## Building for Production

Create a production build:

```bash
npm run build
```

Preview the production build:

```bash
npm run preview
```

## Project Structure

```
NRP_ROS_V2/
 src/                    # Source files
    components/         # React components
    context/           # React context providers
    hooks/             # Custom React hooks
    tools/             # Mission planning tools
    types/             # TypeScript type definitions
    utils/             # Utility functions
    App.tsx            # Main app component
    config.ts          # Configuration
    index.tsx          # Entry point
 index.html             # HTML template
 package.json           # Dependencies and scripts
 vite.config.ts         # Vite configuration
 tsconfig.json          # TypeScript configuration
 tailwind.config.js     # Tailwind CSS configuration
 postcss.config.js      # PostCSS configuration
```

## Features

- Mission planning interface
- Real-time telemetry display
- Waypoint management
- Servo control
- RTK GPS integration
- Mission logging and export

## Technologies

- React 19
- TypeScript
- Vite
- Tailwind CSS
- Socket.IO Client

## License

ISC

## Repository

https://github.com/Vetri2425/NRP_ROS_V2
