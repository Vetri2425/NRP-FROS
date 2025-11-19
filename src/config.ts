// src/config.ts

/**
 * Backend configuration for the rover backend server.
 *
 * To change the backend IP:
 * 1. Set the environment variable VITE_ROS_HTTP_BASE in a .env file (e.g., VITE_ROS_HTTP_BASE=http://192.168.1.100:5001)
 * 2. Or change the default IP below.
 *
 * Priority:
 * 1. VITE_ROS_HTTP_BASE from .env if set
 * 2. Otherwise, use the default IP below
 */
const BACKEND_FROM_ENV = import.meta.env.VITE_ROS_HTTP_BASE;

/**
 * The full URL for the backend API (Socket.IO and HTTP endpoints).
 * Change this default IP if needed, or use .env variable.
 */
export const BACKEND_URL = BACKEND_FROM_ENV || 'http://192.168.1.101:5001';
