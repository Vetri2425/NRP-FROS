// src/config.ts

/**
 * Backend configuration
 * Priority: 
 * 1. Use VITE_ROS_HTTP_BASE from .env if set
 * 2. Otherwise, use fixed IP address 192.168.1.100 with port 5001
 */
const BACKEND_FROM_ENV = import.meta.env.VITE_ROS_HTTP_BASE;

/**
 * Fixed backend IP address for the rover backend server
 */
export const BACKEND_IP = BACKEND_FROM_ENV 
  ? new URL(BACKEND_FROM_ENV).hostname 
  : '192.168.1.101';

/**
 * Defines the port the backend server is running on.
 */
export const BACKEND_PORT = BACKEND_FROM_ENV 
  ? new URL(BACKEND_FROM_ENV).port || '5001'
  : '5001';

/**
 * The full URL for the backend API.
 * Uses .env variable if set, otherwise uses fixed IP 192.168.1.29.
 */
export const BACKEND_URL = BACKEND_FROM_ENV || `http://192.168.1.101:5001`;
