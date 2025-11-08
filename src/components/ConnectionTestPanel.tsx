import React, { useEffect, useState } from 'react';
import { BACKEND_URL } from '../config';

/**
 * ConnectionTestPanel - Quick test to verify backend connectivity
 * 
 * This panel tests:
 * 1. HTTP connection to backend API
 * 2. Socket.IO WebSocket connection
 * 3. Telemetry data reception
 * 
 * Usage: Add <ConnectionTestPanel /> to your App temporarily
 */
const ConnectionTestPanel: React.FC = () => {
  const [httpStatus, setHttpStatus] = useState<'testing' | 'success' | 'failed'>('testing');
  const [socketStatus, setSocketStatus] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');
  const [telemetryReceived, setTelemetryReceived] = useState(false);
  const [lastEventType, setLastEventType] = useState<string | null>(null);
  const [lastEventData, setLastEventData] = useState<any>(null);
  const [isVisible, setIsVisible] = useState(true);

  useEffect(() => {
    // Test HTTP connection
    const testHttp = async () => {
      try {
        const response = await fetch(`${BACKEND_URL}/api/health`, {
          method: 'GET',
          headers: { 'Content-Type': 'application/json' },
        });
        
        if (response.ok) {
          setHttpStatus('success');
        } else {
          setHttpStatus('failed');
        }
      } catch (error) {
        console.error('HTTP test failed:', error);
        setHttpStatus('failed');
      }
    };

    testHttp();

    // Listen for console logs about Socket.IO
    const originalLog = console.log;
    console.log = (...args) => {
      originalLog(...args);
      
      const message = args.join(' ');
      
      if (message.includes('Socket connected successfully')) {
        setSocketStatus('connected');
      } else if (message.includes('Socket disconnected')) {
        setSocketStatus('disconnected');
      } else if (message.includes('[RTK DEBUG] telemetry event received') || 
                 message.includes('[RTK DEBUG] rover_data event received')) {
        setTelemetryReceived(true);
        
        // Extract event type
        if (message.includes('telemetry event received')) {
          setLastEventType('telemetry');
        } else if (message.includes('rover_data event received')) {
          setLastEventType('rover_data');
        }
        
        // Try to extract data
        if (args[1]) {
          setLastEventData(args[1]);
        }
      }
    };

    return () => {
      console.log = originalLog;
    };
  }, []);

  if (!isVisible) {
    return (
      <button
        onClick={() => setIsVisible(true)}
        className="fixed top-4 right-4 z-50 bg-blue-600 hover:bg-blue-700 text-white px-3 py-2 rounded text-sm"
      >
        🔧 Show Connection Test
      </button>
    );
  }

  return (
    <div className="fixed top-4 right-4 z-50 bg-gray-900 border-2 border-blue-500 rounded-lg shadow-2xl p-4 w-96">
      <div className="flex justify-between items-center mb-3 pb-2 border-b border-gray-700">
        <h3 className="text-blue-400 font-bold">🔧 Connection Test</h3>
        <button
          onClick={() => setIsVisible(false)}
          className="text-gray-400 hover:text-white text-xl"
        >
          ×
        </button>
      </div>

      {/* Test Results */}
      <div className="space-y-3 text-sm">
        {/* HTTP Test */}
        <div className="flex items-center justify-between">
          <span className="text-gray-300">HTTP API:</span>
          <span className={`font-bold ${
            httpStatus === 'success' ? 'text-green-400' : 
            httpStatus === 'failed' ? 'text-red-400' : 
            'text-yellow-400'
          }`}>
            {httpStatus === 'success' && '✅ Connected'}
            {httpStatus === 'failed' && '❌ Failed'}
            {httpStatus === 'testing' && '⏳ Testing...'}
          </span>
        </div>

        {/* Socket.IO Test */}
        <div className="flex items-center justify-between">
          <span className="text-gray-300">Socket.IO:</span>
          <span className={`font-bold ${
            socketStatus === 'connected' ? 'text-green-400' : 
            socketStatus === 'disconnected' ? 'text-red-400' : 
            'text-yellow-400'
          }`}>
            {socketStatus === 'connected' && '✅ Connected'}
            {socketStatus === 'disconnected' && '❌ Disconnected'}
            {socketStatus === 'connecting' && '⏳ Connecting...'}
          </span>
        </div>

        {/* Telemetry Test */}
        <div className="flex items-center justify-between">
          <span className="text-gray-300">Telemetry:</span>
          <span className={`font-bold ${telemetryReceived ? 'text-green-400' : 'text-yellow-400'}`}>
            {telemetryReceived ? '✅ Receiving' : '⏳ Waiting...'}
          </span>
        </div>

        {/* Last Event Info */}
        {lastEventType && (
          <div className="mt-3 pt-3 border-t border-gray-700">
            <p className="text-xs text-gray-400 mb-1">Last Event:</p>
            <p className="text-purple-400 font-mono text-xs mb-2">{lastEventType}</p>
            
            {lastEventData && (
              <div className="bg-black bg-opacity-50 rounded p-2 max-h-32 overflow-y-auto">
                <pre className="text-xs text-green-400 font-mono whitespace-pre-wrap">
                  {JSON.stringify(lastEventData, null, 2)}
                </pre>
              </div>
            )}
          </div>
        )}
      </div>

      {/* Backend URL */}
      <div className="mt-3 pt-3 border-t border-gray-700">
        <p className="text-xs text-gray-400">Backend URL:</p>
        <p className="text-xs font-mono text-blue-300 break-all">{BACKEND_URL}</p>
      </div>

      {/* Instructions */}
      <div className="mt-3 pt-3 border-t border-gray-700 text-xs text-gray-400">
        <p className="mb-2">✅ All green = Backend connected and sending data</p>
        <p className="mb-2">❌ HTTP Failed = Backend not running or wrong IP</p>
        <p className="mb-2">❌ Socket.IO Disconnected = WebSocket issue</p>
        <p>⏳ Telemetry Waiting = Connected but no data (check backend logs)</p>
      </div>
    </div>
  );
};

export default ConnectionTestPanel;
