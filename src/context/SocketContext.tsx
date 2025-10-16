// src/context/SocketContext.tsx
import React, { createContext, useContext, useEffect, useRef, useState } from "react";
import { io, Socket } from "socket.io-client";

// 🌍 Where your backend is running
const SOCKET_URL = import.meta.env.VITE_JETSON_BACKEND_URL || "http://localhost:5000";

// 1️⃣ Create a context to hold our single socket connection
const SocketContext = createContext<Socket | null>(null);

// 2️⃣ This provider will start the socket once when the app loads
export const SocketProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const socketRef = useRef<Socket | null>(null);
  const [socket, setSocket] = useState<Socket | null>(null);

  useEffect(() => {
    // Avoid duplicate inits (e.g., hot reload)
    if (socketRef.current) {
      setSocket(socketRef.current);
      return;
    }

    const s = io(SOCKET_URL, {
      reconnection: true,
      reconnectionAttempts: 50,
      reconnectionDelay: 2000,
      reconnectionDelayMax: 15000,
      transports: ["websocket"],
    });

    socketRef.current = s;
    setSocket(s);

    console.log("🔌 Global Socket: connecting to backend...", SOCKET_URL);
    s.on("connect", () => {
      console.log("🟢 Global Socket connected");
      try {
        // Prefer existing active stream; otherwise autostart if the user chose so previously.
        let attempted = false;
        const maybeStart = () => {
          if (attempted) return; attempted = true;
          const should = localStorage.getItem('rtk_should_stream');
          const casterRaw = localStorage.getItem('rtk_caster');
          if (should === 'true' && casterRaw) {
            const caster = JSON.parse(casterRaw);
            if (caster && caster.host && caster.mountpoint && caster.user && caster.password) {
              s.emit('connect_caster', {
                host: caster.host,
                port: parseInt(String(caster.port) || '2101', 10),
                mountpoint: caster.mountpoint,
                user: caster.user,
                password: caster.password,
              });
            }
          }
        };

        // If the backend reports current caster status as Connected, skip autostart
        const onCasterStatus = (msg: { status: string; message?: string }) => {
          const statusLower = (msg?.status || '').toLowerCase();
          if (statusLower !== 'connected') {
            // Not connected yet, proceed with autostart
            maybeStart();
          }
        };
        s.once('caster_status', onCasterStatus);
        // Fallback in case no status arrives
        setTimeout(maybeStart, 700);
      } catch {}
    });
    s.on("disconnect", (reason: string) => console.log("🔴 Global Socket disconnected:", reason));

    // Keep the connection for the app lifetime; disconnect on window unload only
    const handleBeforeUnload = () => {
      try { s.disconnect(); } catch {}
    };
    window.addEventListener("beforeunload", handleBeforeUnload);

    return () => {
      window.removeEventListener("beforeunload", handleBeforeUnload);
      // Do not disconnect here to persist across view changes
    };
  }, []);

  return (
    <SocketContext.Provider value={socket}>
      {children}
    </SocketContext.Provider>
  );
};

// 3️⃣ Helper hook so components can easily use the socket
export const useGlobalSocket = () => {
  return useContext(SocketContext);
};
