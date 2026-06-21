import React, { useEffect, useRef, useCallback, useState } from "react";
import { AuthProvider, useAuth } from "./context/AuthContext";
import { RosbridgeProvider } from "./context/RosbridgeContext";
import { ThemeProvider } from "./context/ThemeContext";
import { ToastProvider, useToast } from "./components/Toast";
import { useRosbridgeContext } from "./context/RosbridgeContext";

import Header from "./components/Header";
import MapCanvas from "./components/MapCanvas";
import VideoFeed from "./components/VideoFeed";
import Teleop from "./components/Teleop";
import ServoGauge from "./components/ServoGauge";
import ConnectionPanel from "./components/ConnectionPanel";
import LoginPage from "./components/LoginPage";

function ConnectionToaster() {
  const { connected, connecting } = useRosbridgeContext();
  const toast = useToast();
  const prevRef = React.useRef(null);

  useEffect(() => {
    const prev = prevRef.current;
    if (prev === null) {
      prevRef.current = connected;
      return;
    }
    if (!prev && connected) toast("Connected to ROS bridge", "success");
    if (prev && !connected && !connecting)
      toast("Connection lost — reconnecting…", "error", 5000);
    prevRef.current = connected;
  }, [connected, connecting, toast]);

  return null;
}

const LEFT_MIN = 260;
const LEFT_MAX = 700;
const LEFT_DEFAULT = 340;

function Dashboard() {
  const gridRef = useRef(null);
  const [leftW, setLeftW] = useState(() => {
    const saved = localStorage.getItem("esibot-left-w");
    const v = saved ? parseInt(saved, 10) : LEFT_DEFAULT;
    return Math.max(LEFT_MIN, Math.min(LEFT_MAX, v));
  });
  const dragging = useRef(false);

  const onPointerDown = useCallback((e) => {
    e.preventDefault();
    dragging.current = true;
    document.body.style.cursor = "col-resize";
    document.body.style.userSelect = "none";
  }, []);

  useEffect(() => {
    const onPointerMove = (e) => {
      if (!dragging.current || !gridRef.current) return;
      const rect = gridRef.current.getBoundingClientRect();
      const x = e.clientX - rect.left - 14;
      const clamped = Math.max(LEFT_MIN, Math.min(LEFT_MAX, x));
      setLeftW(clamped);
    };
    const onPointerUp = () => {
      if (!dragging.current) return;
      dragging.current = false;
      document.body.style.cursor = "";
      document.body.style.userSelect = "";
    };
    window.addEventListener("pointermove", onPointerMove);
    window.addEventListener("pointerup", onPointerUp);
    return () => {
      window.removeEventListener("pointermove", onPointerMove);
      window.removeEventListener("pointerup", onPointerUp);
    };
  }, []);

  useEffect(() => {
    localStorage.setItem("esibot-left-w", String(Math.round(leftW)));
  }, [leftW]);

  return (
    <>
      <ConnectionToaster />
      <Header />
      <main
        ref={gridRef}
        className="dashboard-grid"
        style={{ "--left-w": `${leftW}px` }}
      >
        <div className="col-left">
          <VideoFeed />
          <Teleop />
        </div>
        <div
          className="resize-handle"
          onPointerDown={onPointerDown}
          role="separator"
          aria-orientation="vertical"
          aria-label="Resize left panel"
          tabIndex={0}
        >
          <div className="resize-handle-grip" />
        </div>
        <div className="col-center">
          <MapCanvas />
        </div>
        <div className="col-right">
          <ConnectionPanel />
          <ServoGauge />
        </div>
      </main>
    </>
  );
}

function Root() {
  const { authed, checking } = useAuth();

  if (checking) return null;   // wait for /api/session response
  if (!authed) return <LoginPage />;

  return (
    <RosbridgeProvider>
      <ToastProvider>
        <Dashboard />
      </ToastProvider>
    </RosbridgeProvider>
  );
}

export default function App() {
  return (
    <AuthProvider>
      <ThemeProvider>
        <Root />
      </ThemeProvider>
    </AuthProvider>
  );
}
