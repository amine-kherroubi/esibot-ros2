import React, { useEffect } from 'react'
import { RosbridgeProvider } from './context/RosbridgeContext'
import { ThemeProvider } from './context/ThemeContext'
import { ToastProvider, useToast } from './components/Toast'
import { useRosbridgeContext } from './context/RosbridgeContext'

import Header          from './components/Header'
import MapCanvas       from './components/MapCanvas'
import VideoFeed       from './components/VideoFeed'
import Teleop          from './components/Teleop'
import ServoGauge      from './components/ServoGauge'
import ConnectionPanel from './components/ConnectionPanel'

function ConnectionToaster() {
  const { connected, connecting } = useRosbridgeContext()
  const toast = useToast()
  const prevRef = React.useRef(null)

  useEffect(() => {
    const prev = prevRef.current
    if (prev === null) { prevRef.current = connected; return }
    if (!prev && connected)  toast('Connected to ROS bridge', 'success')
    if (prev  && !connected && !connecting) toast('Connection lost — reconnecting…', 'error', 5000)
    prevRef.current = connected
  }, [connected, connecting, toast])

  return null
}

function Dashboard() {
  return (
    <>
      <ConnectionToaster />
      <Header />
      <main className="dashboard-grid">
        <div className="col-left">
          <VideoFeed />
          <Teleop />
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
  )
}

export default function App() {
  return (
    <ThemeProvider>
      <RosbridgeProvider>
        <ToastProvider>
          <Dashboard />
        </ToastProvider>
      </RosbridgeProvider>
    </ThemeProvider>
  )
}
