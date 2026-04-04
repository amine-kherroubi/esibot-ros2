import React from 'react'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { ROBOT_NAME } from '../config.js'

export default function Header() {
  const { connected, connecting, latency } = useRosbridgeContext()

  const statusColor = connected ? '#22c55e' : connecting ? '#f59e0b' : '#ef4444'
  const statusLabel = connected ? 'Connected' : connecting ? 'Connecting…' : 'Disconnected'

  return (
    <header className="header">
      <div className="header-left">
        <span className="header-icon">🤖</span>
        <span className="header-title">{ROBOT_NAME}</span>
      </div>

      <div className="header-right">
        {connected && latency !== null && (
          <span className="latency-badge">{latency} ms</span>
        )}
        <span className="status-dot" style={{ backgroundColor: statusColor }} />
        <span className="status-label">{statusLabel}</span>
      </div>
    </header>
  )
}
