import React from 'react'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { useTheme } from '../context/ThemeContext'
import { useSystemStatus } from '../hooks/useSystemStatus'
import { ROBOT_NAME } from '../config.js'

const SunIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <circle cx="12" cy="12" r="4"/><path d="M12 2v2"/><path d="M12 20v2"/><path d="m4.93 4.93 1.41 1.41"/><path d="m17.66 17.66 1.41 1.41"/><path d="M2 12h2"/><path d="M20 12h2"/><path d="m6.34 17.66-1.41 1.41"/><path d="m19.07 4.93-1.41 1.41"/>
  </svg>
)

const MoonIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M12 3a6 6 0 0 0 9 9 9 9 0 1 1-9-9Z"/>
  </svg>
)

const LABELS = { driver: 'DRV', radar: 'RAD', map: 'MAP', nav2: 'NAV' }
const TITLES = {
  driver: 'Driver (odom)',
  radar: 'Radar (scan)',
  map: 'Map server',
  nav2: 'Navigation stack',
}

function SystemStatusBadge() {
  const sys = useSystemStatus()
  const keys = ['driver', 'radar', 'map', 'nav2']

  const summaryClass = sys.summary === 'ready' ? 'sys-ready'
    : sys.summary === 'starting' ? 'sys-starting'
    : 'sys-offline'

  return (
    <div className={`sys-status ${summaryClass}`} role="status" aria-label="System status">
      {keys.map(k => (
        <span key={k} className="sys-node" title={`${TITLES[k]}: ${sys[k]}`}>
          <span className={`sys-dot sys-${sys[k]}`} aria-hidden="true" />
          <span className="sys-label">{LABELS[k]}</span>
        </span>
      ))}
    </div>
  )
}

export default function Header() {
  const { connected, connecting, latency } = useRosbridgeContext()
  const { theme, toggle } = useTheme()

  const dotClass = connected ? 'connected' : connecting ? 'connecting' : 'disconnected'
  const statusLabel = connected ? 'Connected' : connecting ? 'Connecting' : 'Disconnected'

  return (
    <header className="header" role="banner">
      <div className="header-left">
        <div className="header-logo" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2a2 2 0 0 1 2 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 0 1 7 7h1a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1h-1.17A7 7 0 0 1 14 22h-4a7 7 0 0 1-6.83-3H2a1 1 0 0 1-1-1v-3a1 1 0 0 1 1-1h1a7 7 0 0 1 7-7h1V5.73A2 2 0 0 1 12 2zm-2 10a2 2 0 1 0 0 4 2 2 0 0 0 0-4zm4 0a2 2 0 1 0 0 4 2 2 0 0 0 0-4z"/>
          </svg>
        </div>
        <h1 className="header-title">{ROBOT_NAME}</h1>
        <span className="header-subtitle">Mission Control</span>
      </div>

      <nav className="header-right" aria-label="Robot status and controls">
        <SystemStatusBadge />
        {connected && latency !== null && (
          <span className="latency-badge" aria-label={`Latency ${latency} milliseconds`}>{latency} ms</span>
        )}
        <div className="status-indicator" aria-live="polite">
          <span className={`status-dot ${dotClass}`} aria-hidden="true" />
          <span className="status-label">{statusLabel}</span>
        </div>
        <button
          className="theme-toggle"
          onClick={toggle}
          aria-label={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
          title={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
        >
          {theme === 'dark' ? <SunIcon /> : <MoonIcon />}
        </button>
      </nav>
    </header>
  )
}
