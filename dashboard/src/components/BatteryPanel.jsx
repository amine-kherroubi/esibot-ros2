import React from 'react'
import { useBattery } from '../hooks/useBattery'
import { useTheme } from '../context/ThemeContext'

function BatteryIcon({ pct, isLight }) {
  const fill = pct === null ? 0 : Math.max(0, Math.min(100, pct * 100))
  const color = fill > 50
    ? (isLight ? '#16a34a' : '#34d399')
    : fill > 20
      ? (isLight ? '#d97706' : '#fbbf24')
      : (isLight ? '#dc2626' : '#f87171')
  const glowColor = fill > 50
    ? 'rgba(52,211,153,0.3)'
    : fill > 20
      ? 'rgba(251,191,36,0.3)'
      : 'rgba(248,113,113,0.3)'
  const innerW = Math.round((fill / 100) * 30)
  const strokeColor = isLight ? 'rgba(15,23,42,0.12)' : 'rgba(148,163,184,0.15)'

  return (
    <svg width="52" height="26" viewBox="0 0 52 26" role="img" aria-label={`Battery level ${Math.round(fill)} percent`}>
      <defs>
        <filter id="battGlow">
          <feGaussianBlur stdDeviation="2" result="glow"/>
          <feMerge><feMergeNode in="glow"/><feMergeNode in="SourceGraphic"/></feMerge>
        </filter>
      </defs>
      <rect x="2" y="2" width="40" height="22" rx="4" ry="4"
        fill="none" stroke={strokeColor} strokeWidth="1.5" />
      <rect x="43" y="8" width="6" height="10" rx="2" fill={strokeColor} />
      <rect x="5" y="5" width={innerW} height="16" rx="2" fill={color}
        filter="url(#battGlow)" style={{ filter: isLight ? 'none' : `drop-shadow(0 0 4px ${glowColor})` }} />
    </svg>
  )
}

export default function BatteryPanel() {
  const { voltage, percentage, estimatedMinutes, charging } = useBattery()
  const { theme } = useTheme()
  const isLight = theme === 'light'
  const pct = percentage ?? null
  const pctPct = pct !== null ? Math.round(pct * 100) : null
  const color = pctPct !== null
    ? pctPct > 50 ? 'var(--green)' : pctPct > 20 ? 'var(--yellow)' : 'var(--red)'
    : 'var(--text-muted)'

  return (
    <div className="card battery-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <rect width="16" height="10" x="2" y="7" rx="2" ry="2"/><line x1="22" x2="22" y1="11" y2="13"/>
          </svg>
          <h2 className="card-heading">Battery</h2>
        </span>
        {charging && <span className="charging-badge">Charging</span>}
      </div>
      <div className="battery-body">
        <BatteryIcon pct={pct} isLight={isLight} />
        <div className="battery-stats">
          <div className="battery-pct" style={{ color }} aria-live="polite" aria-label={pctPct !== null ? `Battery at ${pctPct} percent` : 'Battery unknown'}>
            {pctPct !== null ? `${pctPct}%` : '--'}
          </div>
          {voltage !== null && (
            <div className="battery-voltage">{voltage.toFixed(2)} V</div>
          )}
          {estimatedMinutes !== null && (
            <div className="battery-time">~{estimatedMinutes} min</div>
          )}
        </div>
      </div>
    </div>
  )
}
