import React from 'react'
import { useBattery } from '../hooks/useBattery'

function BatteryIcon({ pct }) {
  const fill = pct === null ? 0 : Math.max(0, Math.min(100, pct * 100))
  const color = fill > 50 ? '#22c55e' : fill > 20 ? '#f59e0b' : '#ef4444'
  const innerW = Math.round((fill / 100) * 28)

  return (
    <svg width="44" height="22" viewBox="0 0 44 22">
      {/* Body */}
      <rect x="1" y="1" width="36" height="20" rx="3" ry="3"
        fill="none" stroke="#64748b" strokeWidth="2" />
      {/* Terminal */}
      <rect x="38" y="7" width="5" height="8" rx="1.5" fill="#64748b" />
      {/* Fill */}
      <rect x="3" y="3" width={innerW} height="16" rx="1.5" fill={color} />
    </svg>
  )
}

export default function BatteryPanel() {
  const { voltage, percentage, estimatedMinutes, charging } = useBattery()
  const pct = percentage ?? null
  const pctPct = pct !== null ? Math.round(pct * 100) : null

  return (
    <div className="card battery-card">
      <div className="card-title">Battery {charging && <span className="charging-badge">⚡ Charging</span>}</div>
      <div className="battery-body">
        <BatteryIcon pct={pct} />
        <div className="battery-stats">
          <div className="battery-pct">
            {pctPct !== null ? `${pctPct}%` : '—'}
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
