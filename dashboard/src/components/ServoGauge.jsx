import React from 'react'
import { useServo } from '../hooks/useServo'
import { useTheme } from '../context/ThemeContext'

export default function ServoGauge() {
  const { angle } = useServo()
  const { theme } = useTheme()
  const isLight = theme === 'light'

  const minDeg = -180
  const maxDeg = 180
  const pct = (angle - minDeg) / (maxDeg - minDeg)

  const trackColor = isLight ? 'rgba(15,23,42,0.08)' : 'rgba(148,163,184,0.12)'
  const fillColor = isLight ? '#0284c7' : '#22d3ee'
  const textColor = isLight ? '#1e293b' : '#e2e8f0'
  const labelColor = isLight ? '#64748b' : '#64748b'
  const glowFilter = isLight
    ? 'none'
    : 'drop-shadow(0 0 4px rgba(34,211,238,0.4))'

  const barW = 200
  const barH = 10
  const barX = 10
  const barY = 38
  const cursorX = barX + pct * barW
  const centerX = barX + 0.5 * barW

  return (
    <div className="card servo-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <circle cx="12" cy="12" r="10"/><path d="M12 2a14.5 14.5 0 0 0 0 20 14.5 14.5 0 0 0 0-20"/><path d="M2 12h20"/>
          </svg>
          <h2 className="card-heading">Radar Servo</h2>
        </span>
      </div>
      <div className="servo-body">
        <svg width="220" height="70" viewBox="0 0 220 70" role="img" aria-label={`Radar servo angle: ${Math.round(angle)} degrees`}>
          {/* Track */}
          <rect x={barX} y={barY} width={barW} height={barH} rx="5" fill={trackColor} />
          {/* Center tick */}
          <line x1={centerX} y1={barY - 2} x2={centerX} y2={barY + barH + 2} stroke={labelColor} strokeWidth="1" opacity="0.5" />
          {/* Cursor */}
          <circle cx={cursorX} cy={barY + barH / 2} r="8" fill={fillColor} style={{ filter: glowFilter }} />
          <circle cx={cursorX} cy={barY + barH / 2} r="3" fill={isLight ? '#fff' : '#0f172a'} />
          {/* Angle text */}
          <text x="110" y="18" textAnchor="middle" fill={textColor} fontSize="20" fontWeight="600"
            fontFamily="'JetBrains Mono', 'Fira Code', monospace">
            {Math.round(angle)}°
          </text>
          {/* Labels */}
          <text x={barX} y={barY + barH + 16} textAnchor="middle" fill={labelColor} fontSize="11"
            fontFamily="Inter, sans-serif">-180°</text>
          <text x={centerX} y={barY + barH + 16} textAnchor="middle" fill={labelColor} fontSize="11"
            fontFamily="Inter, sans-serif">0°</text>
          <text x={barX + barW} y={barY + barH + 16} textAnchor="middle" fill={labelColor} fontSize="11"
            fontFamily="Inter, sans-serif">+180°</text>
        </svg>
      </div>
    </div>
  )
}
