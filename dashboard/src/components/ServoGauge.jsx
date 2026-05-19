import React from 'react'
import { useServo } from '../hooks/useServo'
import { useTheme } from '../context/ThemeContext'

export default function ServoGauge() {
  const { angle } = useServo()
  const { theme } = useTheme()
  const isLight = theme === 'light'

  const pct = angle / 180
  const toRad = (deg) => (deg * Math.PI) / 180
  const cx = 60, cy = 60, r = 44
  const startAngle = toRad(210)
  const endAngle   = toRad(210 + 240 * pct)

  const arcPath = (from, to) => {
    const x1 = cx + r * Math.cos(from)
    const y1 = cy + r * Math.sin(from)
    const x2 = cx + r * Math.cos(to)
    const y2 = cy + r * Math.sin(to)
    const large = to - from > Math.PI ? 1 : 0
    return `M ${x1} ${y1} A ${r} ${r} 0 ${large} 1 ${x2} ${y2}`
  }

  const trackColor = isLight ? 'rgba(15,23,42,0.06)' : 'rgba(148,163,184,0.08)'
  const textColor = isLight ? '#1e293b' : '#e2e8f0'
  const labelColor = isLight ? '#64748b' : '#64748b'
  const gradStart = isLight ? '#0284c7' : '#22d3ee'
  const gradEnd = isLight ? '#0891b2' : '#3b82f6'
  const glowFilter = isLight
    ? 'drop-shadow(0 0 4px rgba(2,132,199,0.2))'
    : 'drop-shadow(0 0 6px rgba(56,189,248,0.3))'

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
        <svg width="120" height="100" viewBox="0 0 120 100" role="img" aria-label={`Radar servo angle: ${Math.round(angle)} degrees`}>
          <defs>
            <linearGradient id="gaugeGrad" x1="0" y1="0" x2="1" y2="0">
              <stop offset="0%" stopColor={gradStart} />
              <stop offset="100%" stopColor={gradEnd} />
            </linearGradient>
            <filter id="gaugeGlow">
              <feGaussianBlur stdDeviation="3" result="glow"/>
              <feMerge><feMergeNode in="glow"/><feMergeNode in="SourceGraphic"/></feMerge>
            </filter>
          </defs>
          <path
            d={arcPath(toRad(210), toRad(450))}
            fill="none" stroke={trackColor} strokeWidth="8" strokeLinecap="round"
          />
          {pct > 0 && (
            <path
              d={arcPath(startAngle, endAngle)}
              fill="none" stroke="url(#gaugeGrad)" strokeWidth="8" strokeLinecap="round"
              filter="url(#gaugeGlow)"
              style={{ filter: glowFilter }}
            />
          )}
          <text x="60" y="56" textAnchor="middle" fill={textColor} fontSize="24" fontWeight="600"
            fontFamily="'JetBrains Mono', 'Fira Code', monospace">
            {Math.round(angle)}°
          </text>
          <text x="60" y="74" textAnchor="middle" fill={labelColor} fontSize="13"
            fontFamily="Inter, sans-serif" letterSpacing="0.08em">
            RADAR
          </text>
        </svg>
        <div className="servo-limits">
          <span>0°</span><span>180°</span>
        </div>
      </div>
    </div>
  )
}
