import React from 'react'
import { useServo } from '../hooks/useServo'

export default function ServoGauge() {
  const { angle } = useServo()

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

  return (
    <div className="card servo-card">
      <div className="card-title">Radar Servo <span className="hint">lecture seule</span></div>
      <div className="servo-body">
        <svg width="120" height="120" viewBox="0 0 120 120">
          <path
            d={arcPath(toRad(210), toRad(450))}
            fill="none" stroke="#334155" strokeWidth="8" strokeLinecap="round"
          />
          <path
            d={arcPath(startAngle, endAngle)}
            fill="none" stroke="#3b82f6" strokeWidth="8" strokeLinecap="round"
          />
          <text x="60" y="58" textAnchor="middle" fill="#f1f5f9" fontSize="18" fontWeight="600">
            {Math.round(angle)}°
          </text>
          <text x="60" y="74" textAnchor="middle" fill="#94a3b8" fontSize="10">
            radar
          </text>
        </svg>
        <div className="servo-limits">
          <span>0°</span><span>180°</span>
        </div>
      </div>
    </div>
  )
}
