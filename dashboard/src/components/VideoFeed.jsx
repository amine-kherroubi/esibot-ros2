import React, { useEffect, useState } from 'react'
import { useCamera } from '../hooks/useCamera'
import { useEsp32Ip } from '../hooks/useEsp32Ip'
import { useDetections } from '../hooks/useDetections'

const TABS = ['Annotated', 'Raw']

export default function VideoFeed() {
  const [selected, setSelected] = useState('Annotated')
  const { imgSrc } = useCamera(selected === 'Annotated' ? '/camera/image_annotated/compressed' : null)
  const { ip, setIp, streamUrl } = useEsp32Ip()
  const { obstacles, signs } = useDetections()

  const [showSettings, setShowSettings] = useState(false)
  const [draft, setDraft] = useState(ip)
  const [rawError, setRawError] = useState(false)

  useEffect(() => { setRawError(false) }, [streamUrl])

  const openSettings = () => {
    setDraft(ip)
    setShowSettings((s) => !s)
  }

  const handleSave = (e) => {
    e.preventDefault()
    setIp(draft)
    setShowSettings(false)
  }

  return (
    <div className="card video-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <path d="m22 8-6 4 6 4V8Z"/><rect width="14" height="12" x="2" y="6" rx="2" ry="2"/>
          </svg>
          <h2 className="card-heading">Camera</h2>
        </span>
        <div className="card-title-actions">
          <div className="topic-switcher">
            {TABS.map((k) => (
              <button
                key={k}
                className={`topic-btn${selected === k ? ' active' : ''}`}
                onClick={() => setSelected(k)}
              >
                {k}
              </button>
            ))}
          </div>
          <button
            type="button"
            className={`icon-btn${showSettings ? ' active' : ''}`}
            onClick={openSettings}
            title="ESP32-CAM IP"
            aria-label="ESP32-CAM IP settings"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
              <circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1Z"/>
            </svg>
          </button>
        </div>
      </div>

      {(obstacles.length > 0 || signs.length > 0) && (
        <div className="detection-badges">
          {obstacles.map((o, i) => (
            <span
              key={`obs-${i}`}
              className={`det-badge ${o.in_lane ? 'det-badge--alert' : 'det-badge--obstacle'}`}
              title={`Confidence: ${Math.round(o.conf * 100)}%`}
            >
              {o.label} · {o.proximity}
            </span>
          ))}
          {signs.map((s, i) => (
            <span
              key={`sign-${i}`}
              className="det-badge det-badge--sign"
              title={`Confidence: ${Math.round(s.conf * 100)}%`}
            >
              {s.label}
            </span>
          ))}
        </div>
      )}

      {showSettings && (
        <form className="cam-settings" onSubmit={handleSave}>
          <label className="sr-only" htmlFor="esp32-ip">ESP32-CAM IP</label>
          <input
            id="esp32-ip"
            type="text"
            className="conn-input"
            value={draft}
            onChange={(e) => setDraft(e.target.value)}
            placeholder="192.168.1.80"
            spellCheck={false}
            autoComplete="off"
          />
          <button type="submit" className="btn btn-primary cam-settings-save">Save</button>
        </form>
      )}

      <div className="video-frame" aria-live="polite">
        {selected === 'Raw' ? (
          rawError ? (
            <div className="video-placeholder">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                <path d="m22 8-6 4 6 4V8Z"/><rect width="14" height="12" x="2" y="6" rx="2" ry="2"/>
              </svg>
              <span>ESP32-CAM unreachable at {ip}</span>
            </div>
          ) : (
            <img
              key={streamUrl}
              src={streamUrl}
              alt="raw camera feed"
              className="video-img"
              onError={() => setRawError(true)}
            />
          )
        ) : imgSrc ? (
          <img src={imgSrc} alt="annotated camera feed" className="video-img" />
        ) : (
          <div className="video-placeholder">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
              <path d="m22 8-6 4 6 4V8Z"/><rect width="14" height="12" x="2" y="6" rx="2" ry="2"/>
            </svg>
            <span>Waiting for camera feed...</span>
          </div>
        )}
      </div>
    </div>
  )
}
