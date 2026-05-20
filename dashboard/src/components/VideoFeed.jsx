import React, { useState } from 'react'
import { useCamera } from '../hooks/useCamera'
import { ESP32_STREAM_URL } from '../config'

const TABS = ['Annotated', 'Raw']

export default function VideoFeed() {
  const [selected, setSelected] = useState('Annotated')
  const { imgSrc } = useCamera(selected === 'Annotated' ? '/camera/image_annotated' : null)

  return (
    <div className="card video-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <path d="m22 8-6 4 6 4V8Z"/><rect width="14" height="12" x="2" y="6" rx="2" ry="2"/>
          </svg>
          <h2 className="card-heading">Camera</h2>
        </span>
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
      </div>

      <div className="video-frame" aria-live="polite">
        {selected === 'Raw' ? (
          <img src={ESP32_STREAM_URL} alt="raw camera feed" className="video-img" />
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
