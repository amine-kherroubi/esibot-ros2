import React, { useState } from 'react'
import { useCamera } from '../hooks/useCamera'

const TOPICS = {
  'Annotated': '/camera/image_annotated',
  'Raw':       '/camera/compressed'
}

export default function VideoFeed() {
  const [selected, setSelected] = useState('Annotated')
  const { imgSrc } = useCamera(TOPICS[selected])

  return (
    <div className="card video-card">
      <div className="card-title">
        Camera
        <div className="topic-switcher">
          {Object.keys(TOPICS).map((k) => (
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

      <div className="video-frame">
        {imgSrc ? (
          <img src={imgSrc} alt="camera feed" className="video-img" />
        ) : (
          <div className="video-placeholder">
            <span>No feed</span>
          </div>
        )}
      </div>
    </div>
  )
}
