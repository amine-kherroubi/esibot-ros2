import React, { useState } from 'react'
import { useRosbridgeContext } from '../context/RosbridgeContext'

export default function ConnectionPanel() {
  const { url, setUrl, connect, disconnect, connected, connecting, latency } = useRosbridgeContext()
  const [draft, setDraft] = useState(url)

  const handleConnect = (e) => {
    e.preventDefault()
    setUrl(draft)
    connect(draft)
  }

  return (
    <div className="card connection-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/><path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><line x1="12" x2="12.01" y1="20" y2="20"/>
          </svg>
          <h2 className="card-heading">Connection</h2>
        </span>
      </div>
      <form className="conn-form" onSubmit={handleConnect}>
        <label className="sr-only" htmlFor="ws-url">WebSocket URL</label>
        <input
          id="ws-url"
          type="url"
          className="conn-input"
          value={draft}
          onChange={(e) => setDraft(e.target.value)}
          placeholder="ws://localhost:9090"
          spellCheck={false}
          autoComplete="url"
        />
        <div className="conn-buttons">
          <button
            type="submit"
            className="btn btn-primary"
            disabled={connecting}
          >
            {connecting ? 'Connecting...' : 'Connect'}
          </button>
          <button
            type="button"
            className="btn btn-secondary"
            onClick={disconnect}
            disabled={!connected && !connecting}
          >
            Disconnect
          </button>
        </div>
      </form>

      <div className="conn-info">
        <div className="conn-row">
          <span className="conn-label">Status</span>
          <span className={`conn-value ${connected ? 'text-green' : 'text-red'}`}>
            {connected ? 'Connected' : connecting ? 'Connecting...' : 'Disconnected'}
          </span>
        </div>
        <div className="conn-row">
          <span className="conn-label">Latency</span>
          <span className="conn-value">
            {connected && latency !== null ? `${latency} ms` : '--'}
          </span>
        </div>
        <div className="conn-row">
          <span className="conn-label">URL</span>
          <span className="conn-value conn-url">{url}</span>
        </div>
      </div>
    </div>
  )
}
