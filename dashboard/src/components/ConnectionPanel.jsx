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
      <div className="card-title">Connection</div>
      <form className="conn-form" onSubmit={handleConnect}>
        <input
          className="conn-input"
          value={draft}
          onChange={(e) => setDraft(e.target.value)}
          placeholder="ws://localhost:9090"
          spellCheck={false}
        />
        <div className="conn-buttons">
          <button
            type="submit"
            className="btn btn-primary"
            disabled={connecting}
          >
            {connecting ? 'Connecting…' : 'Connect'}
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
            {connected ? 'Connected' : connecting ? 'Connecting…' : 'Disconnected'}
          </span>
        </div>
        <div className="conn-row">
          <span className="conn-label">Latency</span>
          <span className="conn-value">
            {connected && latency !== null ? `${latency} ms` : '—'}
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
