import React, { useState } from 'react'
import { useAuth } from '../context/AuthContext'

export default function LoginPage() {
  const { login } = useAuth()
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState(false)
  const [shake, setShake] = useState(false)

  async function handleSubmit(e) {
    e.preventDefault()
    const result = await login(username, password)
    if (result === true) {
      setError(false)
    } else {
      setError(result === 'rate_limited' ? 'Too many attempts — wait 60s' : 'Invalid credentials')
      setShake(true)
      setTimeout(() => setShake(false), 500)
    }
  }

  function handleChange(setter) {
    return e => { setter(e.target.value); setError(false) }
  }

  return (
    <div className="login-page">
      <div className={`login-card${shake ? ' shake' : ''}`}>
        <div className="login-logo" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2a2 2 0 0 1 2 2c0 .74-.4 1.39-1 1.73V7h1a7 7 0 0 1 7 7h1a1 1 0 0 1 1 1v3a1 1 0 0 1-1 1h-1.17A7 7 0 0 1 14 22h-4a7 7 0 0 1-6.83-3H2a1 1 0 0 1-1-1v-3a1 1 0 0 1 1-1h1a7 7 0 0 1 7-7h1V5.73A2 2 0 0 1 12 2zm-2 10a2 2 0 1 0 0 4 2 2 0 0 0 0-4zm4 0a2 2 0 1 0 0 4 2 2 0 0 0 0-4z" />
          </svg>
        </div>
        <h1 className="login-title">EsiBot</h1>

        <form className="login-form" onSubmit={handleSubmit} noValidate>
          <div className="login-field">
            <label className="login-label" htmlFor="login-username">Username</label>
            <input
              id="login-username"
              className={`conn-input login-input${error ? ' input-error' : ''}`}
              type="text"
              value={username}
              onChange={handleChange(setUsername)}
              autoComplete="username"
              autoFocus
              spellCheck={false}
            />
          </div>
          <div className="login-field">
            <label className="login-label" htmlFor="login-password">Password</label>
            <input
              id="login-password"
              className={`conn-input login-input${error ? ' input-error' : ''}`}
              type="password"
              value={password}
              onChange={handleChange(setPassword)}
              autoComplete="current-password"
            />
          </div>
          {error && <p className="login-error" role="alert">{error}</p>}
          <button className="btn btn-primary login-btn" type="submit">
            Sign in
          </button>
        </form>
      </div>
    </div>
  )
}
