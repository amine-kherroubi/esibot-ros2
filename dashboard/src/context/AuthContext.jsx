import React, { createContext, useContext, useEffect, useState } from 'react'

const AuthContext = createContext(null)

export function AuthProvider({ children }) {
  const [authed, setAuthed] = useState(false)
  const [checking, setChecking] = useState(true)

  // On mount: ask the server if the session cookie is still valid
  useEffect(() => {
    fetch('/api/session', { credentials: 'same-origin' })
      .then(r => setAuthed(r.ok && r.status === 200))
      .catch(() => setAuthed(false))
      .finally(() => setChecking(false))
  }, [])

  async function login(username, password) {
    try {
      const r = await fetch('/api/login', {
        method: 'POST',
        credentials: 'same-origin',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username, password }),
      })
      if (r.ok) {
        setAuthed(true)
        return true
      }
      if (r.status === 429) return 'rate_limited'
      return false
    } catch {
      return false
    }
  }

  async function logout() {
    await fetch('/api/logout', { method: 'POST', credentials: 'same-origin' })
      .catch(() => {})
    setAuthed(false)
  }

  return (
    <AuthContext.Provider value={{ authed, checking, login, logout }}>
      {children}
    </AuthContext.Provider>
  )
}

export function useAuth() {
  return useContext(AuthContext)
}
