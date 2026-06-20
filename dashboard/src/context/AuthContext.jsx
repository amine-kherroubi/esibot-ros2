import React, { createContext, useContext, useState } from 'react'
import { AUTH } from '../config.js'

const AuthContext = createContext(null)

export function AuthProvider({ children }) {
  const [authed, setAuthed] = useState(() => sessionStorage.getItem('esibot_auth') === '1')

  function login(username, password) {
    if (username === AUTH.username && password === AUTH.password) {
      sessionStorage.setItem('esibot_auth', '1')
      setAuthed(true)
      return true
    }
    return false
  }

  function logout() {
    sessionStorage.removeItem('esibot_auth')
    setAuthed(false)
  }

  return (
    <AuthContext.Provider value={{ authed, login, logout }}>
      {children}
    </AuthContext.Provider>
  )
}

export function useAuth() {
  return useContext(AuthContext)
}
