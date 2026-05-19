import React, { createContext, useContext, useState, useCallback, useRef } from 'react'

const ToastCtx = createContext(null)

let _id = 0

export function ToastProvider({ children }) {
  const [toasts, setToasts] = useState([])
  const timersRef = useRef({})

  const dismiss = useCallback((id) => {
    setToasts((t) => t.map((x) => x.id === id ? { ...x, exiting: true } : x))
    setTimeout(() => {
      setToasts((t) => t.filter((x) => x.id !== id))
    }, 200)
    if (timersRef.current[id]) {
      clearTimeout(timersRef.current[id])
      delete timersRef.current[id]
    }
  }, [])

  const toast = useCallback((message, type = 'info', duration = 3500) => {
    const id = ++_id
    setToasts((t) => [...t.slice(-4), { id, message, type, exiting: false }])
    timersRef.current[id] = setTimeout(() => dismiss(id), duration)
    return id
  }, [dismiss])

  return (
    <ToastCtx.Provider value={toast}>
      {children}
      <div className="toast-container" aria-live="polite">
        {toasts.map((t) => (
          <div
            key={t.id}
            className={`toast toast-${t.type}${t.exiting ? ' toast-exit' : ''}`}
            onClick={() => dismiss(t.id)}
            onKeyDown={(e) => { if (e.key === 'Enter' || e.key === 'Escape') dismiss(t.id) }}
            tabIndex={0}
            role={t.type === 'error' || t.type === 'warn' ? 'alert' : 'status'}
          >
            {t.message}
            <span className="sr-only">, click to dismiss</span>
          </div>
        ))}
      </div>
    </ToastCtx.Provider>
  )
}

export function useToast() {
  return useContext(ToastCtx)
}
