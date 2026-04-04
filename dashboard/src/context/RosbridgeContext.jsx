import React, {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
  useCallback
} from 'react'
import ROSLIB from 'roslib'
import { ROSBRIDGE_URL } from '../config.js'

export const RosbridgeContext = createContext(null)

export function RosbridgeProvider({ children }) {
  const [connected, setConnected]   = useState(false)
  const [connecting, setConnecting] = useState(false)
  const [latency, setLatency]       = useState(null)
  const [url, setUrl]               = useState(ROSBRIDGE_URL)

  const rosRef         = useRef(null)
  const reconnectTimer = useRef(null)
  const latencyTimer   = useRef(null)
  const mountedRef     = useRef(true)

  const clearReconnect = () => {
    if (reconnectTimer.current) {
      clearTimeout(reconnectTimer.current)
      reconnectTimer.current = null
    }
  }

  const measureLatency = useCallback((ros) => {
    if (!ros) return
    const start = Date.now()
    const svc = new ROSLIB.Service({
      ros,
      name: '/rosapi/topics',
      serviceType: 'rosapi/Topics'
    })
    svc.callService(new ROSLIB.ServiceRequest(), () => {
      if (mountedRef.current) setLatency(Date.now() - start)
    }, () => {
      if (mountedRef.current) setLatency(null)
    })
  }, [])

  const connect = useCallback((targetUrl) => {
    if (rosRef.current) {
      try { rosRef.current.close() } catch (_) {}
      rosRef.current = null
    }
    clearReconnect()

    if (!mountedRef.current) return

    setConnecting(true)
    setConnected(false)

    const ros = new ROSLIB.Ros({ url: targetUrl || url })
    rosRef.current = ros

    ros.on('connection', () => {
      if (!mountedRef.current) return
      setConnected(true)
      setConnecting(false)
      clearReconnect()

      // Start periodic latency measurement
      if (latencyTimer.current) clearInterval(latencyTimer.current)
      measureLatency(ros)
      latencyTimer.current = setInterval(() => measureLatency(ros), 2000)
    })

    ros.on('error', () => {
      if (!mountedRef.current) return
      setConnected(false)
      setConnecting(false)
    })

    ros.on('close', () => {
      if (!mountedRef.current) return
      setConnected(false)
      setConnecting(false)
      if (latencyTimer.current) {
        clearInterval(latencyTimer.current)
        latencyTimer.current = null
      }
      // Auto-reconnect after 5s
      clearReconnect()
      reconnectTimer.current = setTimeout(() => {
        if (mountedRef.current) connect(targetUrl || url)
      }, 5000)
    })
  }, [url, measureLatency])

  const disconnect = useCallback(() => {
    clearReconnect()
    if (latencyTimer.current) {
      clearInterval(latencyTimer.current)
      latencyTimer.current = null
    }
    if (rosRef.current) {
      try { rosRef.current.close() } catch (_) {}
      rosRef.current = null
    }
    setConnected(false)
    setConnecting(false)
  }, [])

  // Initial connection
  useEffect(() => {
    connect(url)
    return () => {
      mountedRef.current = false
      clearReconnect()
      if (latencyTimer.current) clearInterval(latencyTimer.current)
      if (rosRef.current) {
        try { rosRef.current.close() } catch (_) {}
      }
    }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  const value = {
    ros: rosRef.current,
    rosRef,
    connected,
    connecting,
    latency,
    url,
    setUrl,
    connect,
    disconnect
  }

  return (
    <RosbridgeContext.Provider value={value}>
      {children}
    </RosbridgeContext.Provider>
  )
}

export function useRosbridgeContext() {
  return useContext(RosbridgeContext)
}
