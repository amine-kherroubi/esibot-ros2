import { useCallback, useState } from 'react'
import { ESP32_CAM_DEFAULT_IP } from '../config'

const STORAGE_KEY = 'esibot.esp32CamIp'

/**
 * Holds the ESP32-CAM host (IP, optionally with :port), persisted in
 * localStorage so it survives reloads and never needs a rebuild when the
 * network changes. Returns the host plus the direct MJPEG stream URL.
 */
export function useEsp32Ip() {
  const [ip, setIpState] = useState(() => {
    try {
      return localStorage.getItem(STORAGE_KEY) || ESP32_CAM_DEFAULT_IP
    } catch {
      return ESP32_CAM_DEFAULT_IP
    }
  })

  const setIp = useCallback((next) => {
    const trimmed = next.trim()
    setIpState(trimmed)
    try {
      localStorage.setItem(STORAGE_KEY, trimmed)
    } catch {
      // localStorage unavailable (private mode) — keep in-memory only
    }
  }, [])

  return { ip, setIp, streamUrl: `http://${ip}/stream` }
}
