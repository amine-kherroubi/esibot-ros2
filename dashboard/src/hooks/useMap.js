import { useEffect, useRef, useCallback } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to /map (nav_msgs/OccupancyGrid) and draws it onto an offscreen canvas.
 * Returns { offscreenRef, mapMetaRef } for use in MapCanvas.
 */
export function useMap() {
  const { rosRef, connected } = useRosbridgeContext()
  const offscreenRef = useRef(null)   // OffscreenCanvas or null
  const mapMetaRef  = useRef(null)   // { origin, resolution, width, height }
  const subRef      = useRef(null)

  const handleMap = useCallback((msg) => {
    const { info, data } = msg
    const { width, height, resolution, origin } = info

    if (!offscreenRef.current ||
        offscreenRef.current.width  !== width ||
        offscreenRef.current.height !== height) {
      offscreenRef.current = new OffscreenCanvas(width, height)
    }

    mapMetaRef.current = { origin, resolution, width, height }

    const ctx = offscreenRef.current.getContext('2d')
    const imgData = ctx.createImageData(width, height)
    const pixels = imgData.data

    for (let i = 0; i < data.length; i++) {
      const v = data[i]
      let r, g, b
      if (v === -1) {
        // Unknown — gray
        r = 128; g = 128; b = 128
      } else if (v === 0) {
        // Free — white
        r = 255; g = 255; b = 255
      } else {
        // Occupied — scale from gray to black
        const t = 1 - v / 100
        r = Math.round(t * 120)
        g = Math.round(t * 120)
        b = Math.round(t * 120)
      }
      // ROS OccupancyGrid: row 0 = world y_min (south). putImageData places row 0 at
      // canvas top. Flip rows so world south → canvas bottom (north at top, right-side up).
      const row = Math.floor(i / width)
      const col = i % width
      const dst = (height - 1 - row) * width + col
      pixels[dst * 4 + 0] = r
      pixels[dst * 4 + 1] = g
      pixels[dst * 4 + 2] = b
      pixels[dst * 4 + 3] = 255
    }
    ctx.putImageData(imgData, 0, 0)
  }, [])

  useEffect(() => {
    if (!connected || !rosRef.current) return

    subRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      throttle_rate: 2000,
      queue_length: 1
    })
    subRef.current.subscribe(handleMap)

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
    }
  }, [connected, handleMap, rosRef])

  return { offscreenRef, mapMetaRef }
}
