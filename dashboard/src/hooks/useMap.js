import { useEffect, useRef, useCallback, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to /map (nav_msgs/OccupancyGrid) and draws it onto an offscreen canvas.
 * Returns { offscreenRef, mapMetaRef, mapStatsRef, lastUpdateTimeRef, updateSeq }.
 */
export function useMap() {
  const { rosRef, connected } = useRosbridgeContext()
  const offscreenRef       = useRef(null)
  const mapMetaRef         = useRef(null)
  const mapStatsRef        = useRef({ exploredCells: 0, totalCells: 0, widthM: 0, heightM: 0 })
  const lastUpdateTimeRef  = useRef(null)
  const subRef             = useRef(null)
  const [updateSeq, setUpdateSeq] = useState(0)

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

    let exploredCells = 0
    for (let i = 0; i < data.length; i++) {
      const v = data[i]
      let r, g, b
      if (v === -1) {
        r = 100; g = 108; b = 120  // unknown — blue-gray
      } else if (v === 0) {
        r = 240; g = 244; b = 250  // free — off-white
        exploredCells++
      } else {
        // occupied — near-black, stronger at higher probability
        const t = v / 100
        r = Math.round((1 - t) * 60)
        g = Math.round((1 - t) * 60)
        b = Math.round((1 - t) * 70)
        exploredCells++
      }
      const row = Math.floor(i / width)
      const col = i % width
      const dst = (height - 1 - row) * width + col
      pixels[dst * 4 + 0] = r
      pixels[dst * 4 + 1] = g
      pixels[dst * 4 + 2] = b
      pixels[dst * 4 + 3] = 255
    }
    ctx.putImageData(imgData, 0, 0)

    mapStatsRef.current = {
      exploredCells,
      totalCells: data.length,
      widthM: width * resolution,
      heightM: height * resolution,
    }
    lastUpdateTimeRef.current = Date.now()
    setUpdateSeq(s => s + 1)
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

  return { offscreenRef, mapMetaRef, mapStatsRef, lastUpdateTimeRef, updateSeq }
}
