import { useEffect, useRef, useCallback, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

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
    const S = 4

    if (!offscreenRef.current ||
        offscreenRef.current.width  !== width * S ||
        offscreenRef.current.height !== height * S) {
      offscreenRef.current = new OffscreenCanvas(width * S, height * S)
    }

    mapMetaRef.current = { origin, resolution, width, height }

    const ctx = offscreenRef.current.getContext('2d')
    const imgData = ctx.createImageData(width * S, height * S)
    const pixels = imgData.data
    const stride = width * S

    let exploredCells = 0
    for (let i = 0; i < data.length; i++) {
      const v = data[i]
      let r, g, b
      if (v === -1) {
        r = 128; g = 128; b = 128
      } else if (v === 0) {
        r = 255; g = 255; b = 255
        exploredCells++
      } else {
        const t = Math.min(v / 100, 1)
        const shade = Math.round((1 - t) * 80)
        r = shade; g = shade; b = shade
        exploredCells++
      }
      const row = Math.floor(i / width)
      const col = i % width
      const flippedRow = height - 1 - row
      for (let dy = 0; dy < S; dy++) {
        for (let dx = 0; dx < S; dx++) {
          const dst = (flippedRow * S + dy) * stride + (col * S + dx)
          pixels[dst * 4]     = r
          pixels[dst * 4 + 1] = g
          pixels[dst * 4 + 2] = b
          pixels[dst * 4 + 3] = 255
        }
      }
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

    let gotMap = false
    let serviceTimer = null

    subRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      throttle_rate: 2000,
      queue_length: 1
    })
    subRef.current.subscribe((msg) => {
      gotMap = true
      handleMap(msg)
    })

    // Fallback: if no map via topic after 5s, try the service once (nav mode)
    serviceTimer = setTimeout(() => {
      if (gotMap || !rosRef.current) return
      const client = new ROSLIB.Service({
        ros: rosRef.current,
        name: '/map_server/map',
        serviceType: 'nav_msgs/GetMap'
      })
      client.callService(new ROSLIB.ServiceRequest({}), (resp) => {
        if (resp?.map && !gotMap) {
          gotMap = true
          handleMap(resp.map)
        }
      }, () => {})
    }, 5000)

    return () => {
      if (serviceTimer) clearTimeout(serviceTimer)
      subRef.current?.unsubscribe()
      subRef.current = null
    }
  }, [connected, handleMap, rosRef])

  return { offscreenRef, mapMetaRef, mapStatsRef, lastUpdateTimeRef, updateSeq }
}
