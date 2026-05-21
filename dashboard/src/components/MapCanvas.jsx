import React, { useEffect, useRef, useCallback, useState } from 'react'
import ROSLIB from 'roslib'
import { useMap }  from '../hooks/useMap'
import { useOdom } from '../hooks/useOdom'
import { useScan } from '../hooks/useScan'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { useTheme } from '../context/ThemeContext'
import { useToast } from './Toast'
import { SCAN_OVERLAY } from '../config.js'
import {
  worldToCanvas, canvasToWorld,
  drawRobot, drawScan, drawPath, drawGoal,
  drawGrid, drawScaleBar, drawMapInfo
} from '../utils/mapUtils'

const MAX_PATH_LEN = 500
const FPS = 30

export default function MapCanvas() {
  const canvasRef   = useRef(null)
  const wrapRef     = useRef(null)
  const { offscreenRef, mapMetaRef, mapStatsRef, lastUpdateTimeRef, updateSeq } = useMap()
  const { pose } = useOdom()
  const { scan } = useScan()
  const { rosRef, connected } = useRosbridgeContext()
  const { theme } = useTheme()
  const toast = useToast()

  const pathRef     = useRef([])
  const scaleRef    = useRef(2)
  const panRef      = useRef({ x: 0, y: 0 })
  const dragging    = useRef(false)
  const lastPt      = useRef({ x: 0, y: 0 })
  const centeredRef = useRef(false)
  const touchRef    = useRef({ lastDist: 0, lastPt: null })

  const [goalMode, setGoalMode] = useState(false)
  const [goalPt, setGoalPt] = useState(null)
  const [goalStatus, setGoalStatus] = useState(null)
  const goalModeRef = useRef(false)

  const [mapSaveStatus, setMapSaveStatus] = useState(null)

  // "Updated Xs ago" badge
  const [agoLabel, setAgoLabel] = useState(null)
  useEffect(() => {
    const id = setInterval(() => {
      const t = lastUpdateTimeRef.current
      if (!t) { setAgoLabel(null); return }
      const s = Math.round((Date.now() - t) / 1000)
      if (s < 5)       setAgoLabel('just now')
      else if (s < 60) setAgoLabel(`${s}s ago`)
      else             setAgoLabel(`${Math.round(s / 60)}m ago`)
    }, 1000)
    return () => clearInterval(id)
  }, [lastUpdateTimeRef])

  // Toast notification on new map data (debounced 5s, only when area grows)
  const lastNotifiedRef = useRef({ explored: 0, time: 0 })
  useEffect(() => {
    if (updateSeq === 0) return
    const now = Date.now()
    const { exploredCells, totalCells, widthM, heightM } = mapStatsRef.current
    const { explored: prev, time: prevTime } = lastNotifiedRef.current
    const delta = exploredCells - prev
    if (prevTime === 0 || (delta > 0 && now - prevTime > 5000)) {
      lastNotifiedRef.current = { explored: exploredCells, time: now }
      const pct = Math.round(exploredCells / Math.max(totalCells, 1) * 100)
      toast(
        `Map updated — ${pct}% explored (${widthM.toFixed(1)} × ${heightM.toFixed(1)} m)`,
        prevTime === 0 ? 'success' : 'info',
        2500
      )
    }
  }, [updateSeq, mapStatsRef, toast])

  useEffect(() => { goalModeRef.current = goalMode }, [goalMode])

  useEffect(() => {
    if (!rosRef.current) return
    const saveTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/save_map_status',
      messageType: 'std_msgs/String'
    })
    saveTopic.subscribe((msg) => setMapSaveStatus(msg.data))

    const goalStatusTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/nav_goal_status',
      messageType: 'std_msgs/String'
    })
    goalStatusTopic.subscribe((msg) => {
      const s = msg.data
      if (s === 'sending')    setGoalStatus('sending')
      if (s === 'navigating') setGoalStatus('navigating')
      if (s === 'reached')    setGoalStatus('sent')
      if (s === 'error')      setGoalStatus('error')
    })

    return () => { saveTopic.unsubscribe(); goalStatusTopic.unsubscribe() }
  }, [rosRef, connected])

  useEffect(() => {
    const obs = new ResizeObserver(() => {
      const canvas = canvasRef.current
      const wrap   = wrapRef.current
      if (!canvas || !wrap) return
      canvas.width  = wrap.clientWidth
      canvas.height = wrap.clientHeight
      centeredRef.current = false
    })
    if (wrapRef.current) obs.observe(wrapRef.current)
    return () => obs.disconnect()
  }, [])

  useEffect(() => {
    const path = pathRef.current
    const last = path[path.length - 1]
    if (!last || Math.hypot(pose.x - last.x, pose.y - last.y) > 0.05) {
      path.push({ x: pose.x, y: pose.y })
      if (path.length > MAX_PATH_LEN) path.shift()
    }
  }, [pose])

  const autoCenter = useCallback(() => {
    const canvas = canvasRef.current
    const meta   = mapMetaRef.current
    if (!canvas || !meta) return
    const s = scaleRef.current
    const { cx, cy } = worldToCanvas(pose.x, pose.y, meta, s)
    panRef.current = {
      x: canvas.width  / 2 - cx,
      y: canvas.height / 2 - cy
    }
  }, [mapMetaRef, pose])

  const sendGoal = useCallback((wx, wy) => {
    const ros = rosRef.current
    if (!ros) return
    setGoalStatus('sending')
    const topic = new ROSLIB.Topic({
      ros,
      name: '/nav_goal',
      messageType: 'geometry_msgs/PoseStamped'
    })
    // Compute heading from robot's current pose toward the goal
    const yaw = Math.atan2(wy - pose.y, wx - pose.x)
    const qz  = Math.sin(yaw / 2)
    const qw  = Math.cos(yaw / 2)
    topic.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        position:    { x: wx, y: wy, z: 0 },
        orientation: { x: 0,  y: 0,  z: qz, w: qw }
      }
    }))
    setGoalMode(false)
  }, [rosRef, pose])

  useEffect(() => {
    let rafId
    let lastFrame = 0
    const draw = (timestamp) => {
      rafId = requestAnimationFrame(draw)
      if (timestamp - lastFrame < 1000 / FPS) return
      lastFrame = timestamp

      const canvas = canvasRef.current
      if (!canvas) return
      const ctx = canvas.getContext('2d')
      const w = canvas.width
      const h = canvas.height

      // Bilinear smoothing at low zoom (zoomed out), crisp pixels when zoomed in
      ctx.imageSmoothingEnabled = scaleRef.current < 2
      ctx.imageSmoothingQuality = 'high'
      ctx.clearRect(0, 0, w, h)
      ctx.fillStyle = theme === 'light' ? '#e8edf4' : '#0a0f1e'
      ctx.fillRect(0, 0, w, h)

      const meta      = mapMetaRef.current
      const offscreen = offscreenRef.current

      if (meta && offscreen) {
        if (!centeredRef.current) {
          const fitScale = Math.min(
            (w * 0.9) / offscreen.width,
            (h * 0.9) / offscreen.height
          ) * 0.6
          scaleRef.current    = fitScale
          centeredRef.current = true
          autoCenter()
        }

        const s = scaleRef.current
        ctx.save()
        ctx.translate(panRef.current.x, panRef.current.y)
        ctx.drawImage(offscreen, 0, 0, offscreen.width * s, offscreen.height * s)
        drawGrid(ctx, meta, s, theme)
        drawPath(ctx, pathRef.current, meta, s)
        if (SCAN_OVERLAY) drawScan(ctx, scan, pose, meta, s)
        const { cx, cy } = worldToCanvas(pose.x, pose.y, meta, s)
        drawRobot(ctx, cx, cy, pose.yaw, s)
        if (goalPt) {
          const { cx: gcx, cy: gcy } = worldToCanvas(goalPt.wx, goalPt.wy, meta, s)
          drawGoal(ctx, gcx, gcy)
        }
        ctx.restore()
        drawScaleBar(ctx, meta, s, w, h, theme)
        drawMapInfo(ctx, mapStatsRef.current, h, theme)
      } else {
        const cx = w / 2 + panRef.current.x
        const cy = h / 2 + panRef.current.y
        ctx.save()
        if (scan) {
          ctx.strokeStyle = 'rgba(239,68,68,0.5)'
          ctx.lineWidth = 1
          const pxPerMeter = 40
          for (let i = 0; i < scan.ranges.length; i++) {
            const r = scan.ranges[i]
            if (!isFinite(r) || r <= 0 || r >= scan.range_max) continue
            const angle = scan.angle_min + i * scan.angle_increment + pose.yaw
            ctx.beginPath()
            ctx.moveTo(cx, cy)
            ctx.lineTo(cx + r * pxPerMeter * Math.cos(angle), cy - r * pxPerMeter * Math.sin(angle))
            ctx.stroke()
          }
        }
        drawRobot(ctx, cx, cy, pose.yaw, 20)
        ctx.restore()
        const dots = '.'.repeat(Math.floor((Date.now() / 500) % 4))
        ctx.fillStyle = '#64748b'
        ctx.font = '14px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('Waiting for map data' + dots, w / 2, 24)
      }

      if (goalModeRef.current) {
        ctx.fillStyle = 'rgba(251,191,36,0.06)'
        ctx.fillRect(0, 0, w, h)
        ctx.fillStyle = '#fbbf24'
        ctx.font = '15px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('Click on the map to set navigation goal', w / 2, h - 16)
      }
    }
    rafId = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(rafId)
    /* eslint-disable-next-line react-hooks/exhaustive-deps */
  }, [mapMetaRef, offscreenRef, mapStatsRef, pose, scan, autoCenter, goalPt, theme, updateSeq])

  const onWheel = useCallback((e) => {
    const factor = e.deltaY < 0 ? 1.15 : 0.87
    scaleRef.current = Math.min(20, Math.max(0.2, scaleRef.current * factor))
  }, [])

  const onMouseDown = (e) => {
    if (goalModeRef.current) return
    dragging.current = true
    lastPt.current = { x: e.clientX, y: e.clientY }
  }

  const onMouseMove = (e) => {
    if (!dragging.current) return
    panRef.current.x += e.clientX - lastPt.current.x
    panRef.current.y += e.clientY - lastPt.current.y
    lastPt.current = { x: e.clientX, y: e.clientY }
  }

  const onMouseUp = (e) => {
    if (dragging.current) { dragging.current = false; return }
    const meta = mapMetaRef.current
    if (!meta) return
    const rect = canvasRef.current.getBoundingClientRect()
    const scaleRatio = canvasRef.current.width / rect.width
    const cx = (e.clientX - rect.left) * scaleRatio
    const cy = (e.clientY - rect.top)  * scaleRatio
    const { wx, wy } = canvasToWorld(cx, cy, meta, scaleRef.current, panRef.current)
    if (goalModeRef.current) {
      setGoalPt({ cx: cx - panRef.current.x, cy: cy - panRef.current.y, wx, wy })
      sendGoal(wx, wy)
    }
  }

  const onMouseLeave = () => { dragging.current = false }

  const onTouchStart = (e) => {
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX
      const dy = e.touches[0].clientY - e.touches[1].clientY
      touchRef.current.lastDist = Math.hypot(dx, dy)
      touchRef.current.lastPt = null
    } else if (e.touches.length === 1) {
      if (goalModeRef.current) return
      touchRef.current.lastPt = { x: e.touches[0].clientX, y: e.touches[0].clientY }
      dragging.current = true
    }
  }

  const onTouchMove = (e) => {
    e.preventDefault()
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX
      const dy = e.touches[0].clientY - e.touches[1].clientY
      const dist = Math.hypot(dx, dy)
      if (touchRef.current.lastDist) {
        const factor = dist / touchRef.current.lastDist
        scaleRef.current = Math.min(20, Math.max(0.2, scaleRef.current * factor))
      }
      touchRef.current.lastDist = dist
    } else if (e.touches.length === 1 && dragging.current && touchRef.current.lastPt) {
      const t = e.touches[0]
      panRef.current.x += t.clientX - touchRef.current.lastPt.x
      panRef.current.y += t.clientY - touchRef.current.lastPt.y
      touchRef.current.lastPt = { x: t.clientX, y: t.clientY }
    }
  }

  const onTouchEnd = (e) => {
    if (dragging.current && e.changedTouches.length === 1 && !touchRef.current.lastPt) {
      dragging.current = false
      return
    }
    if (goalModeRef.current && e.changedTouches.length === 1) {
      const t = e.changedTouches[0]
      const meta = mapMetaRef.current
      if (!meta || !canvasRef.current) return
      const rect = canvasRef.current.getBoundingClientRect()
      const scaleRatio = canvasRef.current.width / rect.width
      const cx = (t.clientX - rect.left) * scaleRatio
      const cy = (t.clientY - rect.top) * scaleRatio
      const { wx, wy } = canvasToWorld(cx, cy, meta, scaleRef.current, panRef.current)
      setGoalPt({ cx: cx - panRef.current.x, cy: cy - panRef.current.y, wx, wy })
      sendGoal(wx, wy)
    }
    dragging.current = false
    touchRef.current.lastPt = null
    touchRef.current.lastDist = 0
  }
  const recenter = () => { centeredRef.current = false; autoCenter() }

  const toggleGoalMode = () => {
    setGoalMode(m => !m)
    setGoalStatus(null)
  }

  const saveMap = useCallback(() => {
    if (!rosRef.current) return
    setMapSaveStatus('saving')
    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/save_map',
      messageType: 'std_msgs/Empty'
    })
    topic.publish(new ROSLIB.Message({}))
  }, [rosRef])

  const mapSaveLabel = mapSaveStatus === 'saving' ? 'Saving...' : mapSaveStatus === 'saved' ? 'Saved' : mapSaveStatus === 'error' ? 'Error' : 'Save Map'
  const mapSaveColor = mapSaveStatus === 'saved' ? '#34d399' : mapSaveStatus === 'error' ? '#f87171' : undefined
  const goalBtnLabel = goalMode ? 'Cancel' : 'Nav Goal'
  const statusColor = goalStatus === 'sent' ? '#34d399' : goalStatus === 'error' ? '#f87171' : '#fbbf24'
  const statusLabel = goalStatus === 'sending' ? 'Sending...' : goalStatus === 'navigating' ? 'Navigating...' : goalStatus === 'sent' ? 'Goal Reached' : goalStatus === 'error' ? 'Nav2 Error' : null

  return (
    <div className="card map-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <polygon points="3 6 9 3 15 6 21 3 21 18 15 21 9 18 3 21"/><line x1="9" x2="9" y1="3" y2="18"/><line x1="15" x2="15" y1="6" y2="21"/>
          </svg>
          <h2 className="card-heading">Map</h2>
          {agoLabel && (
            <span className={`map-update-badge${agoLabel === 'just now' ? ' fresh' : ''}`}>
              {agoLabel}
            </span>
          )}
        </span>
        <div className="map-controls">
          {statusLabel && (
            <span className="goal-status" style={{ color: statusColor }} aria-live="polite">{statusLabel}</span>
          )}
          <button
            className={`map-btn${goalMode ? ' active' : ''}`}
            onClick={toggleGoalMode}
            disabled={!connected}
            title="Click on the map to send a Nav2 goal"
          >
            {goalBtnLabel}
          </button>
          <button
            className="map-btn"
            onClick={saveMap}
            disabled={!connected}
            style={mapSaveColor ? { color: mapSaveColor, borderColor: mapSaveColor } : undefined}
            title="Save SLAM map"
          >
            {mapSaveLabel}
          </button>
          <button className="map-btn" onClick={recenter} title="Recenter" aria-label="Recenter map">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
              <circle cx="12" cy="12" r="3"/><path d="M12 2v4"/><path d="M12 18v4"/><path d="M2 12h4"/><path d="M18 12h4"/>
            </svg>
          </button>
        </div>
      </div>
      <div ref={wrapRef} className={`map-wrap${goalMode ? ' goal-active' : ''}`}>
        <canvas
          ref={canvasRef}
          className={`map-canvas${goalMode ? ' goal-cursor' : ''}`}
          role="img"
          aria-label={goalMode ? 'Robot map — tap to set navigation goal' : 'Robot map showing position and SLAM data'}
          tabIndex={0}
          onWheel={onWheel}
          onMouseDown={onMouseDown}
          onMouseMove={onMouseMove}
          onMouseUp={onMouseUp}
          onMouseLeave={onMouseLeave}
          onTouchStart={onTouchStart}
          onTouchMove={onTouchMove}
          onTouchEnd={onTouchEnd}
        >
          Robot map view — requires canvas support
        </canvas>
      </div>
    </div>
  )
}
