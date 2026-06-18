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
  drawRobot, drawScan, drawPath, drawGoal, drawInitialPose,
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

  // Nav goal state
  const [goalMode, setGoalMode] = useState(false)
  const [goalPt, setGoalPt] = useState(null)
  const [goalStatus, setGoalStatus] = useState(null)
  const goalModeRef = useRef(false)

  // Init pose state
  const [initPoseMode, setInitPoseMode] = useState(false)
  const [initPosePt, setInitPosePt] = useState(null)
  const initPoseModeRef  = useRef(false)
  const initPoseDragRef  = useRef(null)   // { startCx, startCy, wx, wy, yaw }
  const liveInitPoseRef  = useRef(null)   // { cx, cy, yaw } — canvas coords during drag

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

  // Toast on new map data
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
  useEffect(() => { initPoseModeRef.current = initPoseMode }, [initPoseMode])

  // Subscribe to /save_map_status and /nav_goal_status
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
      if (s === 'reached')    { setGoalStatus('sent'); setGoalPt(null) }
      if (s === 'error')      setGoalStatus('error')
      if (s === 'cancelled')  { setGoalStatus('cancelled'); setGoalPt(null); setTimeout(() => setGoalStatus(null), 2000) }
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

  // ── Send nav goal ────────────────────────────────────────────────────────
  const sendGoal = useCallback((wx, wy) => {
    const ros = rosRef.current
    if (!ros) return
    setGoalStatus('sending')
    const topic = new ROSLIB.Topic({
      ros,
      name: '/nav_goal',
      messageType: 'geometry_msgs/PoseStamped'
    })
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

  // ── Cancel nav goal ───────────────────────────────────────────────────────
  const cancelGoal = useCallback(() => {
    const ros = rosRef.current
    if (!ros) return
    const topic = new ROSLIB.Topic({
      ros,
      name: '/cancel_nav_goal',
      messageType: 'std_msgs/Empty'
    })
    topic.publish(new ROSLIB.Message({}))
  }, [rosRef])

  // ── Send initial pose to AMCL ────────────────────────────────────────────
  const sendInitPose = useCallback((wx, wy, yaw) => {
    const ros = rosRef.current
    if (!ros) return
    const qz = Math.sin(yaw / 2)
    const qw = Math.cos(yaw / 2)
    const topic = new ROSLIB.Topic({
      ros,
      name: '/initialpose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    })
    topic.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        pose: {
          position:    { x: wx, y: wy, z: 0 },
          orientation: { x: 0,  y: 0,  z: qz, w: qw }
        },
        covariance: [
          0.25, 0, 0, 0, 0, 0,
          0, 0.25, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0.07
        ]
      }
    }))
    setInitPosePt({ wx, wy, yaw })
    setInitPoseMode(false)
    toast('Initial pose set — AMCL localizing…', 'success', 3500)
  }, [rosRef, toast])

  // ── Canvas render loop ───────────────────────────────────────────────────
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

      ctx.imageSmoothingEnabled = false
      ctx.clearRect(0, 0, w, h)
      ctx.fillStyle = theme === 'light' ? '#e8edf4' : '#0a0f1e'
      ctx.fillRect(0, 0, w, h)

      const meta      = mapMetaRef.current
      const offscreen = offscreenRef.current

      if (meta && offscreen) {
        if (!centeredRef.current) {
          const fitScale = Math.min(
            (w * 0.85) / meta.width,
            (h * 0.85) / meta.height
          )
          scaleRef.current    = fitScale
          centeredRef.current = true
          autoCenter()
        }

        const s = scaleRef.current
        ctx.save()
        ctx.translate(panRef.current.x, panRef.current.y)
        ctx.drawImage(offscreen, 0, 0, meta.width * s, meta.height * s)
        drawGrid(ctx, meta, s, theme)
        drawPath(ctx, pathRef.current, meta, s)
        if (SCAN_OVERLAY) drawScan(ctx, scan, pose, meta, s)

        // Confirmed init pose marker
        if (initPosePt) {
          const { cx: icx, cy: icy } = worldToCanvas(initPosePt.wx, initPosePt.wy, meta, s)
          drawInitialPose(ctx, icx, icy, initPosePt.yaw)
        }
        // Live drag preview for init pose
        if (liveInitPoseRef.current) {
          const { cx: lcx, cy: lcy, yaw: lyaw } = liveInitPoseRef.current
          drawInitialPose(ctx, lcx, lcy, lyaw)
        }

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

      // Mode overlays
      if (goalModeRef.current) {
        ctx.fillStyle = 'rgba(251,191,36,0.06)'
        ctx.fillRect(0, 0, w, h)
        ctx.fillStyle = '#fbbf24'
        ctx.font = '15px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('Click on the map to set navigation goal', w / 2, h - 16)
      }
      if (initPoseModeRef.current) {
        ctx.fillStyle = 'rgba(34,197,94,0.06)'
        ctx.fillRect(0, 0, w, h)
        ctx.fillStyle = '#22c55e'
        ctx.font = '15px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('Click and drag on the map to set initial pose & orientation', w / 2, h - 16)
      }
    }
    rafId = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(rafId)
    /* eslint-disable-next-line react-hooks/exhaustive-deps */
  }, [mapMetaRef, offscreenRef, mapStatsRef, pose, scan, autoCenter, goalPt, initPosePt, theme, updateSeq])

  // ── Mouse handlers ───────────────────────────────────────────────────────

  const onWheel = useCallback((e) => {
    const rect = canvasRef.current?.getBoundingClientRect()
    if (!rect) return
    const sr = canvasRef.current.width / rect.width
    const mx = (e.clientX - rect.left) * sr
    const my = (e.clientY - rect.top) * sr

    const factor = e.deltaY < 0 ? 1.1 : 1 / 1.1
    const newScale = Math.min(40, Math.max(0.2, scaleRef.current * factor))
    const ratio = newScale / scaleRef.current

    panRef.current.x = mx - ratio * (mx - panRef.current.x)
    panRef.current.y = my - ratio * (my - panRef.current.y)
    scaleRef.current = newScale
  }, [])

  const onMouseDown = (e) => {
    if (goalModeRef.current) return
    if (initPoseModeRef.current) {
      const meta = mapMetaRef.current
      if (!meta || !canvasRef.current) return
      const rect = canvasRef.current.getBoundingClientRect()
      const sr   = canvasRef.current.width / rect.width
      const cx   = (e.clientX - rect.left) * sr
      const cy   = (e.clientY - rect.top)  * sr
      const { wx, wy } = canvasToWorld(cx, cy, meta, scaleRef.current, panRef.current)
      initPoseDragRef.current = { startCx: cx, startCy: cy, wx, wy, yaw: 0 }
      liveInitPoseRef.current = { cx: cx - panRef.current.x, cy: cy - panRef.current.y, yaw: 0 }
      return
    }
    dragging.current = true
    lastPt.current = { x: e.clientX, y: e.clientY }
  }

  const onMouseMove = (e) => {
    if (initPoseDragRef.current) {
      const rect = canvasRef.current.getBoundingClientRect()
      const sr   = canvasRef.current.width / rect.width
      const cx   = (e.clientX - rect.left) * sr
      const cy   = (e.clientY - rect.top)  * sr
      const { startCx, startCy, wx, wy } = initPoseDragRef.current
      const dx = cx - startCx
      const dy = cy - startCy
      const yaw = Math.atan2(-dy, dx)
      initPoseDragRef.current.yaw = yaw
      liveInitPoseRef.current = { cx: startCx - panRef.current.x, cy: startCy - panRef.current.y, yaw }
      return
    }
    if (!dragging.current) return
    panRef.current.x += e.clientX - lastPt.current.x
    panRef.current.y += e.clientY - lastPt.current.y
    lastPt.current = { x: e.clientX, y: e.clientY }
  }

  const onMouseUp = (e) => {
    if (initPoseDragRef.current) {
      const { wx, wy, yaw } = initPoseDragRef.current
      initPoseDragRef.current = null
      liveInitPoseRef.current = null
      sendInitPose(wx, wy, yaw)
      return
    }
    if (dragging.current) { dragging.current = false; return }
    const meta = mapMetaRef.current
    if (!meta) return
    const rect = canvasRef.current.getBoundingClientRect()
    const sr   = canvasRef.current.width / rect.width
    const cx   = (e.clientX - rect.left) * sr
    const cy   = (e.clientY - rect.top)  * sr
    const { wx, wy } = canvasToWorld(cx, cy, meta, scaleRef.current, panRef.current)
    if (goalModeRef.current) {
      setGoalPt({ cx: cx - panRef.current.x, cy: cy - panRef.current.y, wx, wy })
      sendGoal(wx, wy)
    }
  }

  const onMouseLeave = () => {
    dragging.current = false
    if (initPoseDragRef.current) {
      initPoseDragRef.current = null
      liveInitPoseRef.current = null
    }
  }

  // ── Touch handlers ───────────────────────────────────────────────────────

  const onTouchStart = (e) => {
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX
      const dy = e.touches[0].clientY - e.touches[1].clientY
      touchRef.current.lastDist = Math.hypot(dx, dy)
      touchRef.current.lastPt = null
    } else if (e.touches.length === 1) {
      if (goalModeRef.current) return
      if (initPoseModeRef.current) {
        const t = e.touches[0]
        const meta = mapMetaRef.current
        if (!meta || !canvasRef.current) return
        const rect = canvasRef.current.getBoundingClientRect()
        const sr   = canvasRef.current.width / rect.width
        const cx   = (t.clientX - rect.left) * sr
        const cy   = (t.clientY - rect.top)  * sr
        const { wx, wy } = canvasToWorld(cx, cy, meta, scaleRef.current, panRef.current)
        initPoseDragRef.current = { startCx: cx, startCy: cy, wx, wy, yaw: 0 }
        liveInitPoseRef.current = { cx: cx - panRef.current.x, cy: cy - panRef.current.y, yaw: 0 }
        return
      }
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
    } else if (e.touches.length === 1) {
      if (initPoseDragRef.current) {
        const t = e.touches[0]
        const rect = canvasRef.current.getBoundingClientRect()
        const sr   = canvasRef.current.width / rect.width
        const cx   = (t.clientX - rect.left) * sr
        const cy   = (t.clientY - rect.top)  * sr
        const { startCx, startCy } = initPoseDragRef.current
        const yaw = Math.atan2(-(cy - startCy), cx - startCx)
        initPoseDragRef.current.yaw = yaw
        liveInitPoseRef.current = { cx: startCx - panRef.current.x, cy: startCy - panRef.current.y, yaw }
        return
      }
      if (dragging.current && touchRef.current.lastPt) {
        const t = e.touches[0]
        panRef.current.x += t.clientX - touchRef.current.lastPt.x
        panRef.current.y += t.clientY - touchRef.current.lastPt.y
        touchRef.current.lastPt = { x: t.clientX, y: t.clientY }
      }
    }
  }

  const onTouchEnd = (e) => {
    if (initPoseDragRef.current) {
      const { wx, wy, yaw } = initPoseDragRef.current
      initPoseDragRef.current = null
      liveInitPoseRef.current = null
      sendInitPose(wx, wy, yaw)
      return
    }
    if (dragging.current && e.changedTouches.length === 1 && !touchRef.current.lastPt) {
      dragging.current = false
      return
    }
    if (goalModeRef.current && e.changedTouches.length === 1) {
      const t = e.changedTouches[0]
      const meta = mapMetaRef.current
      if (!meta || !canvasRef.current) return
      const rect = canvasRef.current.getBoundingClientRect()
      const sr   = canvasRef.current.width / rect.width
      const cx   = (t.clientX - rect.left) * sr
      const cy   = (t.clientY - rect.top)  * sr
      const { wx, wy } = canvasToWorld(cx, cy, meta, scaleRef.current, panRef.current)
      setGoalPt({ cx: cx - panRef.current.x, cy: cy - panRef.current.y, wx, wy })
      sendGoal(wx, wy)
    }
    dragging.current = false
    touchRef.current.lastPt = null
    touchRef.current.lastDist = 0
  }

  const recenter = () => { centeredRef.current = false; autoCenter() }

  const toggleInitPoseMode = () => {
    setInitPoseMode(m => !m)
    setGoalMode(false)
    setGoalStatus(null)
  }

  const toggleGoalMode = () => {
    setGoalMode(m => !m)
    setInitPoseMode(false)
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

  // ── Label helpers ────────────────────────────────────────────────────────
  const mapSaveLabel = mapSaveStatus === 'saving' ? 'Saving…' : mapSaveStatus === 'saved' ? 'Saved' : mapSaveStatus === 'error' ? 'Error' : 'Save Map'
  const mapSaveColor = mapSaveStatus === 'saved' ? '#34d399' : mapSaveStatus === 'error' ? '#f87171' : undefined
  const isNavigating = goalStatus === 'navigating' || goalStatus === 'sending'
  const goalBtnLabel = goalMode ? 'Cancel' : 'Nav Goal'
  const initBtnLabel = initPoseMode ? 'Cancel' : 'Set Pose'
  const statusColor  = goalStatus === 'sent' ? '#34d399' : goalStatus === 'error' ? '#f87171' : goalStatus === 'cancelled' ? '#fb923c' : '#fbbf24'
  const statusLabel  = goalStatus === 'sending' ? 'Sending…' : goalStatus === 'navigating' ? 'Navigating…' : goalStatus === 'sent' ? 'Goal Reached' : goalStatus === 'error' ? 'Nav2 Error' : goalStatus === 'cancelled' ? 'Cancelled' : null
  const activeMode   = initPoseMode ? 'init' : goalMode ? 'goal' : null

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

          {/* Set Pose — must be done before Nav Goal */}
          <button
            className={`map-btn${initPoseMode ? ' active' : ''}`}
            onClick={toggleInitPoseMode}
            disabled={!connected}
            style={initPoseMode ? { borderColor: '#22c55e', color: '#22c55e' } : undefined}
            title="Click and drag on the map to set AMCL initial pose"
          >
            {initBtnLabel}
          </button>

          {isNavigating ? (
            <button
              className="map-btn active"
              onClick={cancelGoal}
              style={{ borderColor: '#f87171', color: '#f87171' }}
              title="Cancel current navigation goal"
            >
              Stop Nav
            </button>
          ) : (
            <button
              className={`map-btn${goalMode ? ' active' : ''}`}
              onClick={toggleGoalMode}
              disabled={!connected}
              title="Click on the map to send a Nav2 goal"
            >
              {goalBtnLabel}
            </button>
          )}
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
      <div ref={wrapRef} className={`map-wrap${activeMode ? ` ${activeMode}-active` : ''}`}>
        <canvas
          ref={canvasRef}
          className={`map-canvas${activeMode === 'goal' ? ' goal-cursor' : activeMode === 'init' ? ' init-cursor' : ''}`}
          role="img"
          aria-label={
            initPoseMode ? 'Robot map — click and drag to set initial pose' :
            goalMode     ? 'Robot map — tap to set navigation goal' :
                           'Robot map showing position and SLAM data'
          }
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
