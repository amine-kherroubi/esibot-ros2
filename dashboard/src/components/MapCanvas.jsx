import React, { useEffect, useRef, useCallback, useState } from 'react'
import ROSLIB from 'roslib'
import { useMap }  from '../hooks/useMap'
import { useOdom } from '../hooks/useOdom'
import { useScan } from '../hooks/useScan'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { SCAN_OVERLAY } from '../config.js'
import {
  worldToCanvas, canvasToWorld,
  drawRobot, drawScan, drawPath, drawGoal, drawInitialPose
} from '../utils/mapUtils'

const MAX_PATH_LEN = 500
const FPS = 30

export default function MapCanvas() {
  const canvasRef   = useRef(null)
  const wrapRef     = useRef(null)
  const { offscreenRef, mapMetaRef } = useMap()
  const { pose } = useOdom()
  const { scan } = useScan()
  const { rosRef, connected } = useRosbridgeContext()

  const pathRef     = useRef([])
  const scaleRef    = useRef(2)
  const panRef      = useRef({ x: 0, y: 0 })
  const dragging    = useRef(false)
  const lastPt      = useRef({ x: 0, y: 0 })
  const centeredRef = useRef(false)

  const [goalMode,   setGoalMode]   = useState(false)
  const [goalPt,     setGoalPt]     = useState(null)
  const [goalStatus, setGoalStatus] = useState(null)
  const goalModeRef = useRef(false)

  const [poseMode,   setPoseMode]   = useState(false)
  const [posePt,     setPosePt]     = useState(null)
  const [poseStatus, setPoseStatus] = useState(null)
  const poseModeRef = useRef(false)

  const [mapSaveStatus, setMapSaveStatus] = useState(null)

  useEffect(() => { goalModeRef.current = goalMode }, [goalMode])
  useEffect(() => { poseModeRef.current = poseMode }, [poseMode])

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
      if (s === 'reached')    setGoalStatus('sent')
      if (s === 'error')      setGoalStatus('error')
    })

    return () => { saveTopic.unsubscribe(); goalStatusTopic.unsubscribe() }
  }, [rosRef, connected])

  // Match canvas resolution to display size
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

  // Accumulate path
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

  // Publier la pose initiale → /initialpose
  const sendInitialPose = useCallback((wx, wy) => {
    if (!rosRef.current) return
    const pub = new ROSLIB.Topic({
      ros:         rosRef.current,
      name:        '/initialpose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    })
    pub.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        pose: {
          position:    { x: wx, y: wy, z: 0 },
          orientation: { x: 0,  y: 0,  z: 0, w: 1 }
        },
        covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                     0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0.07]
      }
    }))
    setPoseStatus('sent')
    setPoseMode(false)
  }, [rosRef])

  // Send NavigateToPose goal via nav_goal_proxy node (PoseStamped on /nav_goal)
  const sendGoal = useCallback((wx, wy) => {
    const ros = rosRef.current
    if (!ros) return
    setGoalStatus('sending')

    const topic = new ROSLIB.Topic({
      ros,
      name: '/nav_goal',
      messageType: 'geometry_msgs/PoseStamped'
    })
    topic.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        position:    { x: wx, y: wy, z: 0 },
        orientation: { x: 0,  y: 0,  z: 0, w: 1 }
      }
    }))

    setGoalMode(false)
  }, [rosRef])

  // Render loop
  useEffect(() => {
    let timer
    const draw = () => {
      const canvas = canvasRef.current
      if (!canvas) return
      const ctx = canvas.getContext('2d')
      const w = canvas.width
      const h = canvas.height

      ctx.clearRect(0, 0, w, h)
      ctx.fillStyle = '#1e293b'
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
        drawPath(ctx, pathRef.current, meta, s)
        if (SCAN_OVERLAY) drawScan(ctx, scan, pose, meta, s)
        const { cx, cy } = worldToCanvas(pose.x, pose.y, meta, s)
        drawRobot(ctx, cx, cy, pose.yaw, s)
        if (goalPt) {
          const { cx: gcx, cy: gcy } = worldToCanvas(goalPt.wx, goalPt.wy, meta, s)
          drawGoal(ctx, gcx, gcy)
        }
        if (posePt) {
          const { cx: pcx, cy: pcy } = worldToCanvas(posePt.wx, posePt.wy, meta, s)
          drawInitialPose(ctx, pcx, pcy)
        }
        ctx.restore()
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
        ctx.fillStyle = '#475569'
        ctx.font = '12px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('En attente de la carte…', w / 2, 20)
      }

      // Goal mode cursor hint
      if (goalModeRef.current) {
        ctx.fillStyle = 'rgba(245,158,11,0.15)'
        ctx.fillRect(0, 0, w, h)
        ctx.fillStyle = '#f59e0b'
        ctx.font = '13px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('Cliquez sur la carte pour définir la destination', w / 2, h - 12)
      }

      // Pose mode cursor hint
      if (poseModeRef.current) {
        ctx.fillStyle = 'rgba(34,197,94,0.12)'
        ctx.fillRect(0, 0, w, h)
        ctx.fillStyle = '#22c55e'
        ctx.font = '13px Inter, sans-serif'
        ctx.textAlign = 'center'
        ctx.fillText('Cliquez sur la carte pour définir la pose initiale', w / 2, h - 12)
      }

      timer = setTimeout(draw, 1000 / FPS)
    }
    draw()
    return () => clearTimeout(timer)
  }, [mapMetaRef, offscreenRef, pose, scan, autoCenter, goalPt])

  // Zoom
  const onWheel = useCallback((e) => {
    e.preventDefault()
    const factor = e.deltaY < 0 ? 1.15 : 0.87
    scaleRef.current = Math.min(20, Math.max(0.2, scaleRef.current * factor))
  }, [])

  // Click → goal or pan start
  const onMouseDown = (e) => {
    if (goalModeRef.current || poseModeRef.current) return
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
    } else if (poseModeRef.current) {
      setPosePt({ cx: cx - panRef.current.x, cy: cy - panRef.current.y, wx, wy })
      sendInitialPose(wx, wy)
    }
  }

  const onMouseLeave = () => { dragging.current = false }

  const recenter = () => { centeredRef.current = false; autoCenter() }

  const toggleGoalMode = () => {
    setGoalMode(m => !m)
    setPoseMode(false)
    setGoalStatus(null)
  }

  const togglePoseMode = () => {
    setPoseMode(m => !m)
    setGoalMode(false)
    setPoseStatus(null)
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

  const mapSaveLabel = mapSaveStatus === 'saving' ? 'Saving…' : mapSaveStatus === 'saved' ? 'Saved ✓' : mapSaveStatus === 'error' ? 'Erreur' : '💾 Save Map'
  const mapSaveColor = mapSaveStatus === 'saved' ? '#22c55e' : mapSaveStatus === 'error' ? '#ef4444' : undefined

  const goalBtnLabel  = goalMode ? 'Annuler' : '⚑ Envoyer un goal'
  const poseBtnLabel  = poseMode ? 'Annuler' : '⊕ Pose initiale'
  const statusColor   = goalStatus === 'sent' ? '#22c55e' : goalStatus === 'error' ? '#ef4444' : '#f59e0b'
  const statusLabel   = goalStatus === 'sending' ? 'Envoi…' : goalStatus === 'navigating' ? 'Navigation…' : goalStatus === 'sent' ? 'Goal envoyé' : goalStatus === 'error' ? 'Erreur Nav2' : null
  const poseStatusColor = poseStatus === 'sent' ? '#22c55e' : '#f59e0b'
  const poseStatusLabel = poseStatus === 'sent' ? 'Pose envoyée' : null

  return (
    <div className="card map-card">
      <div className="card-title">
        Map
        <div className="map-controls">
          {poseStatusLabel && (
            <span className="goal-status" style={{ color: poseStatusColor }}>{poseStatusLabel}</span>
          )}
          {statusLabel && (
            <span className="goal-status" style={{ color: statusColor }}>{statusLabel}</span>
          )}
          <button
            className={`map-btn${poseMode ? ' active-green' : ''}`}
            onClick={togglePoseMode}
            disabled={!connected}
            title="Cliquer sur la carte pour définir la pose initiale AMCL"
          >
            {poseBtnLabel}
          </button>
          <button
            className={`map-btn${goalMode ? ' active' : ''}`}
            onClick={toggleGoalMode}
            disabled={!connected}
            title="Cliquer sur la carte pour envoyer un goal Nav2"
          >
            {goalBtnLabel}
          </button>
          <button
            className="map-btn"
            onClick={saveMap}
            disabled={!connected}
            style={mapSaveColor ? { color: mapSaveColor, borderColor: mapSaveColor } : undefined}
            title="Sauvegarder la carte SLAM"
          >
            {mapSaveLabel}
          </button>
          <button className="map-btn" onClick={recenter} title="Recentrer">⊙</button>
        </div>
      </div>
      <div ref={wrapRef} className="map-wrap">
        <canvas
          ref={canvasRef}
          className={`map-canvas${goalMode ? ' goal-cursor' : ''}`}
          onWheel={onWheel}
          onMouseDown={onMouseDown}
          onMouseMove={onMouseMove}
          onMouseUp={onMouseUp}
          onMouseLeave={onMouseLeave}
        />
      </div>
    </div>
  )
}
