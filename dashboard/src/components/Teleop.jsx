import React, { useEffect, useRef, useCallback, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { CMD_VEL } from '../config.js'

const PUBLISH_HZ = 10
const STOP_DELAY = 200

const ArrowUp = () => (
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    <path d="m18 15-6-6-6 6"/>
  </svg>
)

const ArrowDown = () => (
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    <path d="m6 9 6 6 6-6"/>
  </svg>
)

const ArrowLeft = () => (
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    <path d="m15 18-6-6 6-6"/>
  </svg>
)

const ArrowRight = () => (
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    <path d="m9 18 6-6-6-6"/>
  </svg>
)

const StopIcon = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
    <rect x="4" y="4" width="16" height="16" rx="2"/>
  </svg>
)

function TeleopBtn({ icon, keyHint, active, onDown, onUp, disabled, label }) {
  return (
    <button
      className={`teleop-btn${active ? ' pressed' : ''}`}
      onPointerDown={onDown}
      onPointerUp={onUp}
      onPointerLeave={onUp}
      disabled={disabled}
      aria-label={label}
    >
      {icon}
      <span className="teleop-key-hint" aria-hidden="true">{keyHint}</span>
    </button>
  )
}

export default function Teleop() {
  const { rosRef, connected } = useRosbridgeContext()
  const pubRef     = useRef(null)
  const keysRef    = useRef(new Set())
  const loopRef    = useRef(null)
  const stopTimer  = useRef(null)
  const [active, setActive] = useState({ fwd: false, bwd: false, lft: false, rgt: false })
  const [velocity, setVelocity] = useState({ lin: 0, ang: 0 })

  useEffect(() => {
    if (!connected || !rosRef.current) {
      pubRef.current = null
      return
    }
    pubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    })
  }, [connected, rosRef])

  const publish = useCallback((linear, angular) => {
    if (!pubRef.current) return
    pubRef.current.publish(new ROSLIB.Message({
      linear:  { x: linear,  y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    }))
    setVelocity({ lin: linear, ang: angular })
  }, [])

  const stopRobot = useCallback(() => {
    publish(0, 0)
  }, [publish])

  const startLoop = useCallback(() => {
    if (loopRef.current) return
    loopRef.current = setInterval(() => {
      const keys = keysRef.current
      let lin = 0, ang = 0
      if (keys.has('ArrowUp')    || keys.has('w')) lin =  CMD_VEL.LINEAR_SPEED
      if (keys.has('ArrowDown')  || keys.has('s')) lin = -CMD_VEL.LINEAR_SPEED
      if (keys.has('ArrowLeft')  || keys.has('a')) ang =  CMD_VEL.ANGULAR_SPEED
      if (keys.has('ArrowRight') || keys.has('d')) ang = -CMD_VEL.ANGULAR_SPEED
      publish(lin, ang)
      setActive({
        fwd: lin > 0,
        bwd: lin < 0,
        lft: ang > 0,
        rgt: ang < 0
      })
    }, 1000 / PUBLISH_HZ)
  }, [publish])

  const stopLoop = useCallback(() => {
    if (loopRef.current) {
      clearInterval(loopRef.current)
      loopRef.current = null
    }
    setActive({ fwd: false, bwd: false, lft: false, rgt: false })
    setVelocity({ lin: 0, ang: 0 })
  }, [])

  useEffect(() => {
    const onKeyDown = (e) => {
      const dirs = ['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d']
      if (!dirs.includes(e.key)) return
      e.preventDefault()
      if (stopTimer.current) { clearTimeout(stopTimer.current); stopTimer.current = null }
      keysRef.current.add(e.key)
      startLoop()
    }
    const onKeyUp = (e) => {
      keysRef.current.delete(e.key)
      if (keysRef.current.size === 0) {
        stopTimer.current = setTimeout(() => {
          stopLoop()
          stopRobot()
        }, STOP_DELAY)
      }
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup',   onKeyUp)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup',   onKeyUp)
      stopLoop()
    }
  }, [startLoop, stopLoop, stopRobot])

  const btnDown = (lin, ang) => {
    if (stopTimer.current) { clearTimeout(stopTimer.current); stopTimer.current = null }
    startLoop()
    publish(lin, ang)
  }
  const btnUp = () => {
    stopLoop()
    stopRobot()
  }

  const isMoving = velocity.lin !== 0 || velocity.ang !== 0

  return (
    <div className="card teleop-card">
      <div className="card-title">
        <span className="card-title-left">
          <svg className="card-title-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
            <circle cx="12" cy="12" r="10"/><polygon points="16.24 7.76 14.12 14.12 7.76 16.24 9.88 9.88 16.24 7.76"/>
          </svg>
          <h2 className="card-heading">Teleop</h2>
        </span>
        <span className="hint">WASD / Arrows</span>
      </div>
      <div className="teleop-grid">
        <div />
        <TeleopBtn icon={<ArrowUp />}    keyHint="W" active={active.fwd} onDown={() => btnDown(CMD_VEL.LINEAR_SPEED, 0)}                      onUp={btnUp} disabled={!connected} label="Move forward" />
        <div />
        <TeleopBtn icon={<ArrowLeft />}  keyHint="A" active={active.lft} onDown={() => btnDown(0, CMD_VEL.ANGULAR_SPEED)}                     onUp={btnUp} disabled={!connected} label="Turn left" />
        <button
          className="teleop-btn stop-btn"
          onPointerDown={stopRobot}
          disabled={!connected}
          aria-label="Emergency stop"
        >
          <StopIcon />
        </button>
        <TeleopBtn icon={<ArrowRight />} keyHint="D" active={active.rgt} onDown={() => btnDown(0, -CMD_VEL.ANGULAR_SPEED)}                    onUp={btnUp} disabled={!connected} label="Turn right" />
        <div />
        <TeleopBtn icon={<ArrowDown />}  keyHint="S" active={active.bwd} onDown={() => btnDown(-CMD_VEL.LINEAR_SPEED, 0)}                     onUp={btnUp} disabled={!connected} label="Move backward" />
        <div />
      </div>
      <div className={`teleop-speed${isMoving ? ' active' : ''}`} aria-live="polite" aria-label={`Linear ${velocity.lin.toFixed(1)} meters per second, angular ${velocity.ang.toFixed(1)} radians per second`}>
        <span className="speed-label">Lin</span>
        <span className="speed-value">{velocity.lin.toFixed(1)}</span>
        <span className="speed-unit">m/s</span>
        <span className="speed-sep" />
        <span className="speed-label">Ang</span>
        <span className="speed-value">{velocity.ang.toFixed(1)}</span>
        <span className="speed-unit">rad/s</span>
      </div>
    </div>
  )
}
