import React, { useEffect, useRef, useCallback, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { CMD_VEL } from '../config.js'

const PUBLISH_HZ = 10
const STOP_DELAY = 200 // ms after key release before stopping

export default function Teleop() {
  const { rosRef, connected } = useRosbridgeContext()
  const pubRef     = useRef(null)
  const keysRef    = useRef(new Set())
  const loopRef    = useRef(null)
  const stopTimer  = useRef(null)
  const [active, setActive] = useState({ fwd: false, bwd: false, lft: false, rgt: false })

  // Setup publisher
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

  // Button press handlers
  const btnDown = (lin, ang) => {
    if (stopTimer.current) { clearTimeout(stopTimer.current); stopTimer.current = null }
    startLoop()
    // Override keys simulation via direct immediate publish
    publish(lin, ang)
  }
  const btnUp = () => {
    stopLoop()
    stopRobot()
  }

  const Btn = ({ label, lin, ang, activeKey }) => (
    <button
      className={`teleop-btn${active[activeKey] ? ' pressed' : ''}`}
      onPointerDown={() => btnDown(lin, ang)}
      onPointerUp={btnUp}
      onPointerLeave={btnUp}
      disabled={!connected}
    >
      {label}
    </button>
  )

  return (
    <div className="card teleop-card">
      <div className="card-title">Teleop <span className="hint">(WASD / arrows)</span></div>
      <div className="teleop-grid">
        <div />
        <Btn label="▲" lin={CMD_VEL.LINEAR_SPEED}  ang={0}                    activeKey="fwd" />
        <div />
        <Btn label="◄" lin={0}                      ang={CMD_VEL.ANGULAR_SPEED} activeKey="lft" />
        <button className="teleop-btn stop-btn" onPointerDown={stopRobot} disabled={!connected}>■</button>
        <Btn label="►" lin={0}                      ang={-CMD_VEL.ANGULAR_SPEED} activeKey="rgt" />
        <div />
        <Btn label="▼" lin={-CMD_VEL.LINEAR_SPEED} ang={0}                    activeKey="bwd" />
        <div />
      </div>
    </div>
  )
}
