import { useEffect, useRef, useState, useCallback } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

const STALE_MS = 15000
const POLL_MS = 5000

function topicStatus(lastSeen) {
  if (!lastSeen) return 'waiting'
  return (Date.now() - lastSeen) < STALE_MS ? 'active' : 'stale'
}

export function useSystemStatus() {
  const { rosRef, connected } = useRosbridgeContext()
  const [state, setState] = useState({
    driver: 'off', radar: 'off', map: 'off', nav2: 'off'
  })

  const ts = useRef({ odom: 0, scan: 0 })
  const nodesAlive = useRef({ map: false, nav2: false })
  const subsRef = useRef([])
  const timerRef = useRef(null)

  const pollNodes = useCallback(() => {
    if (!rosRef.current || !connected) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/rosapi/nodes',
      serviceType: 'rosapi/Nodes'
    })
    svc.callService(new ROSLIB.ServiceRequest(), (result) => {
      const nodes = result.nodes || []
      nodesAlive.current.map = nodes.some(n => n.includes('map_server'))
      nodesAlive.current.nav2 = nodes.some(n => n.includes('bt_navigator'))
    }, () => {
      nodesAlive.current.map = false
      nodesAlive.current.nav2 = false
    })
  }, [rosRef, connected])

  const refresh = useCallback(() => {
    if (!connected) {
      setState({ driver: 'off', radar: 'off', map: 'off', nav2: 'off' })
      return
    }
    setState({
      driver: topicStatus(ts.current.odom),
      radar: topicStatus(ts.current.scan),
      map: nodesAlive.current.map ? 'active' : 'waiting',
      nav2: nodesAlive.current.nav2 ? 'active' : 'waiting',
    })
  }, [connected])

  useEffect(() => {
    if (!connected || !rosRef.current) {
      ts.current = { odom: 0, scan: 0 }
      nodesAlive.current = { map: false, nav2: false }
      setState({ driver: 'off', radar: 'off', map: 'off', nav2: 'off' })
      return
    }

    const ros = rosRef.current
    const subs = []

    const sub = (name, type, key, throttle) => {
      const t = new ROSLIB.Topic({
        ros, name, messageType: type,
        throttle_rate: throttle, queue_length: 1
      })
      t.subscribe(() => { ts.current[key] = Date.now() })
      subs.push(t)
    }

    sub('/odom', 'nav_msgs/Odometry', 'odom', 5000)
    sub('/scan', 'sensor_msgs/LaserScan', 'scan', 5000)

    subsRef.current = subs

    pollNodes()
    timerRef.current = setInterval(() => {
      pollNodes()
      refresh()
    }, POLL_MS)
    refresh()

    return () => {
      subs.forEach(s => { try { s.unsubscribe() } catch (_) {} })
      subsRef.current = []
      if (timerRef.current) clearInterval(timerRef.current)
    }
  }, [connected, rosRef, pollNodes, refresh])

  const allActive = connected &&
    state.driver === 'active' &&
    state.radar === 'active' &&
    state.map === 'active' &&
    state.nav2 === 'active'

  const summary = !connected ? 'offline'
    : allActive ? 'ready'
    : 'starting'

  return { ...state, summary, allActive }
}
