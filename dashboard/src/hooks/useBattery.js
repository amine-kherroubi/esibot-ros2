import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'
import { BATTERY_CAPACITY_MINUTES } from '../config.js'

/**
 * Subscribes to /battery_state (sensor_msgs/BatteryState).
 * Falls back to /diagnostics if BatteryState unavailable.
 * Returns { voltage, percentage, estimatedMinutes, charging }
 */
export function useBattery() {
  const { rosRef, connected } = useRosbridgeContext()
  const [voltage,           setVoltage]           = useState(null)
  const [percentage,        setPercentage]        = useState(null)
  const [estimatedMinutes,  setEstimatedMinutes]  = useState(null)
  const [charging,          setCharging]          = useState(false)
  const subRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) return

    subRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/battery_state',
      messageType: 'sensor_msgs/BatteryState',
      throttle_rate: 2000,
      queue_length: 1
    })

    subRef.current.subscribe((msg) => {
      const v = msg.voltage

      // Pourcentage depuis ROS si vraiment renseigné (> 0),
      // sinon calculé depuis voltage (LiPo 3S : 12.6V=100%, 10.5V=0%)
      let pct = msg.percentage > 0
        ? msg.percentage
        : v > 0
          ? Math.min(1, Math.max(0, (v - 10.5) / (12.6 - 10.5)))
          : null

      setVoltage(v)
      setPercentage(pct)
      setCharging(msg.power_supply_status === 1)
      if (pct !== null) {
        setEstimatedMinutes(Math.round(pct * BATTERY_CAPACITY_MINUTES))
      }
    })

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
    }
  }, [connected, rosRef])

  return { voltage, percentage, estimatedMinutes, charging }
}
