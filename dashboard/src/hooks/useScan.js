import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to /scan (sensor_msgs/LaserScan).
 * Returns { scan } — the raw LaserScan message (ranges, angle_min, angle_max, angle_increment).
 */
export function useScan() {
  const { rosRef, connected } = useRosbridgeContext()
  const [scan, setScan] = useState(null)
  const subRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) {
      setScan(null)
      return
    }

    subRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan',
      throttle_rate: 100,
      queue_length: 1
    })

    subRef.current.subscribe((msg) => {
      setScan({
        ranges:          msg.ranges,
        angle_min:       msg.angle_min,
        angle_max:       msg.angle_max,
        angle_increment: msg.angle_increment,
        range_min:       msg.range_min,
        range_max:       msg.range_max
      })
    })

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
      setScan(null)
    }
  }, [connected, rosRef])

  return { scan }
}
