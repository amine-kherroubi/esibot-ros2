import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to /esibot/servo_angle (lecture seule).
 * Publie le radar_node — angle courant du sweep HC-SR04.
 */
export function useServo() {
  const { rosRef, connected } = useRosbridgeContext()
  const [angle, setAngle] = useState(90)
  const subRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) return

    subRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/servo_angle',
      messageType: 'std_msgs/Float32',
      throttle_rate: 200,
      queue_length: 1
    })
    subRef.current.subscribe((msg) => setAngle(msg.data))

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
    }
  }, [connected, rosRef])

  return { angle }
}
