import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

export function useServo() {
  const { rosRef, connected } = useRosbridgeContext()
  const [angle, setAngle] = useState(0)
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
    subRef.current.subscribe((msg) => {
      setAngle(msg.data)
    })

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
    }
  }, [connected, rosRef])

  return { angle }
}
