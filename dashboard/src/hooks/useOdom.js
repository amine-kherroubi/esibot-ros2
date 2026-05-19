import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to /odom (nav_msgs/Odometry).
 * Returns { pose, twist } — pose: {x, y, yaw}, twist: {linear, angular}
 */
export function useOdom() {
  const { rosRef, connected } = useRosbridgeContext()
  const [pose,  setPose]  = useState({ x: 0, y: 0, yaw: 0 })
  const [twist, setTwist] = useState({ linear: 0, angular: 0 })
  const subRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) return

    subRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/odom',
      messageType: 'nav_msgs/Odometry',
      throttle_rate: 100,
      queue_length: 1
    })

    subRef.current.subscribe((msg) => {
      const { position, orientation } = msg.pose.pose
      // Quaternion → yaw
      const { x: qx, y: qy, z: qz, w: qw } = orientation
      const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

      setPose({ x: position.x, y: position.y, yaw })
      setTwist({
        linear:  msg.twist.twist.linear.x,
        angular: msg.twist.twist.angular.z
      })
    })

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
    }
  }, [connected, rosRef])

  return { pose, twist }
}
