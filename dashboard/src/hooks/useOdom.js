import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Robot pose in the MAP frame (composed from /tf + /odom)
 * + twist from /odom for velocity display.
 */
export function useOdom() {
  const { rosRef, connected } = useRosbridgeContext()
  const [pose,  setPose]  = useState({ x: 0, y: 0, yaw: 0 })
  const [twist, setTwist] = useState({ linear: 0, angular: 0 })
  const mapToOdomRef = useRef({ x: 0, y: 0, yaw: 0 })
  const tfSubRef   = useRef(null)
  const odomSubRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) return

    // /tf: extract map→odom transform (from slam_toolbox or AMCL)
    tfSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/tf',
      messageType: 'tf2_msgs/TFMessage',
      throttle_rate: 50,
      queue_length: 1
    })

    tfSubRef.current.subscribe((msg) => {
      for (const t of msg.transforms) {
        if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') {
          const { x: qx, y: qy, z: qz, w: qw } = t.transform.rotation
          const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
          mapToOdomRef.current = { x: t.transform.translation.x, y: t.transform.translation.y, yaw }
        }
      }
    })

    // /odom: odom→base_footprint position + twist
    odomSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/odom',
      messageType: 'nav_msgs/Odometry',
      throttle_rate: 100,
      queue_length: 1
    })

    odomSubRef.current.subscribe((msg) => {
      const { position, orientation } = msg.pose.pose
      const { x: qx, y: qy, z: qz, w: qw } = orientation
      const odomYaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

      // Compose: map_pose = map_to_odom * odom_to_base
      const m = mapToOdomRef.current
      const cos = Math.cos(m.yaw)
      const sin = Math.sin(m.yaw)
      setPose({
        x:   m.x + cos * position.x - sin * position.y,
        y:   m.y + sin * position.x + cos * position.y,
        yaw: m.yaw + odomYaw
      })
      setTwist({
        linear:  msg.twist.twist.linear.x,
        angular: msg.twist.twist.angular.z
      })
    })

    return () => {
      tfSubRef.current?.unsubscribe()
      tfSubRef.current = null
      odomSubRef.current?.unsubscribe()
      odomSubRef.current = null
    }
  }, [connected, rosRef])

  return { pose, twist }
}
