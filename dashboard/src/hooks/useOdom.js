import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

function quaternionToYaw(qx, qy, qz, qw) {
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
}

function composeTransforms(a, b) {
  const cos = Math.cos(a.yaw)
  const sin = Math.sin(a.yaw)
  return {
    x:   a.x + cos * b.x - sin * b.y,
    y:   a.y + sin * b.x + cos * b.y,
    yaw: a.yaw + b.yaw,
  }
}

/**
 * Robot pose in the MAP frame, derived entirely from /tf.
 * Extracts map→odom and odom→base_footprint from the same TF message
 * to avoid the race condition of composing across two independent subscriptions.
 * /odom is subscribed separately for twist (velocity display) only.
 */
export function useOdom() {
  const { rosRef, connected } = useRosbridgeContext()
  const [pose,  setPose]  = useState({ x: 0, y: 0, yaw: 0 })
  const [twist, setTwist] = useState({ linear: 0, angular: 0 })

  const tfBufferRef = useRef({
    mapToOdom:     { x: 0, y: 0, yaw: 0 },
    odomToBase:    { x: 0, y: 0, yaw: 0 },
    hasMapToOdom:  false,
    hasOdomToBase: false,
  })

  const tfSubRef   = useRef(null)
  const odomSubRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) return

    // Subscribe to /tf and extract both map→odom and odom→base_footprint atomically
    tfSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/tf',
      messageType: 'tf2_msgs/TFMessage',
      throttle_rate: 50,
      queue_length: 1,
    })

    tfSubRef.current.subscribe((msg) => {
      const buf = tfBufferRef.current
      let updated = false

      for (const t of msg.transforms) {
        const { x: qx, y: qy, z: qz, w: qw } = t.transform.rotation
        const yaw = quaternionToYaw(qx, qy, qz, qw)
        const tx = t.transform.translation.x
        const ty = t.transform.translation.y

        if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') {
          buf.mapToOdom    = { x: tx, y: ty, yaw }
          buf.hasMapToOdom = true
          updated = true
        } else if (t.header.frame_id === 'odom' && t.child_frame_id === 'base_footprint') {
          buf.odomToBase    = { x: tx, y: ty, yaw }
          buf.hasOdomToBase = true
          updated = true
        }
      }

      if (updated && buf.hasOdomToBase) {
        setPose(
          buf.hasMapToOdom
            ? composeTransforms(buf.mapToOdom, buf.odomToBase)
            : { x: buf.odomToBase.x, y: buf.odomToBase.y, yaw: buf.odomToBase.yaw }
        )
      }
    })

    // /odom used only for twist (linear/angular velocity display)
    odomSubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/odom',
      messageType: 'nav_msgs/Odometry',
      throttle_rate: 200,
      queue_length: 1,
    })

    odomSubRef.current.subscribe((msg) => {
      setTwist({
        linear:  msg.twist.twist.linear.x,
        angular: msg.twist.twist.angular.z,
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
