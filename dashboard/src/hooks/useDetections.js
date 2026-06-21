import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

export function useDetections() {
  const { rosRef, connected } = useRosbridgeContext()
  const [obstacles, setObstacles] = useState([])
  const [signs, setSigns] = useState([])
  const obsRef = useRef(null)
  const sigRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) return

    obsRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/esibot/obstacles',
      messageType: 'std_msgs/String',
      throttle_rate: 250,
      queue_length: 1,
    })
    obsRef.current.subscribe((msg) => {
      try { setObstacles(JSON.parse(msg.data)) } catch { setObstacles([]) }
    })

    sigRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/esibot/signs',
      messageType: 'std_msgs/String',
      throttle_rate: 500,
      queue_length: 1,
    })
    sigRef.current.subscribe((msg) => {
      try { setSigns(JSON.parse(msg.data)) } catch { setSigns([]) }
    })

    return () => {
      obsRef.current?.unsubscribe()
      sigRef.current?.unsubscribe()
      obsRef.current = null
      sigRef.current = null
      setObstacles([])
      setSigns([])
    }
  }, [connected, rosRef])

  return { obstacles, signs }
}
