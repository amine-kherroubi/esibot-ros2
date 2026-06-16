import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to a sensor_msgs/CompressedImage (JPEG) topic and exposes a
 * data-URL for an <img>. rosbridge delivers `data` already base64-encoded,
 * so there is zero per-pixel decoding in the browser — the feed stays fluid.
 */
export function useCamera(topic = '/camera/image_annotated/compressed') {
  const { rosRef, connected } = useRosbridgeContext()
  const [imgSrc, setImgSrc] = useState(null)
  const subRef = useRef(null)

  useEffect(() => {
    if (!topic || !connected || !rosRef.current) {
      setImgSrc(null)
      return
    }

    const topicObj = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topic,
      messageType: 'sensor_msgs/CompressedImage',
      throttle_rate: 50,
      queue_length: 1
    })
    subRef.current = topicObj

    topicObj.subscribe((msg) => {
      setImgSrc(`data:image/jpeg;base64,${msg.data}`)
    })

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
      setImgSrc(null)
    }
  }, [connected, rosRef, topic])

  return { imgSrc }
}
