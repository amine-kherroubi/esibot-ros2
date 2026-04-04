import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosbridgeContext } from '../context/RosbridgeContext'

/**
 * Subscribes to a camera topic.
 * Handles both:
 *   - sensor_msgs/CompressedImage  → base64 JPEG data
 *   - sensor_msgs/Image            → base64 BGR8 raw data (converted via canvas)
 */
export function useCamera(topic = '/camera/image_annotated/compressed') {
  const { rosRef, connected } = useRosbridgeContext()
  const [imgSrc, setImgSrc] = useState(null)
  const subRef    = useRef(null)
  const canvasRef = useRef(null)

  useEffect(() => {
    if (!connected || !rosRef.current) {
      setImgSrc(null)
      return
    }

    // Detect topic type first
    const topicObj = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topic,
      messageType: 'sensor_msgs/Image',
      throttle_rate: 80,
      queue_length: 1
    })

    subRef.current = topicObj

    subRef.current.subscribe((msg) => {
      const { width, height, encoding, data } = msg

      // If JPEG compressed image (sometimes published as sensor_msgs/Image with jpeg encoding)
      if (encoding === 'jpeg' || encoding === 'rgb8_jpeg') {
        setImgSrc(`data:image/jpeg;base64,${data}`)
        return
      }

      // Raw BGR8 → convert to canvas
      try {
        if (!canvasRef.current) {
          canvasRef.current = document.createElement('canvas')
        }
        const canvas = canvasRef.current
        if (canvas.width !== width || canvas.height !== height) {
          canvas.width  = width
          canvas.height = height
        }
        const ctx = canvas.getContext('2d')
        const imgData = ctx.createImageData(width, height)
        const pixels = imgData.data

        // data is base64 — decode it
        const binary = atob(data)
        const bytes  = new Uint8Array(binary.length)
        for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i)

        const isBGR = encoding === 'bgr8'
        for (let i = 0; i < width * height; i++) {
          const src = i * 3
          pixels[i * 4 + 0] = isBGR ? bytes[src + 2] : bytes[src + 0] // R
          pixels[i * 4 + 1] = bytes[src + 1]                           // G
          pixels[i * 4 + 2] = isBGR ? bytes[src + 0] : bytes[src + 2] // B
          pixels[i * 4 + 3] = 255
        }
        ctx.putImageData(imgData, 0, 0)
        setImgSrc(canvas.toDataURL('image/jpeg', 0.8))
      } catch (e) {
        // ignore frame errors
      }
    })

    return () => {
      subRef.current?.unsubscribe()
      subRef.current = null
      setImgSrc(null)
    }
  }, [connected, rosRef, topic])

  return { imgSrc }
}
