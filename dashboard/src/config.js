export const ROSBRIDGE_URL = `ws://${window.location.hostname}:9090`

export const ROBOT_NAME = 'EsiBot'

export const CMD_VEL = {
  LINEAR_SPEED: 0.4,
  ANGULAR_SPEED: 2.0
}

export const BATTERY_CAPACITY_MINUTES = 45

export const SCAN_OVERLAY = true

// ESP32-CAM raw MJPEG stream. The dashboard connects directly to the camera
// at http://<ip>/stream. This is only the default; the live value is editable
// in the Camera card and persisted in localStorage (see useEsp32Ip).
export const ESP32_CAM_DEFAULT_IP = '10.55.37.10'
