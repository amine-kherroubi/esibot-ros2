/**
 * Convert a canvas pixel coordinate to world coordinate (meters).
 */
export function canvasToWorld(cx, cy, meta, scale, pan) {
  const ox  = meta.origin.position.x
  const oy  = meta.origin.position.y
  const res = meta.resolution
  const mx  = (cx - pan.x) / scale
  const my  = meta.height - (cy - pan.y) / scale
  return {
    wx: ox + mx * res,
    wy: oy + my * res
  }
}

/**
 * Draw initial pose marker (green circle with cross) on canvas.
 */
export function drawInitialPose(ctx, cx, cy) {
  ctx.save()
  ctx.translate(cx, cy)
  ctx.beginPath()
  ctx.arc(0, 0, 8, 0, 2 * Math.PI)
  ctx.fillStyle = '#22c55e'
  ctx.fill()
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 2
  ctx.stroke()
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 2
  ctx.beginPath(); ctx.moveTo(-4, 0); ctx.lineTo(4, 0); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(0, -4); ctx.lineTo(0, 4); ctx.stroke()
  ctx.restore()
}

/**
 * Draw goal marker (flag pin) on canvas.
 */
export function drawGoal(ctx, cx, cy) {
  ctx.save()
  ctx.translate(cx, cy)
  // Pin circle
  ctx.beginPath()
  ctx.arc(0, 0, 8, 0, 2 * Math.PI)
  ctx.fillStyle = '#f59e0b'
  ctx.fill()
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 2
  ctx.stroke()
  // Cross
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 2
  ctx.beginPath(); ctx.moveTo(-4, 0); ctx.lineTo(4, 0); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(0, -4); ctx.lineTo(0, 4); ctx.stroke()
  ctx.restore()
}

/**
 * Convert a world coordinate (meters) to canvas pixel coordinate.
 *
 * @param {number} wx  — world x (meters)
 * @param {number} wy  — world y (meters)
 * @param {object} meta — { origin: {position:{x,y}}, resolution, width, height }
 * @param {number} scale — pixels per cell (canvas zoom)
 * @returns {{ cx: number, cy: number }}
 */
export function worldToCanvas(wx, wy, meta, scale) {
  const ox = meta.origin.position.x
  const oy = meta.origin.position.y
  const res = meta.resolution

  // Map cell indices (note: ROS map y increases upward, canvas y increases downward)
  const mx = (wx - ox) / res
  const my = (wy - oy) / res

  return {
    cx: mx * scale,
    cy: (meta.height - my) * scale
  }
}

/**
 * Draw the robot pose as a filled circle with a direction arrow.
 *
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} cx — canvas x
 * @param {number} cy — canvas y
 * @param {number} yaw — heading in radians
 * @param {number} scale — pixels per cell
 */
export function drawRobot(ctx, cx, cy, yaw, scale) {
  const r = Math.max(10, scale * 0.8)

  ctx.save()
  ctx.translate(cx, cy)

  // Body
  ctx.beginPath()
  ctx.arc(0, 0, r, 0, 2 * Math.PI)
  ctx.fillStyle = '#3b82f6'
  ctx.fill()
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 1.5
  ctx.stroke()

  // Direction arrow
  ctx.rotate(-yaw) // canvas y is flipped vs ROS
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(r * 2.0, 0)
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 2
  ctx.stroke()

  ctx.restore()
}

/**
 * Draw LIDAR scan rays on the canvas.
 *
 * @param {CanvasRenderingContext2D} ctx
 * @param {object} scan  — { ranges, angle_min, angle_increment, range_max }
 * @param {object} pose  — { x, y, yaw }
 * @param {object} meta  — map metadata
 * @param {number} scale
 */
export function drawScan(ctx, scan, pose, meta, scale) {
  if (!scan) return
  const { ranges, angle_min, angle_increment, range_max } = scan
  const { cx: rx, cy: ry } = worldToCanvas(pose.x, pose.y, meta, scale)

  ctx.save()
  ctx.strokeStyle = 'rgba(239,68,68,0.6)'
  ctx.lineWidth = 1

  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (!isFinite(r) || r <= 0 || r >= range_max) continue

    const angle = angle_min + i * angle_increment + pose.yaw
    const wx = pose.x + r * Math.cos(angle)
    const wy = pose.y + r * Math.sin(angle)
    const { cx, cy } = worldToCanvas(wx, wy, meta, scale)

    ctx.beginPath()
    ctx.moveTo(rx, ry)
    ctx.lineTo(cx, cy)
    ctx.stroke()
  }

  ctx.restore()
}

/**
 * Draw the robot path (array of {x,y} world coords) on the canvas.
 *
 * @param {CanvasRenderingContext2D} ctx
 * @param {Array<{x,y}>} path
 * @param {object} meta
 * @param {number} scale
 */
export function drawPath(ctx, path, meta, scale) {
  if (!path || path.length < 2) return

  ctx.save()
  ctx.strokeStyle = 'rgba(59,130,246,0.5)'
  ctx.lineWidth = 1.5
  ctx.setLineDash([3, 3])
  ctx.beginPath()

  path.forEach((pt, idx) => {
    const { cx, cy } = worldToCanvas(pt.x, pt.y, meta, scale)
    if (idx === 0) ctx.moveTo(cx, cy)
    else ctx.lineTo(cx, cy)
  })

  ctx.stroke()
  ctx.restore()
}
