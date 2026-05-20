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
 * Draw LIDAR scan: faint rays + bright endpoint dots.
 */
export function drawScan(ctx, scan, pose, meta, scale) {
  if (!scan) return
  const { ranges, angle_min, angle_increment, range_max } = scan
  const { cx: rx, cy: ry } = worldToCanvas(pose.x, pose.y, meta, scale)

  ctx.save()

  // Faint rays
  ctx.strokeStyle = 'rgba(239,68,68,0.2)'
  ctx.lineWidth = 0.8
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (!isFinite(r) || r <= 0 || r >= range_max) continue
    const angle = angle_min + i * angle_increment + pose.yaw
    const { cx, cy } = worldToCanvas(pose.x + r * Math.cos(angle), pose.y + r * Math.sin(angle), meta, scale)
    ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(cx, cy); ctx.stroke()
  }

  // Bright endpoint dots
  ctx.fillStyle = 'rgba(239,68,68,0.95)'
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (!isFinite(r) || r <= 0 || r >= range_max) continue
    const angle = angle_min + i * angle_increment + pose.yaw
    const { cx, cy } = worldToCanvas(pose.x + r * Math.cos(angle), pose.y + r * Math.sin(angle), meta, scale)
    ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 2 * Math.PI); ctx.fill()
  }

  ctx.restore()
}

/**
 * Draw a 1m grid over the map (inside pan/translate block).
 */
export function drawGrid(ctx, meta, scale, theme) {
  if (!meta) return
  const { origin, resolution, width, height } = meta
  const ox = origin.position.x
  const oy = origin.position.y
  const color = theme === 'light' ? 'rgba(0,0,0,0.08)' : 'rgba(255,255,255,0.06)'

  ctx.save()
  ctx.strokeStyle = color
  ctx.lineWidth = 0.5

  const xMax = ox + width * resolution
  const yMax = oy + height * resolution

  for (let wx = Math.ceil(ox); wx <= xMax; wx++) {
    const px = (wx - ox) / resolution * scale
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, height * scale); ctx.stroke()
  }
  for (let wy = Math.ceil(oy); wy <= yMax; wy++) {
    const py = (height - (wy - oy) / resolution) * scale
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(width * scale, py); ctx.stroke()
  }

  ctx.restore()
}

/**
 * Draw a scale bar HUD in the bottom-right corner (outside pan/translate block).
 */
export function drawScaleBar(ctx, meta, scale, cw, ch, theme) {
  if (!meta) return
  const pxPerMeter = scale / meta.resolution
  if (pxPerMeter < 4) return

  let barM = 1
  if (pxPerMeter * 5 <= 100) barM = 5
  else if (pxPerMeter * 2 <= 100) barM = 2

  const barPx = barM * pxPerMeter
  const x = cw - barPx - 14
  const y = ch - 14
  const tk = 5

  ctx.save()
  const stroke = theme === 'light' ? 'rgba(0,0,0,0.55)' : 'rgba(255,255,255,0.7)'
  ctx.strokeStyle = stroke
  ctx.fillStyle   = stroke
  ctx.lineWidth = 1.5
  ctx.font = '10px Inter, sans-serif'
  ctx.textAlign = 'center'

  ctx.beginPath()
  ctx.moveTo(x, y - tk); ctx.lineTo(x, y)
  ctx.lineTo(x + barPx, y); ctx.lineTo(x + barPx, y - tk)
  ctx.stroke()
  ctx.fillText(`${barM}m`, x + barPx / 2, y - tk - 3)
  ctx.restore()
}

/**
 * Draw map stats overlay in the bottom-left corner (outside pan/translate block).
 */
export function drawMapInfo(ctx, stats, ch, theme) {
  if (!stats || !stats.totalCells) return
  const pct = Math.round(stats.exploredCells / stats.totalCells * 100)
  const text = `${stats.widthM.toFixed(1)} × ${stats.heightM.toFixed(1)} m  ·  ${pct}% explored`

  ctx.save()
  ctx.font = '10px Inter, sans-serif'
  ctx.textAlign = 'left'
  ctx.fillStyle = theme === 'light' ? 'rgba(0,0,0,0.45)' : 'rgba(255,255,255,0.4)'
  ctx.fillText(text, 10, ch - 8)
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
