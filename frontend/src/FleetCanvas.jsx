import { useEffect, useRef } from 'react'

// Renders `docs/PS_AND_ARCHITECTURE.md` §3.6's "position, battery, mode, task status per
// robot" onto a 2D canvas. `robots` is the parsed `FleetSnapshot` JSON (live off `/ws`, or
// a frame of the `/history` replay) — this component only draws it, it never touches the
// socket itself (that's `App.jsx`'s job). Robots move one cell per tick, so instead of
// snapping between cells it eases each robot from its previous drawn position to the new
// one over `glideMs` (about one tick), and keeps a short fading trail behind each.
// The warehouse map (`useMap.js`, a copy of maps/warehouse-10-20-10-2-1.map) is the
// background; operator-console overlays (jobs, blocked cells, placed robots) sit on top
// and a click reports the clicked cell to `App.jsx` via `onCellClick`.

const CELL_PX = 6
const TRAIL_LEN = 40
const MODE_COLOR = {
  Cooperative: '#3fb950',
  Cautious: '#d29922',
  Autonomous: '#f85149',
}
const UNKNOWN_COLOR = '#8b949e'

function batteryColor(pct) {
  if (pct == null) return UNKNOWN_COLOR
  if (pct > 50) return '#3fb950'
  if (pct > 20) return '#d29922'
  return '#f85149'
}

function drawMap(map) {
  const off = document.createElement('canvas')
  off.width = map.width * CELL_PX
  off.height = map.height * CELL_PX
  const ctx = off.getContext('2d')
  ctx.fillStyle = '#0d1117'
  ctx.fillRect(0, 0, off.width, off.height)
  ctx.fillStyle = '#21262d'
  map.rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      if (row[x] !== '.') ctx.fillRect(x * CELL_PX, y * CELL_PX, CELL_PX, CELL_PX)
    }
  })
  return off
}

const TASK_COLOR = { pending: '#8b949e', active: '#58a6ff', done: '#3fb950' }

// Operator-console overlays (Decision 18): jobs as pickup circle -> dropoff square,
// operator-blocked cells, robots placed before Start, and the first click of a new job.
function drawOverlays(ctx, { tasks, blocked, setupRobots, draft }) {
  const c = (v) => (v + 0.5) * CELL_PX
  for (const t of tasks) {
    if (t.status === 'cancelled') continue
    ctx.globalAlpha = t.status === 'done' ? 0.35 : 0.9
    ctx.strokeStyle = TASK_COLOR[t.status] ?? '#8b949e'
    ctx.fillStyle = ctx.strokeStyle
    ctx.lineWidth = 1
    ctx.setLineDash([3, 3])
    ctx.beginPath()
    ctx.moveTo(c(t.pickup[0]), c(t.pickup[1]))
    ctx.lineTo(c(t.dropoff[0]), c(t.dropoff[1]))
    ctx.stroke()
    ctx.setLineDash([])
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.arc(c(t.pickup[0]), c(t.pickup[1]), CELL_PX * 0.9, 0, Math.PI * 2)
    ctx.stroke()
    ctx.fillRect(c(t.dropoff[0]) - CELL_PX * 0.8, c(t.dropoff[1]) - CELL_PX * 0.8, CELL_PX * 1.6, CELL_PX * 1.6)
    ctx.font = '10px sans-serif'
    ctx.textAlign = 'left'
    ctx.fillText(`${t.task_id}`, c(t.pickup[0]) + CELL_PX, c(t.pickup[1]) - CELL_PX)
  }
  ctx.globalAlpha = 1
  ctx.strokeStyle = '#f85149'
  ctx.lineWidth = 2
  for (const [x, y] of blocked) {
    ctx.fillStyle = 'rgba(248,81,73,0.35)'
    ctx.fillRect(x * CELL_PX, y * CELL_PX, CELL_PX, CELL_PX)
    ctx.beginPath()
    ctx.moveTo(x * CELL_PX, y * CELL_PX)
    ctx.lineTo((x + 1) * CELL_PX, (y + 1) * CELL_PX)
    ctx.moveTo((x + 1) * CELL_PX, y * CELL_PX)
    ctx.lineTo(x * CELL_PX, (y + 1) * CELL_PX)
    ctx.stroke()
  }
  setupRobots.forEach(([x, y], i) => {
    ctx.beginPath()
    ctx.arc(c(x), c(y), CELL_PX * 0.9, 0, Math.PI * 2)
    ctx.fillStyle = '#8b949e'
    ctx.fill()
    ctx.fillStyle = '#e6edf3'
    ctx.font = '10px sans-serif'
    ctx.textAlign = 'center'
    ctx.fillText(`#${i + 1}`, c(x), c(y) - CELL_PX * 1.6)
  })
  if (draft) {
    ctx.strokeStyle = '#58a6ff'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.arc(c(draft[0]), c(draft[1]), CELL_PX * 1.2, 0, Math.PI * 2)
    ctx.stroke()
  }
}

export default function FleetCanvas({
  map, robots, glideMs = 100, resetKey = 0,
  tasks = [], blocked = [], setupRobots = [], draft = null, onCellClick,
}) {
  const canvasRef = useRef(null)
  const mapImageRef = useRef(null)
  const anim = useRef({}) // id -> { from, to, t0, trail, robot }
  const glideRef = useRef(glideMs)
  glideRef.current = glideMs
  const overlayRef = useRef({})
  overlayRef.current = { tasks, blocked, setupRobots, draft }

  useEffect(() => {
    mapImageRef.current = map ? drawMap(map) : null
  }, [map])

  // A new replay (or leaving it) starts from clean positions/trails, not a glide from
  // wherever the previous view left off.
  useEffect(() => {
    anim.current = {}
  }, [resetKey])

  useEffect(() => {
    const now = performance.now()
    for (const [id, robot] of Object.entries(robots)) {
      if (!robot.position) continue
      const [x, y] = robot.position
      const a = anim.current[id]
      if (!a) {
        anim.current[id] = { from: [x, y], to: [x, y], t0: now, trail: [[x, y]], robot }
      } else {
        a.robot = robot
        if (a.to[0] !== x || a.to[1] !== y) {
          const p = Math.min(1, (now - a.t0) / glideRef.current)
          a.from = [a.from[0] + (a.to[0] - a.from[0]) * p, a.from[1] + (a.to[1] - a.from[1]) * p]
          a.to = [x, y]
          a.t0 = now
          a.trail.push([x, y])
          if (a.trail.length > TRAIL_LEN) a.trail.shift()
        }
      }
    }
  }, [robots])

  useEffect(() => {
    let raf
    function frame(now) {
      const canvas = canvasRef.current
      if (canvas) {
        const ctx = canvas.getContext('2d')
        ctx.fillStyle = '#0d1117'
        ctx.fillRect(0, 0, canvas.width, canvas.height)
        if (mapImageRef.current) ctx.drawImage(mapImageRef.current, 0, 0)
        drawOverlays(ctx, overlayRef.current)

        for (const [id, a] of Object.entries(anim.current)) {
          const p = Math.min(1, (now - a.t0) / glideRef.current)
          const x = (a.from[0] + (a.to[0] - a.from[0]) * p + 0.5) * CELL_PX
          const y = (a.from[1] + (a.to[1] - a.from[1]) * p + 0.5) * CELL_PX
          const color = MODE_COLOR[a.robot.mode] ?? UNKNOWN_COLOR

          ctx.strokeStyle = color
          a.trail.forEach((pt, i) => {
            if (i === 0) return
            const prev = a.trail[i - 1]
            ctx.globalAlpha = (i / a.trail.length) * 0.6
            ctx.lineWidth = 2
            ctx.beginPath()
            ctx.moveTo((prev[0] + 0.5) * CELL_PX, (prev[1] + 0.5) * CELL_PX)
            ctx.lineTo((pt[0] + 0.5) * CELL_PX, (pt[1] + 0.5) * CELL_PX)
            ctx.stroke()
          })
          ctx.globalAlpha = 1

          ctx.beginPath()
          ctx.arc(x, y, CELL_PX * 0.9, 0, Math.PI * 2)
          ctx.fillStyle = color
          ctx.fill()
          ctx.strokeStyle = '#e6edf3'
          ctx.lineWidth = 1
          ctx.stroke()

          ctx.fillStyle = '#e6edf3'
          ctx.font = '10px sans-serif'
          ctx.textAlign = 'center'
          ctx.fillText(`#${id}`, x, y - CELL_PX * 1.6)
          const battery = a.robot.battery_pct
          ctx.fillStyle = batteryColor(battery)
          ctx.fillText(battery == null ? '?' : `${battery.toFixed(0)}%`, x, y + CELL_PX * 2.6)
          if (a.robot.current_task != null) {
            ctx.fillStyle = '#58a6ff'
            ctx.fillText(`task ${a.robot.current_task}`, x, y + CELL_PX * 4)
          }
        }
      }
      raf = requestAnimationFrame(frame)
    }
    raf = requestAnimationFrame(frame)
    return () => cancelAnimationFrame(raf)
  }, [])

  function handleClick(e) {
    if (!onCellClick) return
    const canvas = canvasRef.current
    const rect = canvas.getBoundingClientRect()
    const x = Math.floor(((e.clientX - rect.left) / rect.width) * (canvas.width / CELL_PX))
    const y = Math.floor(((e.clientY - rect.top) / rect.height) * (canvas.height / CELL_PX))
    onCellClick(x, y)
  }

  return (
    <canvas
      ref={canvasRef}
      width={161 * CELL_PX}
      height={63 * CELL_PX}
      className="fleet-canvas"
      onClick={handleClick}
    />
  )
}
