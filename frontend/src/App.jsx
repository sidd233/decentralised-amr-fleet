import { useEffect, useRef, useState } from 'react'
import ControlPanel from './ControlPanel.jsx'
import FleetCanvas from './FleetCanvas.jsx'
import { PRESETS } from './presets.js'
import useMap from './useMap.js'

// WebSocket hook + top-level layout. Connects to `src/dashboard/server.rs`'s `/ws`
// (proxied by Vite in dev, same-origin in the production build) and re-renders on every
// snapshot. Since Decision 18 (docs/decisions.md) the dashboard is also an operator
// console: the controls below call the server's `/api/*` endpoints, which broadcast
// commands (new jobs, retargets, blocked cells) onto the robots' bus. The robots still
// claim, plan and negotiate on their own.
const EMPTY = { robots: {}, tasks: [], blocked: [], running: false, can_control: false }

function useFleetSnapshot() {
  const [snapshot, setSnapshot] = useState(EMPTY)
  const [connected, setConnected] = useState(false)
  const retryRef = useRef(null)

  useEffect(() => {
    let socket
    let cancelled = false

    function connect() {
      const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws'
      socket = new WebSocket(`${protocol}://${window.location.host}/ws`)

      socket.onopen = () => setConnected(true)
      socket.onmessage = (event) => setSnapshot({ ...EMPTY, ...JSON.parse(event.data) })
      socket.onclose = () => {
        setConnected(false)
        if (!cancelled) {
          retryRef.current = setTimeout(connect, 1000)
        }
      }
      socket.onerror = () => socket.close()
    }

    connect()
    return () => {
      cancelled = true
      clearTimeout(retryRef.current)
      socket?.close()
    }
  }, [])

  return { snapshot, connected }
}

const FRAME_MS = 100 // one logical tick (config.rs TICK_INTERVAL_MS)

// Replays the current run from its first tick. Frames come from the server's read-only
// `/history`; nothing is sent into the fleet.
function useReplay() {
  const [frames, setFrames] = useState(null) // null = live view
  const [index, setIndex] = useState(0)
  const [speed, setSpeed] = useState(1)
  const [runId, setRunId] = useState(0) // bumped on every start/stop so the canvas resets

  useEffect(() => {
    if (!frames || index >= frames.length - 1) return
    const timer = setTimeout(() => setIndex((i) => i + 1), FRAME_MS / speed)
    return () => clearTimeout(timer)
  }, [frames, index, speed])

  async function start() {
    const res = await fetch('/history')
    const data = await res.json()
    if (data.length === 0) return
    setFrames(data)
    setIndex(0)
    setRunId((n) => n + 1)
  }

  return {
    frames, index, speed, setSpeed, start, runId,
    stop: () => {
      setFrames(null)
      setRunId((n) => n + 1)
    },
  }
}

async function api(method, path, body) {
  const res = await fetch(path, {
    method,
    headers: { 'content-type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  const data = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(data.error ?? `request failed (${res.status})`)
  return data
}

export default function App() {
  const { snapshot: live, connected } = useFleetSnapshot()
  const replay = useReplay()
  const map = useMap()
  const shown = replay.frames ? replay.frames[replay.index] : live
  const robots = shown.robots ?? {}
  const robotIds = Object.keys(robots).sort((a, b) => Number(a) - Number(b))

  // Operator console state.
  const [setup, setSetup] = useState({ robots: [], tasks: [] })
  const [mode, setMode] = useState('robot')
  const [draft, setDraft] = useState(null) // first click of a job (its pickup)
  const [retargetId, setRetargetId] = useState(null)
  const [message, setMessage] = useState(null)
  const running = live.running

  useEffect(() => {
    if (running && mode === 'robot') setMode('task')
  }, [running, mode])

  function note(text, error = false) {
    setMessage({ text, error })
  }

  async function call(method, path, body, okText) {
    try {
      await api(method, path, body)
      if (okText) note(okText)
    } catch (e) {
      note(e.message, true)
    }
  }

  function loadPreset(i) {
    const p = PRESETS[i]
    setSetup({ robots: p.robots, tasks: p.tasks })
    setDraft(null)
    note(p.note)
  }

  function start() {
    call('POST', '/api/scenario/start', { robots: setup.robots, tasks: setup.tasks }, 'Scenario started.')
  }

  function onCellClick(x, y) {
    if (!live.can_control || replay.frames || !map) return
    if (mode === 'robot') {
      if (running) return note('Stop the scenario to change robot starts.', true)
      const at = setup.robots.findIndex(([rx, ry]) => rx === x && ry === y)
      if (at >= 0) return setSetup({ ...setup, robots: setup.robots.filter((_, i) => i !== at) })
      if (!map.isFree(x, y)) return note(`(${x}, ${y}) isn't a free cell.`, true)
      return setSetup({ ...setup, robots: [...setup.robots, [x, y]] })
    }
    if (mode === 'block') {
      if (!running) return note('Start the scenario before blocking cells.', true)
      const isBlocked = live.blocked.some(([bx, by]) => bx === x && by === y)
      return call('POST', '/api/block', { cell: [x, y], blocked: !isBlocked })
    }
    // mode === 'task'
    if (!map.isFree(x, y)) return note(`(${x}, ${y}) isn't a free cell.`, true)
    if (!draft) return setDraft([x, y])
    const job = { pickup: draft, dropoff: [x, y] }
    setDraft(null)
    if (retargetId != null) {
      const id = retargetId
      setRetargetId(null)
      return call('PUT', `/api/tasks/${id}`, job, `Job ${id} retargeted.`)
    }
    if (running) return call('POST', '/api/tasks', job, 'Job added.')
    setSetup({ ...setup, tasks: [...setup.tasks, job] })
  }

  // Before Start the job list shows what will be sent; after, the server's live statuses.
  const tasks = running
    ? shown.tasks ?? []
    : setup.tasks.map((t, i) => ({ ...t, task_id: i + 1, status: 'pending', robot: null }))
  const blocked = shown.blocked ?? []

  return (
    <div className="dashboard">
      <header>
        <h1>SIH26123 Fleet Dashboard</h1>
        <span className={connected ? 'status-ok' : 'status-down'}>
          {connected ? 'live' : 'reconnecting…'}
        </span>
      </header>

      {live.can_control && (
        <ControlPanel
          running={running}
          mode={mode}
          setMode={(m) => {
            setMode(m)
            setDraft(null)
            setRetargetId(null)
          }}
          draft={draft}
          retargetId={retargetId}
          tasks={tasks}
          robotCount={setup.robots.length}
          onPreset={loadPreset}
          onStart={start}
          onStop={() => call('POST', '/api/scenario/stop', undefined, 'Scenario stopped.')}
          onRetarget={(id) => {
            setMode('task')
            setDraft(null)
            setRetargetId(id)
          }}
          onCancel={(id) => call('DELETE', `/api/tasks/${id}`, undefined, `Job ${id} cancelled.`)}
          message={message}
        />
      )}

      <div className="replay-controls">
        {replay.frames ? (
          <>
            <button onClick={replay.start}>Restart</button>
            <button onClick={replay.stop}>Back to live</button>
            <select value={replay.speed} onChange={(e) => replay.setSpeed(Number(e.target.value))}>
              <option value={1}>1×</option>
              <option value={4}>4×</option>
              <option value={16}>16×</option>
            </select>
            <span>
              replay {replay.index + 1} / {replay.frames.length}
            </span>
          </>
        ) : (
          <button onClick={replay.start}>Replay from start</button>
        )}
      </div>

      <FleetCanvas
        map={map}
        robots={robots}
        glideMs={replay.frames ? FRAME_MS / replay.speed : FRAME_MS}
        resetKey={replay.runId}
        tasks={tasks}
        blocked={blocked}
        setupRobots={running || replay.frames ? [] : setup.robots}
        draft={draft}
        onCellClick={onCellClick}
      />

      <table className="fleet-table">
        <thead>
          <tr>
            <th>Robot</th>
            <th>Position</th>
            <th>Battery</th>
            <th>Mode</th>
            <th>Task</th>
          </tr>
        </thead>
        <tbody>
          {robotIds.length === 0 && (
            <tr>
              <td colSpan={5}>No robots heard from yet.</td>
            </tr>
          )}
          {robotIds.map((id) => {
            const robot = robots[id]
            return (
              <tr key={id}>
                <td>{id}</td>
                <td>{robot.position ? `(${robot.position[0]}, ${robot.position[1]})` : '—'}</td>
                <td>{robot.battery_pct != null ? `${robot.battery_pct.toFixed(1)}%` : '—'}</td>
                <td>{robot.mode === 'Autonomous' ? 'Autonomous (no peers in range)' : (robot.mode ?? '—')}</td>
                <td>{robot.current_task != null ? robot.current_task : 'idle'}</td>
              </tr>
            )
          })}
        </tbody>
      </table>
    </div>
  )
}
