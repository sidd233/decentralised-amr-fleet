import { PRESETS } from './presets.js'

// Operator console UI (Decision 18): scenario setup/start/stop, click-mode picker and the
// job list with retarget/cancel. Pure presentation — App.jsx owns state and API calls.
const MODES = [
  { id: 'robot', label: 'Place robots', hint: 'Click a free cell to add a robot (click it again to remove). Needs at least 2 robots; before Start only.' },
  { id: 'task', label: 'Add job', hint: 'Click a pickup cell, then a dropoff cell.' },
  { id: 'block', label: 'Block cell', hint: 'Click a free cell to block it (a blocked aisle); click again to unblock.' },
]

export default function ControlPanel({
  running, mode, setMode, draft, retargetId, tasks, robotCount,
  onPreset, onStart, onStop, onRetarget, onCancel, message,
}) {
  const active = MODES.find((m) => m.id === mode)
  return (
    <section className="control-panel">
      <div className="control-row">
        <select defaultValue="" onChange={(e) => e.target.value !== '' && onPreset(Number(e.target.value))} disabled={running}>
          <option value="" disabled>Load a preset…</option>
          {PRESETS.map((p, i) => (
            <option key={p.name} value={i}>{p.name}</option>
          ))}
        </select>
        {running ? (
          <button className="danger" onClick={onStop}>Stop scenario</button>
        ) : (
          <button className="primary" onClick={onStart} disabled={robotCount < 2}>
            Start scenario ({robotCount} robot{robotCount === 1 ? '' : 's'})
          </button>
        )}
        <span className="mode-picker">
          {MODES.map((m) => (
            <button key={m.id} className={mode === m.id ? 'selected' : ''} onClick={() => setMode(m.id)}>
              {m.label}
            </button>
          ))}
        </span>
      </div>
      <p className="hint">
        {retargetId != null
          ? `Retargeting job ${retargetId}: click its new pickup, then its new dropoff.`
          : active.hint}
        {draft && ` Pickup set at (${draft[0]}, ${draft[1]}) — now click the dropoff.`}
      </p>
      {message && <p className={message.error ? 'msg error' : 'msg'}>{message.text}</p>}

      {tasks.length > 0 && (
        <table className="fleet-table">
          <thead>
            <tr><th>Job</th><th>Pickup</th><th>Dropoff</th><th>Status</th><th>Robot</th><th></th></tr>
          </thead>
          <tbody>
            {tasks.map((t) => (
              <tr key={t.task_id} className={`status-${t.status}`}>
                <td>{t.task_id}</td>
                <td>({t.pickup[0]}, {t.pickup[1]})</td>
                <td>({t.dropoff[0]}, {t.dropoff[1]})</td>
                <td>{t.status}</td>
                <td>{t.robot ?? '—'}</td>
                <td>
                  {running && (t.status === 'pending' || t.status === 'active') && (
                    <>
                      <button onClick={() => onRetarget(t.task_id)}>Retarget</button>{' '}
                      <button onClick={() => onCancel(t.task_id)}>Cancel</button>
                    </>
                  )}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </section>
  )
}
