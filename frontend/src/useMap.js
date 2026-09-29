import { useEffect, useState } from 'react'

// Loads the warehouse map the robots run on (`public/map.txt`, a copy of
// maps/warehouse-10-20-10-2-1.map) so the canvas can draw it and the operator controls can
// reject clicks on walls/shelves before ever calling the server (which validates again).
function parseMap(text) {
  const lines = text.split('\n')
  const start = lines.findIndex((l) => l.trim() === 'map')
  const rows = lines.slice(start + 1).filter((l) => l.length > 0)
  const width = Math.max(...rows.map((r) => r.length))
  return {
    rows,
    width,
    height: rows.length,
    isFree: (x, y) => y >= 0 && y < rows.length && rows[y][x] === '.',
  }
}

export default function useMap() {
  const [map, setMap] = useState(null)
  useEffect(() => {
    fetch('/map.txt')
      .then((r) => r.text())
      .then((text) => setMap(parseMap(text)))
      .catch(() => {})
  }, [])
  return map
}
