// Ready-made scenarios for the operator console. Every cell is a free cell on
// maps/warehouse-10-20-10-2-1.map: 1-wide aisles run along y = 1, 4, 7, ... between shelf
// blocks, with single-cell gaps between blocks (the map's natural choke points).
export const PRESETS = [
  {
    name: 'Demo — 3 robots, 3 jobs',
    note: 'The original compose scenario: robots start apart and work independently.',
    robots: [[5, 1], [10, 1], [139, 51]],
    tasks: [
      { pickup: [50, 4], dropoff: [60, 7] },
      { pickup: [70, 10], dropoff: [80, 13] },
      { pickup: [140, 55], dropoff: [150, 58] },
    ],
  },
  {
    name: 'Head-on in an aisle',
    note: 'Two robots must pass each other in the 1-wide aisle at y = 4 — watch the conflict resolution.',
    robots: [[30, 4], [100, 4]],
    tasks: [
      { pickup: [30, 4], dropoff: [100, 4] },
      { pickup: [100, 4], dropoff: [30, 4] },
    ],
  },
  {
    name: 'Blocked aisle — re-route',
    note: 'Start, then pick “Block cell” and click the aisle at about (80, 7) while the robot is on its way.',
    robots: [[10, 7], [10, 10]],
    tasks: [
      { pickup: [30, 7], dropoff: [140, 7] },
      { pickup: [30, 10], dropoff: [140, 10] },
    ],
  },
  {
    name: 'Rush hour — 6 robots, 8 jobs',
    note: 'A crowded left edge sends work to the far side of the warehouse.',
    robots: [[5, 1], [5, 4], [5, 7], [5, 10], [5, 13], [5, 16]],
    tasks: [
      { pickup: [40, 1], dropoff: [130, 4] },
      { pickup: [40, 4], dropoff: [130, 7] },
      { pickup: [40, 7], dropoff: [130, 10] },
      { pickup: [40, 10], dropoff: [130, 13] },
      { pickup: [40, 13], dropoff: [130, 16] },
      { pickup: [40, 16], dropoff: [130, 1] },
      { pickup: [90, 1], dropoff: [20, 13] },
      { pickup: [90, 16], dropoff: [20, 4] },
    ],
  },
]
