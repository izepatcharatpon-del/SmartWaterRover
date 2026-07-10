# USV Simulation

Open **one file** in Chrome or Edge:

`1_Firmware/simulation/index.html`

Or run `start-sim.ps1`

## Controls

| Button | Action |
|--------|--------|
| Start | Begin GRID patrol (waypoint grid) |
| Pause / Resume | Freeze simulation |
| Reset | Return to STANDBY at start position |
| E-Stop | Emergency stop (press Start again to clear) |
| Force RTH | Return to home (only while on mission) |
| Add hotspot | Click map to place pollution zone |

## What you should see

1. Press **Start** - boat **turns bow first** (yellow arrow), then moves toward gray/yellow waypoints
2. Dashed line shows bearing to current target
3. Trail color: blue=grid, purple=spiral, orange=RTH
4. Grid patrol passes through the red zone at center (75,75) to trigger SPIRAL

Logic: fixed 20 Hz, heading rotates toward target before moving (no stuck bow).
