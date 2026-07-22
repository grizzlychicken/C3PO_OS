# Motion CSV Capture

Use this script to log Teensy motion telemetry CSV output into timestamped files.

## Prerequisites

1. Firmware is built with:
   - `DEBUG_MOTION 1`
   - `DEBUG_MOTION_CSV 1`
2. Teensy is connected over USB serial.
3. You know the COM port (for example `COM5`).

## Run

From repo root in PowerShell:

```powershell
.\capture_motion_csv.ps1 -Port COM5
```

Optional parameters:

```powershell
.\capture_motion_csv.ps1 -Port COM5 -Baud 115200 -DurationSec 120 -OutDir logs/motion_csv -Prefix test_head_yaw
```

## Output

- Files are written to `logs/motion_csv` by default.
- Filename format: `motion_yyyyMMdd_HHmmss.csv`.
- The script keeps only CSV telemetry lines and ignores other serial prints.

## Tips

1. Start capture first, then power cycle Teensy if you want the CSV header included from boot.
2. Keep one test scenario per capture file (for example: yaw sweep, torso lean, mixed motion).
3. If there is no data:
   - confirm firmware debug flags are enabled
   - verify COM port is correct
   - ensure no other app has the serial port open
