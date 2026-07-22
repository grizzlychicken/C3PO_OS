param(
    [Parameter(Mandatory = $true)]
    [string]$Port,

    [int]$Baud = 115200,

    [int]$DurationSec = 0,

    [string]$OutDir = "logs/motion_csv",

    [string]$Prefix = "motion"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function New-TimeStamp {
    return (Get-Date -Format "yyyyMMdd_HHmmss")
}

function Ensure-Directory([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) {
        New-Item -ItemType Directory -Path $Path | Out-Null
    }
}

function Is-CsvTelemetryLine([string]$Line) {
    if ([string]::IsNullOrWhiteSpace($Line)) {
        return $false
    }

    $trimmed = $Line.Trim()

    # Header row emitted by firmware when DEBUG_MOTION_CSV_HEADER=1
    if ($trimmed.StartsWith("ms,mode_rc,mode_head,")) {
        return $true
    }

    # Data rows start with milliseconds value
    return $trimmed -match '^[0-9]+,'
}

try {
    Ensure-Directory -Path $OutDir

    $timestamp = New-TimeStamp
    $outFile = Join-Path $OutDir ("{0}_{1}.csv" -f $Prefix, $timestamp)

    $serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
    $serial.NewLine = "`n"
    $serial.ReadTimeout = 250

    Write-Host "Opening $Port at $Baud baud..."
    $serial.Open()
    Write-Host "Connected. Logging CSV telemetry to: $outFile"

    $writer = New-Object System.IO.StreamWriter($outFile, $false, [System.Text.Encoding]::ASCII)

    $start = Get-Date
    $savedCount = 0
    $headerSeen = $false

    Write-Host "Press Ctrl+C to stop."

    while ($true) {
        if ($DurationSec -gt 0) {
            $elapsed = (Get-Date) - $start
            if ($elapsed.TotalSeconds -ge $DurationSec) {
                break
            }
        }

        try {
            $line = $serial.ReadLine()
        }
        catch [System.TimeoutException] {
            continue
        }

        if (-not (Is-CsvTelemetryLine -Line $line)) {
            continue
        }

        $trimmed = $line.Trim()

        # Keep only one header, even if the board reboots and prints again.
        if ($trimmed.StartsWith("ms,mode_rc,mode_head,")) {
            if ($headerSeen) {
                continue
            }
            $headerSeen = $true
        }

        $writer.WriteLine($trimmed)
        $savedCount++

        if (($savedCount % 100) -eq 0) {
            Write-Host "Saved $savedCount lines..."
        }
    }

    Write-Host "Capture finished."
    Write-Host "Saved lines: $savedCount"
    Write-Host "File: $outFile"
}
finally {
    if ($null -ne $writer) {
        $writer.Flush()
        $writer.Close()
        $writer.Dispose()
    }

    if ($null -ne $serial -and $serial.IsOpen) {
        $serial.Close()
        $serial.Dispose()
    }
}
