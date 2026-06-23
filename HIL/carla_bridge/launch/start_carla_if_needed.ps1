param(
    [string]$CarlaExe = "",
    [int]$Port = 2000
)

$ErrorActionPreference = "Stop"

# launch/ → carla_bridge/ → HIL/ → 仓库根（CALRA 在仓库根）
if (-not $CarlaExe) {
    $repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
    $CarlaExe = Join-Path $repoRoot "CALRA\CarlaUE4.exe"
}

$existing = Get-Process | Where-Object { $_.ProcessName -like "*CarlaUE4*" }
if (-not $existing) {
    $resolved = Resolve-Path -LiteralPath $CarlaExe
    $workdir = Split-Path -Parent $resolved
    Write-Host "Starting CARLA: $resolved"
    Start-Process -FilePath $resolved -WorkingDirectory $workdir -ArgumentList "-quality-level=Low","-windowed","-ResX=1280","-ResY=720"
} else {
    Write-Host "CARLA already running."
}

Write-Host "Waiting for CARLA TCP port $Port ..."
for ($i = 0; $i -lt 90; $i++) {
    $r = Test-NetConnection 127.0.0.1 -Port $Port -WarningAction SilentlyContinue
    if ($r.TcpTestSucceeded) {
        Write-Host "CARLA port ready."
        exit 0
    }
    Start-Sleep -Seconds 1
}

throw "CARLA port $Port did not become ready in time."
