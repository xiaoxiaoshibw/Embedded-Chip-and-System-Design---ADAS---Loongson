# record_demo.ps1 — ADAS 演示视频录制辅助
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File record_demo.ps1
#   powershell -ExecutionPolicy Bypass -File record_demo.ps1 -Scenes "lka","aeb","failover"
#   powershell -ExecutionPolicy Bypass -File record_demo.ps1 -NoMerge
#
# 前提：ffmpeg 已安装且在 PATH 中
#   winget install ffmpeg    # 或 choco install ffmpeg

param(
    [string[]]$Scenes = @("lka","acc","aeb","overtake","failover"),
    [string]$OutputDir = "recordings",
    [string]$Resolution = "1280x720",
    [int]$Fps = 30,
    [switch]$NoMerge,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# ── 检查 ffmpeg ──
$ffmpeg = Get-Command ffmpeg -ErrorAction SilentlyContinue
if (-not $ffmpeg) {
    Write-Host "[!] 未找到 ffmpeg，请先安装:" -ForegroundColor Red
    Write-Host "    winget install ffmpeg" -ForegroundColor Yellow
    Write-Host "    或 choco install ffmpeg" -ForegroundColor Yellow
    exit 1
}

# ── 创建输出目录 ──
$outPath = Join-Path $scriptDir $OutputDir
New-Item -ItemType Directory -Force -Path $outPath | Out-Null
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"

Write-Host ""
Write-Host "═══════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "  ADAS 演示视频录制" -ForegroundColor Cyan
Write-Host "  场景: $($Scenes -join ' → ')" -ForegroundColor Cyan
Write-Host "  输出: $outPath" -ForegroundColor Cyan
Write-Host "  分辨率: $Resolution @ ${Fps}fps" -ForegroundColor Cyan
Write-Host "═══════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host ""

# ── 获取 CARLA 窗口信息 ──
$carlaProcess = Get-Process -Name "CarlaUE4*" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $carlaProcess) {
    Write-Host "[!] 未检测到 CARLA 进程，请先启动 CARLA" -ForegroundColor Yellow
    Write-Host "    或运行: python demo_video.py（会自动启动）" -ForegroundColor Yellow
    $hwnd = $null
} else {
    $hwnd = $carlaProcess.MainWindowHandle
    Write-Host "[OK] CARLA 进程: $($carlaProcess.ProcessName) (PID: $($carlaProcess.Id))" -ForegroundColor Green
}

# ── 录制函数 ──
function Start-Recording {
    param(
        [string]$SceneName,
        [string]$OutFile
    )

    # 使用 gdigrab 录制桌面区域（兼容性最好）
    # 如果 CARLA 窗口有句柄，可以只录该窗口
    $ffmpegArgs = @(
        "-f", "gdigrab",
        "-framerate", $Fps,
        "-i", "desktop",
        "-vcodec", "libx264",
        "-preset", "ultrafast",
        "-crf", "23",
        "-pix_fmt", "yuv420p",
        "-t", "180",  # 最长 3 分钟保护
        "-y",
        $OutFile
    )

    Write-Host "[REC] 开始录制: $SceneName → $OutFile" -ForegroundColor Red
    if ($DryRun) {
        Write-Host "  [DRY] ffmpeg $($ffmpegArgs -join ' ')" -ForegroundColor Gray
        return $null
    }

    $proc = Start-Process -FilePath "ffmpeg" -ArgumentList $ffmpegArgs `
        -NoNewWindow -PassThru -RedirectStandardError "$OutFile.log"
    return $proc
}

# ── 主录制循环 ──
$recordings = @()

foreach ($scene in $Scenes) {
    $outFile = Join-Path $outPath "${timestamp}_${scene}.mp4"

    Write-Host ""
    Write-Host "── 场景: $scene ──" -ForegroundColor Yellow
    Write-Host "  按 Enter 开始录制，场景结束后按 Enter 停止" -ForegroundColor Gray

    if (-not $DryRun) {
        Read-Host "  按 Enter 开始录制"
    }

    $recProc = Start-Recording -SceneName $scene -OutFile $outFile

    # 同时启动场景
    Write-Host "  [RUN] python cli.py $scene" -ForegroundColor Green
    if (-not $DryRun) {
        $sceneProc = Start-Process -FilePath "python" `
            -ArgumentList "cli.py", $scene `
            -WorkingDirectory $scriptDir `
            -PassThru
    }

    if (-not $DryRun) {
        Read-Host "  场景结束后按 Enter 停止录制"
        if ($recProc -and -not $recProc.HasExited) {
            # 发送 'q' 到 ffmpeg 停止录制
            $recProc | Stop-Process -Force -ErrorAction SilentlyContinue
        }
        if ($sceneProc -and -not $sceneProc.HasExited) {
            $sceneProc | Stop-Process -Force -ErrorAction SilentlyContinue
        }
    }

    $recordings += $outFile
    Write-Host "[OK] 已保存: $outFile" -ForegroundColor Green
}

# ── 合并视频 ──
if (-not $NoMerge -and $recordings.Count -gt 1) {
    $mergeList = Join-Path $outPath "${timestamp}_filelist.txt"
    $mergedFile = Join-Path $outPath "${timestamp}_full_demo.mp4"

    Write-Host ""
    Write-Host "[...] 合并 $($recordings.Count) 段视频" -ForegroundColor Cyan

    # 创建 ffmpeg concat 文件
    $concatContent = ($recordings | ForEach-Object { "file '$($_ -replace '\\','\\')'" }) -join "`n"
    Set-Content -Path $mergeList -Value $concatContent -Encoding UTF8

    if (-not $DryRun) {
        $mergeArgs = @(
            "-f", "concat",
            "-safe", "0",
            "-i", $mergeList,
            "-c", "copy",
            "-y",
            $mergedFile
        )
        & ffmpeg @mergeArgs
        Write-Host "[OK] 合并完成: $mergedFile" -ForegroundColor Green
    } else {
        Write-Host "  [DRY] ffmpeg -f concat -safe 0 -i $mergeList -c copy -y $mergedFile" -ForegroundColor Gray
    }

    # 清理临时文件
    Remove-Item $mergeList -ErrorAction SilentlyContinue
}

Write-Host ""
Write-Host "═══════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "  录制完成！文件位于: $outPath" -ForegroundColor Cyan
Write-Host "═══════════════════════════════════════════════════" -ForegroundColor Cyan
