$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$wheel = Join-Path $repoRoot "CALRA\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
python -m pip install $wheel
