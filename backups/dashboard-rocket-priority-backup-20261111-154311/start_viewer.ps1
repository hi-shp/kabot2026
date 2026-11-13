param(
    [string]$Jetson = 'jetson',
    [string]$User = 'isv',
    [int]$Port = 8765,
    [int]$MaxPoints = 1800,
    [double]$Hz = 3
)
$ErrorActionPreference = 'Stop'
$viewerEnv = Join-Path $env:LOCALAPPDATA 'KabotSensorViewer\venv'
$viewerPython = Join-Path $viewerEnv 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $viewerPython)) {
    Write-Host 'Preparing the sensor viewer environment...'
    python -m venv $viewerEnv
    if ($LASTEXITCODE -ne 0) { throw 'Python environment creation failed.' }
}
& $viewerPython -c "import importlib.util,sys;sys.exit(any(importlib.util.find_spec(m) is None for m in ('paramiko','selenium','PIL','imageio_ffmpeg','av')))"
if ($LASTEXITCODE -ne 0) {
    & $viewerPython -m pip install -r (Join-Path $PSScriptRoot 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Viewer dependency installation failed.' }
}
$viewerArgs = @((Join-Path $PSScriptRoot 'run_viewer.py'), '--host', $Jetson, '--user', $User, '--port', $Port, '--max-points', $MaxPoints, '--hz', $Hz)
& $viewerPython @viewerArgs
