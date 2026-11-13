param([int]$Port = 8765)
$ErrorActionPreference = 'Stop'
$viewerPython = Join-Path $env:LOCALAPPDATA 'KabotSensorViewer\venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $viewerPython)) {
    $viewerPython = (Get-Command python -ErrorAction Stop).Source
}
& $viewerPython (Join-Path $PSScriptRoot 'run_viewer.py') --host jetson-starlink --key --wait-for-connection --port $Port
if ($LASTEXITCODE -ne 0) { throw 'Starlink dashboard failed to start.' }
