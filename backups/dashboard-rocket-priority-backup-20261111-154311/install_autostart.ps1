param([switch]$NoStart)
$ErrorActionPreference = 'Stop'
$viewerRoot = Join-Path $env:LOCALAPPDATA 'KabotSensorViewer'
$viewerPython = Join-Path $viewerRoot 'venv\Scripts\pythonw.exe'
$viewerScript = Join-Path $PSScriptRoot 'run_viewer.py'
$viewerLog = Join-Path $viewerRoot 'background.log'
if (-not (Test-Path -LiteralPath $viewerPython)) { throw 'Run start_viewer.ps1 once to prepare the environment.' }
# SSH public-key authentication must already be configured for the jetson alias.
$viewerArguments = "`"$viewerScript`" --host jetson --key --wait-for-sensors --log-file `"$viewerLog`""
$runKey = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Run'
New-Item -Path $runKey -Force | Out-Null
New-ItemProperty -Path $runKey -Name 'KabotSensorViewer' -PropertyType String -Value "`"$viewerPython`" $viewerArguments" -Force | Out-Null
if (-not $NoStart) {
    Start-Process -FilePath $viewerPython -ArgumentList $viewerArguments -WindowStyle Hidden
}
Write-Host 'Dashboard autostart configured. Bringup opens or focuses the dashboard.'
