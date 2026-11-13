param([switch]$Install)
$ErrorActionPreference = 'Stop'
$cli = Join-Path $env:ProgramFiles 'Tailscale\tailscale.exe'
if (-not (Test-Path -LiteralPath $cli)) {
    if (-not $Install) { throw 'Run this script with -Install to install Tailscale first.' }
    & winget install --id Tailscale.Tailscale --exact --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) { throw 'Tailscale installation failed.' }
}
if (-not (Test-Path -LiteralPath $cli)) { throw 'Tailscale executable is unavailable.' }
& $cli up --unattended=true --timeout=60s
if ($LASTEXITCODE -ne 0) { throw 'Finish Tailscale login and run this script again.' }
& $cli status
Write-Host 'Register the Jetson on the same Tailscale network as kaboat-jetson.'
Write-Host 'Use welcome_CU on the laptop. Keep Kk on the boat with internet enabled.'
Write-Host 'Then start start_starlink.ps1 and click the Starlink connection button.'
