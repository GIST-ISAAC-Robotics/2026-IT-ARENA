# Launch local window; the existing DPAPI password is used only on stdin and SSH.
param([string]$CredentialFile = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml")
$ErrorActionPreference = 'Stop'
$record = Import-Clixml -LiteralPath $CredentialFile
$discovery = (& "$PSScriptRoot/discover_jetson.ps1" -CredentialFile $CredentialFile) | ConvertFrom-Json
if ($LASTEXITCODE -ne 0 -or -not $discovery.ok) { throw 'Jetson not found or host key did not match.' }
$request = @{
    host = $record.HostName; port = $record.Port; user = $record.Credential.UserName
    password = $record.Credential.GetNetworkCredential().Password
    known_hosts = "$env:USERPROFILE\.ssh\known_hosts"; connect_address = $discovery.verified_address
    command = 'umask 077; cd /home/q/it_arena_steering_setup_20261114 && exec /home/q/it_arena_imu_20261111/tools_venv/bin/python -u scripts/steering_panel_remote.py --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00 --runs .'
}
$request | ConvertTo-Json -Compress | & C:/Python314/python.exe "$PSScriptRoot/steering_panel.py"
exit $LASTEXITCODE
