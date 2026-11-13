<# 바탕화면/터미널용 접속기. 현재 주소를 찾고 기존 SSH 키가 일치할 때 접속한다. #>
param(
    [string]$InterfaceAlias = 'Wi-Fi',
    [string]$ConnectAddress,
    [switch]$ProbeOnly,
    [string]$CredentialFile = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml"
)
$ErrorActionPreference = 'Stop'
$record = Import-Clixml -LiteralPath $CredentialFile
$discoveryOutput = & "$PSScriptRoot/discover_jetson.ps1" -InterfaceAlias $InterfaceAlias -ConnectAddress $ConnectAddress -CredentialFile $CredentialFile
$discoveryExit = $LASTEXITCODE
$discovery = ($discoveryOutput -join "`n") | ConvertFrom-Json
if ($discoveryExit -ne 0 -or -not $discovery.ok) {
    throw "ISAAC Jetson not found or its SSH key did not match ($($discovery.reason)). Check that both devices use the same Wi-Fi and client isolation is disabled. For a known IPv4 address, retry with -ConnectAddress."
}
if ($ProbeOnly) {
    $discovery | ConvertTo-Json -Depth 4
    exit 0
}
Write-Host "Verified ISAAC Jetson: $($discovery.verified_address). Enter the usual SSH password if prompted."
# 비밀번호를 읽거나 명령행에 넣지 않는다. SSH 자체 검증도 다시 수행한다.
& ssh -o "HostKeyAlias=$($record.HostName)" -o StrictHostKeyChecking=yes -o ConnectTimeout=5 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -p $record.Port "$($record.Credential.UserName)@$($discovery.verified_address)"
exit $LASTEXITCODE
