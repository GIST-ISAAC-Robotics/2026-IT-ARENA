param(
    [string]$InterfaceAlias = 'Wi-Fi',
    [string]$ConnectAddress,
    [string]$CredentialFile = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml"
)
$ErrorActionPreference = 'Stop'
$record = Import-Clixml -LiteralPath $CredentialFile
$sourceArgs = @()
if ($ConnectAddress) {
    $sourceArgs = @('--address', $ConnectAddress)
} else {
    $addresses = @(Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 |
        Where-Object { $_.AddressState -eq 'Preferred' -and $_.IPAddress -notmatch '^(169\.254\.|127\.)' })
    if ($addresses.Count -ne 1) { throw 'Specify a connected interface with one preferred IPv4 address.' }
    $network = "$($addresses[0].IPAddress)/$($addresses[0].PrefixLength)"
    $sourceArgs = @('--interface', $network)
}
# 비밀번호를 추출하거나 전달하지 않는다. 발견 후보의 기존 SSH 호스트 키만 대조한다.
& C:/Python314/python.exe "$PSScriptRoot/discover_jetson.py" --host $record.HostName --port $record.Port @sourceArgs --known-hosts "$env:USERPROFILE/.ssh/known_hosts"
exit $LASTEXITCODE
