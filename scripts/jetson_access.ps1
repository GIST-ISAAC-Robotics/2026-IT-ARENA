param(
    [ValidateSet('run', 'put', 'get')][string]$Action = 'run',
    [string]$Command,
    [string]$Source,
    [string]$Destination,
    [string]$Log,
    [switch]$Sudo,
    [int]$Timeout = 900,
    [string]$ConnectAddress,
    [string]$InterfaceAlias = 'Wi-Fi',
    [string]$CredentialFile = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml"
)
$ErrorActionPreference = 'Stop'
$record = Import-Clixml -LiteralPath $CredentialFile
if (-not $ConnectAddress -and $record.HostName.EndsWith('.local')) {
    # 일반 DNS의 잘못된 .local 응답을 우회한다. 인증 전 기존 장비 키로 확인한다.
    $discoveryOutput = & "$PSScriptRoot/discover_jetson.ps1" -InterfaceAlias $InterfaceAlias -CredentialFile $CredentialFile
    $discoveryExit = $LASTEXITCODE
    $discovery = ($discoveryOutput -join "`n") | ConvertFrom-Json
    if ($discoveryExit -ne 0 -or -not $discovery.ok) {
        throw "Jetson discovery failed ($($discovery.reason)). Check Wi-Fi/client isolation or use -ConnectAddress with a known current IPv4 address."
    }
    $ConnectAddress = $discovery.verified_address
}
$request = @{
    host = $record.HostName; user = $record.Credential.UserName; port = $record.Port
    password = $record.Credential.GetNetworkCredential().Password
    action = $Action; command = $Command; source = $Source; destination = $Destination
    log = $Log; sudo = [bool]$Sudo; timeout = $Timeout
    known_hosts = "$env:USERPROFILE\.ssh\known_hosts"
    connect_address = $ConnectAddress
}
# 비밀번호는 명령행/파일/출력에 넣지 않고 자식 프로세스의 표준 입력으로만 전달합니다.
$request | ConvertTo-Json -Compress | & C:/Python314/python.exe "$PSScriptRoot/jetson_access.py"
exit $LASTEXITCODE
