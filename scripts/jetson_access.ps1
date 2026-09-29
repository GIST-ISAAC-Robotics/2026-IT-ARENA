param(
    [ValidateSet('run', 'put', 'get')][string]$Action = 'run',
    [string]$Command,
    [string]$Source,
    [string]$Destination,
    [string]$Log,
    [switch]$Sudo,
    [int]$Timeout = 900,
    [string]$ConnectAddress,
    [string]$CredentialFile = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml"
)
$ErrorActionPreference = 'Stop'
$record = Import-Clixml -LiteralPath $CredentialFile
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
