param(
    [Parameter(Mandatory=$true)][string]$Runbook,
    [string]$Destination = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml"
)
$ErrorActionPreference = 'Stop'
# 원문은 데이터로만 읽으며 포함된 명령을 실행하지 않습니다.
$body = Get-Content -Raw -Encoding UTF8 -LiteralPath $Runbook
$passwordMatch = [regex]::Match($body, '(?m)^password:\s*([^\r\n]+)')
$userMatch = [regex]::Match($body, '(?m)^user:\s*(\S+)')
$hostMatch = [regex]::Match($body, '(?m)^hostname:\s*(\S+)')
if (-not ($passwordMatch.Success -and $userMatch.Success -and $hostMatch.Success)) {
    throw '런북의 접속 필드를 찾지 못했습니다. 원문/비밀번호는 출력하지 않습니다.'
}
if (Test-Path -LiteralPath $Destination) { throw '기존 접속 저장 파일은 덮어쓰지 않습니다.' }
$directory = Split-Path -Parent $Destination
New-Item -ItemType Directory -Force -Path $directory | Out-Null
$sid = [Security.Principal.WindowsIdentity]::GetCurrent().User.Value
& icacls.exe $directory /inheritance:r /grant:r "*${sid}:(OI)(CI)F" '*S-1-5-18:(OI)(CI)F' | Out-Null
if ($LASTEXITCODE -ne 0) { throw '비공개 디렉터리 ACL 설정 실패' }
$secure = ConvertTo-SecureString $passwordMatch.Groups[1].Value.Trim() -AsPlainText -Force
$credential = [PSCredential]::new($userMatch.Groups[1].Value, $secure)
$record = [pscustomobject]@{
    HostName = $hostMatch.Groups[1].Value + '.local'
    Credential = $credential
    Port = 22
    Source = (Resolve-Path -LiteralPath $Runbook).Path
    SavedAt = (Get-Date).ToString('o')
}
# Windows DPAPI: 같은 Windows 사용자/컴퓨터에서만 복호화. 평문 비밀번호 파일 없음.
$record | Export-Clixml -LiteralPath $Destination
$reloaded = Import-Clixml -LiteralPath $Destination
if ($reloaded.Credential.UserName -ne $credential.UserName -or
    $reloaded.Credential.GetNetworkCredential().Password -cne $credential.GetNetworkCredential().Password) {
    throw '암호화 저장 재검증 실패'
}
[pscustomobject]@{Saved=$Destination;HostName=$record.HostName;User=$credential.UserName;Protection='Windows DPAPI + private ACL';Verified=$true} | ConvertTo-Json
