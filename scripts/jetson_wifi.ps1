<# 기본은 저장만. 전환 후 새 망에서 ConfirmTransition을 명시해야 복귀를 취소한다. #>
[CmdletBinding(DefaultParameterSetName='Register')]
param(
    [Parameter(Mandatory,ParameterSetName='Register')][string]$Ssid,
    [Parameter(Mandatory,ParameterSetName='Register')][ValidateSet('WPA2','WPA3')][string]$Security,
    [Parameter(ParameterSetName='Register')][string]$ProfileName = 'IT-ARENA Venue',
    [Parameter(ParameterSetName='Register')][Security.SecureString]$WifiPassword,
    [Parameter(Mandatory,ParameterSetName='Select')][guid]$ConnectionUuid,
    [Parameter(ParameterSetName='Register')][Parameter(ParameterSetName='Select')][switch]$Activate,
    [Parameter(Mandatory,ParameterSetName='Confirm')][switch]$ConfirmTransition,
    [Parameter(Mandatory,ParameterSetName='Confirm')][ValidatePattern('^[0-9a-f]{32}$')][string]$TransitionId,
    [ValidateRange(60,600)][int]$RollbackSeconds = 180,
    [ValidatePattern('^[A-Za-z0-9_.-]{1,32}$')][string]$JetsonInterface = 'wlP1p1s0',
    [string]$InterfaceAlias = 'Wi-Fi',
    [string]$ConnectAddress,
    [string]$RepositoryRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$CredentialFile = "$env:LOCALAPPDATA\IT-Arena\private\jetson-access.clixml",
    [string]$PythonExe = 'C:/Python314/python.exe'
)
$ErrorActionPreference = 'Stop'
if ($PSCmdlet.ParameterSetName -eq 'Confirm' -and -not $ConfirmTransition) {
    throw '새 네트워크 확인 후 -ConfirmTransition을 명시하십시오.'
}
if ($PSCmdlet.ParameterSetName -eq 'Select' -and -not $Activate) {
    throw '기존 UUID 선택은 -Activate와 함께 사용하십시오. 프로필 저장만 하려면 -Ssid를 사용하십시오.'
}
$repo = (Resolve-Path -LiteralPath $RepositoryRoot).Path
# 기존 mDNS/호스트 키 검증을 재사용한다. 명시 주소도 키 검증을 생략하지 않는다.
$discoveryOutput = & "$repo/scripts/discover_jetson.ps1" -InterfaceAlias $InterfaceAlias -ConnectAddress $ConnectAddress -CredentialFile $CredentialFile
$discoveryExit = $LASTEXITCODE
$discovery = ($discoveryOutput -join "`n") | ConvertFrom-Json
if ($discoveryExit -ne 0 -or -not $discovery.ok) { throw 'Jetson 발견/기존 호스트 키 확인 실패. 설정은 변경하지 않았습니다.' }
$record = Import-Clixml -LiteralPath $CredentialFile
$wifi = @{ interface=$JetsonInterface; activate=[bool]$Activate; rollback_seconds=$RollbackSeconds }
$secretPointer = [IntPtr]::Zero
try {
    if ($PSCmdlet.ParameterSetName -eq 'Confirm') {
        $wifi.action='confirm'; $wifi.transition_id=$TransitionId; $wifi.user_confirmed=$true
    } elseif ($PSCmdlet.ParameterSetName -eq 'Select') {
        $wifi.action='select'; $wifi.connection_uuid=$ConnectionUuid.ToString()
    } else {
        if (-not $WifiPassword) { $WifiPassword = Read-Host '대회장 Wi-Fi 암호' -AsSecureString }
        $secretPointer = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($WifiPassword)
        $wifi.action='register'; $wifi.ssid=$Ssid; $wifi.security=$Security; $wifi.profile_name=$ProfileName
        $wifi.psk=[Runtime.InteropServices.Marshal]::PtrToStringBSTR($secretPointer)
    }
    if ($Activate) {
        $wifi.transition_id=[guid]::NewGuid().ToString('N')
        Write-Host "전환 ID: $($wifi.transition_id). 새 망에서 확인하기 전 복귀 타이머를 취소하지 않습니다."
    }
    $request = @{
        host=$record.HostName; user=$record.Credential.UserName; port=$record.Port
        password=$record.Credential.GetNetworkCredential().Password
        known_hosts="$env:USERPROFILE\.ssh\known_hosts"; connect_address=$discovery.verified_address
        wifi=$wifi; repository_root=$repo
    }
    # 평문은 자식 stdin과 SSH stdin에만 존재한다. argv/환경변수/로컬 파일로 보내지 않는다.
    $request | ConvertTo-Json -Depth 5 -Compress | & $PythonExe "$PSScriptRoot/jetson_wifi.py"
    $operationExit = $LASTEXITCODE
} finally {
    if ($secretPointer -ne [IntPtr]::Zero) { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($secretPointer) }
    if ($wifi) { $wifi.Remove('psk') }
    $request=$null
}
exit $operationExit
