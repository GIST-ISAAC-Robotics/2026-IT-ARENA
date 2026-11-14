$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../../../../..')).Path
$fixture = Get-Content -LiteralPath "$PSScriptRoot/fixture.json" -Raw | ConvertFrom-Json
$code = [Convert]::ToBase64String([IO.File]::ReadAllBytes("$PSScriptRoot/audit_remote.py"))
function Audit-Probe($data, $name) {
    $json = $data | ConvertTo-Json -Depth 8 -Compress
    $encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($json))
    $command = "python3 -B -c 'import base64;exec(compile(base64.b64decode(`"$code`"),`"audit_remote.py`",`"exec`"))' '$encoded'"
    $lines = & "$repo/scripts/jetson_access.ps1" -Sudo -Command $command -Timeout 40
    $exitCode = $LASTEXITCODE
    $lines | Set-Content -LiteralPath "$PSScriptRoot/$name.json" -Encoding utf8
    $exitCode | Set-Content -LiteralPath "$PSScriptRoot/$name.exit"
    return ($lines -join "`n" | ConvertFrom-Json)
}
$before = Audit-Probe @{mode='before';fixture=$fixture} 'before'
if (-not $before.probe_name_absent -or $before.active_uuid -ne $fixture.initial_active_uuid) { throw 'preflight_failed_no_registration' }
$publicPassword = ConvertTo-SecureString -String $fixture.psk -AsPlainText -Force
$receipt = & "$repo/scripts/jetson_wifi.ps1" -Ssid $fixture.ssid -Security $fixture.security -ProfileName $fixture.profile_name -WifiPassword $publicPassword
$operationExit = $LASTEXITCODE
$receipt | Set-Content -LiteralPath "$PSScriptRoot/registration.json" -Encoding utf8
$operationExit | Set-Content -LiteralPath "$PSScriptRoot/registration.exit"
$registration = $receipt -join "`n" | ConvertFrom-Json
$publicPassword = $null
$result = Audit-Probe @{mode='after';fixture=$fixture;before=$before;registration=$registration} 'result'
$result | ConvertTo-Json -Depth 8
if (-not $result.passed) { exit 1 }
