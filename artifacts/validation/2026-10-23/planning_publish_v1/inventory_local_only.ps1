$ErrorActionPreference = 'Stop'
$taskCandidates = @(git -c core.quotepath=false ls-files --others --exclude-standard)
$taskRows = foreach ($taskFile in $taskCandidates) {
  if ($taskFile -notmatch '^artifacts/validation/2026-10-(15|16|17|19|21|22|23)/') { continue }
  $taskReason = if ($taskFile -match '/[^/]*source[^/]*\.(zip|tar\.gz)$') { 'local_source_archive' }
    elseif ($taskFile -match '/(?:\w*_driver|driver|repro_\w+|receiver_ready|receiver_stop)$') { 'generated_native_binary' }
    elseif ($taskFile -match '/batched_[^/]+\.obj$') { 'generated_scene_mesh' }
    else { $null }
  if ($null -eq $taskReason) { continue }
  $taskItem = Get-Item -LiteralPath $taskFile
  [ordered]@{path=$taskFile; bytes=$taskItem.Length; sha256=(Get-FileHash -LiteralPath $taskFile -Algorithm SHA256).Hash.ToLowerInvariant(); reason=$taskReason}
}
$taskOutput = 'artifacts/validation/2026-10-23/planning_publish_v1/local_only_manifest.json'
if (Test-Path -LiteralPath $taskOutput) { throw 'Refusing to overwrite preservation inventory' }
[ordered]@{purpose='Publication exclusion inventory; all files retained locally, no deletion'; files=@($taskRows)} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $taskOutput -Encoding utf8
[ordered]@{files=@($taskRows).Count; totalMiB=[math]::Round(($taskRows | ForEach-Object { $_.bytes } | Measure-Object -Sum).Sum/1MB,2); manifest=$taskOutput} | ConvertTo-Json
