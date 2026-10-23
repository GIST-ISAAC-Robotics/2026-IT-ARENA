$ErrorActionPreference = 'Stop'
$taskFiles = @(@(git -c core.quotepath=false diff HEAD --name-only) + @(git -c core.quotepath=false ls-files --others --exclude-standard) | Sort-Object -Unique)
$taskLarge = [Collections.Generic.List[object]]::new()
$taskBinaries = [Collections.Generic.List[string]]::new()
$taskRaw = [Collections.Generic.List[string]]::new()
[long]$taskBytes = 0
foreach ($taskFile in $taskFiles) {
  if (-not (Test-Path -LiteralPath $taskFile -PathType Leaf)) { continue }
  $taskItem = Get-Item -LiteralPath $taskFile
  $taskBytes += $taskItem.Length
  if ($taskItem.Length -ge 100MB) { $taskLarge.Add(@{path=$taskFile; bytes=$taskItem.Length}) }
  if ($taskFile -match '\.(mcap|db3)$|/(receipt\.jsonl|raw_scans\.jsonl|scans_received\.jsonl)$') { $taskRaw.Add($taskFile) }
  $taskStream = [IO.File]::OpenRead($taskItem.FullName)
  try {
    $taskHeader = [byte[]]::new(4)
    if ($taskStream.Read($taskHeader,0,4) -eq 4 -and $taskHeader[0] -eq 127 -and
        $taskHeader[1] -eq 69 -and $taskHeader[2] -eq 76 -and $taskHeader[3] -eq 70) { $taskBinaries.Add($taskFile) }
  } finally { $taskStream.Dispose() }
}
$taskInventory = Get-Content artifacts/validation/2026-10-23/planning_publish_v1/local_only_manifest.json -Raw | ConvertFrom-Json
$taskExcluded = @($taskInventory.files | Where-Object { $_.path -in $taskFiles })
$taskMissing = @($taskInventory.files | Where-Object { -not (Test-Path -LiteralPath $_.path -PathType Leaf) })
$taskResult = [ordered]@{
  scope='Changed and newly publishable files; size/ELF/raw-path audit, not a complete secret or semantic audit'
  fileCount=$taskFiles.Count; totalMiB=[math]::Round($taskBytes/1MB,2)
  atLeast100MiB=@($taskLarge.ToArray()); elfBinaries=@($taskBinaries.ToArray()); rawSensorPaths=@($taskRaw.ToArray())
  excludedStillPublishable=@($taskExcluded | ForEach-Object { $_.path }); localPreservationMissing=@($taskMissing | ForEach-Object { $_.path })
  localOnlyCount=$taskInventory.files.Count
  localOnlyMiB=[math]::Round(($taskInventory.files | Measure-Object bytes -Sum).Sum/1MB,2)
  passed=($taskLarge.Count -eq 0 -and $taskBinaries.Count -eq 0 -and $taskRaw.Count -eq 0 -and $taskExcluded.Count -eq 0 -and $taskMissing.Count -eq 0)
}
$taskResult | ConvertTo-Json -Depth 5
if (-not $taskResult.passed) { exit 1 }
