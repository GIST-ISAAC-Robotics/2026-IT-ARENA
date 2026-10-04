$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../../../..')).Path
Set-Location -LiteralPath $repo
Add-Type -AssemblyName System.IO.Compression.FileSystem

function Assert-G0($condition, $message) {
    if (-not $condition) { throw $message }
}
function Read-G0Json($relative) {
    Get-Content -LiteralPath (Join-Path $repo $relative) -Raw | ConvertFrom-Json
}
function Read-G0ZipHashes($relative) {
    $archive = [IO.Compression.ZipFile]::OpenRead((Join-Path $repo $relative))
    $hashes = [ordered]@{}
    try {
        foreach ($entry in $archive.Entries) {
            $stream = $entry.Open()
            $digest = [Security.Cryptography.SHA256]::Create()
            try { $hashes[$entry.FullName] = [Convert]::ToHexString($digest.ComputeHash($stream)).ToLowerInvariant() }
            finally { $stream.Dispose(); $digest.Dispose() }
        }
    } finally { $archive.Dispose() }
    return $hashes
}

$batch = 'artifacts/validation/2026-10-05/multi_vehicle_g0_gazebo_all_v1'
$finish = 'artifacts/validation/2026-10-05/multi_vehicle_g0_finish_v1'
$sourceHashes = Read-G0ZipHashes "$batch/source_snapshot.zip"
foreach ($name in $sourceHashes.Keys) {
    Assert-G0 ((Get-FileHash -LiteralPath $name -Algorithm SHA256).Hash.ToLowerInvariant() -eq $sourceHashes[$name]) "Current source drift: $name"
}
$baseline = Read-G0Json 'artifacts/validation/2026-10-05/g0_baseline/sha256_manifest.json'
$baselineCode = @($baseline.files | Where-Object { $_.path -match '^(src|scripts|tests|config)/' })
foreach ($entry in $baselineCode) {
    Assert-G0 ((Get-FileHash -LiteralPath $entry.path -Algorithm SHA256).Hash.ToLowerInvariant() -eq $entry.sha256) "Pre-G0 code changed: $($entry.path)"
}

$firstCases = @('empty_r0_corridor', 'central_stop', 'edge_stop', 'moving_lead')
$remainingCases = @('stop_restart', 'low_target', 'rear_mask', 'short_branch_wall')
$rows = @()
foreach ($caseName in ($firstCases + $remainingCases)) {
    if ($firstCases -contains $caseName) {
        $casePath = "$batch/$caseName/report.json"
        $rootPath = $null
        $zipPath = "$batch/source_snapshot.zip"
    } else {
        $rootPath = "$finish/$caseName/report.json"
        $rootReport = Read-G0Json $rootPath
        Assert-G0 ($rootReport.passed -eq $true) "Top-level failed: $caseName"
        Assert-G0 ($rootReport.drive_output_topics.Count -eq 0) "Drive output in $caseName"
        Assert-G0 ($rootReport.r0_baseline.unchanged -eq $true) "R0 changed in $caseName"
        $casePath = "$finish/$caseName/$caseName/report.json"
        $zipPath = "$finish/$caseName/source_snapshot.zip"
    }
    $runHashes = Read-G0ZipHashes $zipPath
    Assert-G0 ($runHashes.Count -eq $sourceHashes.Count) "Snapshot size changed: $caseName"
    foreach ($name in $sourceHashes.Keys) { Assert-G0 ($runHashes[$name] -eq $sourceHashes[$name]) "Snapshot drift: $caseName $name" }
    $report = Read-G0Json $casePath
    Assert-G0 ($report.case -eq $caseName) 'Wrong case label'
    Assert-G0 ($report.rendered_observation_passed -eq $true) "Observation failed: $caseName"
    Assert-G0 ($report.failures.Count -eq 0) "Recorded failures: $caseName"
    foreach ($check in $report.checks.PSObject.Properties) { Assert-G0 ($check.Value -eq $true) "Failed check: $caseName $($check.Name)" }
    Assert-G0 ($report.shutdown.clean -eq $true) "Unclean shutdown: $caseName"
    Assert-G0 ($report.environment.actual_renderer_verified -eq $true) "Renderer unverified: $caseName"
    Assert-G0 ($report.vehicle_stop_verified -eq $false -and $report.collision_safety_verified -eq $false -and $report.swept_curve_verified -eq $false) "Scope overclaim: $caseName"
    foreach ($artifact in $report.raw_artifacts.PSObject.Properties) {
        $raw = $artifact.Value
        Assert-G0 ((Get-Item -LiteralPath $raw.relative_path).Length -eq $raw.bytes) "Raw size drift: $caseName $($artifact.Name)"
        Assert-G0 ((Get-FileHash -LiteralPath $raw.relative_path -Algorithm SHA256).Hash.ToLowerInvariant() -eq $raw.sha256) "Raw hash drift: $caseName $($artifact.Name)"
        $null = git check-ignore -- $raw.relative_path
        Assert-G0 ($LASTEXITCODE -eq 0) "Raw evidence not ignored: $($raw.relative_path)"
    }
    $rows += [pscustomobject][ordered]@{
        case = $caseName
        report = $casePath
        report_sha256 = (Get-FileHash -LiteralPath $casePath -Algorithm SHA256).Hash.ToLowerInvariant()
        top_level_report = $rootPath
        source_snapshot = $zipPath
        rendered_observation_passed = $report.rendered_observation_passed
        checks_passed = @($report.checks.PSObject.Properties).Count
        raw_frames = $report.raw_frame_count
        sequential_scans = $report.sequential_scan_count
        active_scans = $report.active_scan_count
        raw_rate_sim_hz = $report.raw_rate_sim_hz
        scan_rate_sim_hz = $report.sequential_rate_sim_hz
        max_capture_time_error_s = $report.max_capture_time_error_s
        visible_target_rays_min = $report.observation_summary.visible_target_rays_min
        visible_target_rays_max = $report.observation_summary.visible_target_rays_max
        dynamic_phases_required = $report.dynamic_phases.required
        dynamic_phases_verified = $report.dynamic_phase_observation_verified
        clean_shutdown = $report.shutdown.clean
        renderer_verified = $report.environment.actual_renderer_verified
        raw_hashes_and_ignore_verified = $true
    }
}
$pytestLog = 'artifacts/validation/2026-10-05/g0_unit_v7/pytest.log'
Assert-G0 ((Get-Content -LiteralPath $pytestLog -Raw) -match '239 passed') 'Missing regression result'
$collection = [ordered]@{
    schema_version = 1
    collected_at = (Get-Date).ToString('o')
    scope = 'G0-a fixed-ego observation and cross-section geometry only; no driving or vehicle safety validation'
    collection_passed = $true
    completed_case_count = $rows.Count
    interrupted_batch = "$batch/INTERRUPTED.md"
    interruption_note = 'First four completed reports preserved. Original stop_restart attempt interrupted without a verdict; remaining four run independently. Not a claim that the original eight-case batch completed.'
    consistent_source_file_count = $sourceHashes.Count
    source_sha256 = $sourceHashes
    pre_g0_code_files_unchanged = $baselineCode.Count
    pytest_log = $pytestLog
    pytest_log_sha256 = (Get-FileHash -LiteralPath $pytestLog -Algorithm SHA256).Hash.ToLowerInvariant()
    cases = $rows
    vehicle_stop_verified = $false
    collision_safety_verified = $false
    full_course_verified = $false
}
$outputPath = Join-Path $PSScriptRoot 'collection_report.json'
$collection | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $outputPath -Encoding utf8
$rows | Select-Object case,raw_frames,sequential_scans,visible_target_rays_min,visible_target_rays_max,dynamic_phases_verified,clean_shutdown | Format-Table -AutoSize
Write-Output "COLLECTION_OK cases=$($rows.Count) source_files=$($sourceHashes.Count) preserved_code_files=$($baselineCode.Count)"
