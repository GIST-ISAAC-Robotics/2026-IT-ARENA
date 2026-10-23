$ErrorActionPreference = 'Stop'
$taskRoot = (Get-Location).Path
$documents = @('docs/PROJECT_CONTEXT.md', 'docs/README.md',
  'docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md',
  'docs/hardware/SENSOR_PROBE_RUNBOOK.md', 'docs/sensors/B0183_INPUT_HANDOFF_2026_10_23.md',
  'docs/sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md', 'docs/reports/DEVELOPMENT_PLAN_2026_10_04.md')
$failures = [System.Collections.Generic.List[object]]::new()
$count = 0
foreach ($document in $documents) {
  $path = Join-Path $taskRoot $document
  $contents = Get-Content -LiteralPath $path -Raw -Encoding utf8
  foreach ($link in [regex]::Matches($contents, '\[[^\]]+\]\(([^)]+)\)')) {
    $target = $link.Groups[1].Value.Trim('<','>')
    if ($target -match '^[a-zA-Z]+:' -or $target.StartsWith('//')) { continue }
    $parts = $target -split '#', 2
    $file = if ($parts[0]) { [IO.Path]::GetFullPath((Join-Path (Split-Path $path) ([uri]::UnescapeDataString($parts[0])))) } else { $path }
    $count++
    if (-not (Test-Path -LiteralPath $file)) {
      $failures.Add(@{document=$document; target=$target; reason='missing_file'})
      continue
    }
    if ($parts.Count -gt 1 -and [IO.Path]::GetExtension($file) -eq '.md') {
      $seen = @{}
      $anchors = foreach ($line in Get-Content -LiteralPath $file -Encoding utf8) {
        if ($line -match '^#{1,6}\s+(.+)$') {
          $slug = [regex]::Replace($Matches[1].ToLowerInvariant(), '[^\p{L}\p{M}\p{N}_ -]', '').Replace(' ', '-')
          if ($seen.ContainsKey($slug)) { $seen[$slug]++; "$slug-$($seen[$slug])" }
          else { $seen[$slug] = 0; $slug }
        }
      }
      if ([uri]::UnescapeDataString($parts[1]) -notin $anchors) {
        $failures.Add(@{document=$document; target=$target; reason='missing_heading_anchor'})
      }
    }
  }
}
$result = @{documents=$documents.Count; local_links=$count; failures=@($failures.ToArray()); passed=($failures.Count -eq 0)}
$result | ConvertTo-Json -Depth 5
if ($failures.Count) { exit 1 }
