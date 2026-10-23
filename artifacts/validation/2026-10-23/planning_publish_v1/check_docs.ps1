$ErrorActionPreference = 'Stop'
$taskRoot = (Get-Location).Path
$documents = @(
  'docs/PROJECT_CONTEXT.md', 'docs/README.md',
  'docs/reports/DEVELOPMENT_PLAN_2026_10_23.md',
  'docs/reports/DEVELOPMENT_PLAN_2026_10_04.md',
  'docs/autonomy/MULTI_VEHICLE_DESIGN_2026_10_04.md',
  'docs/HARDWARE_AND_RULE_QUESTIONS.md', 'docs/activity/2026-10-23.md',
  'artifacts/validation/2026-10-23/planning_publish_v1/README.md',
  'artifacts/validation/2026-10-23/planning_review_v1/README.md'
)
$failures = [Collections.Generic.List[object]]::new()
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
      $htmlAnchors = @([regex]::Matches((Get-Content -LiteralPath $file -Raw -Encoding utf8), '<a\b[^>]*\bid="([^"]+)"') | ForEach-Object { $_.Groups[1].Value })
      if ([uri]::UnescapeDataString($parts[1]) -notin (@($anchors) + $htmlAnchors)) {
        $failures.Add(@{document=$document; target=$target; reason='missing_heading_anchor'})
      }
    }
  }
}
@{documents=$documents.Count; local_links=$count; failures=@($failures.ToArray()); passed=($failures.Count -eq 0)} | ConvertTo-Json -Depth 5
if ($failures.Count) { exit 1 }
