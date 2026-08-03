param(
  [Parameter(Mandatory = $false)]
  [string]$ApkZip,

  [Parameter(Mandatory = $false)]
  [string]$ExeZip,

  [Parameter(Mandatory = $false)]
  [string]$AabZip,

  [Parameter(Mandatory = $false)]
  [string]$DistDirectory,

  [Parameter(Mandatory = $false)]
  [string]$Sha256Sums,

  [Parameter(Mandatory = $false)]
  [string]$QmsReleaseRecord
)

$ErrorActionPreference = 'Stop'

function Test-ArtifactArchive {
  param(
    [Parameter(Mandatory = $true)][string]$ZipPath,
    [Parameter(Mandatory = $true)][string]$Extension,
    [Parameter(Mandatory = $true)][int64]$MinimumBytes
  )

  if (-not (Test-Path -LiteralPath $ZipPath)) {
    throw "Artifact ZIP not found: $ZipPath"
  }

  $temp = Join-Path ([IO.Path]::GetTempPath()) ("rvt-artifact-" + [Guid]::NewGuid().ToString("N"))
  New-Item -ItemType Directory -Path $temp | Out-Null
  try {
    Expand-Archive -LiteralPath $ZipPath -DestinationPath $temp -Force
    $files = Get-ChildItem -LiteralPath $temp -Recurse -File -Filter "*.$Extension"
    if ($files.Count -lt 1) {
      throw "No .$Extension file found inside $ZipPath"
    }
    foreach ($file in $files) {
      if ($file.Length -lt $MinimumBytes) {
        throw "$($file.Name) is too small: $($file.Length) bytes"
      }
      $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $file.FullName
      [PSCustomObject]@{
        Kind = $Extension.ToUpperInvariant()
        File = $file.Name
        Bytes = $file.Length
        SHA256 = $hash.Hash.ToLowerInvariant()
      }
    }
  }
  finally {
    Remove-Item -LiteralPath $temp -Recurse -Force -ErrorAction SilentlyContinue
  }
}

if ($ApkZip -or $ExeZip -or $AabZip) {
  if (-not $ApkZip -or -not $ExeZip) {
    throw 'ApkZip and ExeZip are both required for archive validation.'
  }
  Test-ArtifactArchive -ZipPath $ApkZip -Extension 'apk' -MinimumBytes 1000000
  if ($AabZip) {
    Test-ArtifactArchive -ZipPath $AabZip -Extension 'aab' -MinimumBytes 1000000
  }
  Test-ArtifactArchive -ZipPath $ExeZip -Extension 'exe' -MinimumBytes 1000000
}

if ($DistDirectory -or $Sha256Sums -or $QmsReleaseRecord) {
  if (-not $DistDirectory) {
    throw 'DistDirectory is required when validating QMS release evidence.'
  }
  $dist = (Resolve-Path -LiteralPath $DistDirectory).Path
  $sumsPath = if ($Sha256Sums) { $Sha256Sums } else { Join-Path $dist 'SHA256SUMS' }
  $recordPath = if ($QmsReleaseRecord) { $QmsReleaseRecord } else { Join-Path $dist 'qms-release-record.json' }
  if (-not (Test-Path -LiteralPath $sumsPath)) { throw "SHA256SUMS not found: $sumsPath" }
  if (-not (Test-Path -LiteralPath $recordPath)) { throw "QMS release record not found: $recordPath" }

  $schemaVerifier = Join-Path $PSScriptRoot 'generate-qms-release-record.mjs'
  & node $schemaVerifier --verify-record $recordPath
  if ($LASTEXITCODE -ne 0) {
    throw "QMS release record failed full JSON Schema validation."
  }

  $record = Get-Content -LiteralPath $recordPath -Raw | ConvertFrom-Json
  if ($record.schema_version -ne 'rvt-qms-release-record-v1') {
    throw "Unexpected QMS release record schema: $($record.schema_version)"
  }
  $tagProductVersion = ($record.release_tag -replace '^v', '') -replace '[-+].*$', ''
  if ($tagProductVersion -ne $record.product_version) {
    throw "Release tag product version $tagProductVersion does not match $($record.product_version)"
  }

  $artifactNames = [Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
  foreach ($artifact in $record.artifacts) {
    $artifactName = [string]$artifact.name
    if ([string]::IsNullOrWhiteSpace($artifactName) -or
        $artifactName -ne [IO.Path]::GetFileName($artifactName) -or
        $artifactName -in @('.', '..') -or
        $artifactName.Contains('/') -or
        $artifactName.Contains('\')) {
      throw "Recorded artifact name must be a contained basename: $artifactName"
    }
    if (-not $artifactNames.Add($artifactName)) {
      throw "Duplicate recorded release artifact: $artifactName"
    }
    $artifactPath = Join-Path $dist $artifactName
    $resolvedArtifactPath = [IO.Path]::GetFullPath($artifactPath)
    $distPrefix = $dist.TrimEnd([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
    if (-not $resolvedArtifactPath.StartsWith($distPrefix, [StringComparison]::OrdinalIgnoreCase)) {
      throw "Recorded artifact path escapes DistDirectory: $artifactName"
    }
    if (-not (Test-Path -LiteralPath $artifactPath -PathType Leaf)) {
      throw "Recorded release artifact not found: $artifactName"
    }
    $file = Get-Item -LiteralPath $artifactPath
    if ([int64]$artifact.size_bytes -ne $file.Length) {
      throw "Recorded byte length mismatch for $artifactName"
    }
    $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $artifactPath).Hash.ToLowerInvariant()
    if ($actualHash -ne [string]$artifact.sha256) {
      throw "Recorded SHA-256 mismatch for $artifactName"
    }
    if (-not $artifact.signing_state) {
      throw "Signature state missing for $artifactName"
    }
  }

  $checksumEntries = @{}
  foreach ($line in Get-Content -LiteralPath $sumsPath) {
    if (-not $line.Trim()) { continue }
    if ($line -notmatch '^([0-9a-fA-F]{64})  (.+)$') {
      throw "Invalid SHA256SUMS line: $line"
    }
    $checksumName = $Matches[2]
    if ([string]::IsNullOrWhiteSpace($checksumName) -or
        $checksumName -ne [IO.Path]::GetFileName($checksumName) -or
        $checksumName -in @('.', '..') -or
        $checksumName.Contains('/') -or
        $checksumName.Contains('\')) {
      throw "SHA256SUMS entry must be a contained basename: $checksumName"
    }
    if ($checksumEntries.ContainsKey($checksumName)) {
      throw "Duplicate SHA256SUMS entry: $checksumName"
    }
    $checksumEntries[$checksumName] = $Matches[1].ToLowerInvariant()
  }
  $expectedChecksumNames = [Collections.Generic.HashSet[string]]::new($artifactNames, [StringComparer]::Ordinal)
  [void]$expectedChecksumNames.Add('qms-release-record.json')
  $missingChecksumNames = @($expectedChecksumNames | Where-Object { -not $checksumEntries.ContainsKey($_) })
  $unexpectedChecksumNames = @($checksumEntries.Keys | Where-Object { -not $expectedChecksumNames.Contains($_) })
  if ($missingChecksumNames.Count -or $unexpectedChecksumNames.Count) {
    throw "SHA256SUMS coverage mismatch. Missing: $($missingChecksumNames -join ', '); unexpected: $($unexpectedChecksumNames -join ', ')"
  }
  foreach ($entry in $checksumEntries.GetEnumerator()) {
    $entryPath = Join-Path $dist $entry.Key
    if (-not (Test-Path -LiteralPath $entryPath -PathType Leaf)) {
      throw "SHA256SUMS file not found: $($entry.Key)"
    }
    $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $entryPath).Hash.ToLowerInvariant()
    if ($actualHash -ne $entry.Value) {
      throw "SHA256SUMS mismatch for $($entry.Key)"
    }
  }

  [PSCustomObject]@{
    Kind = 'QMS'
    File = [IO.Path]::GetFileName($recordPath)
    Bytes = (Get-Item -LiteralPath $recordPath).Length
    SHA256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $recordPath).Hash.ToLowerInvariant()
  }
}
