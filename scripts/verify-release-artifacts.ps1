param(
  [Parameter(Mandatory = $true)]
  [string]$ApkZip,

  [Parameter(Mandatory = $true)]
  [string]$ExeZip,

  [Parameter(Mandatory = $false)]
  [string]$AabZip
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

Test-ArtifactArchive -ZipPath $ApkZip -Extension 'apk' -MinimumBytes 1000000
if ($AabZip) {
  Test-ArtifactArchive -ZipPath $AabZip -Extension 'aab' -MinimumBytes 1000000
}
Test-ArtifactArchive -ZipPath $ExeZip -Extension 'exe' -MinimumBytes 1000000
