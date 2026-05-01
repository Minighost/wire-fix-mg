# Builds a release zip for the Factorio mod portal.
#
# Reads the version from info.json and produces:
#   dist/wire-fix_<version>.zip
# whose top-level folder inside the zip is `wire-fix_<version>/`,
# which is the layout Factorio requires.
#
# Usage (from repo root):
#   pwsh ./package.ps1
#   powershell -ExecutionPolicy Bypass -File ./package.ps1

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path

$info = Get-Content -Raw -LiteralPath (Join-Path $RootDir 'info.json') | ConvertFrom-Json
$ModName    = $info.name
$Version    = $info.version
$FolderName = "${ModName}_${Version}"

$DistDir = Join-Path $RootDir 'dist'
$Staging = Join-Path $DistDir $FolderName
$ZipPath = Join-Path $DistDir "${FolderName}.zip"

# Whitelist: only these top-level items go into the zip.
$Includes = @(
    'info.json',
    'control.lua',
    'data.lua',
    'settings.lua',
    'LICENSE',
    'README.md',
    'changelog.txt',
    'thumbnail.png',
    'locale',
    'scripts',
    'graphics'
)

# Dev-only files inside copied folders that should not ship.
$ExcludePatterns = @(
    '*.py',             # icon generators in graphics/
    '*.psd', '*.xcf',   # source art
    'config-local.lua', # local dev override picked up by scripts/wire-fix.lua
    'archive'           # any archive subfolders left behind
)

if (Test-Path $DistDir) {
    Remove-Item -Recurse -Force $DistDir
}
New-Item -ItemType Directory -Path $Staging | Out-Null

foreach ($item in $Includes) {
    $src = Join-Path $RootDir $item
    if (-not (Test-Path -LiteralPath $src)) {
        Write-Warning "Skipping missing item: $item"
        continue
    }
    Copy-Item -LiteralPath $src -Destination (Join-Path $Staging $item) -Recurse -Force
}

foreach ($pat in $ExcludePatterns) {
    Get-ChildItem -Path $Staging -Recurse -Force -Include $pat -ErrorAction SilentlyContinue |
        ForEach-Object { Remove-Item -Recurse -Force -LiteralPath $_.FullName }
}

if (Test-Path -LiteralPath $ZipPath) {
    Remove-Item -Force -LiteralPath $ZipPath
}

# Compress-Archive on Windows PowerShell writes entry names with backslashes,
# which the Factorio mod portal rejects ("ZIP archive is using Windows-style
# backslashes"). Build the zip via System.IO.Compression so we can write
# forward-slash entry names directly.
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

$stagingFull = (Resolve-Path -LiteralPath $Staging).Path
$parentFull = (Resolve-Path -LiteralPath (Split-Path -Parent $stagingFull)).Path
$zipStream = [System.IO.File]::Open($ZipPath, [System.IO.FileMode]::Create)
try {
    $archive = New-Object System.IO.Compression.ZipArchive($zipStream, [System.IO.Compression.ZipArchiveMode]::Create)
    try {
        Get-ChildItem -LiteralPath $stagingFull -Recurse -File | ForEach-Object {
            $rel = $_.FullName.Substring($parentFull.Length + 1) -replace '\\', '/'
            [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile(
                $archive, $_.FullName, $rel, [System.IO.Compression.CompressionLevel]::Optimal) | Out-Null
        }
    } finally {
        $archive.Dispose()
    }
} finally {
    $zipStream.Dispose()
}

$zipInfo = Get-Item -LiteralPath $ZipPath
Write-Output ""
Write-Output "Built: $ZipPath"
Write-Output ("Size:  {0:N1} KB" -f ($zipInfo.Length / 1KB))
Write-Output "Top-level folder inside zip: $FolderName"
