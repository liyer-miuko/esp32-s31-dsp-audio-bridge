# Run in an exported ESP-IDF v6.1 environment. Never flash or regenerate anchors.
param([string]$Python = 'python')
$ErrorActionPreference = 'Continue'
if (-not $env:IDF_PATH) { throw 'Export the required IDF environment first.' }
$project = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Push-Location $project
try {
    & $Python (Join-Path $PSScriptRoot 'check_core1_frozen.py')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $Python (Join-Path $env:IDF_PATH 'tools/idf.py') -B build-fork build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $Python (Join-Path $PSScriptRoot 'check_core1_frozen.py') --objects
    exit $LASTEXITCODE
} finally { Pop-Location }
