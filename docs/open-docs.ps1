$ErrorActionPreference = "Stop"

$indexPath = Join-Path -Path $PSScriptRoot -ChildPath "index.html"

if (-not (Test-Path -LiteralPath $indexPath)) {
    throw "index.html not found: $indexPath"
}

Start-Process -FilePath $indexPath
Write-Output "Opened in browser: $indexPath"
