# Builds target\package\Driftless-<version>-setup.exe (Windows x64, Inno Setup 6).
#
#   packaging\windows\build-setup.ps1 [-OhneBauen]
#
# -OhneBauen: take the release programs already in target\release (no cargo build).
# The version is the one of [workspace.package] in Cargo.toml. Check the result with
# packaging\windows\check-setup.ps1.
[CmdletBinding()]
param(
    [switch]$OhneBauen
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
# cargo works on the workspace of the current folder; back to the caller's folder at the end.
Push-Location $root
try {
    # The version of [workspace.package] in Cargo.toml.
    function Get-WorkspaceVersion([string]$cargoToml) {
        $inPackage = $false
        foreach ($line in Get-Content -LiteralPath $cargoToml) {
            if ($line -match '^\[') { $inPackage = ($line.Trim() -eq '[workspace.package]') }
            elseif ($inPackage -and $line -match '^version\s*=\s*"([^"]+)"') { return $Matches[1] }
        }
        return $null
    }
    $version = Get-WorkspaceVersion (Join-Path $root 'Cargo.toml')
    if (-not $version -or $version -notmatch '^\d+\.\d+\.\d+') {
        Write-Error 'Keine Version in [workspace.package] von Cargo.toml gefunden.'
    }

    if (-not $OhneBauen) {
        & cargo build --release --locked -p driftless-app -p driftless-cli
        if ($LASTEXITCODE -ne 0) { Write-Error "cargo build ist fehlgeschlagen (Exit $LASTEXITCODE)." }
    }
    foreach ($bin in 'driftless-app.exe', 'driftless.exe') {
        if (-not (Test-Path -LiteralPath (Join-Path $root "target\release\$bin"))) {
            Write-Error "target\release\$bin fehlt (ohne -OhneBauen bauen)."
        }
    }

    # ISCC.exe: on the PATH, else the usual places of Inno Setup 6.
    $iscc = Get-Command 'ISCC.exe' -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Source
    if (-not $iscc) {
        $candidates = @(
            "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
            "$env:ProgramFiles\Inno Setup 6\ISCC.exe",
            "$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe"
        )
        $iscc = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
    }
    if (-not $iscc) {
        Write-Error 'ISCC.exe (Inno Setup 6) nicht gefunden: installieren, z. B. "choco install innosetup".'
    }

    $package = Join-Path $root 'target\package'
    $setup = Join-Path $package "Driftless-$version-setup.exe"
    New-Item -ItemType Directory -Force -Path $package | Out-Null
    if (Test-Path -LiteralPath $setup) { Remove-Item -LiteralPath $setup -Force }

    & $iscc "/DAppVersion=$version" (Join-Path $root 'packaging\windows\driftless.iss')
    if ($LASTEXITCODE -ne 0) { Write-Error "ISCC ist fehlgeschlagen (Exit $LASTEXITCODE)." }
    if (-not (Test-Path -LiteralPath $setup)) { Write-Error "$setup wurde nicht erzeugt." }

    $mb = [math]::Round((Get-Item -LiteralPath $setup).Length / 1MB, 1)
    Write-Output "$setup ($mb MB)"
}
finally {
    Pop-Location
}
