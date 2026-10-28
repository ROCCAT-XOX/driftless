# Checks a setup of build-setup.ps1: size, silent install for the current user, the installed
# files, that both programs bring their C runtime with them, `--version` of both programs, the
# Start menu entry, silent uninstall. Exit != 0 on any fault. Leaves nothing installed behind
# when it passes.
#
#   packaging\windows\check-setup.ps1 target\package\Driftless-<version>-setup.exe
#
# Installs into $env:RUNNER_TEMP\Driftless-Test (else the temp folder of the user).
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Setup
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
# Get-PeImports
. (Join-Path $PSScriptRoot 'pe-imports.ps1')

if (-not $Setup -or -not (Test-Path -LiteralPath $Setup -PathType Leaf)) {
    [Console]::Error.WriteLine('Aufruf: check-setup.ps1 <Driftless-…-setup.exe>')
    exit 2
}
$Setup = (Resolve-Path -LiteralPath $Setup).Path

$base = if ($env:RUNNER_TEMP) { $env:RUNNER_TEMP } else { [IO.Path]::GetTempPath() }
$dir = Join-Path $base 'Driftless-Test'
$installLog = Join-Path $base 'driftless-setup.log'
$uninstallLog = Join-Path $base 'driftless-uninstall.log'
$shortcut = Join-Path ([Environment]::GetFolderPath('Programs')) 'Driftless.lnk'

function Show-Log([string]$path) {
    if (Test-Path -LiteralPath $path) {
        Write-Host "--- $path"
        Get-Content -LiteralPath $path | Write-Host
        Write-Host '---'
    }
}
function Fail([string]$message) {
    Show-Log $installLog
    Show-Log $uninstallLog
    [Console]::Error.WriteLine("FEHLER: $message")
    exit 1
}
function Ok([string]$message) {
    Write-Host "ok: $message"
}

# Runs a program (console or GUI) with its output caught; returns the trimmed output.
function Get-ProgramOutput([string]$exe, [string[]]$arguments) {
    $info = New-Object System.Diagnostics.ProcessStartInfo
    $info.FileName = $exe
    # Plain arguments only (no spaces): .Arguments works in Windows PowerShell 5.1 too.
    $info.Arguments = $arguments -join ' '
    $info.UseShellExecute = $false
    $info.RedirectStandardOutput = $true
    $info.RedirectStandardError = $true
    $info.CreateNoWindow = $true
    $process = [System.Diagnostics.Process]::Start($info)
    $stdout = $process.StandardOutput.ReadToEndAsync()
    $stderr = $process.StandardError.ReadToEndAsync()
    if (-not $process.WaitForExit(60000)) {
        $process.Kill()
        Fail "$exe $($arguments -join ' ') hat nach 60 s nicht geendet."
    }
    if ($process.ExitCode -ne 0) {
        Fail "$exe $($arguments -join ' '): Exit $($process.ExitCode): $($stderr.Result)"
    }
    return $stdout.Result.Trim()
}

# The version of [workspace.package] in Cargo.toml.
function Get-WorkspaceVersion([string]$cargoToml) {
    $inPackage = $false
    foreach ($line in Get-Content -LiteralPath $cargoToml) {
        if ($line -match '^\[') { $inPackage = ($line.Trim() -eq '[workspace.package]') }
        elseif ($inPackage -and $line -match '^version\s*=\s*"([^"]+)"') { return $Matches[1] }
    }
    return $null
}
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$version = Get-WorkspaceVersion (Join-Path $root 'Cargo.toml')
if (-not $version) { Fail 'keine Version in Cargo.toml' }

# 4. (first, it is cheap) the size.
$size = (Get-Item -LiteralPath $Setup).Length
$mb = [math]::Round($size / 1MB, 1)
if ($size -ge 500MB) { Fail "$Setup ist $mb MB groß (Grenze 500 MB)." }
Ok "Größe $mb MB ($size Byte) < 500 MB"

if (Test-Path -LiteralPath $shortcut) {
    Fail "$shortcut gibt es schon (ist Driftless installiert?). Erst deinstallieren."
}
if (Test-Path -LiteralPath $dir) { Remove-Item -LiteralPath $dir -Recurse -Force }
foreach ($log in $installLog, $uninstallLog) {
    if (Test-Path -LiteralPath $log) { Remove-Item -LiteralPath $log -Force }
}

# 1. Silent install for the current user (setup.exe waits for its second stage).
$arguments = @('/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', '/CURRENTUSER',
    "/DIR=`"$dir`"", "/LOG=`"$installLog`"")
$install = Start-Process -FilePath $Setup -ArgumentList $arguments -Wait -PassThru
if ($install.ExitCode -ne 0) { Fail "Stille Installation: Exit $($install.ExitCode)" }
Ok "stille Installation nach $dir"

# 2. The files, both programs and the Start menu entry.
$expected = 'driftless-app.exe', 'driftless.exe', 'driftless.ico', 'LICENSE-APACHE',
    'LICENSE-MIT', 'unins000.exe'
foreach ($name in $expected) {
    if (-not (Test-Path -LiteralPath (Join-Path $dir $name) -PathType Leaf)) {
        Fail "$name fehlt in $dir"
    }
}
Ok "Dateien: $($expected -join ', ')"

$cli = Join-Path $dir 'driftless.exe'
$app = Join-Path $dir 'driftless-app.exe'
# The C runtime is linked in (.cargo/config.toml): the Visual C++ runtime is not part of Windows,
# and this runner has it installed, so only the import tables can show the need for it.
foreach ($exe in $cli, $app) {
    $runtime = @(Get-PeImports $exe | Where-Object { $_ -match '^(vcruntime|msvcp|api-ms-win-crt-)' })
    if ($runtime.Count -gt 0) {
        Fail "$(Split-Path -Leaf $exe) braucht die Visual-C++-Laufzeit ($($runtime -join ', ')): target-feature=+crt-static in .cargo/config.toml?"
    }
}
Ok 'C-Laufzeit eingebunden: kein VCRUNTIME*, MSVCP* oder api-ms-win-crt-* in driftless.exe und driftless-app.exe'

$cliVersion = Get-ProgramOutput $cli @('--version')
if ($cliVersion -notmatch "^driftless $([regex]::Escape($version)) \(") {
    Fail "driftless.exe --version: `"$cliVersion`", erwartet `"driftless $version (…)`""
}
Ok "driftless.exe --version: $cliVersion"
# A GUI program in release builds; with its output caught it still answers on stdout.
$appVersion = Get-ProgramOutput $app @('--version')
if ($appVersion -notmatch "^Driftless $([regex]::Escape($version)) \(") {
    Fail "driftless-app.exe --version: `"$appVersion`", erwartet `"Driftless $version (…)`""
}
Ok "driftless-app.exe --version: $appVersion"

if (-not (Test-Path -LiteralPath $shortcut -PathType Leaf)) { Fail "Startmenü-Eintrag $shortcut fehlt" }
$link = (New-Object -ComObject WScript.Shell).CreateShortcut($shortcut)
if ($link.TargetPath -ne $app) { Fail "$shortcut zeigt auf `"$($link.TargetPath)`" statt auf `"$app`"" }
Ok "Startmenü-Eintrag $shortcut → $($link.TargetPath)"

# 3. Silent uninstall. The uninstaller hands over to a copy of itself in the temp folder and
# ends at once, so wait until the folder and the entry are gone.
$uninstaller = Join-Path $dir 'unins000.exe'
$uninstall = Start-Process -FilePath $uninstaller -Wait -PassThru `
    -ArgumentList @('/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', "/LOG=`"$uninstallLog`"")
if ($uninstall.ExitCode -ne 0) { Fail "Stille Deinstallation: Exit $($uninstall.ExitCode)" }
$deadline = (Get-Date).AddSeconds(120)
while ((Test-Path -LiteralPath $dir) -or (Test-Path -LiteralPath $shortcut)) {
    if ((Get-Date) -gt $deadline) {
        $left = if (Test-Path -LiteralPath $dir) { (Get-ChildItem -LiteralPath $dir -Recurse -Name) -join ', ' } else { '' }
        Fail "nach der Deinstallation noch da: Ordner=$(Test-Path -LiteralPath $dir) ($left), Eintrag=$(Test-Path -LiteralPath $shortcut)"
    }
    Start-Sleep -Milliseconds 500
}
Ok 'stille Deinstallation: Ordner und Startmenü-Eintrag sind weg'

Write-Host "Alles in Ordnung: $Setup ($mb MB)"
