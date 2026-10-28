# The DLLs a Windows program needs at its start, read from the import table and the delay-load
# table of the PE file (32 or 64 bit), with PowerShell alone (no dumpbin). Dot-sourced by
# check-setup.ps1; by hand:
#
#   . packaging\windows\pe-imports.ps1; Get-PeImports target\release\driftless.exe
Set-StrictMode -Version Latest

# The names of the DLLs that the PE file $Path imports, in the order of its tables (the import
# table first, then the delay-load table); wrap the call in @() for an array of one or none.
# Throws if the file is no PE file.
function Get-PeImports {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )
    $bytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Path).Path)
    function U16([long]$at) { [long][BitConverter]::ToUInt16($bytes, $at) }
    function U32([long]$at) { [long][BitConverter]::ToUInt32($bytes, $at) }
    function U64([long]$at) { [long][BitConverter]::ToUInt64($bytes, $at) }

    if ($bytes.Length -lt 0x40 -or (U16 0) -ne 0x5A4D) { throw "$Path ist keine Programmdatei (kein MZ)." }
    $pe = U32 0x3C
    if ($pe + 24 -gt $bytes.Length -or (U32 $pe) -ne 0x4550) { throw "$Path ist keine PE-Datei." }
    $sectionCount = U16 ($pe + 6)
    $optional = $pe + 24
    $sections = $optional + (U16 ($pe + 20))
    # PE32 (0x10B) or PE32+ (0x20B): where the image base and the data directories are.
    switch (U16 $optional) {
        0x10B { $imageBase = U32 ($optional + 28); $directories = $optional + 96 }
        0x20B { $imageBase = U64 ($optional + 24); $directories = $optional + 112 }
        default { throw "${Path}: unbekannter PE-Kopf." }
    }
    $directoryCount = U32 ($directories - 4)

    # The place in the file of a relative virtual address.
    function Get-Offset([long]$rva) {
        for ($i = 0; $i -lt $sectionCount; $i++) {
            $s = $sections + 40 * $i
            $start = U32 ($s + 12)
            $size = [Math]::Max((U32 ($s + 8)), (U32 ($s + 16)))
            if ($rva -ge $start -and $rva -lt $start + $size) { return (U32 ($s + 20)) + $rva - $start }
        }
        throw "${Path}: Adresse 0x$('{0:X}' -f $rva) liegt in keinem Abschnitt."
    }
    function Get-Name([long]$rva) {
        $at = Get-Offset $rva
        $end = $at
        while ($end -lt $bytes.Length -and $bytes[$end] -ne 0) { $end++ }
        [Text.Encoding]::ASCII.GetString($bytes, $at, $end - $at)
    }
    # The address of data directory $index (0 if the file has none).
    function Get-Directory([int]$index) {
        if ($index -ge $directoryCount) { return 0 }
        U32 ($directories + 8 * $index)
    }

    $names = New-Object System.Collections.Generic.List[string]
    # 1: import descriptors, 20 bytes each, the name at +12; an empty one ends the table.
    $table = Get-Directory 1
    if ($table -ne 0) {
        $at = Get-Offset $table
        while ((U32 ($at + 12)) -ne 0) {
            $names.Add((Get-Name (U32 ($at + 12))))
            $at += 20
        }
    }
    # 13: delay-load descriptors, 32 bytes each, the name at +4 (an address relative to the
    # image if bit 0 of the attributes is set, else an absolute one); an empty one ends it.
    $table = Get-Directory 13
    if ($table -ne 0) {
        $at = Get-Offset $table
        while ((U32 ($at + 4)) -ne 0) {
            $name = U32 ($at + 4)
            if (((U32 $at) -band 1) -eq 0) { $name -= $imageBase }
            $names.Add((Get-Name $name))
            $at += 32
        }
    }
    return $names.ToArray()
}
