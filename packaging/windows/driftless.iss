; Inno Setup 6 script of the Windows setup: target\package\Driftless-<version>-setup.exe.
;
; Built by packaging\windows\build-setup.ps1, which takes the version from Cargo.toml:
;   ISCC.exe /DAppVersion=<version> packaging\windows\driftless.iss
; Checked by packaging\windows\check-setup.ps1 (silent install, --version, uninstall).
;
; Without administrator rights by default (into the user's programs folder); the dialog or
; /ALLUSERS on the command line installs for all users. No file associations, no PATH entry.
; Relative paths are relative to this file.

#ifndef AppVersion
  #error AppVersion fehlt: ISCC.exe /DAppVersion=<version> packaging\windows\driftless.iss
#endif
; x64compatible needs Inno Setup 6.3 or newer.
#if VER < EncodeVer(6, 3, 0)
  #error Inno Setup 6.3 oder neuer erforderlich
#endif

[Setup]
; Fixed for good: Windows finds an installed Driftless by it (update, uninstall).
AppId={{9C554EE9-E0C9-4921-9E1F-CCBCD2A07058}
AppName=Driftless
AppVersion={#AppVersion}
AppPublisher=Driftless
AppPublisherURL=https://github.com/ROCCAT-XOX/driftless
DefaultDirName={autopf}\Driftless
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog commandline
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=..\..\target\package
OutputBaseFilename=Driftless-{#AppVersion}-setup
SetupIconFile=..\..\assets\icon\driftless.ico
UninstallDisplayIcon={app}\driftless.ico
UninstallDisplayName=Driftless
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
; The language dialog only when Windows speaks neither German nor English.
ShowLanguageDialog=auto

[Languages]
; English first: the language when Windows speaks neither.
Name: "en"; MessagesFile: "compiler:Default.isl"
Name: "de"; MessagesFile: "compiler:Languages\German.isl"

[Files]
Source: "..\..\target\release\driftless-app.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\..\target\release\driftless.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\..\assets\icon\driftless.ico"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\..\LICENSE-APACHE"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\..\LICENSE-MIT"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{autoprograms}\Driftless"; Filename: "{app}\driftless-app.exe"; IconFilename: "{app}\driftless.ico"
