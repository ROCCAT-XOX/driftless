## Installieren

Dateien: `Driftless-@VERSION@-macos-arm64.dmg` für den Mac, `Driftless-@VERSION@-setup.exe` für
Windows, `THIRD-PARTY-NOTICES.html` mit den Lizenzen der verwendeten Bausteine.

### Mac

1. Das `.dmg` öffnen, **Driftless** auf „Programme“ ziehen und das Image auswerfen. Driftless nicht
   aus dem Image heraus starten.
2. Driftless in „Programme“ doppelklicken. Beim ersten Start blockiert macOS das Programm
   („„Driftless“ nicht geöffnet“), weil es nicht bei Apple beglaubigt ist: **Fertig** klicken.
3. **Systemeinstellungen → „Datenschutz & Sicherheit“**, unten bei „Sicherheit“ steht „„Driftless“
   wurde blockiert …“: **Dennoch öffnen** klicken, mit Passwort oder Touch ID bestätigen und im
   nächsten Fenster noch einmal **Dennoch öffnen**. Startet Driftless nicht von selbst, noch einmal
   doppelklicken.

Statt Schritt 2 und 3 geht auch das Terminal (das gibt auch die Kommandozeile frei):

```
xattr -dr com.apple.quarantine /Applications/Driftless.app
```

Beides ist nach jeder neuen Version noch einmal nötig.

### Windows

1. Das Setup herunterladen. Warnt der Browser, die Datei werde selten heruntergeladen: über „…“
   **Beibehalten** wählen.
2. Das Setup starten. Meldet Windows „Der Computer wurde durch Windows geschützt“:
   **Weitere Informationen → Trotzdem ausführen**.
3. „Installation nur für Sie (empfohlen)“ wählen. Driftless steht danach im Startmenü.

### Kommandozeile

Die Kommandozeile `driftless` ist in beiden Paketen dabei, aber nicht im Suchpfad:

- Mac: `/Applications/Driftless.app/Contents/MacOS/driftless`
- Windows: `%LOCALAPPDATA%\Programs\Driftless\driftless.exe` (installiert für alle Benutzer:
  `C:\Program Files\Driftless\driftless.exe`), in PowerShell z. B.
  `& "$env:LOCALAPPDATA\Programs\Driftless\driftless.exe" --version`

### Voraussetzungen

- Mac mit Apple-Chip (M1 oder neuer) und macOS 14 oder neuer; Intel-Macs gehen nicht.
- Windows 10 oder 11 (64 Bit) mit einer Grafikkarte für DirectX 12 oder Vulkan.
- Arbeitsspeicher: 16 GB empfohlen. „Maximal“ braucht bei großen Aufnahmen knapp 4 GB, eine
  Oberfläche mit 1 cm bis 9 GB.
- Freier Platz: je Scan 1 bis 6 GB für die Ergebnisse, je nach Genauigkeit.

### Projekte nicht in iCloud Drive oder OneDrive

Projekte und Scans auf ein lokales Laufwerk legen, z. B. in den Ordner „Driftless Projekte“ im
Benutzerordner, den Driftless vorschlägt, nicht in iCloud Drive, OneDrive oder einen anderen Ordner,
der abgeglichen wird: Die Ergebnisse sind mehrere GB groß, und Dateien, die nur in der Cloud liegen,
bremsen oder stören das Rechnen.

### Wenn etwas nicht geht

☰ → „Protokoll-Ordner öffnen“ zeigt das Protokoll (`driftless.log`), „Über Driftless“ nennt Version
und Commit. Beides bitte einer Fehlermeldung beilegen.

Was neu ist, steht unten und im [CHANGELOG](@REPO@/blob/main/CHANGELOG.md).
