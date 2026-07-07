# elroboto

Embedded-Steuerung für einen SO-101-Roboterarm auf dem
**NUCLEO-U545RE-Q** mit STM32U545RE. Das Projekt wird mit STM32CubeMX
konfiguriert, über CMake gebaut und per ST-LINK/SWD programmiert und debuggt.

## Aktueller Stand

- Die grüne Benutzer-LED LD2 wechselt alle 500 ms ihren Zustand.
- Der Benutzer-Taster B1 startet nach dem manuellen Positionieren den
  Drive-Home-Test.
- USART1 ist als Virtual COM Port mit 115200 Baud eingerichtet.
- Nach dem Start erscheint `elroboto booted` auf der seriellen Schnittstelle.
- Ein Lebenszeichen mit der vergangenen Laufzeit wird einmal pro Sekunde
  zusammen mit dem App-Zustand ausgegeben.
- LPUART1 ist mit 1 Mbit/s fuer den Waveshare Bus Servo Adapter eingerichtet.
- Ein STS3215 wurde ueber den externen UART erfolgreich als ID 6 gepingt.
- Beim Start liest die Firmware alle Gelenkpositionen und loggt, ob alle
  Gelenke innerhalb der Home-Toleranz liegen.
- Danach entsperrt die Firmware alle konfigurierten Gelenke, damit der Arm von
  Hand in eine Testposition gebracht werden kann.
- B1 ruft danach `Servo_DriveHome()` auf: alle Gelenke werden erneut
  entsperrt, alle sechs Home-Positionen werden kommandiert, und nach Erreichen
  der Home-Positionen werden ID 5 und ID 6 wieder gelockt.

## Verzeichnisübersicht

| Pfad | Inhalt |
| --- | --- |
| `App/` | Eigene Anwendungslogik und Zustände |
| `ServoBus/` | Historischer Bring-up-Code fuer STS3215-Paketformat, Parser und begrenzten UART-Transport |
| `Core/` | Hardwareinitialisierung, Interrupt-Einstieg, System- und C-Laufzeit-Anbindung |
| `docs/` | Hardwarebelegung und protokollierte Bring-up-Erkenntnisse |
| `Drivers/BSP/` | Board Support Package für LED, Taster und Virtual COM Port des Nucleo-Boards |
| `Drivers/CMSIS/` | ARM-Cortex-M33- und STM32U545-Definitionen auf Registerebene |
| `Drivers/STM32U5xx_HAL_Driver/` | Hardware Abstraction Layer von ST für GPIO, UART, Clock, Flash usw. |
| `cmake/` | Toolchain- und von CubeMX erzeugte CMake-Konfiguration |
| `build/` | Lokale Build-Ausgaben; wird nicht in Git versioniert |
| `.settings/` | Gemeinsame Werkzeug- und Bundle-Versionen der STM32-VS-Code-Erweiterung |
| `.vscode/` | Gemeinsame Build- und Debug-Konfiguration für VS Code |

Mehr Details zu den Dateien in `Core/` stehen in
[Core/README.md](Core/README.md). Die finale Servo-Verkabelung steht in
[docs/hardware.md](docs/hardware.md); der gesamte Diagnoseweg ist in
[docs/servo_bus_bringup.md](docs/servo_bus_bringup.md) festgehalten. Die
aktuellen Kinematik- und Gelenkannahmen stehen in
[docs/kinematics.md](docs/kinematics.md).

## Wichtige Dateien

| Datei | Aufgabe |
| --- | --- |
| `elroboto.ioc` | Zentrale CubeMX-Konfiguration für Board, Pins, Clock und Peripherie |
| `App/Src/app.c` | App-Zustandsmaschine fuer Startup-Home-Check, Startup-Unlock, B1-Drive-Home, LED und Logging |
| `App/Src/servo.c` | Gelenktabelle, STS3215-Kommandos und Rohwert-Limits |
| `App/Src/uart.c` | Adapter fuer den CubeMX-initialisierten Servo-UART-Handle |
| `Core/Src/main.c` | Hardwareinitialisierung und zyklischer Aufruf der Anwendung |
| `CMakeLists.txt` | Oberste Buildbeschreibung und Platz für eigene Module |
| `CMakePresets.json` | Vordefinierte Builds `Debug` und `Release` |
| `STM32U545xx_FLASH.ld` | Speicheraufteilung für einen Build, der aus Flash läuft |
| `STM32U545xx_RAM.ld` | Alternative Speicheraufteilung für einen RAM-Build |
| `startup_stm32u545xx.s` | Reset-Einstieg, Vektortabelle und Übergang zur C-Laufzeit |
| `AGENTS.md` | Arbeits-, Architektur- und Sicherheitsregeln für dieses Repository |

## Vom Einschalten bis zur Anwendung

```text
Reset
  -> startup_stm32u545xx.s
  -> SystemInit()
  -> C-Laufzeit initialisiert RAM und globale Variablen
  -> main()
  -> HAL-, Clock-, Power- und Peripherieinitialisierung
  -> Endlosschleife der Anwendung
```

## Bauen und Debuggen

Debug-Build im Terminal:

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Das Ergebnis liegt unter `build/Debug/elroboto.elf`. In VS Code startet `F5`
den ST-LINK-Debugger, programmiert die Firmware und hält zunächst bei `main()`.

Serielle Ausgabe unter macOS öffnen:

```bash
screen /dev/cu.usbmodem1403 115200
```

Der Gerätename kann sich nach einem anderen USB-Anschluss ändern.

## CubeMX und eigener Code

`elroboto.ioc` ist die Quelle für Hardwarekonfigurationen. Änderungen an Pins,
Clocks oder Peripherie werden in CubeMX vorgenommen und anschließend neu
generiert.

In generierten C-/Headerdateien gehört eigener Code ausschließlich in Bereiche
zwischen Markierungen wie:

```c
/* USER CODE BEGIN 0 */
/* eigener Code */
/* USER CODE END 0 */
```

CubeMX erhält diese Bereiche bei einer Neugenerierung. Code außerhalb davon
kann überschrieben werden. Eigene Module liegen außerhalb der generierten
Dateien, beispielsweise unter `App/`, und werden über `CMakeLists.txt`
eingebunden.
