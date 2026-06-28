# elroboto

Embedded-Steuerung für einen SO-101-Roboterarm auf dem
**NUCLEO-U545RE-Q** mit STM32U545RE. Das Projekt wird mit STM32CubeMX
konfiguriert, über CMake gebaut und per ST-LINK/SWD programmiert und debuggt.

## Aktueller Stand

- Die grüne Benutzer-LED LD2 wechselt alle 500 ms ihren Zustand.
- Der Benutzer-Taster B1 wechselt zwischen 500 ms und 100 ms Blinkintervall.
- USART1 ist als Virtual COM Port mit 115200 Baud eingerichtet.
- Nach dem Start erscheint `elroboto booted` auf der seriellen Schnittstelle.
- Ein Lebenszeichen mit der vergangenen Laufzeit wird einmal pro Sekunde
  ausgegeben.

## Verzeichnisübersicht

| Pfad | Inhalt |
| --- | --- |
| `Core/` | Anwendungseinstieg, Interrupts, Systeminitialisierung und C-Laufzeit-Anbindung |
| `Drivers/BSP/` | Board Support Package für LED, Taster und Virtual COM Port des Nucleo-Boards |
| `Drivers/CMSIS/` | ARM-Cortex-M33- und STM32U545-Definitionen auf Registerebene |
| `Drivers/STM32U5xx_HAL_Driver/` | Hardware Abstraction Layer von ST für GPIO, UART, Clock, Flash usw. |
| `cmake/` | Toolchain- und von CubeMX erzeugte CMake-Konfiguration |
| `build/` | Lokale Build-Ausgaben; wird nicht in Git versioniert |
| `.settings/` | Gemeinsame Werkzeug- und Bundle-Versionen der STM32-VS-Code-Erweiterung |
| `.vscode/` | Gemeinsame Build- und Debug-Konfiguration für VS Code |

Mehr Details zu den Dateien in `Core/` stehen in
[Core/README.md](Core/README.md).

## Wichtige Dateien

| Datei | Aufgabe |
| --- | --- |
| `elroboto.ioc` | Zentrale CubeMX-Konfiguration für Board, Pins, Clock und Peripherie |
| `Core/Src/main.c` | Einstieg in die Anwendung und aktuell die LED-/Logging-Logik |
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
kann überschrieben werden. Größere eigene Module sollen später außerhalb der
generierten Dateien angelegt und über `CMakeLists.txt` eingebunden werden.
