# Core-Verzeichnis

`Core/` enthält den projektspezifischen Einstieg in die Firmware. Ein großer
Teil wurde von CubeMX erzeugt. Die Dateien verbinden Startup-Code, STM32 HAL,
Board Support Package und die eigentliche Anwendung.

## Struktur

```text
Core/
├── Inc/    Öffentliche Header und Konfigurationsdateien
└── Src/    Implementierungen und Anwendungseinstieg
```

## Dateien in `Core/Src`

### `main.c`

Startpunkt der Anwendung. Die Funktion `main()` führt derzeit aus:

1. `HAL_Init()` initialisiert HAL und Systemtick.
2. `SystemClock_Config()` konfiguriert den Systemtakt.
3. `SystemPower_Config()` wählt die SMPS-Stromversorgung.
4. `MX_ICACHE_Init()` aktiviert den Instruction Cache.
5. `BSP_LED_Init()` initialisiert die grüne Benutzer-LED.
6. `BSP_PB_Init()` initialisiert den Benutzer-Taster mit Interrupt.
7. `BSP_COM_Init()` richtet USART1 mit 115200 Baud ein.
8. Die Endlosschleife schaltet die LED und schreibt Statusmeldungen.

Die Zeitvergleiche verwenden Differenzen von `HAL_GetTick()`. Dadurch wartet
die CPU nicht blockierend und der Überlauf des 32-Bit-Tickzählers wird korrekt
behandelt.

### `stm32u5xx_hal_msp.c`

MSP bedeutet **MCU Support Package**. Hier werden die hardwarenahen
Voraussetzungen für HAL-Peripherie eingerichtet, beispielsweise:

- Peripherie-Clocks
- GPIO Alternate Functions
- DMA-Kanäle
- Interrupt-Prioritäten

Bei der aktuellen Board-Konfiguration übernimmt das BSP einen Teil der
USART1-Pininitialisierung. Änderungen sollten über CubeMX erfolgen.

### `stm32u5xx_it.c`

Enthält die Interrupt Service Routinen des STM32. Dazu gehören unter anderem
Exception Handler für HardFault, SysTick und konfigurierte Peripherieinterrupts.

Interrupt Handler sollen kurz bleiben. Sie erfassen Ereignisse und übergeben
die eigentliche Verarbeitung an den normalen Programmablauf.

### `system_stm32u5xx.c`

CMSIS-Systemdatei für den STM32U5. Sie stellt insbesondere bereit:

- `SystemInit()` für sehr frühe CPU-Initialisierung
- `SystemCoreClock` mit der aktuellen CPU-Frequenz
- `SystemCoreClockUpdate()` zur Neuberechnung dieser Frequenz

Diese Datei läuft zeitlich noch vor `main()` und wird normalerweise nicht von
der Anwendung geändert.

### `syscalls.c`

Bindet Funktionen der C-Standardbibliothek an die Embedded-Umgebung an.
Beispielsweise ruft `printf()` intern `_write()` auf. `_write()` leitet jedes
Zeichen über `__io_putchar()` an den vom BSP eingerichteten Virtual COM Port
weiter.

Andere Funktionen sind einfache Platzhalter, weil ein Bare-Metal-STM32 kein
Betriebssystem mit Dateien, Prozessen oder Terminals besitzt.

### `sysmem.c`

Stellt mit `_sbrk()` den Heap für die C-Laufzeit bereit. Das ist beispielsweise
für dynamische Speicherallokation relevant. Die Anwendung soll dynamische
Allokation im Laufzeitpfad trotzdem vermeiden.

## Dateien in `Core/Inc`

### `main.h`

Gemeinsamer Projekt-Header. Er bindet STM32 HAL und das Nucleo-BSP ein und
deklariert `Error_Handler()`. Spätere anwendungsweite Pin- oder Konstantennamen
können von CubeMX hier erzeugt werden.

### `stm32u5xx_hal_conf.h`

Konfiguriert die HAL. Die Datei bestimmt unter anderem, welche HAL-Module
aktiviert sind und welche Clock-Werte oder Assertions verwendet werden.

### `stm32u5xx_nucleo_conf.h`

Konfiguriert das Board Support Package. Aktuell aktiviert sie:

- BSP-Unterstützung für den Virtual COM Port
- Weiterleitung von Logausgaben auf diesen COM-Port
- Interrupt-Priorität des Benutzer-Tasters

### `stm32u5xx_it.h`

Deklariert die Interrupt Handler aus `stm32u5xx_it.c`, damit die Vektortabelle
und andere Module sie referenzieren können.

## Welche Ebene macht was?

```text
Eigene Anwendung
  -> BSP: Funktionen des konkreten Nucleo-Boards
  -> HAL: Funktionen einer STM32-Peripherie
  -> CMSIS: CPU- und Registerdefinitionen
  -> Hardware
```

Beispiele:

- `BSP_LED_Toggle(LED_GREEN)` kennt die LED des konkreten Boards.
- `HAL_GPIO_TogglePin(...)` arbeitet mit einem bestimmten GPIO-Port und Pin.
- Ein direkter Registerzugriff arbeitet ohne diese Abstraktionen.

## Sicher eigenen Code ergänzen

In CubeMX-generierten Dateien nur innerhalb von `USER CODE BEGIN/END` arbeiten.
Für kleine Experimente ist `main.c` geeignet. Sobald Logik wächst, sollte sie in
eigene Module ausgelagert werden, zum Beispiel:

```text
App/Inc/app.h
App/Src/app.c
```

Diese Dateien werden dann im obersten `CMakeLists.txt` ergänzt. So bleibt
`main.c` hauptsächlich für Initialisierung und zyklischen Aufruf zuständig.

## Empfohlene Lesereihenfolge

1. `Core/Src/main.c`
2. `Core/Inc/main.h`
3. `Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.h`
4. `Core/Src/stm32u5xx_it.c`
5. `Core/Src/syscalls.c`
6. Erst danach HAL- oder CMSIS-Implementierungen im Detail

Die HAL-Dateien sind umfangreich und hauptsächlich Bibliothekscode. Für das
Verständnis der Anwendung ist es effizienter, zunächst von `main.c` aus den
tatsächlich verwendeten Funktionsaufrufen zu folgen.

