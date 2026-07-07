# Servo-Bus-Bring-up

Dieses Dokument fasst den am 28.06.2026 erreichten Stand der Kommunikation
zwischen NUCLEO-U545RE-Q, Waveshare Bus Servo Adapter (A) und Feetech STS3215
zusammen.

## Validierter Stand

Die vollstaendige Kommunikationskette wurde auf echter Hardware erfolgreich
getestet:

```text
STM32U545 -> LPUART1 -> Waveshare Adapter A -> STS3215
```

Ein einzeln angeschlossener Servo antwortete auf einen reinen Ping:

```text
Servo found: ID=6, baud=1000000, status=0x00
```

Der Test hat weder Register geschrieben noch eine Bewegung angefordert. Die ID
6 ist noch keinem konkreten Gelenk zugeordnet.

## Endgueltige Konfiguration

| Einstellung | Wert |
|---|---|
| STM32-Peripherie | LPUART1 |
| TX | PA2 / Arduino D1 |
| RX | PA3 / Arduino D0 |
| Baudrate | 1.000.000 Baud |
| Datenformat | 8N1 |
| Flow Control | keines |
| LPUART1-Takt | HSI16, 16 MHz |
| SYSCLK/HCLK | HSI16, 16 MHz |
| Waveshare-Jumper | A fuer externen MCU-UART |

Verdrahtung entsprechend der Waveshare-Beschriftung:

```text
Waveshare TX  -> Nucleo D1 / PA2 / LPUART1_TX
Waveshare RX  -> Nucleo D0 / PA3 / LPUART1_RX
Waveshare GND -> Nucleo GND
```

Das Waveshare-Board und der 12-V-Servo werden aus dem separaten 12-V-Netzteil
versorgt. Es wird keine Versorgungsspannung vom Adapter zum Nucleo gefuehrt.

## Historische Implementierung

Die fruehere separate Servo-Bus-Komponente, die Hosttests und die lokalen
macOS-Diagnosewerkzeuge wurden nach der Integration des Servo-Codes in `App/`
entfernt. Dieses Dokument beschreibt den historischen Bring-up-Weg; die
aktuelle App nutzt `App/Src/servo.c`, `App/Src/uart.c` und B1 fuer den
Drive-Home-Test.

## Wichtigste Diagnoseerkenntnis

Die erste Implementierung lief mit nur 4 MHz Systemtakt und rief fuer jedes
empfangene Byte einzeln `HAL_UART_Receive()` auf. Bei 1 Mbit/s trifft alle
10 Mikrosekunden ein neues UART-Byte ein. Dadurch gingen Teile der
sechs Byte langen Servoantwort verloren.

Die funktionierende Loesung besteht aus zwei Aenderungen:

1. SYSCLK und HCLK wurden auf HSI16 mit 16 MHz umgestellt.
2. Die sechs Byte lange Ping-Antwort wird in einem zusammenhaengenden
   `HAL_UART_Receive()`-Aufruf empfangen und erst danach geparst.

Ein Austausch von Adapter, Servo und Kabeln war fuer diesen Fehler nicht
erforderlich.

## Durchgefuehrte Tests

- STM32-Debug-Build ohne Fehler
- Hosttests des Paketformats und Parsers bestanden
- langsamer GPIO-Loopback fuer D1/TX und D0/RX bestanden
- echter LPUART1-Loopback bei 1 Mbit/s bestanden
- alle verwendeten Dupont-Kabel per Loopback getestet
- Waveshare-USB-Ping im Jumpermodus B erfolgreich
- finaler Ping ueber den externen UART im Jumpermodus A erfolgreich

## Sicheres Wiederaufnehmen

1. Nur einen Servo anschliessen und mechanisch freilegen.
2. Nucleo per USB starten und serielles Logging pruefen.
3. Gemeinsame Masse herstellen.
4. Waveshare-Jumper A sowie TX-D1 und RX-D0 kontrollieren.
5. 12 V zuletzt einschalten.
6. B1 einmal druecken und die gefundene ID dokumentieren.
7. Nach dem Test 12 V wieder ausschalten.

## Naechste Schritte

1. Die restlichen Servos einzeln pingen und ID sowie Gelenkzuordnung erfassen.
2. Read-Befehle fuer Position, Spannung und Temperatur implementieren.
3. Transport auf Interrupt oder DMA umstellen und Timeouts als Zustandslogik
   behandeln.
4. Erst danach zentrale Limits, Kalibrierwerte und sichere Schreibbefehle
   planen.
5. Vor dem ersten Bewegungsbefehl mechanische Limits, Drehrichtung und eine
   physische Abschaltmoeglichkeit festlegen.
