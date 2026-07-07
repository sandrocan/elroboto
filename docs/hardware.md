# Hardware-Anbindung

## Zielboard

- ST NUCLEO-U545RE-Q
- STM32U545RE
- integrierter ST-LINK fuer Flashen und SWD-Debugging

## Diagnose-UART

USART1 ist mit dem Virtual COM Port des ST-LINK verbunden. Die Firmware nutzt
diese Schnittstelle mit 115200 Baud fuer `printf()`-Diagnosen.

## Servo-Bus

Der Waveshare Bus Servo Adapter (A), Modell WSH-SBS-01, wird ueber LPUART1
angebunden:

| Funktion | STM32-Pin | Adapter-Beschriftung |
|---|---|---|
| Senden | PA2 / LPUART1_TX / Arduino D1 (CN9 Pin 2) | TX |
| Empfangen | PA3 / LPUART1_RX / Arduino D0 (CN9 Pin 1) | RX |
| Bezugspotential | GND | GND |

Die TX-zu-TX-/RX-zu-RX-Zuordnung folgt der Beschriftung und Dokumentation des
Waveshare-Adapters. Vor dem Einschalten muss der Adapter-Jumper auf Stellung A
fuer die externe MCU-Schnittstelle stehen.

UART-Konfiguration:

- 1.000.000 Baud
- 8 Datenbits
- keine Paritaet
- 1 Stopbit
- kein Hardware Flow Control
- HSI16 als LPUART1-Taktquelle

Die Servoversorgung erfolgt separat mit 12 V. Die 12-V-Leitung darf nicht mit
einem Versorgungspin des Nucleo verbunden werden. Nucleo und Adapter benoetigen
aber eine gemeinsame Masse. Fuer die erste Inbetriebnahme wird nur ein Servo
angeschlossen; die 12-V-Versorgung wird zuletzt eingeschaltet.

Am 28.06.2026 wurde die Kommunikationskette auf echter Hardware validiert. Ein
einzeln angeschlossener STS3215 antwortete bei 1.000.000 Baud als ID 6 mit
Status `0x00`. Der Servo wurde dabei nur per Ping gelesen; es wurde kein
Bewegungs- oder Schreibbefehl gesendet. Fuer den zuverlaessigen Empfang laeuft
der STM32-Systemtakt mit 16 MHz und die sechs Byte lange Ping-Antwort wird als
zusammenhaengender UART-Block empfangen.

## Offene Konfiguration

- `TODO(alessandro):` Die getestete ID 6 und die IDs der restlichen STS3215 den
  jeweiligen Gelenken zuordnen und dokumentieren. Die Firmware enthaelt aktuell
  die aus dem Kinematikstand importierte vorlaeufige Zuordnung ID 1 bis 6.
- `TODO(alessandro):` Vor Schreib- oder Bewegungsbefehlen die tatsaechliche
  Baudrate und ID jedes Servos einzeln durch einen reinen Lesezugriff pruefen.
- `TODO(alessandro):` Den Drive-Home-Test aus `docs/kinematics.md`
  mit entlastetem Arm, kleiner Geschwindigkeit und physischer Abschaltung
  durchfuehren und Ergebnis hier dokumentieren.
