# Hardware-Anbindung

## Zielboard

- ST NUCLEO-U545RE-Q
- STM32U545RE
- integrierter ST-LINK fuer Flashen und SWD-Debugging

## Diagnose-UART

USART1 ist mit dem Virtual COM Port des ST-LINK verbunden. Die Firmware nutzt
diese Schnittstelle mit 115200 Baud fuer `printf()`-Diagnosen.

## E-Skin-UART

Der ESP32-C6 sendet den maximalen E-Skin-Naeherungswert als ASCII-Zeile an
UART4. Die am 21.07.2026 auf Hardware validierte Verbindung ist:

| Funktion | ESP32-C6 | STM32 / Nucleo |
|---|---|---|
| Sensordaten | TX / GPIO16 | PC11 / UART4_RX / CN7 Pin 2 |
| Bezugspotential | GND | GND, zum Beispiel am POWER-Header |

UART4 verwendet 115200 Baud, 8 Datenbits, keine Paritaet, ein Stopbit und kein
Hardware Flow Control. Akzeptiert werden Zeilen wie `0.123\n` und
`0.123\r\n`. Im Hardwaretest wurden fortlaufend gueltige Pakete ohne UART-,
Parser- oder Neustartfehler empfangen. Der unbelastete Sensorwert lag bei etwa
`0.011`; bei Annaeherung stieg er bis ueber `0.8`.

Der UART4-Interrupt puffert nur die empfangene ASCII-Zeile. Die Umwandlung in
`float` erfolgt ausserhalb der ISR im normalen Kontrollpfad. Dadurch kann die
E-Skin-Verarbeitung die dicht aufeinanderfolgenden Servoantworten bei 1 Mbit/s
nicht durch eine rechenintensive `strtof()`-Auswertung im Interrupt
unterbrechen.

Die Demo verwendet vorlaeufig `0.050` als Stoppschwellwert. Beim Stopp liest die
Firmware die Positionen der vier aktiven Gelenke und schreibt sie als
Halteziele zurueck. Sinkt der Messwert anschliessend fuer mindestens 500 ms auf
hoechstens `0.020`, wird derselbe Demoversuch automatisch neu berechnet und
fortgesetzt. Die Hysterese verhindert ein direktes Ein-/Ausschalten im
Grenzbereich.

Vor der ersten Bewegung muss innerhalb von zwei Sekunden ein gueltiges Paket
ankommen. Eine Datenluecke von mehr als einer Sekunde fuehrt zum verriegelten
Fehler und wird nicht automatisch freigegeben. Auch ein Skin-Stopp waehrend der
Startpositionierung bleibt verriegelt. Diese Reaktionen sind nur nicht
zertifizierte Software-Stopps und weder ein Not-Halt noch eine Garantie fuer
mechanische Sicherheit.

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
- RX-FIFO aktiviert, um kurze Unterbrechungen durch andere Interrupts zu
  ueberbruecken

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
