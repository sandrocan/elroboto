# AGENTS.md

## Geltungsbereich

Diese Datei gilt fuer das gesamte Repository. Sie beschreibt die verbindlichen
Arbeitsregeln fuer Menschen und Coding-Agents in diesem Projekt.

## Projektziel

Dieses Repository enthaelt die vollstaendig eingebettete Steuerung eines
SO-101-Roboterarms mit sechs Freiheitsgraden. Die Firmware soll direkt auf dem
STM32 laufen und folgende Aufgaben integrieren:

- Kommunikation mit sechs Feetech-STS3215-Servos ueber ein Waveshare Serial Bus
  Servo Driver Board
- Umrechnung einer Zielpose des Greifers in Gelenkwinkel (inverse Kinematik)
- Ansteuerung, Rueckmeldung, Kalibrierung und Begrenzung der sechs Gelenke
- eine deterministische Hauptzustandsmaschine fuer die Abschlussdemo

Der Schwerpunkt von Alessandro ist MCU-Konfiguration und Integration: Projekt-
und Peripherie-Setup, Hardwarekommunikation, Softwarearchitektur,
Hauptsteuerung, Debugging, Tests und Dokumentation. Die Kinematik- und
Robot-Control-Schnittstellen werden gemeinsam mit Niklas integriert.

## Zielhardware

- Board: **ST NUCLEO-U545RE-Q** (Nucleo-64)
- MCU: **STM32U545RE**, Arm Cortex-M33, bis 160 MHz, 512 KiB Flash,
  274 KiB SRAM, Hardware-FPU
- Debugger: integrierter ST-LINK
- Servoaktorik: 6 x Feetech STS3215
- Servoschnittstelle: Waveshare Serial Bus Servo Driver Board ueber UART

Die Kurzbezeichnung `NUCLEO-U54RE-Q` ist nicht die offizielle ST-Bezeichnung.
Behandle `NUCLEO-U545RE-Q` als Zielboard. Falls die Beschriftung der realen
Hardware davon abweicht, keine Konfiguration erzeugen oder flashen, sondern
zuerst mit Alessandro klaeren.

## Technische Grundlage

- Bevorzugte Toolchain: STM32CubeIDE mit integriertem STM32CubeMX und
  STM32CubeU5/HAL. Bereits im Projekt festgelegte Versionen nicht stillschweigend
  wechseln.
- Sprache: C fuer Firmware; C11 oder der vom erzeugten STM32-Projekt vorgegebene
  kompatible Dialekt. C++ nur nach einer bewussten Projektentscheidung.
- Startarchitektur: Bare Metal mit kooperativer, zeitgesteuerter
  Zustandsmaschine. Ein RTOS nur einfuehren, wenn ein nachgewiesener Bedarf und
  ein Integrationsplan vorliegen.
- Die `.ioc`-Datei ist die Quelle fuer Pin-, Clock-, DMA-, Interrupt- und
  Peripheriekonfiguration. Aenderungen an generierten Dateien muessen innerhalb
  der `USER CODE`-Bereiche bleiben.
- Keine dynamische Speicherallokation im Laufzeitpfad. Speicherbedarf statisch
  und nachvollziehbar halten.
- Physikalische Werte immer mit Einheit benennen, zum Beispiel `distance_mm`,
  `timeout_ms`, `joint_angle_rad` oder `speed_ticks_per_s`.

## Bevorzugte Softwarearchitektur

Passe dich an die von CubeMX erzeugte Struktur an. Halte eigenen Code von
generiertem Code getrennt. Wenn noch keine Struktur existiert, verwende
sinngemaess folgende Module:

- `platform`: STM32-/HAL-nahe Adapter, Zeitbasis und Peripherie-Handles
- `servo_bus`: Paketformat, UART-Transport, Timeouts, Pruefsummen und Antworten
- `robot_control`: Gelenkziele, Limits, Kalibrierung und Bewegungsbefehle
- `kinematics`: Vorwaerts-/Inverse-Kinematik ohne HAL-Abhaengigkeiten
- `safety`: Grenzwerte, Fehlerverriegelung und sichere Stoppentscheidung
- `app`: Hauptzustandsmaschine und Integration aller Module

Abhaengigkeitsrichtung:

`app -> safety/robot_control -> servo_bus/kinematics -> platform/HAL`

Kinematik, Paketparser und Zustandslogik sollen ohne reale Hardware
testbar bleiben. Sie duerfen keine versteckten Abhaengigkeiten auf globale
HAL-Handles besitzen. Oeffentliche Modul-Schnittstellen gehoeren in Header;
interne Helfer bleiben `static` in der jeweiligen Quelldatei.

## Peripherie- und Kommunikationsregeln

### Servo-Bus

- Das Waveshare-Board wird ueber UART angebunden. Konkrete UART-Instanz,
  Alternate Functions, Pins, Pegel, Baudrate und Verdrahtung erst nach Abgleich
  mit Board-Handbuch, Schaltplan und realem Aufbau in der `.ioc` festlegen.
- Das Adapterboard kann eine ungewoehnliche RX-zu-RX-/TX-zu-TX-Verdrahtung
  vorgeben. Beschriftungen nicht anhand ueblicher UART-Konventionen erraten.
- STS3215 nutzt einen seriellen Bus mit Antworten. Richtungswechsel, Antwortzeit,
  Paketlaenge, Pruefsumme und Timeout explizit behandeln.
- Baudrate und Servo-IDs sind Konfiguration, keine verstreuten Magic Numbers.
  Der Servo nennt 1 Mbit/s als Werkseinstellung; vor dem ersten Buszugriff die
  tatsaechliche Konfiguration aller sechs Servos pruefen.
- Senden/Empfangen bevorzugt interrupt- oder DMA-basiert implementieren. Parser
  muessen Teilpakete, ungueltige Laengen, falsche IDs, Pruefsummenfehler und
  Timeouts kontrolliert behandeln.
- Nie Broadcast-Schreibbefehle verwenden, solange IDs, Limits und
  Bewegungsrichtung nicht einzeln validiert wurden.

### Zeitverhalten

- In der zyklischen Steuerung kein `HAL_Delay()` und kein unbeschraenktes
  Busy-Waiting. Kurze, begruendete Delays sind nur waehrend einer sicheren
  Initialisierung zulaessig.
- Zeitvergleiche muessen den Ueberlauf der Tick-Zeit korrekt behandeln.
- ISR kurz halten: Status erfassen bzw. Puffern und Verarbeitung in den normalen
  Kontrollpfad verschieben.
- Jeder externe Zugriff braucht einen endlichen Timeout und einen definierten
  Fehlerpfad.

## Zustands- und Sicherheitsmodell

Die Anwendung soll mindestens die Zustaende `INIT`, `IDLE`, `MOVING` und
`FAULT` unterscheiden. Weitere Zustaende wie `HOMING` oder `DECELERATING` nur
mit klaren Ein- und Austrittsbedingungen ergaenzen.

Sicherheitsentscheidungen haben Vorrang vor Bewegungsbefehlen:

- Gelenk-, Servo- und Geschwindigkeitslimits vor dem Senden pruefen.
- Kommunikationsfehler duerfen keine unkontrollierte Weiterbewegung ausloesen.
- Ein verriegelter Fehler darf nur durch einen bewussten, dokumentierten
  Resetpfad geloest werden.
- Software-Stopp nicht als zertifizierten Not-Halt oder funktional sichere
  Einrichtung bezeichnen.
- Den Arm niemals allein durch Firmware als energiefrei oder mechanisch sicher
  betrachten.

Agents duerfen Firmware bauen und statisch pruefen. Flashen, Servos bestromen,
Servo-IDs/Kalibrierwerte schreiben oder eine Bewegung ausloesen ist nur nach
ausdruecklicher Zustimmung des Benutzers erlaubt. Bei ersten Bewegungstests:
Arm entlasten bzw. aufbocken, Arbeitsraum freihalten, kleine Geschwindigkeiten
und konservative Limits verwenden und eine physische Abschaltmoeglichkeit
bereithalten.

## Konfiguration und Dokumentation

- Noch unbekannte Hardwaredaten nicht erfinden. Offene Entscheidungen mit
  `TODO(alessandro):` und einer konkreten Frage markieren.
- Sobald festgelegt, Pinout, Spannungen, Masseverbindung, UART-Parameter und
  Servo-IDs in `docs/hardware.md` dokumentieren.
- Kalibrierwerte und mechanische Limits zentral ablegen; Rohwerte, Winkel und
  Vorzeichenkonventionen eindeutig beschreiben.
- Koordinatensysteme, Achsreihenfolge und Winkeleinheiten in
  `docs/kinematics.md` dokumentieren, bevor IK-Ergebnisse den realen Arm bewegen.
- Wesentliche Architektur- oder Sicherheitsentscheidungen kurz in `docs/adr/`
  festhalten, sobald dieser Ordner existiert.
- Keine Zugangsdaten, lokalen IDE-Metadaten, Build-Artefakte oder absoluten
  Rechnerpfade committen.

## Arbeitsweise fuer Aenderungen

1. Vor einer Aenderung Repository, `.ioc`, Buildsystem und vorhandene
   Dokumentation lesen.
2. Die kleinste in sich geschlossene Aenderung umsetzen; generierten und
   handgeschriebenen Code nicht unnoetig vermischen.
3. Vorhandenen Stil und bestehende Schnittstellen beibehalten oder eine
   notwendige Abweichung begruenden.
4. Fehler immer bis zu einer sichtbaren Status-/Diagnoseschnittstelle
   weiterreichen; Fehlercodes nicht still verwerfen.
5. Build und passende Tests ausfuehren. Wenn Hardware fehlt, klar zwischen
   kompiliert, host-getestet und auf Hardware validiert unterscheiden.
6. Dokumentation bei Aenderungen an Pinout, Protokoll, Zustandslogik, Limits oder
   Buildablauf im selben Arbeitsschritt aktualisieren.

## Tests und Definition of Done

Eine Aenderung ist erst fertig, wenn:

- das konfigurierte STM32-Ziel ohne neue Warnungen oder Fehler baut,
- reine Logik soweit praktikabel durch Host-Unit-Tests abgedeckt ist,
- Protokolltests auch ungueltige Pakete, Timeouts und Teilpakete enthalten,
- Sicherheitslogik mindestens Gelenkgrenzen und Kommunikationsfehler abdeckt,
- keine reale Bewegung fuer einen automatisierten Test vorausgesetzt wird,
- notwendige Hardwaretests als reproduzierbare Schritte dokumentiert sind und
- im Abschlussbericht klar steht, was nicht auf echter Hardware getestet wurde.
