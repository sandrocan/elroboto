# Kinematik und Gelenkzuordnung

## Aktueller Integrationsstand

Der importierte Kinematikstand liefert aktuell Kalibrierwerte, eine erste
URDF-basierte Modellbeschreibung und eine Servo-Gelenktabelle. Die Firmware
nutzt daraus noch keine inverse Kinematik fuer Bewegungsbefehle.

Der aktuelle Hardwaretest prueft beim Start, ob alle konfigurierten Gelenke
innerhalb der Home-Toleranz liegen. Die Firmware loggt das Ergebnis, faehrt beim
Start aber nicht automatisch los. Danach entsperrt sie alle Gelenke fuer
manuelle Positionierung. Nach manueller Positionierung ruft B1
`Servo_DriveHome()` auf.

## Rohwert-Kalibrierung

Die STS3215-Positionen werden als Encoder-Rohwerte im Bereich 0 bis 4095
gefuehrt. Die folgenden Werte sind die gemessenen Drive-Home-Werte und sind in
`App/Src/servo.c` als zentrale Gelenktabelle abgelegt.

| Gelenk | Servo-ID | Name | Min Raw | Home / Fixed Raw | Max Raw | Nutzung |
|---|---:|---|---:|---:|---:|---|
| J1 | 1 | `shoulder_pan` | 729 | 2047 | 3391 | aktiv |
| J2 | 2 | `shoulder_lift` | 787 | 2047 | 3308 | aktiv |
| J3 | 3 | `elbow_flex` | 900 | 2047 | 2998 | aktiv |
| J4 | 4 | `wrist_flex` | 933 | 2047 | 3228 | aktiv |
| J5 | 5 | `wrist_roll` | 2000 | 2047 | 2200 | fixiert |
| J6 | 6 | `gripper` | 810 | 810 | 2200 | fixiert / geschlossen |

`TODO(alessandro):` Servo-IDs 1 bis 6 an echter Hardware den physischen
Gelenken zuordnen und danach `docs/hardware.md` aktualisieren.

## Modellquelle

Die nominale Kinematik basiert auf der SO-101-URDF-Beschreibung. URDF-Urspruenge
werden als Parent-zu-Child-Transformationen interpretiert, nicht als globale
Koordinaten. Die aktive erste IK-Version soll positionsbasiert arbeiten und nur
die vier Gelenke `shoulder_pan`, `shoulder_lift`, `elbow_flex` und
`wrist_flex` berechnen. `wrist_roll` und `gripper` bleiben fuer diesen Stand
fixiert.

Transformationsreihenfolge fuer URDF-Gelenke:

```text
T(parent -> child) = Trans(x, y, z) * RotZ(yaw) * RotY(pitch) * RotX(roll) * RotZ(q)
```

Verwendete Kette:

```text
origin
  -> shoulder_pan
  -> shoulder_lift
  -> elbow_flex
  -> wrist_flex
  -> wrist_roll (fixiert)
  -> gripper_link
  -> gripper_frame_joint
  -> gripper_frame_link
```

## Voraussetzung fuer Bewegung

Bevor IK-Ergebnisse reale Servos bewegen duerfen, muessen mindestens diese
Punkte abgeschlossen sein:

- Servo-IDs und Gelenkzuordnung fuer alle sechs Achsen read-only validieren.
- Drehrichtungen, Rohwert-zu-Winkel-Konvention und Vorzeichen dokumentieren.
- Gelenklimits aus Firmware, Mechanik und URDF gegeneinander pruefen.
- Einen reproduzierbaren Hardwaretest mit aufgebocktem Arm, kleiner
  Geschwindigkeit, konservativen Limits und physischer Abschaltmoeglichkeit
  dokumentieren.

## Drive-Home-Test

`App/Src/app.c` enthaelt aktuell einen Drive-Home-Test fuer die
Hardwarevalidierung. Ablauf:

1. Firmware starten und serielle Ausgabe beobachten.
2. Die Firmware liest alle Gelenkpositionen und vergleicht sie mit Home.
3. Falls mindestens ein Gelenk ausserhalb der Toleranz liegt, wird dies nur
   geloggt.
4. Danach entsperrt die Firmware alle konfigurierten Gelenke, inklusive ID 5
   und ID 6.
5. Arm von Hand in eine reproduzierbare Testposition bewegen.
6. B1 druecken: `Servo_DriveHome()` wird aufgerufen.
7. `Servo_DriveHome()` entsperrt zuerst erneut alle Gelenke, lockt und
   kommandiert dann alle sechs Home-Positionen, pollt die Encoderpositionen
   bis alle innerhalb der Toleranz liegen und lockt danach ID 5 und ID 6.

Aktuelle Home-Konstanten:

```text
SERVO_HOME_SPEED = 120
SERVO_HOME_ACCELERATION = 20
SERVO_HOME_POSITION_TOLERANCE = 50
SERVO_HOME_TIMEOUT_MS = 15000
HOME_CHECK_TOLERANCE_TICKS = 50
```

Vor diesem Test muessen Arm und Servo mechanisch entlastet sein, der
Arbeitsraum frei sein und eine physische Abschaltmoeglichkeit bereitliegen.
Der Test ersetzt keinen Not-Halt und keine funktional sichere Einrichtung.
