import serial

SERIAL_PORT = '/dev/cu.usbserial-0001'  
BAUD_RATE = 115200

def main():
    print(f"Verbinde mit {SERIAL_PORT} bei {BAUD_RATE} Baud...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Verbindung hergestellt. Lese Daten (Abbruch mit Ctrl+C)...")
        print("-" * 40)

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(line)
                    
    except serial.SerialException as e:
        print(f"Fehler beim Öffnen des Ports: {e}")
    except KeyboardInterrupt:
        print("\nProgramm beendet.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()