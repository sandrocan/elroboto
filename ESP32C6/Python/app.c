app_process_button(now_ms);

    // =======================================================
    // UART-EMPFÄNGER (Entspricht dem Python-Skript)
    // =======================================================
    {
        extern UART_HandleTypeDef *UartCell_GetHandle(void);
        UART_HandleTypeDef *my_cell = UartCell_GetHandle();

        if (my_cell != NULL)
        {
            uint8_t rx_byte;
            // Statische Variablen behalten ihren Wert zwischen den Funktionsaufrufen
            static char rx_buffer[32];
            static uint8_t rx_index = 0;

            // Timeout 0: Alle aktuell im Kabel wartenden Bytes sofort lesen
            while (HAL_UART_Receive(my_cell, &rx_byte, 1, 0) == HAL_OK) 
            {
                // Wenn Zeilenende (\n) erreicht ist (wie Python readline)
                if (rx_byte == '\n') 
                {
                    rx_buffer[rx_index] = '\0'; // String sauber terminieren
                    
                    if (rx_index > 0)
                    {
                        // 1. Text in eine Float-Zahl umwandeln
                        float empfangener_wert = strtof(rx_buffer, NULL);
                        
                        // 2. Den Wert an die globale Roboter-Logik übergeben
                        skin_distance = empfangener_wert;
                        
                        // 3. Ausgabe im Terminal (zur Kontrolle)
                        printf("Empfangen: %.3f\r\n", skin_distance);
                    }
                    
                    // Puffer für die nächste Zeile zurücksetzen
                    rx_index = 0;
                }
                // Unsichtbare Zeichen (\r) ignorieren (wie Python strip)
                else if (rx_byte != '\r') 
                {
                    // Zeichen in den Puffer schreiben (mit Überlaufschutz)
                    if (rx_index < sizeof(rx_buffer) - 1)
                    {
                        rx_buffer[rx_index++] = (char)rx_byte;
                    }
                    else
                    {
                        rx_index = 0; 
                    }
                }
            }
        }
    }
    