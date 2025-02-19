import serial
import time

# Configuración del puerto serial
PORT = '/dev/ttyUSB0'
BAUDRATE = 9600  # Ajusta este valor según el baudrate actual del módulo
TIMEOUT = 1

# Inicializar la conexión serial
def init_serial():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"Conexión establecida en {PORT} a {BAUDRATE} baudios.")
        return ser
    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")
        return None

# Enviar comando al módulo GPS
def send_command(ser, command):
    if ser is not None:
        print(f"Enviando comando: {command}")
        ser.write((command + '\r\n').encode())  # Asegúrate de agregar \r\n
        time.sleep(1)  # Esperar respuesta
        response = ser.read_all()  # Lee los datos en bruto (bytes)
        if response:
            print(f"Respuesta (bytes): {response}")
            print(f"Respuesta (hex): {response.hex()}")
        else:
            print("No se recibió respuesta.")
    else:
        print("No se pudo enviar el comando. Conexión serial no establecida.")

# Cambiar baudrate
def change_baudrate(ser, new_baudrate):
    # Comando UBX para cambiar el baudrate
    command = f"$PUBX,41,1,0007,0003,{new_baudrate},0"
    send_command(ser, command)
    print(f"Cambiando baudrate a {new_baudrate}...")
    time.sleep(2)  # Esperar a que el módulo procese el cambio
    ser.baudrate = new_baudrate  # Cambiar el baudrate en la conexión serial
    print(f"Baudrate cambiado a {new_baudrate}. Verifica la conexión.")

# Cambiar frecuencia de actualización
def change_update_rate(ser, rate):
    # Comando UBX para cambiar la frecuencia de actualización
    # UBX-CFG-RATE (0x06 0x08)
    # Payload: measRate (2 bytes), navRate (2 bytes), timeRef (2 bytes)
    measRate = int(1000 / rate)  # Tiempo de medición en ms
    navRate = 1  # Número de ciclos de medición por actualización de navegación
    timeRef = 0  # Referencia de tiempo (0 = UTC, 1 = GPS)

    # Crear el mensaje UBX
    header = b"\xB5\x62"  # Sync chars
    class_id = b"\x06\x08"  # UBX-CFG-RATE
    length = b"\x06\x00"  # Longitud del payload (6 bytes)
    payload = measRate.to_bytes(2, 'little') + navRate.to_bytes(2, 'little') + timeRef.to_bytes(2, 'little')
    checksum = calculate_checksum(class_id + length + payload)
    message = header + class_id + length + payload + checksum

    # Enviar el mensaje UBX
    ser.write(message)
    print(f"Frecuencia de actualización cambiada a {rate} Hz.")

# Calcular el checksum para el mensaje UBX
def calculate_checksum(data):
    CK_A = 0
    CK_B = 0
    for byte in data:
        CK_A = (CK_A + byte) & 0xFF
        CK_B = (CK_B + CK_A) & 0xFF
    return bytes([CK_A, CK_B])

# Reiniciar módulo
def factory_reset(ser):
    # Comando UBX para reiniciar a estado de fábrica
    command = "$PUBX,41,1,0007,0003,9600,0"
    send_command(ser, command)
    print("Reiniciando módulo a estado de fábrica...")

# Obtener información del módulo
def get_module_info(ser):
    # Comando UBX para obtener información del módulo
    disable_nmea(ser)
    time.sleep(1)
    command = "$PUBX,00"
    send_command(ser, command)
    time.sleep(1)
    enable_nmea(ser)

# Desactivar mensajes NMEA
def disable_nmea(ser):
    # Comando UBX para desactivar todos los mensajes NMEA
    command = b"\xB5\x62\x06\x01\x08\x00\xF0\x00\x00\x00\x00\x00\x00\x01\x00\x24"  # UBX-CFG-MSG
    ser.write(command)
    time.sleep(1)

# Reactivar mensajes NMEA
def enable_nmea(ser):
    # Comando UBX para reactivar los mensajes NMEA
    command = b"\xB5\x62\x06\x01\x08\x00\xF0\x00\x00\x00\x00\x00\x00\x01\x01\x25"  # UBX-CFG-MSG
    ser.write(command)
    time.sleep(1)

# Menú de opciones
def menu():
    print("\n--- Menú de Comandos para NEO-7M-0 ---")
    print("1. Cambiar frecuencia de actualización")
    print("2. Reiniciar a estado de fábrica")
    print("3. Cambiar baudios de transmisión")
    print("4. Obtener información del módulo")
    print("5. Salir")

# Función principal
def main():
    ser = init_serial()
    if ser is None:
        return

    while True:
        menu()
        choice = input("Selecciona una opción: ")

        if choice == '1':
            rate = input("Introduce la nueva frecuencia de actualización (en Hz, ej. 5): ")
            change_update_rate(ser, int(rate))
        elif choice == '2':
            factory_reset(ser)
        elif choice == '3':
            baudrate = input("Introduce el nuevo baudrate (ej. 9600, 38400, 57600, 115200): ")
            change_baudrate(ser, int(baudrate))
        elif choice == '4':
            get_module_info(ser)
        elif choice == '5':
            print("Saliendo...")
            break
        else:
            print("Opción no válida. Inténtalo de nuevo.")

    ser.close()

if __name__ == "__main__":
    main()