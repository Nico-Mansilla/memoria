import socket
import time

# Configura el servidor para escuchar en todas las interfaces
server_ip = '0.0.0.0'  # Escucha en todas las interfaces
server_port = 5000

# Crea un socket TCP
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((server_ip, server_port))
sock.listen(1)

print("Escuchando en {}:{}...".format(server_ip, server_port))

# Acepta la conexión de la Raspberry Pi
while True:
    try:
        conn, addr = sock.accept()
        print("Conexión establecida con {}".format(addr))
        # Aquí puedes añadir el código para manejar la conexión
        try:
            while True:
                # Recibe los datos enviados por la Raspberry Pi
                data = conn.recv(1024)
                if not data:
                    break
                # Decodifica los datos de bytes a str
                decoded_data = data.decode('utf-8')
                print("Datos recibidos: " + decoded_data)
        except KeyboardInterrupt:
            print("Cerrando conexión...")
            conn.close()
    except Exception as e:
        print("Error en la conexión: {}".format(e))
        # Espera un momento antes de intentar aceptar otra conexión
        time.sleep(1)