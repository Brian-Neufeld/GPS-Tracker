import socket

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server.bind(("10.0.0.15", 5000))
server.listen(1)

print("Waiting for ESP32...")

while True:
    conn, addr = server.accept()

    #print("Connected:", addr)

    while True:
        data = conn.recv(1024)

        if not data:
            break

        print("Received:", data.decode())

        with open(r"C:\Users\Brian\ESP-IDF_GPS_LOGGING\logfile.txt", "a", encoding="utf-8") as file:
            file.write(data.decode())

        #conn.sendall(data)

conn.close()
