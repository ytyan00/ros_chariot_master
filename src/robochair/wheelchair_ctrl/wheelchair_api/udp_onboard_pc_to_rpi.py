# import socket

# class RaspberryPiUDPClient:
#     def __init__(self, host: str, port: int = 5005):
#         """Initialize UDP client with Raspberry Pi IP and port."""
#         self.host = host
#         self.port = port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.connect((self.host, self.port))

#     def send_command(self, command):
#         """Send a formatted command to the Raspberry Pi."""
#         self.sock.send(command.encode())

#     def close(self):
#         """Close the socket."""
#         self.sock.close()

# if __name__ == "__main__":
#     client = RaspberryPiUDPClient(host='192.168.1.101', port=5005)

#     try:
#         while True:
#             client.send_command(f'B|{200}')
#             # time.sleep(0.5)  # Optional delay
#     except KeyboardInterrupt:
#         print("Stopped by user.")
#     finally:
#         client.close()


import socket
import time

class RaspberryPiUDPClient:
    def __init__(self, host: str, port: int = 5005):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, command):
        self.sock.sendto(command.encode(), (self.host, self.port))

    def close(self):
        self.sock.close()

if __name__ == "__main__":
    client = RaspberryPiUDPClient(host='192.168.1.101', port=5005)

    try:
        while True:
            client.send_command(f'F|{50}')
            print("send")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        client.close()
