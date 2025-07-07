import socket

class RaspberryPiUDPClient:
    def __init__(self, host: str, port: int = 5005):
        """Initialize UDP client with Raspberry Pi IP and port."""
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((self.host, self.port))

    def send_command(self, command):
        """Send a formatted command to the Raspberry Pi."""
        self.sock.send(command.encode())

    def close(self):
        """Close the socket."""
        self.sock.close()

if __name__ == "__main__":
    client = RaspberryPiUDPClient(host='192.168.1.10', port=5005)

    try:
        while True:
            client.send_command(f'F|{10}')
            # time.sleep(0.5)  # Optional delay
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        client.close()
