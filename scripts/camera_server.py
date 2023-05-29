import io
import socket
import struct
import time
import picamera


# Set the server's IP address and port
server_ip = "10.9.72.244"  # Use "0.0.0.0" to bind to all available network interfaces
server_port = 8000

# Create a socket and bind it to the IP and port
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))
server_socket.listen(0)

# Wait for a connection
print("Waiting for a connection...")
connection = server_socket.accept()[0].makefile("wb")
print("Connected!")

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24

        # Allow the camera to warm up
        time.sleep(2)

        # Create an in-memory stream
        stream = io.BytesIO()

        for _ in camera.capture_continuous(stream, "jpeg", use_video_port=True):
            # Write the length of the captured image
            connection.write(struct.pack("<L", stream.tell()))
            connection.flush()

            # Rewind the stream and send the image data
            stream.seek(0)
            connection.write(stream.read())

            # Reset the stream
            stream.seek(0)
            stream.truncate()
finally:
    connection.close()
    server_socket.close()
