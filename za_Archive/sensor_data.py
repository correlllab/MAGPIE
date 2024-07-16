import socket
import binascii

def read_sensor_data(sensorip, port):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((sensorip, port))
            data = s.recv(1024)
            return binascii.hexlify(data).decode()
    except socket.error as e:
        print(f"Socket Error: {e}")
        return None
    


    """
    port = 
    data = read_sensor_data(sensorip, port)
    if(data):
        print(data)
"""

