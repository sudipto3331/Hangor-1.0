import socket
from pymavlink import mavutil
import time
import subprocess

def check_ping(ip):
    """ Check if the device is reachable via ping """
    print(f"[INFO] Pinging {ip}...")
    response = subprocess.run(['ping', '-n', '1', ip], stdout=subprocess.PIPE)
    if "TTL" in response.stdout.decode():
        print(f"[SUCCESS] {ip} is reachable!")
        return True
    else:
        print(f"[ERROR] {ip} is not reachable.")
        return False

def check_port(ip, port):
    """ Check if a specific UDP port is open on the given IP """
    print(f"[INFO] Checking if port {port} is open on {ip}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1)
    try:
        sock.sendto(b'ping', (ip, port))
        sock.recvfrom(1024)  # waiting for response
        print(f"[SUCCESS] Port {port} is open on {ip}")
        return True
    except socket.timeout:
        print(f"[ERROR] Port {port} is not open on {ip}")
        return False
    except Exception as e:
        print(f"[ERROR] Error while checking port {port}: {str(e)}")
        return False

def test_connection():
    ip = "192.168.2.2"  # BlueOS IP
    port = 14550  # MAVLink port
    local_ip = "192.168.2.125"  # Your local machine's IP
    
    # Step 1: Check if BlueOS is reachable
    if not check_ping(ip):
        return "[ERROR] BlueOS device is not reachable. Please check network configuration."
    
    # Step 2: Check if the port is open on BlueOS
    if not check_port(ip, port):
        return f"[ERROR] Port {port} on {ip} is not accessible. Please check firewall settings or network configuration."
    
    # Step 3: Try to establish MAVLink connection
    try:
        print("[INFO] Attempting to connect to MAVLink...")
        master = mavutil.mavlink_connection(f'udpin:{ip}:{port}')
        master.wait_heartbeat()
        print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")
        return "[SUCCESS] MAVLink connection established."
    except Exception as e:
        return f"[ERROR] Failed to connect via MAVLink: {str(e)}"

# Main function to run the troubleshooting steps
if __name__ == "__main__":
    result = test_connection()
    print(result)
