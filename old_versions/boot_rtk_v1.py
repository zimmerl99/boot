import socket
import base64
import serial
import threading
import time
from datetime import datetime

class NTRIPClient:
    def __init__(self, caster_ip, caster_port, mountpoint, username, password, 
                 xsens_port, baudrate=921600):

        # set all variables for connection
        self.caster_ip = caster_ip
        self.caster_port = caster_port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.xsens_port = xsens_port
        self.baudrate = baudrate
        
        self.ntrip_socket = None
        self.serial_xsens = None
        self.running = False
        self.bytes_received = 0
        self.bytes_forwarded = 0
        self.last_status_time = time.time()
        
    def connect_ntrip(self):
        print(f"\n Connecting to NTRIP caster: {self.caster_ip} at port:{self.caster_port}")
        
        # Create TCP socket
        self.ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ntrip_socket.settimeout(10)
        
        try:
            # Connect to caster
            self.ntrip_socket.connect((self.caster_ip, self.caster_port))
            
            # Create HTTP request with authentication
            auth_string = f"{self.username}:{self.password}"
            auth_bytes = auth_string.encode('utf-8')
            auth_b64 = base64.b64encode(auth_bytes).decode('ascii')
            
            request = (
                f"GET /{self.mountpoint} HTTP/1.1\r\n"
                f"Host: {self.caster_ip}\r\n"
                f"Ntrip-Version: Ntrip/2.0\r\n"
                f"User-Agent: NTRIP PythonClient/1.0\r\n"
                f"Authorization: Basic {auth_b64}\r\n"
                f"Connection: close\r\n"
                f"\r\n"
            )
            
            # Send request
            self.ntrip_socket.sendall(request.encode('utf-8'))
            
            # Receive response
            response = self.ntrip_socket.recv(1024).decode('utf-8', errors='ignore')
            
            # Check if successful
            if "200 OK" in response or "ICY 200 OK" in response:
                print(f"NTRIP authentication successful - Connected to mountpoint: {self.mountpoint}")
                self.ntrip_socket.settimeout(None)
                return True
            else:
                print(f"Authentication failed - Response: {response[:200]}")
                return False
                
        except socket.timeout:
            print("Connection timeout")
            return False
        except ConnectionRefusedError:
            print("Connection refused")
            return False
        except Exception as e:
            print(f"Error: {e}")
            return False
    
    def connect_xsens(self):
        print(f"\nConnecting to Xsens on {self.xsens_port}")
        
        try:
            self.serial_xsens = serial.Serial(
                port=self.xsens_port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=1
            )
            print(f"Xsens connected at {self.baudrate} baudrate")
            
            # Clear any existing data in buffer
            self.serial_xsens.reset_input_buffer()
            self.serial_xsens.reset_output_buffer()
            return True
            
        except serial.SerialException as e:
            print(f"Error: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def forward_rtcm_data(self):
        print("\n[RTCM Thread] Started - forwarding corrections to Xsens")
        buffer_size = 4096
        rtcm_packet_count = 0
        
        while self.running:
            try:
                # Receive RTCM data from NTRIP
                data = self.ntrip_socket.recv(buffer_size)
                
                if not data:
                    print("[RTCM Thread] NTRIP connection closed by server")
                    self.running = False
                    break
                
                self.bytes_received += len(data)
                
                # Count RTCM packets (they start with 0xD3)
                rtcm_packet_count += data.count(b'\xD3')
                # print(data)
                
                # Forward to Xsens
                bytes_written = self.serial_xsens.write(data)
                self.bytes_forwarded += bytes_written
                
                # Print status every 2 seconds
                current_time = time.time()
                if current_time - self.last_status_time >= 2.0:
                    kbytes = self.bytes_forwarded / 1024
                    rate = self.bytes_forwarded / (current_time - self.last_status_time) / 1024
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"RTCM: {kbytes:.1f} KB forwarded | "
                          f"{rtcm_packet_count} packets | "
                          f"{rate:.1f} KB/s")
                    rtcm_packet_count = 0
                    self.bytes_forwarded = 0
                    self.last_status_time = current_time
                
            except socket.timeout:
                continue
            except serial.SerialTimeoutException:
                print("[RTCM Thread] Serial write timeout")
                continue
            except Exception as e:
                print(f"[RTCM Thread] Error: {e}")
                self.running = False
                break
        
        print("[RTCM Thread] Stopped")
    
    def read_xsens_data(self):
        print("[Xsens Monitor] Started - monitoring Xsens output")
        
        packet_count = 0
        last_print = time.time()
        
        while self.running:
            try:
                # Check if data is available
                if self.serial_xsens.in_waiting > 0:
                    # Read available data
                    data = self.serial_xsens.read(self.serial_xsens.in_waiting)
                    
                    # Look for Xsens packet preambles
                    # MTData2 packets start with 0xFA 0xFF
                    if b'\xFA\xFF' in data:
                        packet_count += data.count(b'\xFA\xFF')
                    
                    # Print status every 5 seconds
                    current_time = time.time()
                    if current_time - last_print >= 5.0:
                        if packet_count > 0:
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                                  f"Xsens: Received {packet_count} data packets")
                            packet_count = 0
                        last_print = current_time
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"[Xsens Monitor] Error: {e}")
                break
        
        print("[Xsens Monitor] Stopped")
    
    def start(self):
        print("\nNTRIP RTK CLIENT FOR XSENS MTI-680G")
        
        # Step 1: Connect to NTRIP
        if not self.connect_ntrip():
            print("\nFailed to connect to NTRIP. Exiting.")
            return False
        
        # Step 2: Connect to Xsens
        if not self.connect_xsens():
            print("\nFailed to connect to Xsens. Exiting.")
            return False
        
        # Step 3: Start threads
        print("\nStarting data forwarding")
        self.running = True
        self.last_status_time = time.time()
        
        # Thread 1: Forward RTCM data from NTRIP to Xsens
        rtcm_thread = threading.Thread(target=self.forward_rtcm_data, daemon=True)
        rtcm_thread.start()
        
        # # Thread 2: Monitor Xsens data output
        data_thread = threading.Thread(target=self.read_xsens_data, daemon=True)
        data_thread.start()
    
        print("\nNTRIP Client Running")
    
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()
        
        return True
    
    def stop(self):
        self.running = False
        time.sleep(0.5) 
        
        # Close connections
        if self.ntrip_socket:
            try:
                self.ntrip_socket.close()
                print("Closed NTRIP connection")
            except:
                pass
        
        if self.serial_xsens:
            try:
                self.serial_xsens.close()
                print("Closed Xsens serial port")
            except:
                pass
        
        print(f"\nStatistics:")
        print(f"Total RTCM data received: {self.bytes_received / 1024:.1f} KB")
        print(f"Total RTCM data forwarded: {self.bytes_forwarded / 1024:.1f} KB")


if __name__ == "__main__":
    client = NTRIPClient(
        caster_ip="167.131.109.57",      # ODOT  IP
        caster_port=9879,                 # ODOT port
        mountpoint="LCS1_GNSS",           # Albany MP
        username="GFR1",                  #  username
        password="GFR1",                   # pass 
        xsens_port="/dev/ttyUSB0",         # Xsens usb port on raspi
        baudrate=115200                   # baudrate for serial bus
    )
    
    client.start()