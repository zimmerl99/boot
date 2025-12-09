import socket
import base64
import serial
import threading
import time
from datetime import datetime

class NTRIPClient:
    def __init__(self, caster_ip, caster_port, mountpoint, username, password, 
                 xsens_port, baudrate=921600, log_file=None):

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
        
        # Setup logging
        if log_file is None:
            log_file = f"/home/gfrpi/boot/ntrip_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        
        self.log_file = log_file
        
        self.log_to_file(f"\n{'='*60}")
        self.log_to_file(f"NTRIP Client Session Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self.log_to_file(f"{'='*60}\n")
        
    def log_to_file(self, message):
        # write message to log file
        with open(self.log_file, 'a') as f:
            f.write(message + '\n')
        
    def log(self, message):
        # print to terminal and write to log file
        print(message)
        self.log_to_file(message)

    def check_and_reconnect_serial(self):
        # check if serial port still has connection, reconnect if needed
        try:
            if not self.serial_xsens or not self.serial_xsens.is_open:
                self.log("[Serial] Port closed, attempting reconnect...")
                return self.connect_xsens()
            return True
        except:
            self.log("[Serial] Port disconnected, attempting reconnect...")
            return self.connect_xsens()
        
    def connect_ntrip(self):
        self.log(f"\n Connecting to NTRIP caster: {self.caster_ip} at port:{self.caster_port}")
        
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
                self.log(f"NTRIP authentication successful - Connected to mountpoint: {self.mountpoint}")
                self.ntrip_socket.settimeout(None)
                return True
            else:
                self.log(f"Authentication failed - Response: {response[:200]}")
                return False
                
        except socket.timeout:
            self.log("Connection timeout")
            return False
        except ConnectionRefusedError:
            self.log("Connection refused")
            return False
        except Exception as e:
            self.log(f"Error: {e}")
            return False
    
    def connect_xsens(self):
        self.log(f"\nConnecting to Xsens on {self.xsens_port}")
        
        try:
            self.serial_xsens = serial.Serial(
                port=self.xsens_port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=1
            )
            self.log(f"Xsens connected at {self.baudrate} baudrate")
            
            # Clear any existing data in buffer
            self.serial_xsens.reset_input_buffer()
            self.serial_xsens.reset_output_buffer()
            return True
            
        except serial.SerialException as e:
            self.log(f"Error: {e}")
            return False
        except Exception as e:
            self.log(f"Unexpected error: {e}")
            return False
    
    def forward_rtcm_data(self):
        self.log("\n[RTCM Thread] Started - forwarding corrections to Xsens")
        buffer_size = 4096
        rtcm_packet_count = 0
        reconnect_attempts = 0
        max_reconnect_attempts = 5

        # timer to check if network is disrupted
        last_data_time = time.time() 
        data_timeout = 30
        
        while self.running and reconnect_attempts < max_reconnect_attempts:
            try:
                self.ntrip_socket.settimeout(5)

                # Receive RTCM data from NTRIP
                data = self.ntrip_socket.recv(buffer_size)
                
                if not data:
                    self.log("[RTCM Thread] NTRIP connection closed by server")
                    self.log("[RTCM Thread] Attempting to reconnect...")
                    
                    # Close old socket
                    self.ntrip_socket.close()
                    
                    # Try to reconnect
                    if self.connect_ntrip():
                        self.log("[RTCM Thread] Reconnected successfully")
                        reconnect_attempts = 0
                        continue
                    else:
                        reconnect_attempts += 1
                        self.log(f"[RTCM Thread] Reconnection attempt {reconnect_attempts} failed")
                        time.sleep(10)
                        continue
                reconnect_attempts = 0
                last_data_time = time.time()

                self.bytes_received += len(data)
                
                # Count RTCM packets (they start with 0xD3)
                rtcm_packet_count += data.count(b'\xD3')
                
                # Forward to Xsens
                try:
                    if not self.check_and_reconnect_serial():
                        self.log("[RTCM Thread] Failed to reconnect to Xsens")
                        time.sleep(5)
                        continue
                    bytes_written = self.serial_xsens.write(data)
                    self.bytes_forwarded += bytes_written
                except serial.SerialException as e:
                    self.log(f"[RTCM Thread] Serial error: {e}")
                    try:
                        self.serial_xsens.close()
                    except:
                        pass
                    self.serial_xsens = None
                    time.sleep(5)
                    continue
                except OSError as e:
                    self.log(f"[RTCM Thread] Serial I/O error: {e}")
                    try:
                        self.serial_xsens.close()
                    except:
                        pass
                    self.serial_xsens = None
                    time.sleep(5)
                    continue
                
                # Print status every 2 seconds
                current_time = time.time()
                if current_time - self.last_status_time >= 2.0:
                    kbytes = self.bytes_forwarded / 1024
                    rate = self.bytes_forwarded / (current_time - self.last_status_time) / 1024
                    self.log(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"RTCM: {kbytes:.1f} KB forwarded | "
                          f"{rtcm_packet_count} packets | "
                          f"{rate:.1f} KB/s")
                    rtcm_packet_count = 0
                    self.bytes_forwarded = 0
                    self.last_status_time = current_time
                
            except socket.timeout:
                # Check if exceeded the timeout
                if time.time() - last_data_time > data_timeout:
                    self.log("[RTCM Thread] No data received for 30s, attempting reconnect...")
                    
                    # Close old socket
                    try: 
                        self.ntrip_socket.close() 
                    except: 
                        pass
                    
                    # Try to reconnect
                    if self.connect_ntrip():
                        self.log("[RTCM Thread] Reconnected successfully after timeout")
                        reconnect_attempts = 0 
                        last_data_time = time.time()
                    else:
                        reconnect_attempts += 1
                        self.log(f"[RTCM Thread] Reconnection attempt {reconnect_attempts} failed")
                        time.sleep(10) 
                continue
            except serial.SerialTimeoutException:
                self.log("[RTCM Thread] Serial write timeout")
                continue
            except Exception as e:
                self.log(f"[RTCM Thread] Error: {e}")
                self.running = False
                break
        
        self.log("[RTCM Thread] Stopped")
    
    def read_xsens_data(self):
        self.log("[Xsens Monitor] Started - monitoring Xsens output")
        
        packet_count = 0
        last_print = time.time()
        reconnect_attempts = 0
        max_reconnect_attempts = 5
        
        while self.running and reconnect_attempts < max_reconnect_attempts:
            try:
                # Check if usb is connected
                if not self.serial_xsens or not self.serial_xsens.is_open:
                    self.log("[Xsens Monitor] Serial port closed, waiting for reconnection...")
                    time.sleep(5)
                    continue

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
                            self.log(f"[{datetime.now().strftime('%H:%M:%S')}] "
                                  f"Xsens: Received {packet_count} data packets")
                            packet_count = 0
                        last_print = current_time
                # Reset reconnect counter on successful read
                reconnect_attempts = 0
                time.sleep(0.1)
            except (serial.SerialException, OSError) as e:
                self.log(f"[Xsens Monitor] Error: {e}")
                reconnect_attempts += 1
                self.log(f"[Xsens Monitor] Waiting for serial reconnection (attempt {reconnect_attempts})...")
                time.sleep(5)
                continue
            except Exception as e:
                self.log(f"[Xsens Monitor] Unexpected error: {e}")
                reconnect_attempts += 1
                time.sleep(5)
                continue
        
        if reconnect_attempts >= max_reconnect_attempts:
            self.log("[Xsens Monitor] Max reconnection attempts reached")
        self.log("[Xsens Monitor] Stopped")
    
    def start(self):
        self.log("\nNTRIP RTK CLIENT FOR XSENS MTI-680G")
        
        # Step 1: Connect to NTRIP
        attempts = 0
        connected = False
        while attempts < 10 and not connected:
            if self.connect_ntrip():
                connected = True
                break
            else:
                attempts += 1
                self.log(f"\nFailed to connect to NTRIP (attempt {attempts}). Waiting 10 seconds...")
                if attempts < 10:
                    time.sleep(10)
        if not connected:
            self.log("\nFailed to connect to NTRIP after 10 attempts. Ending script...")
            return False
        

        # Step 2: Connect to Xsens
        attempts = 0
        connected = False
        while attempts < 10 and not connected:
            if self.connect_xsens():
                connected = True
                break
            else:
                attempts += 1
                self.log(f"\nFailed to connect to Xsens (attempt {attempts}). Waiting 10 seconds...")
                if attempts < 10:
                    time.sleep(10)
        if not connected:
            self.log("\nFailed to connect to Xsens after 10 attempts. Ending script...")
            return False
        

        # Step 3: Start threads
        self.log("\nStarting data forwarding")
        self.running = True
        self.last_status_time = time.time()
        
        # Thread 1: Forward RTCM data from NTRIP to Xsens
        rtcm_thread = threading.Thread(target=self.forward_rtcm_data, daemon=True)
        rtcm_thread.start()
        
        # Thread 2: Monitor Xsens data output
        data_thread = threading.Thread(target=self.read_xsens_data, daemon=True)
        data_thread.start()
    
        self.log("\nNTRIP Client Running - Press Ctrl+C to stop\n")
    
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()
        
        return True
    
    def stop(self):
        self.log("\n\nShutting down...")
        self.running = False
        time.sleep(0.5) 
        
        # Close connections
        if self.ntrip_socket:
            try:
                self.ntrip_socket.close()
                self.log("Closed NTRIP connection")
            except:
                pass
        
        if self.serial_xsens:
            try:
                self.serial_xsens.close()
                self.log("Closed Xsens serial port")
            except:
                pass
        
        self.log(f"\nSession Statistics:")
        self.log(f"  Total RTCM data received: {self.bytes_received / 1024:.1f} KB")
        self.log(f"  Total RTCM data forwarded: {self.bytes_forwarded / 1024:.1f} KB")
        self.log(f"\nSession ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self.log(f"{'='*60}\n")


if __name__ == "__main__":
    client = NTRIPClient(
        caster_ip="167.131.109.57",      # ODOT  IP
        caster_port=9879,                 # ODOT port
        mountpoint="LCS1_GNSS",           # Albany MP
        username="GFR1",                  #  username
        password="GFR1",                   # pass 
        xsens_port="/dev/ttyUSB0",         # Xsens usb port
        baudrate=115200,                   # baudrate for serial bus
        log_file="/home/gfrpi/boot/ntrip_session.log"  # log file path
    )
    
    client.start()