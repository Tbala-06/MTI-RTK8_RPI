#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MTi IMU Parser Library with NTRIP RTK Support
==============================================
Use this as a Python module to read specific MTi sensor data
and connect to NTRIP caster for RTK corrections.
"""

import time
import struct
import serial
import socket
import base64
import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict


@dataclass
class IMUData:
    """Container for IMU sensor data"""
    timestamp: float = field(default_factory=time.time)
    packet_counter: Optional[int] = None
    sample_time_fine: Optional[int] = None
    quaternion: Optional[Tuple[float, float, float, float]] = None
    euler_angles: Optional[Tuple[float, float, float]] = None
    acceleration: Optional[Tuple[float, float, float]] = None
    delta_v: Optional[Tuple[float, float, float]] = None
    free_acceleration: Optional[Tuple[float, float, float]] = None
    rate_of_turn: Optional[Tuple[float, float, float]] = None
    delta_q: Optional[Tuple[float, float, float, float]] = None
    magnetic_field: Optional[Tuple[float, float, float]] = None
    baro_pressure: Optional[float] = None
    status_word: Optional[int] = None
    latitude_longitude: Optional[Tuple[float, float]] = None

    def has_data(self) -> bool:
        return any([self.acceleration, self.rate_of_turn, self.quaternion, self.euler_angles])

    def get_gps_info(self) -> Dict:
        if self.status_word is None:
            return {"fix": "Unknown", "satellites": 0}
        gps_fix = (self.status_word >> 2) & 0x07
        num_sats = (self.status_word >> 8) & 0xFF
        fix_types = {0: "No fix", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK"}
        return {"fix": fix_types.get(gps_fix, f"Unknown ({gps_fix})"), "satellites": num_sats}


class NTRIPClient:
    """NTRIP client for RTK corrections"""
    
    def __init__(self, host, port, mountpoint, username, password):
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.socket = None
        self.running = False
        self.thread = None
        self.correction_callback = None
        self.stats = {
            "bytes_received": 0,
            "corrections_sent": 0,
            "connection_errors": 0
        }
    
    def connect(self) -> bool:
        """Connect to NTRIP caster"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.host, self.port))
            
            # Send NTRIP request
            auth_string = f"{self.username}:{self.password}"
            auth_encoded = base64.b64encode(auth_string.encode()).decode()
            
            request = (
                f"GET /{self.mountpoint} HTTP/1.0\r\n"
                f"User-Agent: NTRIP PythonClient/1.0\r\n"
                f"Authorization: Basic {auth_encoded}\r\n"
                f"\r\n"
            )
            
            self.socket.sendall(request.encode())
            
            # Wait for response
            response = self.socket.recv(1024).decode()
            
            if "200 OK" in response or "ICY 200 OK" in response:
                print(f"✓ Connected to NTRIP caster: {self.host}:{self.port}/{self.mountpoint}")
                return True
            else:
                print(f"✗ NTRIP connection failed: {response}")
                self.socket.close()
                return False
                
        except Exception as e:
            print(f"✗ NTRIP connection error: {e}")
            self.stats["connection_errors"] += 1
            return False
    
    def start_corrections(self, callback):
        """Start receiving corrections in background thread"""
        self.correction_callback = callback
        self.running = True
        self.thread = threading.Thread(target=self._correction_loop, daemon=True)
        self.thread.start()
        print("✓ NTRIP correction thread started")
    
    def _correction_loop(self):
        """Background thread to receive and forward corrections"""
        while self.running:
            try:
                data = self.socket.recv(4096)
                if data:
                    self.stats["bytes_received"] += len(data)
                    if self.correction_callback:
                        self.correction_callback(data)
                        self.stats["corrections_sent"] += 1
                else:
                    print("⚠ NTRIP connection closed by server")
                    break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"✗ NTRIP error: {e}")
                self.stats["connection_errors"] += 1
                break
        
        self.running = False
    
    def stop(self):
        """Stop corrections and disconnect"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.socket:
            self.socket.close()
        print("✓ NTRIP client stopped")
    
    def is_connected(self) -> bool:
        return self.running and self.socket is not None


class MTiParser:
    """Parser for MTi continuous data stream with RTK support"""

    PREAMBLE = 0xFA
    BUS_ID = 0xFF
    MSG_MTDATA2 = 0x36
    MSG_ERROR = 0x42

    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.buffer = bytearray()
        self.ntrip = None
        self.stats = {
            "packets_found": 0,
            "packets_parsed": 0,
            "errors": 0,
            "bytes_processed": 0,
            "rtk_corrections": 0
        }

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.3)
            print(f"✓ MTi connected on {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"✗ MTi connection failed: {e}")
            return False

    def disconnect(self):
        if self.ntrip:
            self.ntrip.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✓ MTi disconnected")

    def enable_rtk(self, host, port, mountpoint, username, password) -> bool:
        """
        Enable RTK corrections via NTRIP
        
        Args:
            host: NTRIP caster hostname (e.g., 'rtk2go.com')
            port: NTRIP caster port (usually 2101)
            mountpoint: Mountpoint name (e.g., 'MyBase')
            username: NTRIP username
            password: NTRIP password
        
        Returns:
            True if RTK enabled successfully
        """
        if not self.serial or not self.serial.is_open:
            print("✗ MTi must be connected before enabling RTK")
            return False
        
        self.ntrip = NTRIPClient(host, port, mountpoint, username, password)
        
        if not self.ntrip.connect():
            self.ntrip = None
            return False
        
        # Start forwarding corrections to MTi
        self.ntrip.start_corrections(self._send_rtk_correction)
        return True
    
    def _send_rtk_correction(self, data: bytes):
        """Send RTK correction data to MTi device"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(data)
                self.stats["rtk_corrections"] += 1
        except Exception as e:
            print(f"✗ Failed to send RTK correction: {e}")
    
    def get_rtk_status(self) -> Dict:
        """Get RTK connection status and statistics"""
        if not self.ntrip:
            return {
                "enabled": False,
                "connected": False,
                "bytes_received": 0,
                "corrections_sent": 0
            }
        
        return {
            "enabled": True,
            "connected": self.ntrip.is_connected(),
            "bytes_received": self.ntrip.stats["bytes_received"],
            "corrections_sent": self.ntrip.stats["corrections_sent"],
            "connection_errors": self.ntrip.stats["connection_errors"]
        }

    # --- Internal helpers ---
    def find_message(self) -> Optional[bytes]:
        while len(self.buffer) >= 4:
            if self.buffer[0] != self.PREAMBLE:
                self.buffer.pop(0)
                continue
            if len(self.buffer) < 4:
                return None
            if self.buffer[1] != self.BUS_ID:
                self.buffer.pop(0)
                continue
            mid = self.buffer[2]
            length = self.buffer[3]
            if mid not in [self.MSG_MTDATA2, self.MSG_ERROR]:
                self.buffer.pop(0)
                continue
            msg_size = 4 + length + 1
            if len(self.buffer) < msg_size:
                return None
            message = bytes(self.buffer[:msg_size])
            if self.calculate_checksum(message[1:-1]) != message[-1]:
                self.buffer.pop(0)
                continue
            self.buffer = self.buffer[msg_size:]
            self.stats["packets_found"] += 1
            return message
        return None

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        return (256 - (sum(data) % 256)) & 0xFF

    def parse_message(self, message: bytes) -> Optional[IMUData]:
        if len(message) < 5:
            return None
        mid = message[2]
        if mid == self.MSG_ERROR:
            self.stats["errors"] += 1
            return None
        if mid != self.MSG_MTDATA2:
            return None
        data = IMUData()
        payload = message[4:-1]
        offset = 0
        while offset + 3 <= len(payload):
            data_id = struct.unpack('>H', payload[offset:offset+2])[0]
            length = payload[offset+2]
            offset += 3
            if offset + length > len(payload):
                break
            field_data = payload[offset:offset+length]
            try:
                if data_id == 0x2030 and length == 12:
                    data.euler_angles = struct.unpack('>fff', field_data)
                elif data_id == 0x5040 and length >= 8:
                    lat = struct.unpack('>f', field_data[0:4])[0]
                    lon = struct.unpack('>f', field_data[4:8])[0]
                    data.latitude_longitude = (lat, lon)
                elif data_id == 0x2010 and length == 16:
                    data.quaternion = struct.unpack('>ffff', field_data)
                elif data_id == 0x4020 and length == 12:
                    data.acceleration = struct.unpack('>fff', field_data)
                elif data_id == 0x8020 and length == 12:
                    data.rate_of_turn = struct.unpack('>fff', field_data)
                elif data_id == 0x1020 and length == 2:
                    data.packet_counter = struct.unpack('>H', field_data)[0]
                elif data_id == 0xE020 and length == 4:
                    data.status_word = struct.unpack('>I', field_data)[0]
            except struct.error:
                pass
            offset += length
        return data if data.has_data() or data.latitude_longitude else None

    # --- Public API ---
    def read_euler(self, timeout=2.0) -> Optional[Tuple[float, float, float]]:
        data = self.read_data(timeout)
        return data.euler_angles if data else None

    def read_latlon(self, timeout=2.0) -> Optional[Tuple[float, float]]:
        data = self.read_data(timeout)
        return data.latitude_longitude if data else None

    def read_data(self, timeout=2.0) -> Optional[IMUData]:
        if not self.serial or not self.serial.is_open:
            return None
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                self.buffer.extend(self.serial.read(self.serial.in_waiting))
                self.stats["bytes_processed"] += len(self.buffer)
            message = self.find_message()
            if message:
                parsed = self.parse_message(message)
                if parsed:
                    self.stats["packets_parsed"] += 1
                    return parsed
            time.sleep(0.01)
        return None
    
    def print_stats(self):
        """Print connection and RTK statistics"""
        print("\n" + "="*50)
        print("MTi Statistics:")
        print(f"  Packets found: {self.stats['packets_found']}")
        print(f"  Packets parsed: {self.stats['packets_parsed']}")
        print(f"  Errors: {self.stats['errors']}")
        print(f"  Bytes processed: {self.stats['bytes_processed']}")
        
        rtk = self.get_rtk_status()
        print("\nRTK Status:")
        print(f"  Enabled: {rtk['enabled']}")
        print(f"  Connected: {rtk['connected']}")
        if rtk['enabled']:
            print(f"  Corrections received: {rtk['bytes_received']} bytes")
            print(f"  Corrections sent to MTi: {rtk['corrections_sent']}")
            print(f"  Connection errors: {rtk['connection_errors']}")
        print("="*50 + "\n")


# Example usage
if __name__ == "__main__":
    # Initialize parser
    mti = MTiParser(port='COM3', baudrate=115200)  # Adjust port for your system
    
    if not mti.connect():
        exit(1)
    
    # Enable RTK corrections (optional)
    # Uncomment and configure with your NTRIP caster details
    """
    if mti.enable_rtk(
        host='rtk2go.com',          # NTRIP caster
        port=2101,                   # NTRIP port
        mountpoint='YourMountpoint', # Your mountpoint
        username='your_email',       # Your email/username
        password='none'              # Password (often 'none' for public)
    ):
        print("✓ RTK corrections enabled")
        print("⏳ Waiting for RTK fix...")
    """
    
    print("\nReading MTi data (Ctrl+C to stop)...\n")
    
    try:
        while True:
            data = mti.read_data(timeout=1.0)
            
            if data:
                if data.latitude_longitude:
                    lat, lon = data.latitude_longitude
                    gps_info = data.get_gps_info()
                    print(f"GPS: {lat:.6f}, {lon:.6f} | "
                          f"Fix: {gps_info['fix']} | "
                          f"Sats: {gps_info['satellites']}")
                
                if data.euler_angles:
                    roll, pitch, yaw = data.euler_angles
                    print(f"Euler: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n⏹ Stopping...")
    
    finally:
        mti.print_stats()
        mti.disconnect()
        print("✓ Done")
