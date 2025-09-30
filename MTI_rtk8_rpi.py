#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MTi IMU Parser - Thonny Compatible Version
===========================================
Simple, clean parser for MTi sensor data
Works on Raspberry Pi with Thonny IDE
"""

import time
import struct
import serial
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
        return {"fix": fix_types.get(gps_fix, "Unknown ({})".format(gps_fix)), "satellites": num_sats}


class MTiParser:
    """Parser for MTi continuous data stream"""
    
    # Preamble and message IDs
    PREAMBLE = 0xFA
    BUS_ID = 0xFF
    MSG_MTDATA2 = 0x36
    MSG_ERROR = 0x42
    
    # Data field identifiers
    DATA_IDS = {
        0x1020: ("PacketCounter", 2),
        0x1060: ("SampleTimeFine", 4),
        0x2010: ("Quaternion", 16),
        0x2030: ("EulerAngles", 12),
        0x4020: ("Acceleration", 12),
        0x4010: ("DeltaV", 12),
        0x4030: ("FreeAcceleration", 12),
        0x8020: ("RateOfTurn", 12),
        0x8030: ("DeltaQ", 16),
        0xC020: ("MagneticField", 12),
        0x3010: ("BaroPressure", 4),
        0xE020: ("StatusWord", 4),
        0x5040: ("LatLon", 8),
        0x5042: ("LatLon", 12),
    }
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.buffer = bytearray()
        self.stats = {
            "packets_found": 0,
            "packets_parsed": 0,
            "errors": 0,
            "bytes_processed": 0
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
            print("Connected to {} at {} baud".format(self.port, self.baudrate))
            return True
        except Exception as e:
            print("Connection failed: {}".format(e))
            return False
    
    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")
    
    def find_message(self) -> Optional[bytes]:
        """Find and extract a complete message from buffer"""
        while len(self.buffer) >= 4:
            # Look for preamble
            if self.buffer[0] != self.PREAMBLE:
                self.buffer.pop(0)
                continue
            
            # Check if we have enough for header
            if len(self.buffer) < 4:
                return None
            
            # Validate header: FA FF [MID] [LEN]
            if self.buffer[1] != self.BUS_ID:
                self.buffer.pop(0)
                continue
            
            mid = self.buffer[2]
            length = self.buffer[3]
            
            # Check if it's a known message type
            if mid not in [self.MSG_MTDATA2, self.MSG_ERROR]:
                self.buffer.pop(0)
                continue
            
            # Check if we have the complete message
            msg_size = 4 + length + 1
            if len(self.buffer) < msg_size:
                return None
            
            # Extract message
            message = bytes(self.buffer[:msg_size])
            
            # Verify checksum
            checksum = self.calculate_checksum(message[1:-1])
            if checksum != message[-1]:
                print("Checksum error")
                self.buffer.pop(0)
                continue
            
            # Valid message found
            self.buffer = self.buffer[msg_size:]
            self.stats["packets_found"] += 1
            
            return message
        
        return None
    
    def calculate_checksum(self, data: bytes) -> int:
        """Calculate MTi checksum"""
        return (256 - (sum(data) % 256)) & 0xFF
    
    def parse_message(self, message: bytes) -> Optional[IMUData]:
        """Parse a complete message into IMUData"""
        if len(message) < 5:
            return None
        
        mid = message[2]
        
        # Handle error messages
        if mid == self.MSG_ERROR:
            error_code = message[4]
            self.stats["errors"] += 1
            print("Error message: code={}".format(error_code))
            return None
        
        # Handle MTData2 messages
        if mid != self.MSG_MTDATA2:
            return None
        
        try:
            data = IMUData()
            payload = message[4:-1]
            offset = 0
            fields_parsed = 0
            
            while offset + 3 <= len(payload):
                # Read data ID and length
                data_id = struct.unpack('>H', payload[offset:offset+2])[0]
                length = payload[offset+2]
                offset += 3
                
                # Check if we have enough data
                if offset + length > len(payload):
                    break
                
                field_data = payload[offset:offset+length]
                
                # Parse known fields
                try:
                    if data_id == 0x1020 and length == 2:
                        data.packet_counter = struct.unpack('>H', field_data)[0]
                        fields_parsed += 1
                    
                    elif data_id == 0x1060 and length == 4:
                        data.sample_time_fine = struct.unpack('>I', field_data)[0]
                        fields_parsed += 1
                    
                    elif data_id == 0x2010 and length == 16:
                        data.quaternion = struct.unpack('>ffff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x2030 and length == 12:
                        data.euler_angles = struct.unpack('>fff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x4020 and length == 12:
                        data.acceleration = struct.unpack('>fff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x4010 and length == 12:
                        data.delta_v = struct.unpack('>fff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x4030 and length == 12:
                        data.free_acceleration = struct.unpack('>fff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x8020 and length == 12:
                        data.rate_of_turn = struct.unpack('>fff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x8030 and length == 16:
                        data.delta_q = struct.unpack('>ffff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0xC020 and length == 12:
                        data.magnetic_field = struct.unpack('>fff', field_data)
                        fields_parsed += 1
                    
                    elif data_id == 0x3010 and length == 4:
                        data.baro_pressure = struct.unpack('>I', field_data)[0]
                        fields_parsed += 1
                    
                    elif data_id == 0xE020 and length == 4:
                        data.status_word = struct.unpack('>I', field_data)[0]
                        fields_parsed += 1
                    
                    elif data_id == 0x5040 and length == 8:
                        # Standard format: 2x 32-bit floats
                        lat = struct.unpack('>f', field_data[0:4])[0]
                        lon = struct.unpack('>f', field_data[4:8])[0]
                        data.latitude_longitude = (lat, lon)
                        fields_parsed += 1
                    
                    elif data_id == 0x5042 and length == 12:
                        # Extended format: 2x 32-bit floats + extra
                        lat = struct.unpack('>f', field_data[0:4])[0]
                        lon = struct.unpack('>f', field_data[4:8])[0]
                        data.latitude_longitude = (lat, lon)
                        fields_parsed += 1
                
                except struct.error as e:
                    print("Parse error for ID {:04X}: {}".format(data_id, e))
                
                offset += length
            
            if fields_parsed >= 2:
                self.stats["packets_parsed"] += 1
                return data
            
            return None
            
        except Exception as e:
            print("Message parse error: {}".format(e))
            return None
    
    def read_data(self, timeout=2.0) -> Optional[IMUData]:
        """Read and parse a data packet"""
        if not self.serial or not self.serial.is_open:
            return None
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Read available data
            if self.serial.in_waiting > 0:
                new_data = self.serial.read(self.serial.in_waiting)
                self.buffer.extend(new_data)
                self.stats["bytes_processed"] += len(new_data)
            
            # Try to find and parse a message
            message = self.find_message()
            if message:
                parsed = self.parse_message(message)
                if parsed and parsed.has_data():
                    return parsed
            
            time.sleep(0.01)
        
        return None
    
    def print_data(self, data: IMUData):
        """Print IMU data"""
        print("\n" + "="*70)
        
        if data.packet_counter is not None:
            print("Packet: {:5d}  ".format(data.packet_counter), end="")
        if data.sample_time_fine is not None:
            print("Time: {:10d}  ".format(data.sample_time_fine), end="")
        print("Timestamp: {:.3f}".format(data.timestamp))
        
        if data.quaternion:
            q0, q1, q2, q3 = data.quaternion
            print("Quat:   q0={:8.5f}  q1={:8.5f}  q2={:8.5f}  q3={:8.5f}".format(q0, q1, q2, q3))
        
        if data.euler_angles:
            roll, pitch, yaw = data.euler_angles
            print("Euler:  Roll={:7.2f} deg  Pitch={:7.2f} deg  Yaw={:7.2f} deg".format(roll, pitch, yaw))
        
        if data.acceleration:
            ax, ay, az = data.acceleration
            mag = (ax**2 + ay**2 + az**2)**0.5
            print("Accel:  X={:8.4f}  Y={:8.4f}  Z={:8.4f} m/s2  |a|={:7.4f}".format(ax, ay, az, mag))
        
        if data.rate_of_turn:
            gx, gy, gz = data.rate_of_turn
            print("Gyro:   X={:8.5f}  Y={:8.5f}  Z={:8.5f} rad/s".format(gx, gy, gz))
        
        if data.magnetic_field:
            mx, my, mz = data.magnetic_field
            print("Mag:    X={:8.5f}  Y={:8.5f}  Z={:8.5f} a.u.".format(mx, my, mz))
        
        if data.latitude_longitude:
            lat, lon = data.latitude_longitude
            print("GPS:    Lat={:11.7f} deg  Lon={:11.7f} deg".format(lat, lon))
        
        if data.baro_pressure:
            print("Baro:   {} Pa".format(data.baro_pressure))
        
        if data.status_word is not None:
            gps_info = data.get_gps_info()
            print("Status: GPS {}, {} sats  (0x{:08X})".format(
                gps_info['fix'], gps_info['satellites'], data.status_word))
    
    def run_continuous(self, duration=30):
        """Continuously read and display data"""
        print("\n" + "="*70)
        print("Reading data for {} seconds...".format(duration))
        print("="*70)
        
        start_time = time.time()
        sample_count = 0
        last_print = 0
        
        try:
            while time.time() - start_time < duration:
                data = self.read_data(timeout=0.5)
                
                if data:
                    sample_count += 1
                    self.print_data(data)
                else:
                    elapsed = time.time() - start_time
                    if elapsed - last_print > 1.0:
                        print("\rWaiting... {}s  Buffer: {} bytes  Found: {}  Parsed: {}  Errors: {}".format(
                            int(elapsed), len(self.buffer), 
                            self.stats['packets_found'], self.stats['packets_parsed'], 
                            self.stats['errors']), end="", flush=True)
                        last_print = elapsed
                
                time.sleep(0.02)
        
        except KeyboardInterrupt:
            print("\n\nStopped by user")
        
        print("\n" + "="*70)
        print("Statistics:")
        print("  Samples displayed: {}".format(sample_count))
        print("  Packets found: {}".format(self.stats['packets_found']))
        print("  Packets parsed: {}".format(self.stats['packets_parsed']))
        print("  Error messages: {}".format(self.stats['errors']))
        print("  Bytes processed: {}".format(self.stats['bytes_processed']))
        if self.stats['packets_found'] > 0:
            rate = (self.stats['packets_parsed'] / self.stats['packets_found']) * 100
            print("  Parse success rate: {:.1f}%".format(rate))
        print("="*70)
        
        return sample_count


def main():
    """Main function"""
    print("MTi IMU Parser - Thonny Compatible")
    print("=" * 70)
    
    parser = MTiParser(port='/dev/serial0', baudrate=115200)
    
    if not parser.connect():
        print("Failed to connect")
        return
    
    print("\nChecking for data...")
    time.sleep(0.5)
    
    if parser.serial.in_waiting > 0:
        sample = parser.serial.read(min(50, parser.serial.in_waiting))
        print("Receiving data: {}".format(' '.join('{:02X}'.format(b) for b in sample[:32])))
    else:
        print("No data received - device may need configuration")
        parser.disconnect()
        return
    
    try:
        parser.run_continuous(duration=60)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        parser.disconnect()


if __name__ == "__main__":
    main()
