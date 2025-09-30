#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MTi IMU Parser Library
======================
Use this as a Python module to read specific MTi sensor data.
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
        return {"fix": fix_types.get(gps_fix, f"Unknown ({gps_fix})"), "satellites": num_sats}


class MTiParser:
    """Parser for MTi continuous data stream"""

    PREAMBLE = 0xFA
    BUS_ID = 0xFF
    MSG_MTDATA2 = 0x36
    MSG_ERROR = 0x42

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
            return True
        except Exception as e:
            print("Connection failed:", e)
            return False

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

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
                    return parsed
            time.sleep(0.01)
        return None
