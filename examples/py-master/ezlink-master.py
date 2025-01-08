
"""
EZLinkPy - Simple communication protocol Python implementation (Master side)

This module provides a lightweight implementation of the EZLink protocol
for Python, focusing on the master side functionality.
"""

import struct
import enum
from typing import Optional, Tuple, Union, Dict
from dataclasses import dataclass

# Protocol constants
START_OF_FRAME = 0xAA
MAX_FRAME_SIZE = 32
FRAME_OVERHEAD = 5  # SOF + LEN + ID + CRC16

class ProtoType(enum.Enum):
    """Message types supported by EZLink"""
    MESSAGE = 0       # Simple message without response
    MESSAGE_ACK = 1   # Message requiring acknowledgment
    REQUEST = 2       # Request expecting a specific response
    RESPONSE = 3      # Response to a request

class EZLinkError(Exception):
    """Base class for EZLink exceptions"""
    pass

class FrameError(EZLinkError):
    """Raised when there's an error building or parsing a frame"""
    pass

@dataclass
class Frame:
    """Represents a EZLink frame"""
    id: int
    payload: bytes
    
    @property
    def length(self) -> int:
        """Total frame length including overhead"""
        return len(self.payload) + FRAME_OVERHEAD

    def is_response(self) -> bool:
        """Check if this frame is a response (ID bit 7 set)"""
        return bool(self.id & 0x80)

def calculate_crc16(data: bytes) -> int:
    """Calculate CRC16 for given data"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class EZLink:
    """
    EZLink protocol implementation for Python (Master side)
    
    This class provides methods to build and parse EZLink frames,
    with a focus on the master side functionality (sending requests
    and receiving responses).
    """
    
    def __init__(self):
        """Initialize the EZLink instance"""
        # Keep track of registered message types for parsing
        self._message_types: Dict[int, type] = {}
    
    def register_message(self, msg_class: type) -> None:
        """
        Register a message class for parsing.
        The class must have:
            - id: class variable with message ID (1-127)
            - format: class variable with struct format string
            
        Example:
            @dataclass
            class SetLedMsg:
                id = 1
                format = 'B'  # Single unsigned byte
                state: int
        """
        if not hasattr(msg_class, 'id') or not hasattr(msg_class, 'format'):
            raise ValueError("Message class must have 'id' and 'format' attributes")
        
        msg_id = msg_class.id
        if not 0 < msg_id < 128:
            raise ValueError("Message ID must be between 1 and 127")
            
        self._message_types[msg_id] = msg_class
    
    def build_frame(self, message) -> bytes:
        """
        Build a EZLink frame for a message object.
        The message object must be an instance of a registered message class.
        """
        msg_class = type(message)
        if msg_class not in self._message_types.values():
            raise FrameError("Message type not registered")
            
        # Pack the payload according to the format
        try:
            if hasattr(message, '__dataclass_fields__'):
                # If it's a dataclass, get fields in order
                values = [getattr(message, field) for field in message.__dataclass_fields__]
                payload = struct.pack(message.format, *values)
            else:
                # Otherwise assume it's a simple message with all attributes in format
                payload = struct.pack(message.format, *vars(message).values())
        except struct.error as e:
            raise FrameError(f"Failed to pack message: {e}")
            
        frame = Frame(id=message.id, payload=payload)
        
        if frame.length > MAX_FRAME_SIZE:
            raise FrameError("Frame too large")
            
        # Build the complete frame
        frame_data = bytes([
            START_OF_FRAME,
            frame.length,
            frame.id
        ]) + frame.payload
        
        # Calculate and append CRC16
        crc = calculate_crc16(frame_data)
        frame_data += struct.pack('>H', crc)
        
        return frame_data
        
    def parse_frame(self, data: bytes) -> Tuple[Frame, Optional[object]]:
        """
        Parse raw bytes into a Frame object and optionally the decoded message.
        
        Returns:
            tuple: (Frame object, Decoded message object or None if type not registered)
            
        Raises:
            FrameError: If the frame is invalid
        """
        if len(data) < FRAME_OVERHEAD:
            raise FrameError("Frame too short")
            
        if data[0] != START_OF_FRAME:
            raise FrameError("Invalid start of frame")
            
        length = data[1]
        if length > MAX_FRAME_SIZE or length != len(data):
            raise FrameError("Invalid frame length")
            
        # Verify CRC
        received_crc = struct.unpack('>H', data[-2:])[0]
        calculated_crc = calculate_crc16(data[:-2])
        if received_crc != calculated_crc:
            raise FrameError("CRC mismatch")
            
        frame = Frame(
            id=data[2],
            payload=data[3:-2]
        )
        
        # Try to decode the message if we have the type registered
        message = None
        msg_id = frame.id & 0x7F  # Clear response bit
        if msg_id in self._message_types:
            msg_class = self._message_types[msg_id]
            try:
                values = struct.unpack(msg_class.format, frame.payload)
                if hasattr(msg_class, '__dataclass_fields__'):
                    # If it's a dataclass, construct with field names
                    fields = list(msg_class.__dataclass_fields__.keys())
                    message = msg_class(**dict(zip(fields, values)))
                else:
                    # Otherwise construct with positional arguments
                    message = msg_class(*values)
            except (struct.error, TypeError) as e:
                raise FrameError(f"Failed to unpack message: {e}")
                
        return frame, message