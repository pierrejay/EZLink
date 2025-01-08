# Python EZLink Library Documentation

> A lightweight Python implementation of the EZLink protocol for master devices

## Overview

EZLink Python is a minimalist library that implements the EZLink protocol for master devices (typically a Raspberry Pi or computer) communicating with microcontrollers. It handles frame creation, parsing, and basic request/response patterns.

## Installation

### Prerequisites
```bash
pip install pyserial dataclasses
```

### Quick Start
Copy the `ezlink.py` file to your project and import it:
```python
from ezlink import EZLink, EZLinkError
```

## Basic Usage

### 1. Define Message Types
Use Python dataclasses to define your messages. Each message must have:
- `id`: A class variable with the message ID (1-127)
- `format`: A class variable with the struct format string
- Fields that match your microcontroller's structure

```python
from dataclasses import dataclass

@dataclass
class SetLedMsg:
    id = 1           # Must match slave's proto ID
    format = 'B'     # Single unsigned byte
    state: int       # The actual field

@dataclass
class GetStatusMsg:
    id = 3
    format = ''      # Empty payload for simple requests

@dataclass
class StatusResponse:
    id = 3           # Same as request
    format = 'BI'    # Byte + Unsigned Int
    state: int
    uptime: int
```

### 2. Initialize EZLink
```python
comm = EZLink()

# Register all your message types
comm.register_message(SetLedMsg)
comm.register_message(GetStatusMsg)
comm.register_message(StatusResponse)
```

### 3. Send Messages
```python
# Create a message
led_on = SetLedMsg(state=1)

# Build the frame
frame = comm.build_frame(led_on)

# Send it using pyserial
serial.write(frame)
```

### 4. Parse Received Frames
```python
try:
    frame, message = comm.parse_frame(received_data)
    if message:
        print(f"Received: {message}")
except FrameError as e:
    print(f"Invalid frame: {e}")
```

## Complete Example

Here's a complete example showing a master implementation with serial communication:

```python
class EZLinkMaster:
    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = serial.Serial(port, baudrate)
        self.comm = EZLink()
        
        # Register your message types
        self.comm.register_message(SetLedMsg)
        self.comm.register_message(GetStatusMsg)
        self.comm.register_message(StatusResponse)
    
    def send_message(self, message, expect_response=False, timeout=1.0):
        """Send a message and optionally wait for response"""
        frame = self.comm.build_frame(message)
        self.serial.write(frame)
        
        if expect_response:
            start_time = time.time()
            buffer = bytearray()
            
            while (time.time() - start_time) < timeout:
                if self.serial.in_waiting:
                    buffer.extend(self.serial.read(self.serial.in_waiting))
                    try:
                        frame, response = self.comm.parse_frame(buffer)
                        if frame.is_response():
                            return response
                    except FrameError:
                        continue
                time.sleep(0.01)
            raise TimeoutError("No response received")
    
    def close(self):
        self.serial.close()

# Usage example
master = EZLinkMaster('/dev/ttyUSB0')
try:
    # Simple message
    led_on = SetLedMsg(state=1)
    master.send_message(led_on)
    
    # Request with response
    request = GetStatusMsg()
    response = master.send_message(request, expect_response=True)
    print(f"Status: state={response.state}, uptime={response.uptime}ms")
finally:
    master.close()
```

## Protocol Details

### Frame Format
```
+--------+--------+--------+------------+---------+
|  SOF   |  LEN   |   ID   |  PAYLOAD   |  CRC16  |
+--------+--------+--------+------------+---------+
  0xAA     N+5    1-127     N bytes     2 bytes

- SOF: Start of Frame marker (0xAA)
- LEN: Total frame length (including all fields)
- ID: Message identifier (1-127 for requests, 128-255 for responses)
- PAYLOAD: Message data
- CRC16: Frame integrity check
```

### Message Types
The protocol supports several types of messages:
- `MESSAGE`: Simple one-way messages
- `MESSAGE_ACK`: Messages requiring acknowledgment
- `REQUEST`: Messages expecting a specific response
- `RESPONSE`: Responses to requests

### Error Handling
The library provides several error types:
- `EZLinkError`: Base exception class
- `FrameError`: Raised for frame construction/parsing errors

Common error cases:
- Invalid frame length
- CRC mismatch
- Unknown message ID
- Invalid message format
- Communication timeout

## Important Notes

1. **Message IDs**
   - Must be between 1 and 127
   - Bit 7 (0x80) is automatically set for responses
   - Must match the microcontroller's protocol definition

2. **Struct Format**
   - Must exactly match the C struct on the microcontroller
   - Use Python's struct format characters:
     - 'B': unsigned char
     - 'H': unsigned short
     - 'I': unsigned int
     - etc.

3. **Frame Size**
   - Maximum frame size is 32 bytes by default
   - Includes protocol overhead (5 bytes)
   - Can be adjusted if needed

4. **CRC16**
   - Automatically calculated and verified
   - Uses the same polynomial as the C++ implementation

## Best Practices

1. **Message Definition**
   - Keep message definitions in sync with the microcontroller
   - Document the struct format clearly
   - Use meaningful field names

2. **Error Handling**
   - Always handle potential exceptions
   - Implement appropriate timeouts
   - Clean up resources (close serial port)

3. **Buffer Management**
   - Clear receive buffers before expecting responses
   - Handle partial frames appropriately
   - Don't assume frame boundaries match read operations

4. **Testing**
   - Test with actual hardware
   - Verify frame integrity
   - Check timeout behavior

## Limitations

- Only implements master-side functionality
- No automatic retransmission
- Basic timeout handling
- No built-in message queuing

## Contributing

The library is designed to be simple and extensible. Feel free to:
- Add new features
- Improve error handling
- Optimize performance
- Add more examples

## License

MIT License - Feel free to use and modify as needed.