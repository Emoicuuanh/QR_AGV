#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Wareshare Modbus TCP Communication Module
Provides read and write functions for Modbus TCP communication
"""
import socket
import time
import rospy

class WareshareModbus:
    """Class to handle Modbus TCP communication with Wareshare device"""
    
    def __init__(self, host='192.168.1.200', port=502, device_address=0x01):
        """
        Initialize Wareshare Modbus connection
        
        Args:
            host (str): IP address of the Modbus device
            port (int): Port number (default 502 for Modbus TCP)
            device_address (int): Modbus device/slave address (default 0x01)
        """
        self.host = host
        self.port = port
        self.device_address = device_address
        self.socket = None
        self.connected = False
        
    def connect(self):
        """Establish connection to Modbus device"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            rospy.loginfo(f"Connected to Wareshare at {self.host}:{self.port}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to connect to Wareshare: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close connection to Modbus device"""
        if self.socket:
            try:
                self.socket.close()
                self.connected = False
                rospy.loginfo("Disconnected from Wareshare")
            except Exception as e:
                rospy.logerr(f"Error disconnecting: {e}")
    
    def _build_modbus_frame(self, function_code, coil_address, value):
        """
        Build Modbus TCP frame
        
        Args:
            function_code (int): Modbus function code (0x05 for write single coil)
            coil_address (int): Address of the coil (0-7)
            value (int): Value to write (0xFF00 for ON, 0x0000 for OFF)
            
        Returns:
            bytearray: Modbus TCP frame
        """
        frame = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        # MBAP Header
        frame[0] = 0x00  # Transaction ID (high byte)
        frame[1] = 0x00  # Transaction ID (low byte)
        frame[2] = 0x00  # Protocol ID (high byte) - always 0 for Modbus
        frame[3] = 0x00  # Protocol ID (low byte) - always 0 for Modbus
        frame[4] = 0x00  # Length (high byte)
        frame[5] = 0x06  # Length (low byte) - 6 bytes following
        
        # PDU (Protocol Data Unit)
        frame[6] = self.device_address  # Device/Unit address
        frame[7] = function_code         # Function code
        frame[8] = (coil_address >> 8) & 0xFF  # Coil address (high byte)
        frame[9] = coil_address & 0xFF         # Coil address (low byte)
        frame[10] = (value >> 8) & 0xFF        # Value (high byte)
        frame[11] = value & 0xFF               # Value (low byte)
        
        return bytearray(frame)
    
    def write_single_coil(self, coil_address, state):
        """
        Write single coil (Modbus function code 0x05)
        
        Args:
            coil_address (int): Coil address (0-7)
            state (bool): True for ON (0xFF00), False for OFF (0x0000)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.connected:
            rospy.logwarn("Not connected to Wareshare. Call connect() first.")
            return False
        
        try:
            # 0xFF00 for ON, 0x0000 for OFF
            value = 0xFF00 if state else 0x0000
            frame = self._build_modbus_frame(0x05, coil_address, value)
            
            rospy.logdebug(f"Writing coil {coil_address}: {'ON' if state else 'OFF'} - Frame: {list(frame)}")
            self.socket.send(frame)
            
            # Optional: Read response
            # response = self.socket.recv(12)
            # rospy.logdebug(f"Response: {list(response)}")
            
            return True
        except Exception as e:
            rospy.logerr(f"Error writing coil {coil_address}: {e}")
            return False
    
    def write_coils_sequence(self, coil_addresses, state):
        """
        Write multiple coils in sequence
        
        Args:
            coil_addresses (list): List of coil addresses to write
            state (bool): True for ON, False for OFF
            delay (float): Delay between writes in seconds
            
        Returns:
            bool: True if all writes successful
        """
        success = True
        for addr in coil_addresses:
            if not self.write_single_coil(addr, state):
                success = False
        return success
    
    def write_all_coils(self, state):
        """
        Write all coils (0-7) to the same state
        
        Args:
            state (bool): True for ON, False for OFF
        """
        return self.write_coils_sequence(range(8), state)
    
    def read_coils(self, start_address=0, count=8, retry=3):
        """
        Read relay status (Command 0x01)
        Based on Wareshare protocol documentation
        
        Send code format: 01 01 00 00 00 08 3D CC
        - 01: Device Address
        - 01: Command (Query relay status)
        - 00 00: Relay Start Address (0x0000 - 0x0007)
        - 00 08: Relay Number (number of relays to read)
        - 3D CC: CRC16 checksum
        
        Args:
            start_address (int): Starting relay address (0-7)
            count (int): Number of relays to read (max 8)
            retry (int): Number of retry attempts
            
        Returns:
            list: List of relay states (True/False), or None if error
        """
        if not self.connected:
            rospy.logwarn("Not connected to Wareshare. Call connect() first.")
            return None
        
        for attempt in range(retry):
            try:
                # Flush any pending data in receive buffer
                self.socket.settimeout(0.1)
                try:
                    while True:
                        old_data = self.socket.recv(1024)
                        if not old_data:
                            break
                except socket.timeout:
                    pass  # Buffer is empty, good
                
                # Build read relay status frame
                frame = bytearray(8)
                frame[0] = self.device_address  # Device address (0x01)
                frame[1] = 0x01                 # Command: Query relay status
                frame[2] = (start_address >> 8) & 0xFF  # Start address high byte
                frame[3] = start_address & 0xFF         # Start address low byte
                frame[4] = (count >> 8) & 0xFF          # Relay number high byte
                frame[5] = count & 0xFF                 # Relay number low byte
                
                # Calculate CRC16
                crc = self._calculate_crc16(frame[:6])
                frame[6] = crc & 0xFF           # CRC low byte
                frame[7] = (crc >> 8) & 0xFF    # CRC high byte
                
                # Debug: print the frame
                frame_hex = ' '.join([f'{b:02X}' for b in frame])
                print(f"[Attempt {attempt+1}] Sending: {frame_hex}")
                rospy.loginfo(f"Sending read command: {frame_hex}")
                
                # Set socket timeout for response
                self.socket.settimeout(2.0)
                
                # Send command
                sent = self.socket.send(frame)
                print(f"Sent {sent} bytes")
                
                # Small delay to let device process
                time.sleep(0.05)
                
                # Receive response
                # Response format: 01 01 01 00 51 88
                response = self.socket.recv(1024)
                
                if len(response) < 5:
                    print(f"Response too short: {len(response)} bytes")
                    continue
                
                # Debug: print the response
                response_hex = ' '.join([f'{b:02X}' for b in response])
                print(f"Received: {response_hex} ({len(response)} bytes)")
                rospy.loginfo(f"Received response: {response_hex}")
                
                # Parse response
                device_addr = response[0]
                command = response[1]
                byte_count = response[2]
                
                if device_addr != self.device_address or command != 0x01:
                    print(f"Invalid header: device={device_addr:02X}, cmd={command:02X}")
                    continue
                
                # Extract relay status bytes
                relay_states = []
                for i in range(byte_count):
                    status_byte = response[3 + i]
                    # Each bit represents one relay (bit0=relay0, bit1=relay1, etc.)
                    for bit in range(8):
                        if len(relay_states) < count:
                            relay_states.append(bool(status_byte & (1 << bit)))
                
                print(f"✓ Success! Relay states: {relay_states}")
                return relay_states[:count]
                
            except socket.timeout:
                print(f"[Attempt {attempt+1}] Timeout waiting for response")
                if attempt < retry - 1:
                    time.sleep(0.2)
                    continue
            except Exception as e:
                print(f"[Attempt {attempt+1}] Error: {e}")
                if attempt < retry - 1:
                    time.sleep(0.2)
                    continue
        
        # All retries failed
        rospy.logerr("Failed to read relays after all retries")
        print("✗ Failed to read relays after all retries")
        return None
    
    def _calculate_crc16(self, data):
        """
        Calculate CRC16 checksum for Modbus RTU
        
        Args:
            data (bytearray): Data to calculate CRC for
            
        Returns:
            int: CRC16 value
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

# ============================================================
# Example usage and testing
# ============================================================
if __name__ == "__main__":
    # Initialize ROS node (optional, for logging)
    try:
        rospy.init_node('wareshare_modbus_test', anonymous=True)
    except:
        pass
    
    # Create Wareshare Modbus instance
    wareshare = WareshareModbus(host='192.168.1.200', port=502, device_address=0x01)
    
    # Connect to device
    if not wareshare.connect():
        print("Failed to connect!")
        exit(1)
    while True:
        # wareshare.write_single_coil(0, False)
        # time.sleep(3)
        # wareshare.write_single_coil(0, True)
        # time.sleep(3)
        # wareshare.write_all_coils(False)
        # time.sleep(3)
        print("Reading coil status...")
        coil_states = wareshare.read_coils(0, 8)
        print("Reading coil status...")
        if coil_states:
            for i, state in enumerate(coil_states):
                print(f"  Coil {i}: {'ON' if state else 'OFF'}")
    
    # try:
    #     print("\n" + "="*60)
    #     print("Testing Wareshare Modbus Communication")
    #     print("="*60 + "\n")
        
    #     # Test 1: Turn all coils ON
    #     print("Test 1: Turning all coils ON...")
    #     wareshare.write_all_coils(True)
    #     time.sleep(3)
        
    #     # Test 2: Turn all coils OFF
    #     print("Test 2: Turning all coils OFF...")
    #     wareshare.write_all_coils(False)
    #     time.sleep(3)
        
        # # Test 3: Blink coils 0-3
        # print("Test 3: Blinking coils 0-3...")
        # wareshare.blink_coils([0, 1, 2, 3], cycles=3, on_time=0.3, off_time=0.3)
        
        # # Test 4: Read coil status (if supported)
        # print("Test 4: Reading coil status...")
        # coil_states = wareshare.read_coils(0, 8)
        # if coil_states:
        #     for i, state in enumerate(coil_states):
        #         print(f"  Coil {i}: {'ON' if state else 'OFF'}")
        
        # print("\n" + "="*60)
        # print("Testing completed!")
        # print("="*60 + "\n")
        
    # except KeyboardInterrupt:
    #     print("\nInterrupted by user")
    # finally:
    #     # Cleanup: Turn all coils OFF
    #     print("Cleanup: Turning all coils OFF...")
    #     wareshare.write_all_coils(False)
    #     wareshare.disconnect()