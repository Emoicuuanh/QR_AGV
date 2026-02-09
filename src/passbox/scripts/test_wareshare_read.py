#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Test script to debug Wareshare read function
"""
import socket
import time

def calculate_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def test_read():
    host = '192.168.1.200'
    port = 502
    
    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("✓ Connected!\n")
    
    # First, write something to make sure device is responsive
    print("Step 1: Testing WRITE (turn ON relay 0)...")
    write_frame = bytearray(12)
    write_frame[5] = 0x06
    write_frame[6] = 0x01  # Device address
    write_frame[7] = 0x05  # Write single coil
    write_frame[8] = 0x00
    write_frame[9] = 0x00  # Coil 0
    write_frame[10] = 0xFF
    write_frame[11] = 0x00  # ON
    
    print(f"  Sending: {' '.join([f'{b:02X}' for b in write_frame])}")
    sock.send(write_frame)
    time.sleep(0.5)
    
    # Try to read response from write
    sock.settimeout(1.0)
    try:
        response = sock.recv(1024)
        print(f"  Write response: {' '.join([f'{b:02X}' for b in response])}")
    except socket.timeout:
        print("  No response from write (this is OK for some devices)")
    
    print("\nStep 2: Testing READ...")
    
    # Build read frame
    read_frame = bytearray(8)
    read_frame[0] = 0x01  # Device address
    read_frame[1] = 0x01  # Command: Read relay status
    read_frame[2] = 0x00  # Start address high
    read_frame[3] = 0x00  # Start address low
    read_frame[4] = 0x00  # Count high
    read_frame[5] = 0x08  # Count low (8 relays)
    
    # Calculate CRC
    crc = calculate_crc16(read_frame[:6])
    read_frame[6] = crc & 0xFF
    read_frame[7] = (crc >> 8) & 0xFF
    
    print(f"  Sending: {' '.join([f'{b:02X}' for b in read_frame])}")
    print(f"  Expected: 01 01 00 00 00 08 3D CC")
    
    # Flush buffer
    sock.settimeout(0.1)
    try:
        while True:
            old = sock.recv(1024)
            if not old:
                break
    except socket.timeout:
        pass
    
    # Send read command
    sock.settimeout(5.0)  # Long timeout
    sent = sock.send(read_frame)
    print(f"  Sent {sent} bytes")
    
    # Wait for response
    print("  Waiting for response...")
    try:
        response = sock.recv(1024)
        print(f"\n✓ SUCCESS! Received {len(response)} bytes:")
        print(f"  Response: {' '.join([f'{b:02X}' for b in response])}")
        
        # Parse response
        if len(response) >= 5:
            device_addr = response[0]
            command = response[1]
            byte_count = response[2]
            status_byte = response[3] if len(response) > 3 else 0
            
            print(f"\n  Device Address: 0x{device_addr:02X}")
            print(f"  Command: 0x{command:02X}")
            print(f"  Byte Count: {byte_count}")
            print(f"  Status Byte: 0x{status_byte:02X} = {status_byte:08b}b")
            print(f"\n  Relay states:")
            for bit in range(8):
                state = bool(status_byte & (1 << bit))
                print(f"    Relay {bit}: {'ON' if state else 'OFF'}")
        
    except socket.timeout:
        print("\n✗ TIMEOUT - No response received")
        print("\nPossible issues:")
        print("  1. Device doesn't support read command")
        print("  2. Wrong device address")
        print("  3. Device is busy or not ready")
        print("  4. Need to wait longer after write")
    
    sock.close()
    print("\nConnection closed")

if __name__ == "__main__":
    try:
        test_read()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
