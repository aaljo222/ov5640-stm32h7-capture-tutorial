#!/usr/bin/env python3
"""
UART Debug Monitor - View raw UART data to diagnose frame capture issues
"""

import serial
import argparse
import time
from datetime import datetime

def hex_dump(data, prefix=""):
    """Print hex dump of data"""
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_str = ' '.join(f'{b:02x}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f"{prefix}{i:04x}: {hex_str:<48}  {ascii_str}")

def monitor_uart(port, baud, show_hex=False, show_text=True):
    """Monitor UART and display both hex and text data"""
    print(f"Opening {port} at {baud} baud...")
    print(f"Mode: hex={show_hex}, text={show_text}")
    print("=" * 80)

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        return

    print(f"Serial port open. Monitoring...\n")
    ser.reset_input_buffer()

    buffer = b''
    last_print = time.time()

    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer += data

                # Print text data immediately
                if show_text:
                    try:
                        text = data.decode('utf-8', errors='replace')
                        if text:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            print(f"[{timestamp}] {text}", end='')
                    except:
                        pass

                # Check for frame header
                if b'\xAA\x55\xAA\x55' in buffer:
                    idx = buffer.find(b'\xAA\x55\xAA\x55')
                    print(f"\n\n*** FRAME HEADER FOUND at offset {idx} ***\n")
                    if show_hex:
                        hex_dump(buffer[max(0, idx-16):idx+32], "  ")
                    buffer = buffer[idx+4:]

            # Print hex dump periodically
            if show_hex and buffer and (time.time() - last_print) > 2.0:
                print(f"\n--- Buffer dump ({len(buffer)} bytes) ---")
                hex_dump(buffer[:256])
                print()
                last_print = time.time()

    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        ser.close()
        print("Serial port closed")

def main():
    parser = argparse.ArgumentParser(
        description='UART Debug Monitor - diagnose OV5640 frame capture'
    )
    parser.add_argument('--port', '-p', required=True,
                        help='Serial port (e.g., COM6, /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=2000000,
                        help='Baud rate (default: 2000000)')
    parser.add_argument('--hex', action='store_true',
                        help='Show hex dump of received data')
    parser.add_argument('--no-text', action='store_true',
                        help='Don\'t show text output')

    args = parser.parse_args()

    monitor_uart(
        port=args.port,
        baud=args.baud,
        show_hex=args.hex,
        show_text=not args.no_text
    )

if __name__ == '__main__':
    main()
