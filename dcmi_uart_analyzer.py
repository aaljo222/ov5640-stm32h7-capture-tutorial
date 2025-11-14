#!/usr/bin/env python3
"""
DCMI UART Frame Analyzer for OV5640 STM32H7 Capture
Receives RGB565 frames from STM32 via UART and saves them as images.

Protocol:
  Header: 0xAA 0x55 0xAA 0x55 (4 bytes)
  Length: 4 bytes (little endian)
  Payload: frame data (RGB565)
"""

import serial
import argparse
import struct
import time
from pathlib import Path
import numpy as np
from PIL import Image

# Frame configuration (must match STM32 firmware)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
BYTES_PER_PIXEL = 2
EXPECTED_FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL

# Protocol constants
HEADER_MAGIC = b'\xAA\x55\xAA\x55'
HEADER_SIZE = 8  # 4 bytes magic + 4 bytes length


def rgb565_to_rgb888(data):
    """Convert RGB565 byte array to RGB888 numpy array"""
    # Parse RGB565 data (little endian)
    rgb565 = np.frombuffer(data, dtype=np.uint16)

    # Extract R, G, B components
    r = ((rgb565 >> 11) & 0x1F) << 3  # 5 bits -> 8 bits
    g = ((rgb565 >> 5) & 0x3F) << 2   # 6 bits -> 8 bits
    b = (rgb565 & 0x1F) << 3          # 5 bits -> 8 bits

    # Combine into RGB888 image
    rgb888 = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    rgb888[:, :, 0] = r.reshape((FRAME_HEIGHT, FRAME_WIDTH))
    rgb888[:, :, 1] = g.reshape((FRAME_HEIGHT, FRAME_WIDTH))
    rgb888[:, :, 2] = b.reshape((FRAME_HEIGHT, FRAME_WIDTH))

    return rgb888


def save_frame(frame_data, frame_number, output_dir):
    """Save frame as PNG image"""
    try:
        rgb888 = rgb565_to_rgb888(frame_data)
        img = Image.fromarray(rgb888, 'RGB')

        output_path = output_dir / f"frame_{frame_number:06d}.png"
        img.save(output_path)
        print(f"  → Saved: {output_path}")

    except Exception as e:
        print(f"  ✗ Error saving frame: {e}")


def find_header(ser, timeout=5.0):
    """Search for frame header in serial stream"""
    start_time = time.time()
    buffer = b''

    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            buffer += byte

            # Keep only last 4 bytes
            if len(buffer) > 4:
                buffer = buffer[-4:]

            # Check for header
            if buffer == HEADER_MAGIC:
                return True

    return False


def receive_frame(ser, frame_number, output_dir, save_frames=True):
    """Receive and process a single frame"""
    try:
        # Read frame length (4 bytes, little endian)
        length_bytes = ser.read(4)
        if len(length_bytes) < 4:
            print(f"  ✗ Timeout reading length")
            return False

        frame_size = struct.unpack('<I', length_bytes)[0]

        if frame_size != EXPECTED_FRAME_SIZE:
            print(f"  ⚠ Unexpected frame size: {frame_size} (expected {EXPECTED_FRAME_SIZE})")

        # Read frame data
        print(f"  Reading {frame_size} bytes...", end='', flush=True)
        frame_data = ser.read(frame_size)

        if len(frame_data) < frame_size:
            print(f" ✗ Incomplete ({len(frame_data)}/{frame_size} bytes)")
            return False

        print(f" ✓ Complete")

        # Save frame
        if save_frames:
            save_frame(frame_data, frame_number, output_dir)

        return True

    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def monitor_uart(port, baud, output_dir, save_frames=True, max_frames=None):
    """Main monitoring loop"""
    print(f"Opening {port} at {baud} baud...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
    except serial.SerialException as e:
        print(f"✗ Failed to open serial port: {e}")
        return

    print(f"✓ Serial port open")
    print(f"Waiting for camera frames...")

    # Flush any existing data
    ser.reset_input_buffer()

    frame_count = 0
    start_time = time.time()

    try:
        while max_frames is None or frame_count < max_frames:
            # Look for frame header
            if find_header(ser, timeout=10.0):
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0

                print(f"\n[Frame #{frame_count}] @ {fps:.2f} FPS")

                # Receive and process frame
                if receive_frame(ser, frame_count, output_dir, save_frames):
                    pass  # Success
                else:
                    # Frame receive failed, resync
                    print("  Re-syncing...")
                    ser.reset_input_buffer()
                    time.sleep(0.1)
            else:
                print(".", end='', flush=True)

                # Check for debug messages
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    try:
                        text = data.decode('utf-8', errors='ignore')
                        if text.strip():
                            print(f"\n[DEBUG] {text}", end='')
                    except:
                        pass

        print(f"\n\n✓ Captured {frame_count} frames")

    except KeyboardInterrupt:
        print(f"\n\n✓ Interrupted. Captured {frame_count} frames")
    finally:
        ser.close()
        print("Serial port closed")


def main():
    parser = argparse.ArgumentParser(
        description='DCMI UART Frame Analyzer for OV5640 STM32H7'
    )
    parser.add_argument('--port', '-p', required=True,
                        help='Serial port (e.g., COM6, /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=2000000,
                        help='Baud rate (default: 2000000)')
    parser.add_argument('--output', '-o', default='captured_frames',
                        help='Output directory for frames (default: captured_frames)')
    parser.add_argument('--no-save', action='store_true',
                        help='Don\'t save frames to disk (monitoring only)')
    parser.add_argument('--max-frames', '-n', type=int,
                        help='Maximum number of frames to capture')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output)
    if not args.no_save:
        output_dir.mkdir(exist_ok=True)
        print(f"Output directory: {output_dir.absolute()}")

    # Start monitoring
    monitor_uart(
        port=args.port,
        baud=args.baud,
        output_dir=output_dir,
        save_frames=not args.no_save,
        max_frames=args.max_frames
    )


if __name__ == '__main__':
    main()
