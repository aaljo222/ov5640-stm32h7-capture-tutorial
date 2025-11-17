#!/usr/bin/env python3
"""
OV5640 Camera Viewer - USB CDC + UART Debug
- USB CDC: 이미지 데이터 수신
- UART (PD8): 디버그 메시지 표시
"""

import serial
import serial.tools.list_ports
import numpy as np
import cv2
import threading
import struct

# Configuration
FRAME_W = 320
FRAME_H = 240
FRAME_SIZE = FRAME_W * FRAME_H * 2  # RGB565


class CameraViewer:
    def __init__(self, usb_port, uart_port):
        self.usb_ser = serial.Serial(usb_port, baudrate=115200, timeout=1)
        self.uart_ser = serial.Serial(uart_port, baudrate=115200, timeout=0.1)
        self.running = True
        self.frame_count = 0

    def rgb565_to_bgr(self, data):
        """RGB565 → BGR888 변환"""
        rgb565 = np.frombuffer(data, dtype=np.uint16)

        r = ((rgb565 & 0xF800) >> 11) << 3
        g = ((rgb565 & 0x07E0) >> 5) << 2
        b = (rgb565 & 0x001F) << 3

        img = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
        img[:, :, 2] = r.reshape(FRAME_H, FRAME_W)
        img[:, :, 1] = g.reshape(FRAME_H, FRAME_W)
        img[:, :, 0] = b.reshape(FRAME_H, FRAME_W)

        return img

    def uart_reader_thread(self):
        """UART 디버그 메시지 읽기"""
        print("=== UART Debug Monitor Started ===\n")
        while self.running:
            try:
                line = self.uart_ser.readline()
                if line:
                    print(f"[UART] {line.decode('utf-8', errors='ignore').strip()}")
            except:
                pass

    def find_header(self):
        """헤더 찾기: 0xAA 0x55 0xAA 0x55"""
        while self.running:
            b = self.usb_ser.read(1)
            if not b:
                continue
            if b == b'\xAA':
                next_bytes = self.usb_ser.read(3)
                if next_bytes == b'\x55\xAA\x55':
                    return True
        return False

    def receive_frame(self):
        """프레임 수신"""
        # 헤더 찾기
        if not self.find_header():
            return None

        # 크기 읽기 (4 bytes, little endian)
        size_bytes = self.usb_ser.read(4)
        if len(size_bytes) != 4:
            print("[ERROR] Size read failed")
            return None

        size = struct.unpack('<I', size_bytes)[0]

        if size != FRAME_SIZE:
            print(f"[WARN] Wrong size: {size} (expected {FRAME_SIZE})")
            # 잘못된 데이터 버리기
            self.usb_ser.read(size)
            return None

        # 프레임 데이터 읽기
        frame_data = bytearray()
        remaining = size

        while remaining > 0:
            chunk = self.usb_ser.read(min(remaining, 4096))
            if not chunk:
                print("[ERROR] Frame data incomplete")
                return None
            frame_data.extend(chunk)
            remaining -= len(chunk)

        # 푸터 읽기 (0x55 0xAA 0x55 0xAA)
        footer = self.usb_ser.read(4)
        if footer != b'\x55\xAA\x55\xAA':
            print(f"[WARN] Wrong footer: {footer.hex()}")

        return bytes(frame_data)

    def run(self):
        """메인 실행"""
        # UART 스레드 시작
        uart_thread = threading.Thread(target=self.uart_reader_thread, daemon=True)
        uart_thread.start()

        print("=== USB CDC Image Viewer Started ===")
        print(f"Frame size: {FRAME_W}x{FRAME_H} ({FRAME_SIZE} bytes)")
        print("Press 'q' to quit\n")

        while self.running:
            frame_data = self.receive_frame()

            if frame_data:
                self.frame_count += 1

                # RGB565 → BGR 변환
                img = self.rgb565_to_bgr(frame_data)

                # 2배 확대 (640x480)
                img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LINEAR)

                # 프레임 번호 표시
                cv2.putText(img, f"Frame: {self.frame_count}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow('OV5640 Camera', img)
                print(f"[USB] Frame #{self.frame_count} received")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.running = False
        self.usb_ser.close()
        self.uart_ser.close()
        cv2.destroyAllWindows()
        print("\n=== Stopped ===")


def find_stm32_ports():
    """STM32 포트 자동 검색"""
    ports = serial.tools.list_ports.comports()
    usb_port = None
    uart_port = None

    for port in ports:
        desc = port.description.lower()

        # USB CDC (STMicroelectronics Virtual COM Port)
        if 'stm' in desc or 'virtual' in desc:
            usb_port = port.device

        # UART (FTDI, CH340, CP210x 등)
        if 'usb serial' in desc or 'ftdi' in desc or 'ch340' in desc:
            uart_port = port.device

    return usb_port, uart_port


if __name__ == "__main__":
    print("Scanning for STM32 ports...")

    usb_port, uart_port = find_stm32_ports()

    if not usb_port:
        print("[ERROR] USB CDC port not found!")
        print("Available ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device}: {port.description}")
        exit(1)

    if not uart_port:
        print("[WARN] UART debug port not found, using USB CDC for debug")
        uart_port = usb_port

    print(f"USB CDC: {usb_port}")
    print(f"UART:    {uart_port}\n")

    try:
        viewer = CameraViewer(usb_port, uart_port)
        viewer.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"[ERROR] {e}")