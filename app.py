"""
OV5640 카메라 스트리밍 서버
- STM32H753에서 UART로 RAW 이미지 수신
- 웹 브라우저로 실시간 스트리밍
"""

import serial
import struct
import threading
import time
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

# =============================
# UART 설정
# =============================
ser = serial.Serial(
    port="COM6",  # STM32 연결 포트
    baudrate=2000000,  # 2Mbps
    timeout=0.1
)

FRAME_HEADER = b"\xAA\x55\xAA\x55"

# =============================
# Flask & SocketIO 설정
# =============================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ov5640_camera_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

current_frame = None
frame_count = 0
last_frame_time = time.time()


# =============================
# UART 수신 스레드
# =============================
def uart_reader():
    global current_frame, frame_count, last_frame_time
    buffer = b""

    print("🎥 UART Reader started...")
    print(f"📡 Port: {ser.port}, Baudrate: {ser.baudrate}")
    print(f"🔍 Waiting for frame header: {FRAME_HEADER.hex()}")
    print()

    while True:
        try:
            # 데이터 읽기
            data = ser.read(4096)
            if not data:
                continue

            buffer += data

            while True:
                # 헤더 찾기
                start = buffer.find(FRAME_HEADER)
                if start == -1:
                    # 헤더 없음, 버퍼 크기 제한
                    if len(buffer) > 100000:
                        buffer = buffer[-10000:]
                    break

                # 헤더 뒤에 크기 정보 확인
                if len(buffer) < start + 8:
                    break

                # 프레임 크기 읽기 (Little Endian)
                size_bytes = buffer[start + 4:start + 8]
                size = struct.unpack("<I", size_bytes)[0]

                # 프레임 크기 검증
                if size > 1000000 or size == 0:  # 1MB 초과 또는 0이면 무시
                    print(f"⚠️  Invalid frame size: {size} bytes, skipping...")
                    buffer = buffer[start + 8:]
                    continue

                # 전체 프레임이 버퍼에 있는지 확인
                if len(buffer) < start + 8 + size:
                    break

                # 프레임 데이터 추출
                frame_data = buffer[start + 8:start + 8 + size]

                # 사용한 부분 버퍼에서 제거
                buffer = buffer[start + 8 + size:]

                # 프레임 통계
                frame_count += 1
                current_time = time.time()
                fps = 1.0 / (current_time - last_frame_time) if (current_time - last_frame_time) > 0 else 0
                last_frame_time = current_time

                print(f"✅ Frame #{frame_count}: {size} bytes ({size / 1024:.1f} KB), FPS: {fps:.1f}")

                # 현재 프레임 업데이트
                current_frame = frame_data

                # 웹 클라이언트로 전송
                socketio.emit('frame', frame_data, broadcast=True)

        except Exception as e:
            print(f"❌ Error in UART reader: {e}")
            time.sleep(0.1)


# =============================
# 웹 라우트
# =============================
@app.route("/")
def index():
    return render_template("index.html")


@socketio.on('connect')
def handle_connect():
    print(f"🔌 Client connected")


@socketio.on('disconnect')
def handle_disconnect():
    print(f"🔌 Client disconnected")


# =============================
# 메인 실행
# =============================
if __name__ == "__main__":
    # UART 리더 스레드 시작
    uart_thread = threading.Thread(target=uart_reader, daemon=True)
    uart_thread.start()

    print()
    print("=" * 50)
    print("🚀 OV5640 Camera Streaming Server")
    print("=" * 50)
    print(f"📡 UART: {ser.port} @ {ser.baudrate} baud")
    print(f"🌐 Web: http://localhost:5000")
    print(f"🔍 Frame header: {FRAME_HEADER.hex()}")
    print("=" * 50)
    print()

    # Flask 서버 시작
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)