import serial
import time

# ---------------------------------------
# USB CDC 포트 설정
# ---------------------------------------
PORT = "COM7"   # 👉 여기 자신의 COM포트 번호로 변경
BAUD = 115200   # USB CDC는 실제 baud 무관하지만 설정은 해야 함

def main():
    print(f"Opening {PORT} ...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print("포트 열기 실패:", e)
        return

    print("✔ USB-CDC 연결 성공! 로그 출력 시작\n")

    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode(errors='ignore').strip()
            if data:
                print("[STM32] ", data)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
