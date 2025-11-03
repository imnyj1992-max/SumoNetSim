from __future__ import annotations
import subprocess
import time

TARGET_FILE = "SumoNetSim1.1.4\\dataset_scenario.py"

# 재실행 대기 시간 (초)
RESTART_DELAY = 10

# 실행 횟수 카운트
max_run_count = 1000

def run_forever():
    cnt = 0
    while cnt < max_run_count:
        cnt += 1
        try:
            print(f"[INFO] 실행 시작: {TARGET_FILE}")
            # 자식 프로세스로 실행
            process = subprocess.Popen(["C:/Users/user/python/venv/Scripts/python.exe", TARGET_FILE])

            # 자식 프로세스가 끝날 때까지 대기
            process.wait()

            print(f"[WARNING] 프로세스 종료됨 (returncode={process.returncode})")

            # 비정상 종료인지 확인
            if process.returncode != 0:
                print(f"[ERROR] 예외 발생 가능. {RESTART_DELAY}초 후 재실행합니다.")
            else:
                print(f"[INFO] 정상 종료 감지. {RESTART_DELAY}초 후 재실행합니다.")

        except Exception as e:
            print(f"[CRITICAL] 감시 스크립트 오류: {e}")

        # 재실행 전 대기
        time.sleep(RESTART_DELAY)

if __name__ == "__main__":
    print("[INFO] 시뮬레이션 감시 시작")
    run_forever()
