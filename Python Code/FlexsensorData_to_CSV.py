import serial
import pandas as pd
import time
import os

# 아두이노와 시리얼 통신 설정
ser = serial.Serial('COM4', 9600)  # 아두이노 포트를 COM3으로 설정 (포트 번호는 실제 환경에 맞게 변경)
time.sleep(2)  # 연결 안정화 대기

# 상황별 폴더 설정
situation_names = {1: "손 접기", 2: "손 접은 상태로 고정", 0: "손 피기", 3: "손 핀 상태로 고정"}
base_dir = r"C:\Users\hj-Choi\Desktop\연구자료\flexsensor_데이터"  # 데이터를 저장할 기본 폴더 경로 (원하는 위치로 변경 가능)
os.makedirs(base_dir, exist_ok=True)

# 상황별 폴더 경로 생성
situation_dirs = {key: os.path.join(base_dir, name) for key, name in situation_names.items()}
for dir_path in situation_dirs.values():
    os.makedirs(dir_path, exist_ok=True)
#초기화
s = [[] for _ in range(5)]
loop_count = 1
current_situation = None

while True:
    try:
        line = ser.readline().decode().strip()
 
        # 상황 시작 감지
        if line in ['1', '0']:
            current_situation = int(line)
            data = {
                    'Sensor1': s[0],
                    'Sensor2': s[1],
                    'Sensor3': s[2],
                    'Sensor4': s[3],
                    'Sensor5': s[4]
                }
            df = pd.DataFrame(data)
            filename = f'{situation_dirs[current_situation]}/data_loop_{loop_count}_{situation_names[current_situation]}.csv'
            df.to_csv(filename, index=False)
            print(f"{filename}에 데이터 저장 완료")
            loop_count += 1

            #초기화
            s = [[] for _ in range(5)]
            print(f"{situation_names[current_situation]} 데이터 수집 중...")
        
        # 센서 데이터 수신 및 저장
        elif current_situation is not None:
            sensor_values = line.split(", ")
            
            # 센서 값이 5개인지 확인
            if len(sensor_values) == 5:
                for i, val in enumerate(sensor_values):
                    s[i].append(val)
                

    except KeyboardInterrupt:
        print("프로그램 종료")
        ser.close()
        break
