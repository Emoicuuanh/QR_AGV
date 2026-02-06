#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import psutil
import rospy
import time
import csv, os, datetime
from std_stamped_msgs.msg import StringStamped
# CẤU HÌNH
THRESHOLD = 99  # Ngưỡng CPU (Ví dụ 99%)
CSV_FILE = os.path.join(os.path.dirname(__file__), "check_cpu.csv")
if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "pid", "cpu", "name"])

def log(pid, cpu, name):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, pid, cpu, name])
def robot_status_cb():
    json_str = msg.data
    data_dict = json.loads(json_str)
    detail_value = data_dict.get("detail", "UNKNOWN")
    if detail_value == "LOST_LABEL" or detail_value == "IO_BOARD_DISCONNECT":
        log(pid,cpu,command_str)
        log("=","=","============================")

def check_cpu():
    while(True):
        # print("Đang đo CPU trong 1 giây...")
        # interval=1: Đo trong 1 giây để có con số chính xác (nếu để 0 nó sẽ trả về 0 ngay lập tức)
        cpu_usage = psutil.cpu_percent(interval=0.5)

        if cpu_usage >= THRESHOLD:
            print(f"{'PID':<8} {'%CPU':<6} {'COMMAND (File đang chạy)'}")
            print("-" * 60)

            # Lấy danh sách tất cả tiến trình
            processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'cmdline']):
                try:
                    # Lấy thông tin process
                    if proc.info['cpu_percent'] > 0:
                        processes.append(proc.info)
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass

            # Sắp xếp theo cpu_percent giảm dần và lấy 3 cái đầu
            # key=lambda x: x['cpu_percent'] nghĩa là lấy giá trị cpu_percent để so sánh
            top_3 = sorted(processes, key=lambda x: x['cpu_percent'], reverse=True)[:10]
            for p in top_3:
                pid = p['pid']
                cpu = p['cpu_percent']
                cmdline_raw = p['cmdline'] # Đây là một list, ví dụ: ['python', 'main.py']

                # XỬ LÝ ĐỂ LẤY TÊN FILE
                if cmdline_raw:
                    # Cách 1: Hiển thị toàn bộ câu lệnh
                    # command_str = " ".join(cmdline_raw)
                    
                    # Cách 2: Lọc thông minh (Khuyên dùng cho trường hợp của bạn)
                    # Nếu là python, thường tên file nằm ở vị trí thứ 2 (index 1)
                    # Ví dụ: python /home/mkac/script.py -> Lấy /home/mkac/script.py
                    if 'python' in cmdline_raw[0] and len(cmdline_raw) > 1:
                        # Lấy phần tử thứ 2 trở đi
                        command_str = " ".join(cmdline_raw[1:])
                    else:
                        # Nếu không phải python (ví dụ chrome, systemd) thì lấy tên file chạy đầu tiên
                        command_str = " ".join(cmdline_raw)
                else:
                    # Trường hợp không lấy được cmdline (thường là process hệ thống)
                    command_str = p['name']
                print(f"{pid:<8} {cpu:<6} {command_str}")
                log(pid,cpu,command_str)
            log("=","=","============================")
        else:
            print(f"Hệ thống ổn định. CPU: {cpu_usage}%")

if __name__ == "__main__":
    rospy.Subscriber("/robot_status",StringStamped, robot_status_cb)
    check_cpu()