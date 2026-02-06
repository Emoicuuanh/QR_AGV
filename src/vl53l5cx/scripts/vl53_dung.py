#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, csv, rospy, os, datetime
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from vl53l5cx.msg import Vl53l5cxRanges
from geometry_msgs.msg import Twist

# ================== CẤU HÌNH ==================
CSV_FILE = os.path.join(os.path.dirname(__file__), "modbus_log.csv")
SERIAL_PORT = "/dev/ttyS1"          # hoặc /dev/ttyUSB0
BAUDRATE = 57600                   # ĐÃ ĐÚNG CHO VL53L5CX
WRITE_ADDRESS = 130
WRITE_LENGTH = 64
NUM_REGISTERS = 65

FORWARD_SLAVES = [1, 2]    # Khi tiến
BACKWARD_SLAVES = [3, 4]   # Khi lùi (hoặc ngược lại tùy bạn gắn)

# ================== KẾT NỐI CLIENT ==================
client = ModbusSerialClient(
    method='rtu',
    port=SERIAL_PORT,
    baudrate=BAUDRATE,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=0.5,            # Tăng timeout cho baud cao
    strict=True
)

# ================== LOG CSV ==================
if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "slave_id", "status", "message"])

def log(slave_id, status, message=""):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, slave_id, status, message])

# ================== ĐỌC MODBUS ==================
def read_slave(slave_id):
    try:
        t0 = time.time()
        response = client.read_holding_registers(65, 64, unit=slave_id)
        elapsed = time.time() - t0

        if response.isError():
            print(f"\033[91m[Slave {slave_id}] LỖI: {response}\033[0m")
            log(slave_id, "ERROR", str(response))
            return None, elapsed
        else:
            print(f"\033[92m[Slave {slave_id}] THÀNH CÔNG - {len(response.registers)} giá trị (thời gian: {elapsed:.3f}s)\033[0m")
            log(slave_id, "OK", f"Read {len(response.registers)} regs in {elapsed:.3f}s")
            return response.registers, elapsed
    except ModbusException as e:
        print(f"\033[91m[Slave {slave_id}] ModbusException: {e}\033[0m")
        log(slave_id, "ERROR", str(e))
        return None, 0.0
    except Exception as e:
        print(f"\033[91m[Slave {slave_id}] Lỗi không xác định: {e}\033[0m")
        log(slave_id, "ERROR", str(e))
        return None, 0.0

# ================== IN MA TRẬN 5x8 (TÙY CHỌN) ==================
def print_matrix_5x8(registers, slave_id):
    if not registers or len(registers) < 64:
        return
    rows = [registers[i*8:(i+1)*8] for i in range(8)][:5]
    print(f"[Slave {slave_id}] Ma trận 5×8:")
    for i, row in enumerate(rows, 1):
        print(f"  Hàng {i}: {' '.join(f'{v:6d}' for v in row)}")

# ================== ROS CALLBACK ==================
current_linear = 0.0
def cmd_vel_callback(msg):
    global current_linear
    current_linear = msg.linear.x

# ================== MAIN ==================
if __name__ == "__main__":
    rospy.init_node("tof_modbus_dual_publisher", anonymous=False)

    pub_dict = {
        1: rospy.Publisher("vl53l5cx_r3", Vl53l5cxRanges, queue_size=2),
        2: rospy.Publisher("vl53l5cx_r4", Vl53l5cxRanges, queue_size=2),
        3: rospy.Publisher("vl53l5cx_r1", Vl53l5cxRanges, queue_size=2),
        4: rospy.Publisher("vl53l5cx_r2", Vl53l5cxRanges, queue_size=2),
    }

    rospy.Subscriber("/final_cmd_vel_mux/output", Twist, cmd_vel_callback)

    if not client.connect():
        print("Không kết nối được Modbus RTU!")
        log(0, "FATAL", "Cannot connect to serial port")
        exit()

    print("\033[96mĐÃ KẾT NỐI THÀNH CÔNG Modbus RTU!\033[0m")
    log(0, "INFO", "Modbus connected")

    rate = rospy.Rate(15)  # 15Hz

    while not rospy.is_shutdown():
        # Chọn nhóm slave theo hướng chạy
        active_slaves = FORWARD_SLAVES if current_linear >= 0 else BACKWARD_SLAVES

        print(f"\n--- Đang đọc nhóm slave {'TIẾN' if current_linear >= 0 else 'LÙI'} (ID: {active_slaves}) ---")

        for sid in active_slaves:
            time.sleep(0.015)  # Tránh dồn request quá nhanh
            registers, _ = read_slave(sid)

            if registers:
                # Tạo message ROS
                msg = Vl53l5cxRanges()
                msg.stamp = rospy.Time.now()
                rows = [registers[i*8:(i+1)*8] for i in range(8)][:5]
                rows = list(reversed(rows))  # Đảo ngược hàng nếu cần
                msg.range = [float(v) for row in rows for v in row]
                pub_dict[sid].publish(msg)

                # Uncomment dòng dưới nếu muốn in ma trận mỗi lần đọc
                # print_matrix_5x8(registers, sid)
            else:
                print(f"\033[93m[Slave {sid}] Không có dữ liệu lần này\033[0m")

        rate.sleep()




# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# import time, csv, rospy, os, datetime
# from pymodbus.client.sync import ModbusSerialClient
# from pymodbus.exceptions import ModbusException
# from vl53l5cx.msg import Vl53l5cxRanges
# from geometry_msgs.msg import Twist
# # ================== CẤU HÌNH ==================
# CSV_FILE = os.path.join(os.path.dirname(__file__), "modbus_log.csv")
# SERIAL_PORT = "/dev/ttyS0" # hoặc /dev/ttyUSB0
# BAUDRATE = 57600 # ĐÃ ĐÚNG CHO VL53L5CX
# WRITE_ADDRESS = 130
# WRITE_LENGTH = 64
# NUM_REGISTERS = 65
# FORWARD_SLAVES = [1, 2] # Khi tiến
# BACKWARD_SLAVES = [3, 4] # Khi lùi (hoặc ngược lại tùy bạn gắn)
# # ================== KẾT NỐI CLIENT ==================
# client = ModbusSerialClient(
#     method='rtu',
#     port=SERIAL_PORT,
#     baudrate=BAUDRATE,
#     bytesize=8,
#     parity='N',
#     stopbits=1,
#     timeout=0.5, # Tăng timeout cho baud cao
#     strict=True
# )
# # ================== LOG CSV ==================
# if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
#     with open(CSV_FILE, "w", newline="") as f:
#         writer = csv.writer(f)
#         writer.writerow(["timestamp", "slave_id", "status", "message"])
# def log(slave_id, status, message=""):
#     timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
#     with open(CSV_FILE, "a", newline="") as f:
#         writer = csv.writer(f)
#         writer.writerow([timestamp, slave_id, status, message])
# # ================== ĐỌC MODBUS ==================
# def read_slave(slave_id):
#     try:
#         t0 = time.time()
#         response = client.read_holding_registers(65, 64, unit=slave_id)
#         elapsed = time.time() - t0
#         if response.isError():
#             print(f"\033[91m[Slave {slave_id}] LỖI: {response}\033[0m")
#             log(slave_id, "ERROR", str(response))
#             return None, elapsed
#         else:
#             print(f"\033[92m[Slave {slave_id}] THÀNH CÔNG - {len(response.registers)} giá trị (thời gian: {elapsed:.3f}s)\033[0m")
#             log(slave_id, "OK", f"Read {len(response.registers)} regs in {elapsed:.3f}s")
#             return response.registers, elapsed
#     except ModbusException as e:
#         print(f"\033[91m[Slave {slave_id}] ModbusException: {e}\033[0m")
#         log(slave_id, "ERROR", str(e))
#         return None, 0.0
#     except Exception as e:
#         print(f"\033[91m[Slave {slave_id}] Lỗi không xác định: {e}\033[0m")
#         log(slave_id, "ERROR", str(e))
#         return None, 0.0
# # ================== IN MA TRẬN 5x8 (TÙY CHỌN) ==================
# def print_matrix_5x8(registers, slave_id):
#     if not registers or len(registers) < 64:
#         return
#     rows = [registers[i*8:(i+1)*8] for i in range(8)][:5]
#     print(f"[Slave {slave_id}] Ma trận 5×8:")
#     for i, row in enumerate(rows, 1):
#         print(f" Hàng {i}: {' '.join(f'{v:6d}' for v in row)}")
# # ================== ROS CALLBACK ==================
# current_linear = 0.0
# def cmd_vel_callback(msg):
#     global current_linear
#     current_linear = msg.linear.x
# # ================== MAIN ==================
# if __name__ == "__main__":
#     rospy.init_node("tof_modbus_dual_publisher", anonymous=False)
#     pub_dict = {
#         1: rospy.Publisher("vl53l5cx_r3", Vl53l5cxRanges, queue_size=2),
#         2: rospy.Publisher("vl53l5cx_r4", Vl53l5cxRanges, queue_size=2),
#         3: rospy.Publisher("vl53l5cx_r1", Vl53l5cxRanges, queue_size=2),
#         4: rospy.Publisher("vl53l5cx_r2", Vl53l5cxRanges, queue_size=2),
#     }
#     rospy.Subscriber("/final_cmd_vel_mux/output", Twist, cmd_vel_callback)
#     if not client.connect():
#         print("Không kết nối được Modbus RTU!")
#         log(0, "FATAL", "Cannot connect to serial port")
#         exit()
#     print(f"\033[96mĐÃ KẾT NỐI THÀNH CÔNG Modbus RTU {BAUDRATE} bps!\033[0m")
#     log(0, "INFO", "Modbus connected")
#     rate = rospy.Rate(15) # 15Hz
#     while not rospy.is_shutdown():
#         # Chọn nhóm slave theo hướng chạy
#         active_slaves = FORWARD_SLAVES if current_linear >= 0 else BACKWARD_SLAVES
#         print(f"\n--- Đang đọc nhóm slave {'TIẾN' if current_linear >= 0 else 'LÙI'} (ID: {active_slaves}) ---")
#         for sid in active_slaves:
#             time.sleep(0.015) # Tránh dồn request quá nhanh
#             registers, _ = read_slave(sid)
#             if registers:
#                 # Tạo message ROS
#                 msg = Vl53l5cxRanges()
#                 msg.stamp = rospy.Time.now()
#                 rows = [registers[i*8:(i+1)*8] for i in range(8)][:5]
#                 rows = list(reversed(rows)) # Đảo ngược hàng nếu cần
#                 msg.range = [float(v) for row in rows for v in row]
#                 pub_dict[sid].publish(msg)
#                 # Uncomment dòng dưới nếu muốn in ma trận mỗi lần đọc
#                 # print_matrix_5x8(registers, sid)
#             else:
#                 print(f"\033[93m[Slave {sid}] Không có dữ liệu lần này\033[0m")
#         rate.sleep()





# #!/usr/bin/env python3
# import time, csv, rospy, os, datetime
# from pymodbus.client.sync import ModbusSerialClient
# from pymodbus.exceptions import ModbusException
# from vl53l5cx.msg import Vl53l5cxRanges
# from geometry_msgs.msg import Twist  # để đọc topic /final_cmd_vel_mux/output

# # ================== CẤU HÌNH ==================
# CSV_FILE = os.path.join(os.path.dirname(__file__), "modbus_log.csv")
# SERIAL_PORT = "/dev/ttyS0"
# BAUDRATE = 57600
# WRITE_ADDRESS = 130
# WRITE_LENGTH = 64
# NUM_REGISTERS = 65

# # Khi tiến (linear >= 0) dùng ID 1, 2. Khi lùi (linear < 0) dùng ID 3, 4
# FORWARD_SLAVES = [1, 2]
# BACKWARD_SLAVES = [3, 4]

# # ================== KẾT NỐI CLIENT ==================
# client = ModbusSerialClient(
#     method='rtu',
#     port=SERIAL_PORT,
#     baudrate=BAUDRATE,
#     bytesize=8,
#     parity='N',
#     stopbits=1,
#     timeout=0.1
# )

# # ====== KHỞI TẠO FILE LOG ======
# if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
#     with open(CSV_FILE, "w", newline="") as f:
#         writer = csv.writer(f)
#         writer.writerow(["timestamp", "slave_id", "status", "message"])

# # ====== HÀM GHI LOG ======
# def log(slave_id, status, message=""):
#     timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
#     with open(CSV_FILE, "a", newline="") as f:
#         writer = csv.writer(f)
#         writer.writerow([timestamp, slave_id, status, message])

# # ====== HÀM ĐỌC MODBUS ======
# def process_FC3(slave_id, start_address, num_registers):
#     try:
#         response = client.read_holding_registers(start_address, num_registers, unit=slave_id)
#         if response.isError():
#             msg = f"Error reading registers: {response}"
#             print(f"[Slave {slave_id}]  {msg}")
#             log(slave_id, "ERROR", msg)
#             return None
#         return response.registers
#     except ModbusException as e:
#         msg = f"Modbus exception: {e}"
#         print(f"[Slave {slave_id}]  {msg}")
#         log(slave_id, "ERROR", msg)
#         return None

# # ====== IN MA TRẬN & LOG ======
# def print_matrix_5x8(registers, slave_id):
#     if not registers or len(registers) < 64:
#         msg = f"Not enough 64 elements (only {len(registers) if registers else 0})"
#         print(f"[Slave {slave_id}] {msg}")
#         log(slave_id, "ERROR", msg)
#         return

#     # Chia thành 8 hàng
#     rows = [registers[i*8:(i+1)*8] for i in range(8)]

#     # Giữ 5 hàng đầu → bỏ 3 hàng cuối
#     rows = rows[:5]

#     print(f"[Slave {slave_id}]  Ma trận 5×8 (bỏ 3 hàng cuối):")
#     for row in rows:
#         print(" ".join(f"{val:5d}" for val in row))

#     # Log 40 phần tử
#     data_str = ",".join(str(v) for row in rows for v in row)
#     log(slave_id, "INFO", data_str)

# # ================== CALLBACK TỪ CMD_VEL ==================
# current_linear = 0.0

# def cmd_vel_callback(msg):
#     global current_linear
#     current_linear = msg.linear.x  # lấy giá trị vận tốc tuyến tính
#     # rospy.loginfo_throttle(1.0, f"Linear speed: {current_linear:.3f}")

# # ================== MAIN ==================
# if __name__ == "__main__":
#     rospy.init_node("tof_modbus_dual_publisher")
#     pub_dict = {
#         3: rospy.Publisher("vl53l5cx_r1", Vl53l5cxRanges, queue_size=2),
#         4: rospy.Publisher("vl53l5cx_r2", Vl53l5cxRanges, queue_size=2),
#         1: rospy.Publisher("vl53l5cx_r3", Vl53l5cxRanges, queue_size=2),
#         2: rospy.Publisher("vl53l5cx_r4", Vl53l5cxRanges, queue_size=2),
#     }

#     rospy.Subscriber("/final_cmd_vel_mux/output", Twist, cmd_vel_callback)

#     if client.connect():
#         print("Connected to Modbus slaves via Serial!")

#         # Gửi giá trị khởi tạo
#         init_vals = [60] * WRITE_LENGTH
#         for sid in FORWARD_SLAVES + BACKWARD_SLAVES:
#             client.write_registers(WRITE_ADDRESS, init_vals, unit=sid)

#         rate = rospy.Rate(15)  # 15Hz
#         while not rospy.is_shutdown():
#             # Xác định nhóm slave theo hướng di chuyển
#             active_slaves = BACKWARD_SLAVES if current_linear >= 0 else FORWARD_SLAVES

#             for sid in active_slaves:
#                 time.sleep(0.01)
#                 t0 = time.time()
#                 registers = process_FC3(sid, 65, 64)
#                 if registers:
#                     # print_matrix_5x8(registers, sid)
#                     msg = Vl53l5cxRanges()
#                     msg.stamp = rospy.Time.now()
#                     rows = [registers[i*8:(i+1)*8] for i in range(8)]
#                     rows = rows[:5]     # Lấy 5 hàng đầu
#                     rows = list(reversed(rows))
#                     msg.range = [float(v) for row in rows for v in row]
#                     pub_dict[sid].publish(msg)
#                     log(sid, "INFO", f"Published {len(registers)} values with timestamp")
#                 elapsed = time.time() - t0
#                 log(sid, "INFO", f"Read cycle: {elapsed:.3f}s")
#             rate.sleep()
#     else:
#         print("Failed to connect to Modbus slaves via Serial.")
#         log(0, "ERROR", "Failed to connect to Modbus slaves via Serial.")
