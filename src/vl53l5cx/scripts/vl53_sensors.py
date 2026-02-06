#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, csv, rospy, os, datetime, sys, rospkg, json
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from vl53l5cx.msg import Vl53l5cxRanges
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_stamped_msgs.msg import StringStamped
from safety_msgs.msg import SafetyStatus
from sensor_msgs.msg import LaserScan
from safety_msgs.msg import SafetyJob
from threading import Lock

common_folder = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_folder)
from common_function import find_soft_port

# ================== CẤU HÌNH ==================
CSV_FILE = os.path.join(os.path.dirname(__file__), "modbus_log.csv")
WRITE_ADDRESS = 130
WRITE_LENGTH = 64
NUM_REGISTERS = 65

FORWARD_SLAVES = [3, 4]    # Cảm biến phía TRƯỚC (khi tiến)
BACKWARD_SLAVES = [1, 2]   # Cảm biến phía SAU (khi lùi)
ALL_SLAVES = FORWARD_SLAVES + BACKWARD_SLAVES
modbus_lock = Lock()

lock_flag = True

# ================== BIẾN TRẠNG THÁI HƯỚNG ==================
current_linear = 0.0
last_direction = 1  # 1 = đang hướng tiến, -1 = đang hướng lùi (mặc định ban đầu là tiến)
status_robot =None  
vl53_data = {
    1: None,
    2: None,
    3: None,
    4: None
}
last_sent_states = {
    1: None,
    2: None,
    3: None,
    4: None
}

# ================== KẾT NỐI MODBUS CLIENT - SẼ KHỞI TẠO SAU ==================
client = None

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
#================== GHI MODBUS ==================
# def write_slave(slave_id, start_addr, values):
#     global modbus_lock
#     try:
#         with modbus_lock:
#             response = client.write_registers(start_addr, values, unit=slave_id)
#         if response.isError():
#             print(f"\033[91m[Slave {slave_id}] WRITE ERROR: {response}\033[0m")
#             log(slave_id, "WRITE_ERROR", str(response))
#             return False
#         else:
#             # print(f"\033[92m[Slave {slave_id}] WRITE OK - sent {len(values)} regs\033[0m")
#             # log(slave_id, "WRITE_OK", f"Sent {len(values)} registers")
#             return True
#     except Exception as e:
#         print(f"\033[91m[Slave {slave_id}] WRITE EXCEPTION: {e}\033[0m")
#         log(slave_id, "WRITE_EXCEPTION", str(e))
#         return False
def write_slave(slave_id, start_addr, values):
    global modbus_lock
    max_retries = 3  # Thử lại tối đa 3 lần nếu lỗi
    
    for attempt in range(max_retries):
        try:
            with modbus_lock:
                response = client.write_registers(start_addr, values, unit=slave_id)
            if response.isError():
                rospy.logwarn(f"[Slave {slave_id}] Write attempt {attempt+1} failed: {response}")
                continue # Thử lại vòng lặp kế tiếp
            else:
                # Ghi thành công
                # print(f"\033[92m[Slave {slave_id}] WRITE OK\033[0m")
                return True
        except Exception as e:
            rospy.logwarn(f"[Slave {slave_id}] Write Exception attempt {attempt+1}: {e}")
    # Nếu hết 3 lần vẫn lỗi thì mới log lỗi và trả về False
    log(slave_id, "WRITE_ERROR", f"Failed after {max_retries} retries")
    print(f"\033[91m[Slave {slave_id}] WRITE FAILED after retries\033[0m")
    return False
# ================== ĐỌC MODBUS ==================
def read_slave(slave_id):
    global modbus_lock
    try:
        t0 = time.time()
        with modbus_lock:
            response = client.read_holding_registers(65, 64, unit=slave_id)
            # log(slave_id, "WRITE_ERROR", "no error")
        elapsed = time.time() - t0

        if response.isError():
            print(f"\033[91m[Slave {slave_id}] LỖI: {response}\033[0m")
            log(slave_id, "ERROR", str(response))
            return None, elapsed
        else:
            # print(f"\033[92m[Slave {slave_id}] OK - {len(response.registers)} regs ({elapsed:.3f}s)\033[0m")
            # log(slave_id, "OK", f"Read {len(response.registers)} regs in {elapsed:.3f}s")
            return response.registers, elapsed
    except ModbusException as e:
        print(f"\033[91m[Slave {slave_id}] ModbusException: {e}\033[0m")
        log(slave_id, "ERROR", str(e))
        return None, 0.0
    except Exception as e:
        print(f"\033[91m[Slave {slave_id}] Lỗi: {e}\033[0m")
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

# ================== ROS CALLBACK - CẬP NHẬT HƯỚNG ==================
def cmd_vel_callback(msg):
    global current_linear, last_direction
    current_linear = msg.linear.x
    if current_linear > 0.01:
        last_direction = 1          # Đang tiến
    elif current_linear < -0.01:
        last_direction = -1         # Đang lùi
    # Nếu |v| ≤ 0.01 → giữ nguyên hướng cũ (rất quan trọng khi dừng)
def robot_status_cb(msg):
    global last_sent_states
    try:
        json_str = msg.data
        data_dict = json.loads(json_str)
        detail_value = data_dict.get("detail", "UNKNOWN")
        target_lidars = [1, 2, 3, 4] 
        for lidar_id in target_lidars:
            if detail_value == "SAFETY_STOP":
                if has_point_over_threshold(vl53_data[lidar_id]):
                    target_command = 1
                else:
                    target_command = 0
            else: target_command = 0
            if last_sent_states[lidar_id] != target_command:         
                rospy.loginfo(f"Slave {lidar_id}: Trạng thái đổi từ {last_sent_states[lidar_id]} -> {target_command}. Đang gửi Modbus...")
                # Gửi lệnh
                success = write_slave(lidar_id, WRITE_ADDRESS, [target_command])
                # Nếu gửi thành công, cậ0p nhật trạng thái cũ để lần sau không gửi lại
                if success:
                    last_sent_states[lidar_id] = target_command
            else:
                # Nếu trạng thái giống nhau, không làm gì cả (Tiết kiệm băng thông RS485)
                pass
    except Exception as e:
        rospy.logerr(f"JSON parse error: {e}")
def lidar3_cb(msg):
    vl53_data[1] = msg.ranges
def lidar4_cb(msg):
    vl53_data[2] = msg.ranges
def lidar1_cb(msg):
    vl53_data[3] = msg.ranges
def lidar2_cb(msg):
    vl53_data[4] = msg.ranges
    
def has_point_over_threshold(lidar_msg):
    if lidar_msg is None:
        return False     
    else:
        count = 0
        for i in lidar_msg:
            if i !=2.5:
                count = count + 1
        if count >=3: return True 
        else: return False
# ================== MAIN ==================
if __name__ == "__main__":
    rospy.init_node("tof_modbus_dual_publisher", anonymous=False)

    # Lấy cấu hình từ ROS param
    SERIAL_PORT = rospy.get_param("~port", "/dev/ttyS0")
    BAUDRATE = rospy.get_param("~baud", 57600)
    rospy.loginfo("Set port: {}".format(SERIAL_PORT))
    SERIAL_PORT = find_soft_port(SERIAL_PORT)
    rospy.loginfo("Connecting port: {}".format(SERIAL_PORT))

    # Khởi tạo Modbus client
    client = ModbusSerialClient(
        method='rtu',
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=0.5,
        strict=True
    )

    # Publisher cho từng slave (tùy theo cách bạn đặt tên topic)
    pub_dict = {
        1: rospy.Publisher("vl53l5cx_r3", Vl53l5cxRanges, queue_size=2),
        2: rospy.Publisher("vl53l5cx_r4", Vl53l5cxRanges, queue_size=2),
        3: rospy.Publisher("vl53l5cx_r1", Vl53l5cxRanges, queue_size=2),
        4: rospy.Publisher("vl53l5cx_r2", Vl53l5cxRanges, queue_size=2),
    }
    rospy.Subscriber("/final_cmd_vel_mux/output", Twist, cmd_vel_callback)
    # rospy.Subscriber("/safety_status",SafetyStatus, robot_status_cb)
    rospy.Subscriber("/robot_status",StringStamped, robot_status_cb)
    rospy.Subscriber("/lidar1", LaserScan, lidar1_cb)
    rospy.Subscriber("/lidar2", LaserScan, lidar2_cb)
    rospy.Subscriber("/lidar3", LaserScan, lidar3_cb)
    rospy.Subscriber("/lidar4", LaserScan, lidar4_cb)
    # rospy.Subscriber()

    if not client.connect():
        print("\033[91mKhông kết nối được Modbus RTU!\033[0m")
        log(0, "FATAL", "Cannot connect to serial port")
        rospy.signal_shutdown("Modbus connect failed")
        exit()

    print("\033[96mĐÃ KẾT NỐI THÀNH CÔNG Modbus RTU!\033[0m")
    log(0, "INFO", "Modbus connected")
    rate = rospy.Rate(15)  # 15 Hz

    active_slaves = []
    while not rospy.is_shutdown():
        # Chọn nhóm slave theo hướng chạy
        if current_linear > 0.01:
            active_slaves = FORWARD_SLAVES
        elif current_linear < -0.01:
            active_slaves = BACKWARD_SLAVES
        else :
            active_slaves = ALL_SLAVES

        if active_slaves:
            rospy.logdebug(f"Reading {'FORWARD' if current_linear > 0 else 'BACKWARD'} slaves: {active_slaves}")
        else:
            rospy.logdebug("AGV stopped, skipping sensor read")

        for sid in active_slaves:
            time.sleep(0.015)  # Tránh flood bus
            registers, _ = read_slave(sid)

            if registers and len(registers) >= 64:
                msg = Vl53l5cxRanges()
                msg.stamp = rospy.Time.now()

                # Lấy 5 hàng đầu tiên (40 giá trị), đảo ngược nếu cần định hướng đúng
                rows = [registers[i*8:(i+1)*8] for i in range(8)][:5]
                rows = list(reversed(rows))  # Đảo ngược để phù hợp góc nhìn robot
                msg.range = [float(v) for row in rows for v in row]

                pub_dict[sid].publish(msg)

                # print_matrix_5x8(registers, sid)  # Bỏ comment nếu muốn in ma trận
            else:
                print(f"\033[93m[Slave {sid}] Không nhận được dữ liệu hợp lệ\033[0m")


        rate.sleep()
