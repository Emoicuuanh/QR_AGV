from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import rospy
import time
import threading

modbus_lock = threading.Lock()

def log(slave_id, msg_type, message):
    """Simple logging function"""
    rospy.loginfo(f"[Slave {slave_id}] {msg_type}: {message}")

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
                return True
        except Exception as e:
            rospy.logwarn(f"[Slave {slave_id}] Write Exception attempt {attempt+1}: {e}")
    # Nếu hết 3 lần vẫn lỗi thì mới log lỗi và trả về False
    log(slave_id, "WRITE_ERROR", f"Failed after {max_retries} retries")
    print(f"\033[91m[Slave {slave_id}] WRITE FAILED after retries\033[0m")
    return False

# ================== ĐỌC MODBUS ==================
def read_slave(slave_id, start_addr=65, count=64):
    global modbus_lock
    try:
        t0 = time.time()
        with modbus_lock:
            response = client.read_holding_registers(start_addr, count, unit=slave_id)
        elapsed = time.time() - t0

        if response.isError():
            print(f"\033[91m[Slave {slave_id}] LỖI: {response}\033[0m")
            log(slave_id, "ERROR", str(response))
            return None, elapsed
        else:
            return response.registers, elapsed
    except ModbusException as e:
        print(f"\033[91m[Slave {slave_id}] ModbusException: {e}\033[0m")
        log(slave_id, "ERROR", str(e))
        return None, 0.0
    except Exception as e:
        print(f"\033[91m[Slave {slave_id}] Lỗi: {e}\033[0m")
        log(slave_id, "ERROR", str(e))
        return None, 0.0
    
if __name__ == "__main__":
    rospy.init_node("modbus_tcp_publisher", anonymous=False)

    # Lấy cấu hình từ ROS param cho Modbus TCP/IP
    PLC_IP = rospy.get_param("~plc_ip", "192.168.3.250")
    PLC_PORT = rospy.get_param("~plc_port", 502)
    rospy.loginfo("Connecting to PLC: {}:{}".format(PLC_IP, PLC_PORT))

    # Khởi tạo Modbus TCP client
    client = ModbusTcpClient(
        host=PLC_IP,
        port=PLC_PORT,
        timeout=3.0
    )

    if not client.connect():
        print("\033[91mKhông kết nối được Modbus TCP!\033[0m")
        log(0, "FATAL", "Cannot connect to PLC")
        rospy.signal_shutdown("Modbus connect failed")
        exit()
    
    rospy.loginfo("✓ Connected to PLC successfully!")
    rate = rospy.Rate(15)  # 15 Hz

    active_slaves = []
    while not rospy.is_shutdown():
        # Ghi vào thanh ghi 200
        # result = write_slave(slave_id=1, start_addr=200, values=[3])
        # if result:
        #     rospy.loginfo("✓ Successfully wrote value 1 to register 200")
        # else:
        #     rospy.logerr("✗ Failed to write to register 200")
        
        # Đọc lại để kiểm tra (optional)
        data, elapsed = read_slave(slave_id=1, start_addr=201, count=1)
        if data:
            rospy.loginfo(f"Read back from register 200: {data[0]}")
        
        rate.sleep()
    
    # Đóng kết nối khi shutdown
    client.close()
    rospy.loginfo("Modbus TCP connection closed.")
