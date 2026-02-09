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
    log(slave_id, "WRITE_ERROR", f"Failed after {max_retries} retries")
    print(f"\033[91m[Slave {slave_id}] WRITE FAILED after retries\033[0m")
    return False

# ================== ĐỌC MODBUS ==================
def read_slave(slave_id, start_addr, count):
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