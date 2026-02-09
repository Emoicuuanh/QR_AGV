from pymodbus.client.sync import ModbusTcpClient
import time

# --- CẤU HÌNH THÔNG SỐ ---
SERVER_IP = '192.168.1.200'  # Thay bằng IP thực tế của board
PORT = 502
UNIT_ID = 1  # Thường là 1 cho Waveshare
DELAY = 0.1  # Tốc độ phản hồi (0.1 giây mỗi lần quét)

def sync_in_out():
    # Khởi tạo kết nối
    client = ModbusTcpClient(SERVER_IP, port=PORT)
    
    print(f"Bắt đầu chương trình đồng bộ tại {SERVER_IP}...")
    print("Nhấn Ctrl+C để dừng chương trình.")
    
    previous_input = [None] * 8
    previous_output = [None] * 8

    while True:
        try:
            if not client.connect():
                print("Mất kết nối! Đang thử kết nối lại...")
                time.sleep(2)
                continue
            input_data = client.read_discrete_inputs(0, 8, unit=UNIT_ID)
            if not input_data.isError():

                input_states = input_data.bits[:8]
                client.write_coils(address=0, values=input_states, slave=UNIT_ID)
                for i, state in enumerate(input_states):
                    if state != previous_input[i]:
                        print(f"[IN{i}]: {'ON' if state else 'OFF'}")
                        previous_input[i] = state
                output_data = client.read_coils(address=0, count=8, slave=UNIT_ID)
                if not output_data.isError():
                    output_states = output_data.bits[:8]
                    for i, state in enumerate(output_states):
                        if state != previous_output[i]:
                            print(f"[OUT{i}]: {'ON' if state else 'OFF'}")
                            previous_output[i] = state
            else:
                print("Lỗi khi đọc dữ liệu từ board.")

        except KeyboardInterrupt:
            print("\nĐang dừng chương trình...")
            break
        except Exception as e:
            print(f"Lỗi phát sinh: {e}")
            time.sleep(1)
        
        # Chờ một khoảng thời gian ngắn trước khi quét lại
        time.sleep(DELAY)

    client.close()
    print("Đã đóng kết nối.")

if __name__ == "__main__":
    sync_in_out()