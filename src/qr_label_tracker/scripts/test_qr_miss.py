import rospy
import os
from datetime import datetime
from agv_msgs.msg import DataMatrixStamped

MASTER_PATH = [
    (36000, 41900), (35500, 41900),
    (35000, 41900), (34000, 41900), (33000, 41900), (32000, 41900),
    (31000, 41900), (30000, 41900), (29000, 41900), (28000, 41900),
    (27000, 41900), (26000, 41900), (25000, 41900), (24000, 41900),
    (23000, 41900), (22000, 41900), (21500, 41900), (21000, 41900)
]

class QrMissDetector:
    def __init__(self):
        rospy.init_node('qr_miss_checker', anonymous=True)
        
        self.last_index = -1 
        self.report_path = os.path.expanduser('~/qr_miss_report.txt')
        
        # Tạo file header nếu chưa tồn tại
        if not os.path.exists(self.report_path):
            self.create_log_header()
        
        rospy.Subscriber("/data_gls621", DataMatrixStamped, self.callback)
        rospy.loginfo(f"Node started. Log file: {self.report_path}")

    def create_log_header(self):
        """Tạo header cho file log lần đầu"""
        try:
            with open(self.report_path, "w") as f:
                f.write("=" * 70 + "\n")
                f.write("LOG CÁC ĐIỂM QR BỊ MISS\n")
                f.write("=" * 70 + "\n")
                f.write(f"{'TIMESTAMP':<25} | {'TOẠ ĐỘ QR BỊ MISS':<20}\n")
                f.write("-" * 70 + "\n")
        except Exception as e:
            rospy.logerr(f"Lỗi tạo file log: {e}")

    def append_miss_log(self, point):
        """Ghi thêm một dòng log miss vào cuối file"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with open(self.report_path, "a") as f:
                f.write(f"{timestamp:<25} | {str(point):<20}\n")
        except Exception as e:
            rospy.logerr(f"Lỗi ghi log: {e}")

    def callback(self, msg):
        try:
            # Sửa lỗi typo: lable → label
            current_x = int(msg.label.x)
            current_y = int(msg.label.y)
            current_qr = (current_x, current_y)

            if current_qr not in MASTER_PATH: 
                return
                
            current_index = MASTER_PATH.index(current_qr)

            if self.last_index == -1:
                self.last_index = current_index
                rospy.loginfo(f"Bắt đầu tại: {current_qr}")
                return

            diff = current_index - self.last_index

            if abs(diff) > 1:
                missed_list = []
                if diff > 0: 
                    missed_list = MASTER_PATH[self.last_index + 1 : current_index]
                else: 
                    missed_list = MASTER_PATH[self.last_index - 1 : current_index : -1]

                for point in missed_list:
                    rospy.logwarn(f"MISS: {point}")
                    # Ghi ngay vào file với timestamp
                    self.append_miss_log(point)

            self.last_index = current_index

        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    try:
        detector = QrMissDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
