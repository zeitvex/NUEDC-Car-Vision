from maix import uart, pinmap 
import time 
import math 
 
# 使用UART0
pinmap.set_pin_function("A17", "UART0_RX") 
pinmap.set_pin_function("A16", "UART0_TX") 
 
serial1 = uart.UART("/dev/ttyS0", 115200)  # 使用115200波特率
 
# 舵机控制类 
class ServoController: 
    def __init__(self, serial_port): 
        """初始化舵机控制器""" 
        self.ser = serial_port 
        self.SERVO_CENTER = 2048  # 舵机中心位置 
 
    def angle_to_encoder(self, angle): 
        """将角度转换为舵机编码器值""" 
        if angle > 180.0: 
            angle = 180.0 
        if angle < -180.0: 
            angle = -180.0 
 
        encoder = self.SERVO_CENTER + (angle * 11.37777777) 
        if encoder > 4095: 
            encoder = 4095 
        if encoder < 0: 
            encoder = 0 
 
        return int(encoder) 
 
    def calculate_servo_position(self, x, y, distance): 
        """将坐标转换为舵机位置""" 
        if distance < 0.1: 
            distance = 0.1 
 
        yaw = math.atan2(x, distance) * 180.0 / math.pi 
        projected_distance = math.sqrt(x * x + distance * distance) 
        pitch = math.atan2(y, projected_distance) * 180.0 / math.pi 
 
        yaw_encoder = self.angle_to_encoder(yaw) 
        pitch_encoder = self.angle_to_encoder(pitch) 
 
        return (yaw_encoder, pitch_encoder) 
 
    def move_servo(self, id, position, speed=0xe803): 
        """发送舵机控制命令""" 
        # 计算校验和
        checksum = id + 0x09 + 0x03 + 0x2A + (position & 0xFF) + ((position >> 8) & 0xFF) + 0x00 + 0x00 + (speed & 0xFF) + ((speed >> 8) & 0xFF)
        checksum = (~checksum) & 0xFF
        
        # 构建数据包 - 使用bytes
        cmd = bytes([
            0xFF, 0xFF,           # 字头 
            id,                   # ID 
            0x09,                 # 长度 
            0x03,                 # 指令 
            0x2A,                 # 参数首地址 
            position & 0xFF,      # 位置低字节 
            (position >> 8) & 0xFF, # 位置高字节 
            0x00,                 # 预留低字节 
            0x00,                 # 预留高字节 
            speed & 0xFF,         # 速度低字节 
            (speed >> 8) & 0xFF,  # 速度高字节
            checksum              # 校验和
        ])
        
        print(f"发送到舵机 {id}: {[hex(b) for b in cmd]}")
        # 发送数据
        self.ser.write(cmd)
        time.sleep(0.01)  # 短暂延时，避免指令冲突
        
        return True
 
    def set_position(self, x_pos, y_pos, speed=0xe803): 
        """设置云台的X和Y位置""" 
        self.move_servo(1, x_pos, speed) 
        time.sleep(0.01)  # 短暂延时，避免指令冲突
        self.move_servo(2, y_pos, speed) 
 
    def set_position_by_angle(self, yaw_angle, pitch_angle, speed=0xe803): 
        """通过角度设置云台位置""" 
        yaw_pos = self.angle_to_encoder(yaw_angle) 
        pitch_pos = self.angle_to_encoder(pitch_angle) 
        print(f"角度转换: 偏航={yaw_angle}° -> {yaw_pos}, 俯仰={pitch_angle}° -> {pitch_pos}")
        self.set_position(yaw_pos, pitch_pos, speed) 
 
    def set_position_by_coordinates(self, x, y, distance, speed=0xe803): 
        """通过坐标设置云台位置""" 
        yaw_pos, pitch_pos = self.calculate_servo_position(x, y, distance) 
        self.set_position(yaw_pos, pitch_pos, speed) 
 
    def center_position(self): 
        """将云台归位到中心位置""" 
        self.set_position(self.SERVO_CENTER, self.SERVO_CENTER) 
        print("云台已归位到中心位置") 
 
# 创建舵机控制器实例 
servo_controller = ServoController(serial1) 
 
# 主程序 
try: 
    print("检查串口是否正常打开")
    if serial1.is_open():
        print("串口已成功打开")
    else:
        print("串口未打开，请检查连接")
        
    # 将云台归位到中心 
    print("归位到中心") 
    servo_controller.center_position() 
    time.sleep(1) 
 
    # 控制云台移动到不同位置 
    print("移动到左侧") 
    servo_controller.set_position_by_angle(-30, 0) 
    time.sleep(5)  # 增加等待时间
 
    print("移动到右侧") 
    servo_controller.set_position_by_angle(30, 0) 
    time.sleep(5)  # 增加等待时间
 
    print("移动到上方") 
    servo_controller.set_position_by_angle(0, -30) 
    time.sleep(5)  # 增加等待时间
 
    print("移动到下方") 
    servo_controller.set_position_by_angle(0, 30) 
    time.sleep(5)  # 增加等待时间
 
    # 最后回到中心位置 
    print("回到中心位置") 
    servo_controller.center_position() 
 
except Exception as e: 
    print(f"错误: {e}")
