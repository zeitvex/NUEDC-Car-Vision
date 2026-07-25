'''
    基于MaixPy的云台目标追踪系统
    使用传统计算机视觉算法 + P+前馈控制
'''

from maix import camera, display, image, app, time, uart, pinmap
import math

# 初始化串口
print("初始化串口...")
pinmap.set_pin_function("A17", "UART0_RX") 
pinmap.set_pin_function("A16", "UART0_TX") 
serial1 = uart.UART("/dev/ttyS0", 115200)
print("串口初始化完成")

class ServoController:
    def __init__(self, serial_port):
        """初始化舵机控制器"""
        print("初始化舵机控制器...")
        self.ser = serial_port
        self.SERVO_CENTER = 2048
        self.current_yaw_angle = 0.0
        self.current_pitch_angle = 0.0
        self.last_command_time = time.ticks_ms()
        print("舵机控制器初始化完成")

    def angle_to_encoder(self, angle):
        """将角度转换为舵机编码器值"""
        angle = max(min(angle, 180.0), -180.0)
        encoder = self.SERVO_CENTER + (angle * 11.37777777)
        return int(max(min(encoder, 4095), 0))

    def move_servo(self, id, position, speed=0xefff):
        """发送舵机控制命令"""
        # 计算校验和
        checksum = id + 0x09 + 0x03 + 0x2A + (position & 0xFF) + ((position >> 8) & 0xFF) + 0x00 + 0x00 + (speed & 0xFF) + ((speed >> 8) & 0xFF)
        checksum = (~checksum) & 0xFF
        
        # 构建数据包
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
        
        try:
            self.ser.write(cmd)
        except Exception as e:
            print(f"舵机指令发送失败: {e}")
        time.sleep_ms(10)

    def set_position_by_angle(self, yaw_angle, pitch_angle, speed=0xe803):
        """通过角度设置云台位置"""
        # 限制舵机命令频率
        current_time = time.ticks_ms()
        elapsed = time.ticks_diff(current_time, self.last_command_time)
        if elapsed < 10:
            time.sleep_ms(10 - elapsed)
            
        yaw_pos = self.angle_to_encoder(yaw_angle) 
        pitch_pos = self.angle_to_encoder(pitch_angle)
        
        self.move_servo(1, yaw_pos, speed)
        time.sleep_ms(10)
        self.move_servo(2, pitch_pos, speed)
        
        self.current_yaw_angle = yaw_angle
        self.current_pitch_angle = pitch_angle
        self.last_command_time = time.ticks_ms()

class PFeedforwardController:
    def __init__(self):
        print("初始化P+前馈控制器...")
        # P控制参数
        self.kp_x = 0.08   # X轴比例系数
        self.kp_y = 0.06   # Y轴比例系数
        
        # 前馈控制参数
        self.kf_x = 0.02   # X轴前馈系数
        self.kf_y = 0.015  # Y轴前馈系数
        
        # 死区设置
        self.deadzone_x = 3.0
        self.deadzone_y = 3.0
        
        # 历史误差，用于前馈计算
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_time = time.ticks_ms()
        
        # 增量限制
        self.max_increment_x = 45.0
        self.max_increment_y = 30.0
        
        # 误差变化率滤波
        self.error_rate_filter_x = 0.0
        self.error_rate_filter_y = 0.0
        self.filter_alpha = 0.7  # 滤波系数
        
        print("P+前馈控制器初始化完成")

    def calculate(self, error):  # 修改：方法名从 compute_increment 改为 calculate
        """计算控制增量"""
        error_x, error_y = error  # 修改：解包error参数
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        
        # 应用死区
        if abs(error_x) < self.deadzone_x:
            error_x = 0.0
        if abs(error_y) < self.deadzone_y:
            error_y = 0.0
        
        # 计算误差变化率（用于前馈）
        if dt > 0:
            error_rate_x = (error_x - self.last_error_x) / dt
            error_rate_y = (error_y - self.last_error_y) / dt
            
            # 误差变化率滤波
            self.error_rate_filter_x = self.filter_alpha * self.error_rate_filter_x + (1 - self.filter_alpha) * error_rate_x
            self.error_rate_filter_y = self.filter_alpha * self.error_rate_filter_y + (1 - self.filter_alpha) * error_rate_y
        else:
            self.error_rate_filter_x = 0
            self.error_rate_filter_y = 0
        
        # P + 前馈控制计算
        increment_x = self.kp_x * error_x + self.kf_x * self.error_rate_filter_x
        increment_y = self.kp_y * error_y + self.kf_y * self.error_rate_filter_y
        
        # 限制增量大小
        increment_x = max(min(increment_x, self.max_increment_x), -self.max_increment_x)
        increment_y = max(min(increment_y, self.max_increment_y), -self.max_increment_y)
        
        # 更新历史值
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_time = current_time
        
        return increment_x, increment_y  # 修改：返回元组而不是列表

class TraditionalVisionDetector:
    def __init__(self):
        print("初始化传统视觉检测器...")
        # 检测参数
        self.rect_threshold_base = 12000  # 基础矩形检测阈值
        self.aspect_ratio_target = 1.5    # 目标长宽比
        self.aspect_ratio_tolerance = 0.6 # 长宽比容差
        
        # 尺寸筛选参数
        self.min_width = 15
        self.max_width = 120
        self.min_height = 12
        self.max_height = 80
        
        # 激光偏差补偿参数（从OpenMV代码移植）
        self.laser_offset_x = 10
        self.laser_offset_y = 12
        self.calibration_rect_width = 83  # 50cm处标定矩形宽度
        
        print("传统视觉检测器初始化完成")
    
    def detect_target(self, img):
        """检测目标矩形"""
        # 自适应阈值（根据环境光照调整）
        ambient_light = img.get_statistics().l_mean()
        rect_threshold = max(8000, min(15000, 20000 - ambient_light * 100))
        
        best_rect = None
        best_score = 0
        
        # 查找矩形
        rects = img.find_rects(threshold=rect_threshold)
        
        for r in rects:
            # 长宽比筛选
            aspect_ratio = r.w() / r.h() if r.h() > 0 else 0
            if not (self.aspect_ratio_target - self.aspect_ratio_tolerance < aspect_ratio < self.aspect_ratio_target + self.aspect_ratio_tolerance):
                continue
            
            # 尺寸筛选
            if not (self.min_width < r.w() < self.max_width and self.min_height < r.h() < self.max_height):
                continue
            
            # 计算评分（面积越大越好，长宽比越接近目标越好）
            area_score = r.w() * r.h()
            ratio_score = 1.0 / (1.0 + abs(aspect_ratio - self.aspect_ratio_target))
            total_score = area_score * ratio_score
            
            if total_score > best_score:
                best_score = total_score
                best_rect = r
        
        return best_rect
    
    def calculate_target_center(self, rect, img_width, img_height):
        """计算目标中心点和误差"""
        if rect is None:
            return None, None, None
        
        # 计算矩形中心
        center_x = rect.x() + rect.w() // 2
        center_y = rect.y() + rect.h() // 2
        
        # 计算激光偏差补偿
        scale_factor = rect.w() / self.calibration_rect_width
        compensated_offset_x = int(self.laser_offset_x * scale_factor)
        compensated_offset_y = int(self.laser_offset_y * scale_factor)
        
        # 计算误差（目标中心相对于图像中心的偏差）
        error_x = center_x - img_width // 2 - compensated_offset_x
        error_y = center_y - img_height // 2 - compensated_offset_y
        
        return (center_x, center_y), (error_x, error_y), rect

class TargetTracker:
    def __init__(self):
        print("初始化目标追踪器...")
        
        # 初始化摄像头
        self.cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
        self.cam.constrast(80)  # 设置对比度
        
        # 初始化显示
        self.disp = display.Display()
        
        # 初始化各个组件
        self.servo_controller = ServoController(serial1)
        self.controller = PFeedforwardController()
        self.detector = TraditionalVisionDetector()
        
        # 追踪状态
        self.target_lost_count = 0
        self.max_lost_count = 10  # 修改：使用正确的属性名
        self.tracking_active = False
        
        print("目标追踪器初始化完成")
    
    def run(self):
        """主运行循环"""
        print("开始目标追踪...")
        
        while not app.need_exit():
            try:
                # 获取图像
                img = self.cam.read()
                
                # 检测目标
                target_rect = self.detector.detect_target(img)
                
                if target_rect is not None:
                    # 计算目标中心和误差
                    center, error, rect = self.detector.calculate_target_center(
                        target_rect, img.width(), img.height()
                    )
                    
                    if center is not None and error is not None:
                        # 绘制目标矩形
                        img.draw_rect(rect.x(), rect.y(), rect.w(), rect.h(), color=image.COLOR_RED, thickness=2)
                        
                        # P+前馈控制计算
                        control_output = self.controller.calculate(error)  # 现在调用正确的方法
                        
                        # 计算新的云台角度
                        new_yaw = self.servo_controller.current_yaw_angle + control_output[0]
                        new_pitch = self.servo_controller.current_pitch_angle + control_output[1]
                        
                        # 限制角度范围
                        new_yaw = max(min(new_yaw, 90.0), -90.0)
                        new_pitch = max(min(new_pitch, 45.0), -45.0)
                        
                        # 设置舵机位置
                        self.servo_controller.set_position_by_angle(new_yaw, new_pitch)
                        
                        # 重置丢失计数
                        self.target_lost_count = 0
                        self.tracking_active = True
                        
                        # 显示跟踪信息
                        img.draw_string(10, 10, f"Error: ({error[0]:.0f}, {error[1]:.0f})", color=image.COLOR_WHITE, scale=1)
                        img.draw_string(10, 30, f"Angle: ({new_yaw:.1f}, {new_pitch:.1f})", color=image.COLOR_WHITE, scale=1)
                    else:
                        self.target_lost_count += 1
                        
                        # 如果丢失目标超过阈值，停止跟踪
                        if self.target_lost_count > self.max_lost_count:  # 修改：使用正确的属性名
                            self.tracking_active = False
                        
                        # 显示丢失状态
                        img.draw_string(10, 10, "Target Lost", color=image.COLOR_RED, scale=2)
                        img.draw_string(10, 50, f"Lost Count: {self.target_lost_count}", color=image.COLOR_RED, scale=1)
                
                # 显示图像
                self.disp.show(img)
                
            except Exception as e:
                print(f"追踪循环错误: {e}")
                time.sleep_ms(100)

# 主程序入口
if __name__ == "__main__":
    try:
        tracker = TargetTracker()
        tracker.run()
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        print("程序结束")