'''
    2025电赛E题检测+云台控制
'''

from maix import camera, display, image, nn, app, time, uart, pinmap
import cv2
import numpy as np
import os
import math

# 初始化串口
print("初始化串口...")
pinmap.set_pin_function("A17", "UART0_RX") 
pinmap.set_pin_function("A16", "UART0_TX") 
serial1 = uart.UART("/dev/ttyS0", 115200)  # 使用115200波特率
print("串口初始化完成")
class DistanceAwareController:
    def __init__(self):
        print("初始化距离感知控制器...")
        
        # 基准距离下的映射系数
        self.base_x_angle_per_pixel = 0.01
        self.base_y_angle_per_pixel = 0.005
        
        # 非线性校正因子
        self.x_nonlinear_factor = 1.05
        self.y_nonlinear_factor = 0.95
        
        # 角度变化限制
        self.min_angle_change = 0.02
        self.max_angle_change_x = 1.0
        self.max_angle_change_y = 0.5
        
        # 误差死区
        self.deadzone_x = 5.0
        self.deadzone_y = 5.0
        
        print("距离感知控制器初始化完成")
    
    def estimate_distance(self, rect_area, rect_width, rect_height):
        """
        估计目标距离
        
        参数:
        rect_area - 检测到的矩形区域面积
        rect_width - 检测到的矩形宽度
        rect_height - 检测到的矩形高度
        
        返回:
        distance_factor - 距离因子，值越大表示距离越远
        """
        # 使用矩形面积估计距离
        base_area = 40000  # 基准面积，需要通过实验确定
        
        # 计算距离因子
        if rect_area > 0:
            distance_factor = (base_area / rect_area) ** 0.5
        else:
            distance_factor = 1.0  # 默认值
            
        # 也可以结合宽高比进行校正
        aspect_ratio = rect_width / rect_height if rect_height > 0 else 0.707
        aspect_error = abs(aspect_ratio - 0.707)
        
        # 如果宽高比异常，可能是因为视角问题，调整距离因子
        if aspect_error > 0.2:  # 宽高比误差较大
            distance_factor *= (1 + aspect_error * 0.5)  # 适当增加距离因子
            
        return distance_factor
    
    def compute_angle_change(self, error_x, error_y, rect_info=None):
        """
        根据误差和距离信息计算角度变化
        
        参数:
        error_x - X方向像素误差
        error_y - Y方向像素误差
        rect_info - 包含矩形信息的字典 {area, width, height}
        """
        # 计算距离因子
        distance_factor = 1.0
        if rect_info:
            distance_factor = self.estimate_distance(
                rect_info['area'], 
                rect_info['width'], 
                rect_info['height']
            )
            
        # 根据距离调整映射系数
        x_angle_per_pixel = self.base_x_angle_per_pixel * distance_factor
        y_angle_per_pixel = self.base_y_angle_per_pixel * distance_factor
        
        print(f"距离因子: {distance_factor:.2f}, X系数: {x_angle_per_pixel:.4f}, Y系数: {y_angle_per_pixel:.4f}")
        
        # 应用死区
        if abs(error_x) < self.deadzone_x:
            angle_change_x = 0
        else:
            # 基本线性映射
            angle_change_x = error_x * x_angle_per_pixel
            
            # 非线性校正（大误差时提供更强的校正）
            if abs(error_x) > 30:
                angle_change_x *= self.x_nonlinear_factor
            
            # 限制最大变化
            angle_change_x = max(min(angle_change_x, self.max_angle_change_x), -self.max_angle_change_x)
            
            # 如果变化太小但不在死区内，确保至少有最小变化
            if 0 < abs(angle_change_x) < self.min_angle_change:
                angle_change_x = self.min_angle_change * (1 if angle_change_x > 0 else -1)
        
        # Y方向同理
        if abs(error_y) < self.deadzone_y:
            angle_change_y = 0
        else:
            angle_change_y = error_y * y_angle_per_pixel
            
            if abs(error_y) > 30:
                angle_change_y *= self.y_nonlinear_factor
                
            angle_change_y = max(min(angle_change_y, self.max_angle_change_y), -self.max_angle_change_y)
            
            if 0 < abs(angle_change_y) < self.min_angle_change:
                angle_change_y = self.min_angle_change * (1 if angle_change_y > 0 else -1)
        
        return angle_change_x, angle_change_y



class ServoController:
    def __init__(self, serial_port):
        """初始化舵机控制器"""
        print("初始化舵机控制器...")
        self.ser = serial_port
        self.SERVO_CENTER = 2048  # 舵机中心位置
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
        print(f"移动舵机 ID:{id} 位置:{position} 速度:{speed}")
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
        
        # 发送数据
        try:
            bytes_written = self.ser.write(cmd)
            print(f"舵机指令已发送，写入 {bytes_written} 字节")
        except Exception as e:
            print(f"舵机指令发送失败: {e}")
        time.sleep_ms(10)  # 短暂延时，避免指令冲突

    def set_position_by_angle(self, yaw_angle, pitch_angle, speed=0xe803):
        """通过角度设置云台位置"""
        print(f"设置舵机角度: Yaw={yaw_angle:.2f}°, Pitch={pitch_angle:.2f}°")
        # 限制舵机命令频率
        current_time = time.ticks_ms()
        elapsed = time.ticks_diff(current_time, self.last_command_time)
        if elapsed < 10:  # 确保至少10ms间隔
            time.sleep_ms(10 - elapsed)
            
        yaw_pos = self.angle_to_encoder(yaw_angle) 
        pitch_pos = self.angle_to_encoder(pitch_angle)
        
        self.move_servo(1, yaw_pos, speed)
        time.sleep_ms(10)  # 短暂延时，避免指令冲突
        self.move_servo(2, pitch_pos, speed)
        
        self.current_yaw_angle = yaw_angle
        self.current_pitch_angle = pitch_angle
        self.last_command_time = time.ticks_ms()
        
        print(f"当前舵机角度: Yaw={yaw_angle:.2f}°, Pitch={pitch_angle:.2f}°")
        print(f"编码值: Yaw={yaw_pos}, Pitch={pitch_pos}")

    def center_position(self):
        """将云台归位到中心位置"""
        print("将云台归位到中心位置...")
        self.set_position_by_angle(0.0, 0.0)
        print("云台已归位到中心位置")
class DistanceAwareController:
    def __init__(self):
        print("初始化距离感知控制器...")
        
        # 基准距离下的映射系数
        self.base_x_angle_per_pixel = 0.05
        self.base_y_angle_per_pixel = 0.04
        
        # 非线性校正因子
        self.x_nonlinear_factor = 1.2
        self.y_nonlinear_factor = 1.1
        
        # 角度变化限制
        self.min_angle_change = 0.1
        self.max_angle_change_x = 15.0
        self.max_angle_change_y = 12.0
        
        # 误差死区
        self.deadzone_x = 2.0
        self.deadzone_y = 2.0
        
        print("距离感知控制器初始化完成")
    
    def estimate_distance(self, rect_area, rect_width, rect_height):
        """
        估计目标距离
        
        参数:
        rect_area - 检测到的矩形区域面积
        rect_width - 检测到的矩形宽度
        rect_height - 检测到的矩形高度
        
        返回:
        distance_factor - 距离因子，值越大表示距离越远
        """
        # 使用矩形面积估计距离
        # A4纸的标准尺寸是固定的，所以其在图像中的面积与距离平方成反比
        # 我们可以用一个基准面积来计算相对距离
        base_area = 40000  # 基准面积，需要通过实验确定
        
        # 计算距离因子
        if rect_area > 0:
            distance_factor = (base_area / rect_area) ** 0.5
        else:
            distance_factor = 1.0  # 默认值
            
        # 也可以结合宽高比进行校正
        # 标准A4纸的宽高比约为0.707 (210mm/297mm)
        aspect_ratio = rect_width / rect_height if rect_height > 0 else 0.707
        aspect_error = abs(aspect_ratio - 0.707)
        
        # 如果宽高比异常，可能是因为视角问题，调整距离因子
        if aspect_error > 0.2:  # 宽高比误差较大
            distance_factor *= (1 + aspect_error * 0.5)  # 适当增加距离因子
            
        return distance_factor
    
    def compute_angle_change(self, error_x, error_y, rect_info=None):
        """
        根据误差和距离信息计算角度变化
        
        参数:
        error_x - X方向像素误差
        error_y - Y方向像素误差
        rect_info - 包含矩形信息的字典 {area, width, height}
        """
        # 计算距离因子
        distance_factor = 1.0
        if rect_info:
            distance_factor = self.estimate_distance(
                rect_info['area'], 
                rect_info['width'], 
                rect_info['height']
            )
            
        # 根据距离调整映射系数
        x_angle_per_pixel = self.base_x_angle_per_pixel * distance_factor
        y_angle_per_pixel = self.base_y_angle_per_pixel * distance_factor
        
        print(f"距离因子: {distance_factor:.2f}, X系数: {x_angle_per_pixel:.4f}, Y系数: {y_angle_per_pixel:.4f}")
        
        # 应用死区
        if abs(error_x) < self.deadzone_x:
            angle_change_x = 0
        else:
            # 基本线性映射
            angle_change_x = error_x * x_angle_per_pixel
            
            # 非线性校正（大误差时提供更强的校正）
            if abs(error_x) > 30:
                angle_change_x *= self.x_nonlinear_factor
            
            # 限制最大变化
            angle_change_x = max(min(angle_change_x, self.max_angle_change_x), -self.max_angle_change_x)
            
            # 如果变化太小但不在死区内，确保至少有最小变化
            if 0 < abs(angle_change_x) < self.min_angle_change:
                angle_change_x = self.min_angle_change * (1 if angle_change_x > 0 else -1)
        
        # Y方向同理
        if abs(error_y) < self.deadzone_y:
            angle_change_y = 0
        else:
            angle_change_y = error_y * y_angle_per_pixel
            
            if abs(error_y) > 30:
                angle_change_y *= self.y_nonlinear_factor
                
            angle_change_y = max(min(angle_change_y, self.max_angle_change_y), -self.max_angle_change_y)
            
            if 0 < abs(angle_change_y) < self.min_angle_change:
                angle_change_y = self.min_angle_change * (1 if angle_change_y > 0 else -1)
        
        return angle_change_x, angle_change_y

class PDController:
    def __init__(self):
        print("初始化PD控制器...")
        # PD控制参数
        self.kp_x = 0.1  # X轴比例系数
        self.kp_y = 0.05  # Y轴比例系数

        self.kd_x = 0.010  # X轴微分系数
        self.kd_y = 0.015  # Y轴微分系数

        # 死区设置
        self.deadzone_x = 2.0
        self.deadzone_y = 2.0

        # 历史误差，用于计算变化率
        self.last_error_x = 0.0
        self.last_error_y = 0.0

        # 增量限制
        self.max_increment_x = 1.5
        self.max_increment_y = 1.0

        # 不同误差区域的增益调整
        self.large_error_threshold = 70  # 大误差阈值
        self.gain_large = 2.5  # 大误差增益
        self.gain_small = 0.5  # 小误差增益
        print("PD控制器初始化完成")

    def compute_increment(self, error, axis='x', error_rate=0.0):
        # 应用死区
        if axis == 'x' and abs(error) < self.deadzone_x:
            return 0.0
        if axis == 'y' and abs(error) < self.deadzone_y:
            return 0.0

        # 根据误差大小选择增益
        gain = self.gain_large if abs(error) > self.large_error_threshold else self.gain_small

        # PD控制计算
        if axis == 'x':
            p_term = self.kp_x * error
            d_term = self.kd_x * error_rate
            increment = (p_term - d_term) * gain

            # 限制增量大小
            max_inc = self.max_increment_x
            if abs(error) > self.large_error_threshold:
                max_inc = 2.0
            increment = max(min(increment, max_inc), -max_inc)
        else:
            p_term = self.kp_y * error
            d_term = self.kd_y * error_rate
            increment = (p_term - d_term) * gain

            # 限制增量大小
            max_inc = self.max_increment_y
            if abs(error) > self.large_error_threshold:
                max_inc = 1.5
            increment = max(min(increment, max_inc), -max_inc)

        return increment

class FindRectCircle:
    DEBUG=False
    PRINT_TIME = False
    debug_draw_err_line = False
    debug_draw_err_msg = False
    debug_draw_circle = False
    debug_draw_rect = False
    debug_show_hires = False

    ################################ config #########################################

    # DEBUG=True                 # 打开调试模式，取消注释即可
    # PRINT_TIME = True          # 打印每一步消耗的时间，取消注释即可
    debug_draw_err_line = True   # 画出圆心和画面中心的误差线，需要消耗1ms左右时间
    # debug_draw_err_msg = True    # 画出圆心和画面中心的误差值和 FPS 信息，需要消耗7ms左右时间，慎用
    debug_draw_circle = True       # 画出圆圈，实际是画点，需要再打开变量, debug 模式都会画，耗费时间比较多，慎用
    # debug_draw_rect = True         # 画出矩形框
    # debug_show_hires = True        # 显示结果在高分辨率图上，而不是小分辨率图上， 开启了 hires_mode 才生效


    crop_padding = 12            # 裁切图时的外扩距离，调试到保证最近和最远位置整个黑框在检测框里，可以打开 DEBUG 模式看
    rect_min_limit = 12          # 找到的大黑边框四个点最小距离必须大于这个值才有效，防止找到错误的值，可以放到最远位置测试
    std_from_white_rect = True   # 裁切标准图是裁切自A4纸内部白色部分（更精准），False则是带黑框的外围框（整个A4纸）（更快一点点）
    circle_num_points = 50       # 生成的第三个圆圈的点数量，控制圆边的平滑程度，可以用来巡迹
    std_res = [int(29.7 / 21 * 80), 80]        # 找中心点和圆圈的分辨率，越大越精确，更慢，A4 29.7 x 21cm
    hires_mode = True           # 高分辨模式，适合 find_circle 模式使用，帧率会更低但是找圆圈更精准
                                # 不 find_circle 也可以使用，找4个角点更精准，需要配合设置合理的 std_res
                                # 注意开启了这个模式，输出的误差值也是基于大图的分辨率
    high_res = 448               # 高分辨率模式宽高,越高越清晰但是帧率越低，注意 std_res 也要跟着改大点
    model_path = "/root/models/model_3356.mud" # 检测黑框模型路径，从 https://maixhub.com/model/zoo/1159 下载并传到开发板的 /root/models 目录
    model_dual_buff_mode = True  # 模型双缓冲模式，开启了帧率会高一帧处理的时间，但是延迟也会高一帧

    find_circle = False          # 在找到黑框以内白框后是否继续找圆，如果圆圈画得标准，在纸正中心则不用找，如果画点不在纸正中心则需要找。
                                # 建议把A4纸制作正确就不用找了，帧率更高。
                                # 可以用hires_mode 更清晰才能识别到，另外设置合理的 std_res
    cam_buff_num = 1             # 摄像头缓冲， 1 延迟更低帧率慢一点点， 2延迟更高帧率高一点点
    find_laser = False           # 找激光点（未测试），实际使用时直接把摄像头中心和激光点保持移植就好了，不需要找激光点

    auto_awb = True                             # 自动白平衡或者手动白平衡
    awb_gain = [0.134, 0.0625, 0.0625, 0.1139]  # 手动白平衡，auto_awb为False才生效， R GR GB B 的值，调 R 和 B 即可
    contrast = 80                               # 对比度，会影响到检测，阴影和圆圈痕迹都会更重

    # 舵机控制相关参数
    target_error = [-25, -15]    # 目标误差值
    error_tolerance = 3.0        # 误差容许范围
    jump_threshold = 80          # 大幅调整阈值
    max_lost_count = 10          # 最大丢失计数

    ###################################################################################

    def __init__(self, disp):
        print("初始化FindRectCircle...")
        if not os.path.exists(self.model_path):
            model_path1 = "model/model_3356.mud"
            if not os.path.exists(model_path1):
                print(f"load model failed, please put model in {self.model_path}, or {os.getcwd()}/{model_path1}")
            model_path = model_path1

        self.disp = disp
        print("初始化检测器...")
        # 初始化摄像头
        self.detector = nn.YOLOv5(model=self.model_path, dual_buff = self.model_dual_buff_mode)
        print("检测器初始化完成")
         # 初始化距离感知控制器
        self.distance_controller = DistanceAwareController()
        print("初始化摄像头...")
        # 初始化摄像头
        if self.hires_mode:
            self.cam = camera.Camera(self.high_res, self.high_res, self.detector.input_format(), buff_num=self.cam_buff_num)
        else:
            self.cam = camera.Camera(self.detector.input_width(), self.detector.input_height(), self.detector.input_format(), buff_num=self.cam_buff_num)
        if not self.auto_awb:
            self.cam.awb_mode(camera.AwbMode.Manual)
            self.cam.set_wb_gain(self.awb_gain)
        self.cam.constrast(self.contrast)
        print("摄像头初始化完成")
        # cam.set_windowing([448, 448])

        self._t = time.ticks_ms()

        self.err_center = [0, 0] # 距离中心的误差
        self.center_pos = [self.cam.width() // 2, self.cam.height() // 2] # 画面的中心
        self.last_center = self.center_pos # 上一次检测到的圆心距离
        self.last_circle3_points = []
        self.last_center_small = [self.detector.input_width() // 2, self.detector.input_height() // 2] # 高清模式时，在小图的中心坐标
        self.updated = False
        self.center_pos_small = [self.detector.input_width() // 2, self.detector.input_height() // 2] # 画面的中心
        # 注意这里只考虑到了拉伸缩放(iamge.Fit.FILL)
        self.img_ai_scale = [self.cam.width() / self.detector.input_width(), self.cam.height() / self.detector.input_height()]
        
        # 舵机控制相关
        self.servo_controller = ServoController(serial1)
        self.pd_controller = PDController()
        
        # 目标跟踪状态
        self.target_found = False
        self.stable_count = 0
        self.target_lost_count = 0
        self.initial_adjustment_done = False
        self.last_error_time = time.ticks_ms()
        
        print("FindRectCircle初始化完成")

    def get_res(self):
        return [self.cam.width(), self.cam.height()]

    def debug_time(self, msg):
        if self.PRINT_TIME:
            print(f"t: {time.ticks_diff(time.ticks_ms(), self._t):4d} {msg}")
            self._t = time.ticks_ms()

    def find_laser_point(self, img, original_img):
        '''
            随便写的，有需要请自己修改算法
        '''

        # 这里需要调阈值
        ths = [[0, 100, -128, 127, -128, -18]]
        blobs = self.img_std.find_blobs(ths, x_stride=2, y_stride=2)
        max_s = 0
        max_b = None
        for b in blobs:
            s = b.w() * b.h()
            if s > max_s:
                max_s = s
                max_b = b
        if self.DEBUG:
            laser_binary = img.binary(ths, copy=True)
            original_img.draw_image(original_img.width() - laser_binary.width(), original_img.height() - laser_binary.height(), laser_binary)
        return max_b

    def control_servo(self):
        """根据检测到的误差直接控制舵机到目标位置"""
        # 如果没有更新，不进行控制
        if not self.updated:
            self.target_lost_count += 1
            if self.target_lost_count >= self.max_lost_count:
                print("目标丢失，等待重新检测...")
                self.target_found = False
            return

        # 重置目标丢失计数
        self.target_lost_count = 0
        
        # 计算与目标误差的差值
        error_diff = [
            self.err_center[0] - self.target_error[0],
            self.err_center[1] - self.target_error[1]
        ]
        
        error_magnitude = math.sqrt(error_diff[0]**2 + error_diff[1]**2)
        print(f"误差差值: X={error_diff[0]:.1f}, Y={error_diff[1]:.1f}, 误差大小={error_magnitude:.1f}")
        
        # 检查是否达到目标
        if abs(error_diff[0]) < self.error_tolerance and abs(error_diff[1]) < self.error_tolerance:
            self.stable_count += 1
            if self.stable_count >= 3:
                if not self.target_found:
                    self.target_found = True
                    print(f"目标误差 ({self.target_error[0]}, {self.target_error[1]}) 达成")
        else:
            self.stable_count = 0
            self.target_found = False
        
        # 如果已经达到目标且稳定，不进行调整
        if self.target_found:
            print("已达到目标位置")
            return
        
        # 收集矩形信息
        rect_info = None
        if hasattr(self, 'last_rect_info'):
            rect_info = self.last_rect_info
        
        # 使用距离感知控制器计算角度变化
        angle_change_x, angle_change_y = self.distance_controller.compute_angle_change(
            error_diff[0], error_diff[1], rect_info
        )
        
        # 计算新的角度
        yaw_angle = self.servo_controller.current_yaw_angle + angle_change_x
        pitch_angle = self.servo_controller.current_pitch_angle + angle_change_y
        
        # 限制角度范围
        yaw_angle = max(min(yaw_angle, 180.0), -180.0)
        pitch_angle = max(min(pitch_angle, 180.0), -180.0)
        
        print(f"距离感知调整: Yaw增量={angle_change_x:.2f}°, Pitch增量={angle_change_y:.2f}°")
        
        # 设置舵机位置
        self.servo_controller.set_position_by_angle(yaw_angle, pitch_angle)


    def run(self):
        '''
            Return; 数组 [圆心坐标xy, 画面中心坐标, 圆心和画面中心xy误差, 
                         第三个圆圈的点坐标， 此次是否更新了圆心坐标]
                    误差是识别到的圆心坐标减去屏幕中心坐标的值。
        '''
        print("开始检测循环...")
        self.updated = False
        self.debug_time("start")
        try:
            img = self.cam.read()
            print("摄像头读取成功")
        except Exception as e:
            print(f"摄像头读取失败: {e}")
            return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
            
        self.debug_time("cam read")


        # AI 检测外框
        try:
            if self.hires_mode:
                img_ai = img.resize(self.detector.input_width(), self.detector.input_height())
            else:
                img_ai = img # new copy
            print("图像调整大小成功")
        except Exception as e:
            print(f"图像调整大小失败: {e}")
            return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
            
        self.debug_time("resize")
        
        try:
            objs = self.detector.detect(img_ai, conf_th = 0.5, iou_th = 0.45)
            print(f"检测到 {len(objs)} 个目标")
        except Exception as e:
            print(f"目标检测失败: {e}")
            return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
            
        max_idx = -1
        max_s = 0
        for i, obj in enumerate(objs):
            s = obj.w * obj.h
            if s > max_s:
                max_s = s
                max_idx = i
        
        self.debug_time("detect")
        
        if max_idx >= 0:
            print(f"找到最大目标，索引 {max_idx}")
            obj = objs[max_idx]
            w = obj.w + self.crop_padding * 2
            h = obj.h + self.crop_padding * 2
            w = w + 1 if w % 2 != 0 else w
            h = h + 1 if h % 2 != 0 else h
            x = obj.x - self.crop_padding
            y = obj.y - self.crop_padding
            if x < 0:
                w += x
                x = 0
            if y < 0:
                h += y
                y = 0
            if x + w > img_ai.width():
                w = img_ai.width() - x
            if y + h > img_ai.height():
                h = img_ai.height() - y
                
            try:
                crop_ai = img_ai.crop(x, y, w, h)
                crop_ai_rect = [x, y, w, h]
                print(f"裁剪图像成功: x={x}, y={y}, w={w}, h={h}")
            except Exception as e:
                print(f"裁剪图像失败: {e}")
                return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                
            # 算出裁切范围对应在大图的位置
            crop_rect = [int(obj.x * self.img_ai_scale[0]), int(obj.y * self.img_ai_scale[1]), 
                         int(obj.w * self.img_ai_scale[0]), int(h * self.img_ai_scale[0])]
            
            try:
                img_cv = image.image2cv(img, False, False)
                crop_ai_cv = image.image2cv(crop_ai, False, False)
                print("图像转换为OpenCV格式成功")
            except Exception as e:
                print(f"图像转换为OpenCV格式失败: {e}")
                return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                
            self.debug_time("crop")

            try:
                gray = crop_ai.to_format(image.Format.FMT_GRAYSCALE)
                gray_cv = image.image2cv(gray, False, False)
                print("灰度图转换成功")
            except Exception as e:
                print(f"灰度图转换失败: {e}")
                return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                
            self.debug_time("gray")

            try:
                binary = cv2.adaptiveThreshold(gray_cv, 255,
                            cv2.ADAPTIVE_THRESH_MEAN_C,
                            cv2.THRESH_BINARY_INV, 27, 31)
                print("二值化图像成功")
            except Exception as e:
                print(f"二值化图像失败: {e}")
                return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                
            self.debug_time("binary")

            if self.std_from_white_rect:
                try:
                    # 执行洪泛填充找出内白色轮廓
                    h, w = binary.shape[:2]
                    mask = np.zeros((h + 2, w + 2), np.uint8)
                    # 设置种子点（左上角和右下角），如果环境好，可以只点一个角
                    seed_point = (2, 2)
                    seed_point2 = (w - 2, h - 2)
                    # 设置填充值（白色 255）
                    fill_value = 255
                    # 执行洪泛填充（以左上角像素值为基准）
                    cv2.floodFill(binary, mask, seed_point, fill_value, loDiff=5, upDiff=5, flags=4)
                    cv2.floodFill(binary, mask, seed_point2, fill_value, loDiff=5, upDiff=5, flags=4)
                    binary = cv2.bitwise_not(binary)
                    print("洪泛填充成功")
                except Exception as e:
                    print(f"洪泛填充失败: {e}")
                    return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                    
                self.debug_time("fill")

            # 查找轮廓4个角点
            approx = None
            try:
                contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                print(f"找到 {len(contours)} 个轮廓")
            except Exception as e:
                print(f"查找轮廓失败: {e}")
                return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                
            if len(contours) > 0:
                # 筛选出最大的轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                # 近似多边形
                epsilon = 0.02 * cv2.arcLength(largest_contour, True)
                approx = cv2.approxPolyDP(largest_contour, epsilon, True)
                self.debug_time("find countours")
                # 如果找到的是一个四边形
                if len(approx) == 4:
                    # 获取矩形四个角点
                    # 对角点进行排序：左上、右上、右下、左下
                    corners = approx.reshape((4, 2))
                    # 按顺序排列角点（左上、右上、右下、左下）
                    rect = np.zeros((4, 2), dtype="float32")
                    s = corners.sum(axis=1)
                    rect[0] = corners[np.argmin(s)] # 最小和，左上
                    rect[2] = corners[np.argmax(s)] # 最大和，右下
                    diff = np.diff(corners, axis=1) # y - x
                    rect[3] = corners[np.argmax(diff)] # 差最大，左下
                    rect[1] = corners[np.argmin(diff)] # 差最小，右上
                    minW = min(rect[1][0] - rect[0][0], rect[2][0] - rect[3][0])
                    minH = min(rect[3][1] - rect[0][1], rect[2][1] - rect[1][1])
                    if minH > self.rect_min_limit and minW > self.rect_min_limit:
                        self.debug_time("find rect")
                        print(f"找到有效矩形: minW={minW}, minH={minH}")
                        
                                # 在找到有效矩形后添加
                        if minH > self.rect_min_limit and minW > self.rect_min_limit:
                            # 保存矩形信息用于距离估计
                            self.last_rect_info = {
                                'area': minW * minH,
                                'width': minW,
                                'height': minH
            }
                        maxWidth = self.std_res[0]
                        maxHeight = self.std_res[1]

                        # rect 映射到大图, 从大图中得到标准内框图
                        rect[:, 0] += crop_ai_rect[0]
                        rect[:, 1] += crop_ai_rect[1]
                        rect[:, 0] *= self.img_ai_scale[0]
                        rect[:, 1] *= self.img_ai_scale[1]
                        # 透视变换
                        try:
                            dst = np.array([
                                [0, 0],
                                [maxWidth - 1, 0],
                                [maxWidth - 1, maxHeight - 1],
                                [0, maxHeight - 1]], dtype="float32")
                            M = cv2.getPerspectiveTransform(rect, dst)
                            M_inv = np.linalg.inv(M)
                            img_std_cv = cv2.warpPerspective(img_cv, M, (maxWidth, maxHeight))
                            img_std = image.cv2image(img_std_cv, False, False)
                            print("透视变换成功")
                        except Exception as e:
                            print(f"透视变换失败: {e}")
                            return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                            
                        self.debug_time("get std img")

                        # 计算圆圈间距
                        if self.std_from_white_rect:
                            circle_dist = int(img_std.height() * 0.1149425287356322)
                        else:
                            circle_dist = img_std.height() * 0.09523809523809523
                            
                        if circle_dist > 0:
                            center = [img_std.width() // 2, img_std.height() // 2]
                            # 是否找圆和圆心
                            center_new = None
                            if self.find_circle:
                                try:
                                    img_std_gray_cv = cv2.cvtColor(img_std_cv, cv2.COLOR_RGB2GRAY)
                                    w = h = int(circle_dist * 3)
                                    roi = [center[0] - w // 2, center[1] - h // 2, w, h]
                                    img_small_circle_cv = img_std_gray_cv[roi[1]:roi[1] + roi[3], roi[0]:roi[0]+roi[2]]
                                    print("准备霍夫变换找圆")
                                except Exception as e:
                                    print(f"准备霍夫变换失败: {e}")
                                    return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                                
                                if self.DEBUG:
                                    img_small_circle = image.cv2image(img_small_circle_cv, False, False)
                                    img.draw_image(crop_ai.width(), img_std.height(), img_small_circle)

                                # 用霍夫变换找圆
                                try:
                                    circles = cv2.HoughCircles(img_small_circle_cv, cv2.HOUGH_GRADIENT, dp=1.2,
                                                            minDist=roi[2] // 2,
                                                            param1=100, param2=20,
                                                            minRadius=roi[2] // 4, maxRadius=roi[2] // 2)
                                    print("霍夫变换找圆完成")
                                except Exception as e:
                                    print(f"霍夫变换找圆失败: {e}")
                                    return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                                    
                                # 把找圆范围画出来
                                if self.DEBUG:
                                    img_std.draw_rect(roi[0], roi[1], roi[2], roi[3], image.COLOR_ORANGE)
                                    cv2.circle(img_std_cv, center, 1, (0, 255, 0), -1)
                                # 若检测到圆，得到中心和半径
                                circle_dist_new = 0
                                if circles is not None:
                                    circles = np.uint16(np.around(circles))
                                    for c in circles[0, :]:
                                        center_new = (c[0] + roi[0], c[1] + roi[1])  # 圆心坐标偏移回原图
                                        circle_dist_new = c[2]
                                        if self.DEBUG:
                                            cv2.circle(img_std_cv, center_new, circle_dist_new, (0, 255, 0), 1)
                                            cv2.circle(img_std_cv, center_new, 1, (0, 0, 255), 3)  # 圆心
                                        # 这里认为只能检测到一个圆，如果多个，那画面有问题，或者再优化这里的代码
                                        break
                                if center_new:
                                    # 更新圆环中心和圆环间距离
                                    center = center_new
                                    circle_dist = circle_dist_new
                                    # 在标准图中画出新中心和第三个圈
                                    if self.DEBUG:
                                        cv2.circle(img_std_cv, center, 1, (0, 255, 0), -1)
                                        cv2.circle(img_std_cv, center, circle_dist * 3, (0, 255, 0), 1)
                                self.debug_time("find circle")

                            # 如果不找圆心，或者找到了圆心
                            if (not self.find_circle) or (center_new):
                                try:
                                    # 原图画圆中心
                                    std_center_points = np.array([[center]], dtype=np.float32)
                                    original_center_point = cv2.perspectiveTransform(std_center_points, M_inv)[0][0].astype(np.int32).tolist()
                                    self.err_center = [
                                        original_center_point[0] - self.center_pos[0],
                                        original_center_point[1] - self.center_pos[1],
                                    ]
                                    self.last_center = original_center_point
                                    self.last_center_small = [int(self.last_center[0] / self.img_ai_scale[0]), int(self.last_center[1] / self.img_ai_scale[1])]
                                    self.updated = True
                                    print(f"计算误差成功: X={self.err_center[0]:.1f}, Y={self.err_center[1]:.1f}")
                                except Exception as e:
                                    print(f"计算误差失败: {e}")
                                    return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                                    
                                # 原图画圆
                                radius = circle_dist * 3 # 第三个圈的半径
                                # 构造圆上的轮廓点
                                self.debug_time("get points 3")
                                try:
                                    angles = np.linspace(0, 2 * np.pi, self.circle_num_points, endpoint=False)  # endpoint=False 避免首尾重复
                                    cos_vals = np.cos(angles)
                                    sin_vals = np.sin(angles)

                                    # 向量方式生成所有点
                                    x = center[0] + radius * cos_vals
                                    y = center[1] + radius * sin_vals
                                    circle_pts = np.stack((x, y), axis=1).astype(np.float32)  # shape: (N, 2)
                                    circle_pts = circle_pts[np.newaxis, :, :]  # reshape to (1, N, 2)
                                    print("生成圆点成功")
                                except Exception as e:
                                    print(f"生成圆点失败: {e}")
                                    return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                                    
                                self.debug_time("get points 1")

                                # 反变换回原图
                                try:
                                    self.last_circle3_points = cv2.perspectiveTransform(circle_pts, M_inv)
                                    print("反变换圆点成功")
                                except Exception as e:
                                    print(f"反变换圆点失败: {e}")
                                    return [self.last_center, self.center_pos, self.err_center, self.last_circle3_points, False]
                                    
                                self.debug_time("get points")

                                # 找激光点
                                original_lasert_point = None
                                if self.find_laser:
                                    laser_point = self.find_laser_point(img_std, img if self.DEBUG else img_ai)
                                    if laser_point:
                                        # 原图坐标
                                        points = np.array([[[laser_point.x(), laser_point.y()]]], dtype=np.float32)
                                        original_lasert_point = cv2.perspectiveTransform(points, M_inv)[0][0]
                                # 画在大图上
                                if self.DEBUG or self.debug_show_hires:
                                    img.draw_circle(original_center_point[0], original_center_point[1], 4, image.COLOR_RED, thickness=-1)
                                    pts = np.round(self.last_circle3_points[0]).astype(np.int32)
                                    cv2.polylines(img_cv, [pts], isClosed=True, color=(0, 0, 255), thickness=1)
                                    if original_lasert_point is not None:
                                        img.draw_circle(original_lasert_point[0], original_lasert_point[1], 3, image.COLOR_GREEN, thickness=1)
                                else:
                                # 画在小图上显示
                                    # too slow
                                    # center_ai = image.resize_map_pos(img.width(), img.height(), img_ai.width(), img_ai.height(), image.Fit.FIT_FILL, original_center_point[0], original_center_point[1])
                                    if not self.debug_draw_err_line:
                                        img_ai.draw_circle(self.center_pos_small[0], self.center_pos_small[1], 3, image.COLOR_GREEN, thickness=-1)
                                        img_ai.draw_circle(self.last_center_small[0], self.last_center_small[1], 3, image.COLOR_RED, thickness=-1)
                                    pts = self.last_circle3_points[0]  # shape: (N, 2)
                                    scaled_pts = (pts / self.img_ai_scale).astype(np.int32)  # shape: (N, 2)
                                    points = scaled_pts.reshape(-1).tolist()  # 转为 Python list（与原结果相同）
                                    if self.debug_draw_circle:
                                        img_ai.draw_keypoints(points, image.COLOR_RED, 1, line_thickness=1)
                                self.debug_time("draw points")
                            if self.DEBUG:
                                img.draw_image(crop_ai.width(), 0, img_std)
                        else:
                            print("detected circle too small", img_std.width(), img_std.height())
                    else:
                        print(minW, minH, "rect not valid")

            # 绘制路径
            if approx is not None:
                cv2.drawContours(crop_ai_cv, [approx], -1, (255, 255, 255), 1)
            if self.DEBUG:
                img.draw_image(0, 0, crop_ai)
                img2 = image.cv2image(binary, False, False)
                img.draw_image(0, crop_ai.height(), img2)

            if self.debug_draw_rect:
                img.draw_rect(crop_rect[0], crop_rect[1], crop_rect[2], crop_rect[3], color = image.COLOR_RED, thickness=2)
            self.debug_time("draw")
        
        # 显示处理结果
        try:
            if self.DEBUG or self.debug_show_hires:
                if self.debug_draw_err_line:
                    img.draw_line(self.center_pos[0], self.center_pos[1], self.last_center[0], self.last_center[1], image.COLOR_RED, thickness=3)
                if self.debug_draw_err_msg:
                    img.draw_string(2, img.height() - 32, f"err: {self.err_center[0]:5.1f}, {self.err_center[1]:5.1f}, fps: {time.fps():2.0f}", image.COLOR_RED, scale=1.5, thickness=2)
                self.disp.show(img)
            else:
                if self.debug_draw_err_line:
                    img_ai.draw_line(self.center_pos_small[0], self.center_pos_small[1], self.last_center_small[0], self.last_center_small[1], image.COLOR_RED, thickness=3)
                if self.debug_draw_err_msg:
                    img_ai.draw_string(2, img.height() - 32, f"err: {self.err_center[0]:5.1f}, {self.err_center[1]:5.1f}, fps: {time.fps():2.0f}", image.COLOR_RED, scale=1.5, thickness=2)
                self.disp.show(img_ai)
            print("显示图像成功")
        except Exception as e:
            print(f"显示图像失败: {e}")
            
        self.debug_time("display img")
        
        # 如果检测到了目标，控制舵机
        if self.updated:
            try:
                self.control_servo()
                print("舵机控制成功")
            except Exception as e:
                print(f"舵机控制失败: {e}")
        
        return [
            self.last_center,
            self.center_pos,
            self.err_center,
            self.last_circle3_points,
            self.updated
        ]

if __name__ == "__main__":
    print("程序开始运行...")
    disp = display.Display()
    finder = FindRectCircle(disp)
    
    # 初始化舵机位置
    try:
        finder.servo_controller.center_position()
        time.sleep_ms(500)  # 等待舵机到位
    except Exception as e:
        print(f"舵机初始化失败: {e}")
    
    print("系统启动，开始检测A4纸...")
    
    while not app.need_exit():
        try:
            # 检测A4纸和圆心
            circle_center, screen_center, err_center, circle3, updated = finder.run()
            
            # 打印误差信息
            if updated:
                print(f"误差: X={err_center[0]:.1f}, Y={err_center[1]:.1f}, FPS: {time.fps():.1f}")
        except Exception as e:
            print(f"主循环异常: {e}")
            
        # 控制循环频率
        time.sleep_ms(10)  # 约50Hz的控制频率

