# 导入必要的模块
import spidev  # 用于SPI通信
import time  # 控制时间延迟
import cv2  # 用于图像处理
import numpy as np  # 用于处理数组数据
import threading  # 实现多线程

# 初始化 SPI
spi = spidev.SpiDev()  # 创建SPI对象
spi.open(0, 0)  # 打开设备，(bus, device) 为 (0, 0)
spi.max_speed_hz = 115200  # 设置SPI通信速度

# 定义用于摄像头捕获的线程类
class CameraCapture(threading.Thread):
    def __init__(self, camera_index=0):
        # 调用父类的初始化方法，创建线程
        threading.Thread.__init__(self)
        # 打开指定索引的摄像头
        self.cap = cv2.VideoCapture(camera_index)
        # 初始化图像捕获的状态
        self.ret = False  # 是否成功读取图像
        self.frame = None  # 存储读取的图像帧
        self.running = True  # 控制线程运行的标志
        self.lock = threading.Lock()  # 定义线程锁，避免竞争条件

    def run(self):
        """线程运行的主循环，用于连续捕获摄像头帧。"""
        while self.running:
            # 从摄像头读取一帧图像
            ret, frame = self.cap.read()
            if ret:
                # 加锁以保证多线程访问的安全性
                with self.lock:
                    self.ret, self.frame = ret, frame

    def stop(self):
        """停止摄像头捕获并释放资源。"""
        self.running = False  # 停止标志置为 False
        self.cap.release()  # 释放摄像头资源

    def get_frame(self):
        """获取最新的图像帧和状态。"""
        with self.lock:  # 加锁确保线程安全
            return self.ret, self.frame  # 返回图像捕获结果和帧

# 主循环函数，处理视频帧
def process_frames(camera_capture):
    center_x, center_y = None, None  # 图像中心的坐标
    # 黄色物体的HSV颜色范围
    lower_yellow = np.array([20, 100, 100])  # 黄色的最低HSV值
    upper_yellow = np.array([30, 255, 255])  # 黄色的最高HSV值

    try:
        while True:
            # 从摄像头线程获取最新帧
            ret, frame = camera_capture.get_frame()
            if not ret or frame is None:  # 如果读取失败，跳过本次循环
                continue

            # 初始化中心坐标（仅执行一次）
            if center_x is None or center_y is None:
                height, width = frame.shape[:2]  # 获取图像尺寸
                center_x = width // 2  # 图像中心的x坐标
                center_y = height // 2  # 图像中心的y坐标

            # 将图像转换为HSV颜色空间，并创建黄色物体的掩膜
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # 寻找掩膜中的轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 遍历所有找到的轮廓
            for contour in contours:
                # 只处理面积大于500的轮廓，过滤掉噪声
                if cv2.contourArea(contour) > 500:
                    # 获取包围轮廓的最小外接圆的中心坐标
                    (x, y), _ = cv2.minEnclosingCircle(contour)

                    # 计算物体相对于图像中心的距离
                    distance_x = int(x) - center_x  # x方向的距离
                    distance_y = int(y) - center_y  # y方向的距离

                    # 将距离限制在 -125 到 125 之间，防止溢出
                    distance_x = max(-125, min(125, distance_x))
                    distance_y = max(-125, min(125, distance_y))

                    # 打印距离信息
                    print(distance_x, distance_y)
                    time.sleep(0.05)  # 延迟50毫秒

                    # 准备并通过SPI发送数据
                    send_data = [distance_x & 0xFF, distance_y & 0xFF]  # 将数据限制为8位
                    spi.xfer2(send_data)  # 通过SPI发送数据

            # 按下 'q' 键退出程序
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        # 捕获中断信号以安全退出
        pass
    finally:
        # 停止摄像头捕获线程并关闭SPI
        camera_capture.stop()
        spi.close()

# 创建并启动摄像头捕获线程
camera_capture = CameraCapture()  # 实例化摄像头捕获对象
camera_capture.start()  # 启动线程

# 开始处理视频帧
process_frames(camera_capture)

# 等待摄像头线程结束
camera_capture.join()  # 阻塞主线程，直到摄像头线程结束

