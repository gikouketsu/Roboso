import cv2
import numpy as np
import threading

# 定义一个线程类，用于从摄像头实时捕获视频帧
class CameraCapture(threading.Thread):
    def __init__(self, camera_index=0):
        # 初始化线程，并指定摄像头的索引（默认为0，即系统默认摄像头）
        threading.Thread.__init__(self)
        self.cap = cv2.VideoCapture(camera_index)  # 打开摄像头
        self.ret = False  # 用于保存读取帧的状态（是否成功）
        self.frame = None  # 保存当前捕获的帧
        self.running = True  # 控制线程运行状态的标志

    def run(self):
        # 重载Thread类的run方法，开始循环捕获视频帧
        while self.running:
            self.ret, self.frame = self.cap.read()  # 从摄像头读取帧
            if not self.ret:  # 如果读取失败，输出错误信息并停止线程
                print("Cannot read frame, stopping...")
                self.running = False

    def stop(self):
        # 停止捕获并释放摄像头资源
        self.running = False
        self.cap.release()

# 主处理循环，用于对捕获的帧进行处理
def process_frames(camera_capture):
    while True:
        # 从捕获线程获取当前帧
        frame = camera_capture.frame
        if frame is None:  # 如果帧为空，跳过本次循环
            continue

        # 将帧从BGR颜色空间转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义黑色的HSV颜色范围（根据需求调整范围）
        lower_black = np.array([0, 0, 46])  # 黑色的最小HSV值
        upper_black = np.array([180, 43, 220])  # 黑色的最大HSV值

        # 创建一个掩膜，仅保留在黑色范围内的像素
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # 查找掩膜中的轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 遍历找到的所有轮廓
        for contour in contours:
            # 过滤掉小轮廓，避免噪声（仅处理面积大于500的轮廓）
            if cv2.contourArea(contour) > 500:
                # 获取最小包围圆的圆心和半径
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))  # 将圆心坐标转换为整数

                # 获取帧的宽度和高度
                height, width = frame.shape[:2]
                center_x = width // 2  # 计算帧的中心X坐标
                center_y = height // 2  # 计算帧的中心Y坐标

                # 计算目标相对于帧中心的水平和垂直偏移量
                distance_x = int(x) - center_x
                distance_y = int(y) - center_y

                # 输出偏移量
                print(f"x = {distance_x}, y = {distance_y}")

        # 每次循环等待按键输入，若按下'q'键，则退出循环
        if cv2.waitKey(1) == ord('q'):
            break

    # 停止摄像头捕获线程
    camera_capture.stop()

# 创建并启动摄像头捕获线程
camera_capture = CameraCapture()
camera_capture.start()

# 处理捕获的帧
process_frames(camera_capture)

# 等待摄像头线程结束
camera_capture.join()

