import vrep
import numpy as np
import cv2
import time
import math
import threading
from PIL import Image

vrep.simxFinish(-1)
clientID = -1
while(clientID == -1):
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP

color_range = {
                'black': [(0, 0, 0), (62, 255, 250)],
                'white': [(217, 0, 0), (255, 255, 250)],
              }

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0, sample_time=0.00):
        self.Kp = P                             # 比例增益Kp
        self.Ki = I                             # 积分增益Ki
        self.Kd = D                             # 微分增益Kd
        self.sample_time = sample_time          # 采样间隔
        self.current_time = time.time()         # 当前时间
        self.previous_time = self.current_time  # 历史采样时间点
        self.clear()

    def clear(self):
        self.P_term = 0.0            
        self.I_term = 0.0            # 积分项
        self.D_term = 0.0            # 微分项
        self.previous_error = 0.0   # 历史偏差量
        self.SetPoint = 0.0         # 设定目标值
        self.windup_guard = 20.0
        self.output = 0.0           # 输出

    def update(self, feedback):
        '''
        利用PID算法根据设定的目标值和输入的反馈值更新输出调整小车的速度
        :param feedback: 闭环系统的反馈值
        '''
        error = self.SetPoint - feedback                        # 设定目标值与反馈值之间的偏差

        self.current_time = time.time()                         # 当前采样时间

        delta_time = self.current_time - self.previous_time     # 时间变化量
        delta_error = error - self.previous_error               # 偏差变化量

        if (delta_time >= self.sample_time):
            self.P_term = self.Kp * error                        
            self.I_term += error * delta_time                    # 更新积分项
            self.D_term = 0.0
            if delta_time > 0:
                self.D_term = delta_error / delta_time           # 更新微分项

            if (self.I_term < -self.windup_guard):
                self.I_term = -self.windup_guard
            elif (self.I_term > self.windup_guard):
                self.I_term = self.windup_guard

            self.previous_time = self.current_time              # 更新历史采样时间点
            self.previous_error = error                         # 更新历史偏差
            self.output = self.P_term + (self.Ki * self.I_term) + (self.Kd * self.D_term)
            

#检测并返回最大面积
def getAreaMaxContour(contours, area=1):
        max_area = 0
        ans = None

        for c in contours :
            temp = math.fabs(cv2.contourArea(c))
            if temp > max_area :
                max_area = temp
                if temp > area:
                    ans = c
        return ans

get_line = False
center_x = 0
speed = 0
left_speed, right_speed = 0, 0

x_pid = PID(P=0.5, I=0.3, D=0.5)

def Tracing(img0):
    global get_line, center_x, speed
    global right_speed, left_speed
    # cv2.imshow('img0', img0)
    img1 = cv2.GaussianBlur(img0, (3, 3), 0)  # 高斯模糊，去噪
    # cv2.imshow('img1', img1)
    img2 = cv2.cvtColor(img1, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    img3 = cv2.inRange(img2, color_range['black'][0],
                                color_range['black'][1])  # 根据hsv值对图片进行二值化
    # cv2.imshow('img3', img3)
    img4 = cv2.morphologyEx(img3, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
    img5 = cv2.morphologyEx(img4, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
    # cv2.imshow('img5', img5)

    center_x = 0
    final = img5
    cnts = cv2.findContours(final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓
    cnt_largest = getAreaMaxContour(cnts)  # 找到最大面积的轮廓
    if cnt_largest is not None:
        rect = cv2.minAreaRect(cnt_largest)  # 最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
        p1_x, p1_y = box[0, 0], box[0, 1]
        p3_x, p3_y = box[2, 0], box[2, 1]
        # cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)  # 画出四个点组成的矩形
        center_x, center_y = (p1_x + p3_x) / 2, (p1_y + p3_y) / 2  # 中心点
        # cv2.circle(img, (int(center_x), int(center_y)), 10, (0, 0, 255), -1)  # 画出中心点
        x_pid.SetPoint = img0.shape[-2]/2  # 图像中心

        x_pid.update(center_x)  # 将当前获取的跑道中心值作为输入值

        # pid输出
        out = int(x_pid.output)

        # 确定速度，直道快，弯道慢
        if abs(x_pid.SetPoint - center_x) < 5:
            speed = 7.5 # 直道
        else:
            speed = 4 # 弯道

        # 限制输出大小
        if speed - out < -10:
            out = 10 + speed
        elif speed - out > 10:
            out = -10 + speed
        if speed + out < -10:
            out = -10 - speed
        elif speed + out > 10:
            out = 10 - speed

        # 根据速度调整拐弯幅度
        if speed >= 7:
            out = out / speed
        else:
            out = out / 3
        
        get_line = True
        # 计算左右速度
        left_speed = speed - out
        right_speed = speed + out


def move():
    global get_line, left_speed, right_speed
    while True:
        if get_line:
            get_line = False
            vrep.simxSetJointTargetVelocity(clientID, left_motor, left_speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, right_motor, right_speed, vrep.simx_opmode_streaming)
        else:
            time.sleep(0.001)

th = threading.Thread(target=move)
th.setDaemon(True)
th.start()
if clientID != -1:
    print('Connected to remote API server')
    # Get the handle of Pioneer_p3dx, Pioneer_p3dx_leftMotor, Pioneer_p3dx_rightMotor, Vision_sensor
    _, body = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
    _, left_motor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    _, right_motor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
    _, camera = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)

    _, position = vrep.simxGetObjectPosition(clientID, body, -1, vrep.simx_opmode_streaming)
    _, orientation = vrep.simxGetObjectOrientation(clientID, body, -1, vrep.simx_opmode_streaming)

    
    # 获取传感器的图像
    res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)
    while (vrep.simxGetConnectionId(clientID) != -1):
        res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
        if res == 0:
            # 将图像变成标准格式
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[1], resolution[0], 3])
            # 将图像缩小，提高之后对图像的处理速度
            img=cv2.resize(img,(128,128))

            # 调整图像方向
            # img=cv2.flip(img,1)
            # img=cv2.flip(img,0)
            # img=cv2.flip(img,1)

            # 截取有效视野，排除干扰
            img = img[ (img.shape[0]//10) : (img.shape[0]//4) , img.shape[1]//4 : (img.shape[1]//4)*3]
            Tracing(img)
            # 显示图像
            # cv2.imshow('img', img)
            cv2.waitKey(1)
    cv2.destroyAllWindows()