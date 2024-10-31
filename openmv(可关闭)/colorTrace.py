# 颜色追踪时需要控制环境光线稳定，避免识别标志物的色彩阈值发生改变
import sensor, image, time,math
from pyb import UART, Pin,Timer
import kinematics,robotMoveCmd

class ColorTrace():
    def __init__(self):
        self.run_app_status = 0  # 将run_app_status初始化为实例变量

    red_threshold=(0, 100, 40, 127, 0, 127)
    blue_threshold=(0, 100, -128, 127, -128, -15)

    trace_color_threshold = blue_threshold#识别的颜色

    sensor.reset() #初始化摄像头
    sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565
    sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
    sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡
    clock = time.clock() #追踪帧率

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(100)
    led_status=0

    move_x=0
    move_y=0
    move_z=100

    servo0=1500
    servo1=1500
    servo2=1500
    servo3=860

    servo_option=1#操作舵机选择

    adjust_position=0#用调整身位

    loop_cnt_flag_x=0#标记色块追踪抓取的记号
    loop_cnt_flag_z=0

    #调整中点，用于调整机械臂抓取物块中点
    #如果机械臂抓取偏右，mid_block_cx减小，反之增加
    #如果机械臂抓取偏前，mid_block_cy减小，反之增加
    #如果机械臂抓取偏下或偏上，就调节机械臂第2，3个舵机偏差
    mid_block_cx=80.5
    mid_block_cy=60.5

    grasp_cnt=0#寻找到物块后计数判断

    #机器人运动
    robot_move_cmd = robotMoveCmd.RobotMoveCmd()
    kinematic = kinematics.Kinematics()

    def init(self,cx=80.5,cy=60.5):#初始化巡线配置，传入两个参数调整中位值
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.uart.init(115200, bits=8, parity=None, stop=1 )
        self.led_dac.pulse_width_percent(50)

        self.cap_color_status=0#抓取物块颜色标志，用来判断物块抓取
        #机械臂移动位置
        self.move_x=0
        self.move_y=160
        self.move_z=100

        self.mid_block_cx=cx
        self.mid_block_cy=cy

        self.mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差

        self.move_status=0#机械臂移动的方式

        self.oled_show_flag=0#oled显示字符

        self.servo0=1500
        self.servo1=1500
        self.servo2=1500
        self.servo3=860

        self.servo_option=1#操作舵机选择

        self.adjust_position=0#用调整身位

        self.loop_cnt_flag_x=0#标记色块追踪抓取的记号
        self.loop_cnt_flag_z=0

        self.grasp_cnt=0#寻找到物块后计数判断

        self.kinematic.kinematics_move(self.move_x,self.move_y,self.move_z,1000)
        time.sleep_ms(1000)

    def run_trace_grasp(self):#蓝球追踪抓取
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy

        # 获取图像
        img = sensor.snapshot()
        circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10, r_margin=10, r_min=2, r_max=100, r_step=2)

        if circles:
            # 取第一个识别到的圆
            circle = circles[0]
            # 增大识别范围
            roi_x = max(0, circle.x() - 20)
            roi_y = max(0, circle.y() - 20)
            roi_w = min(img.width() - roi_x, circle.r() * 2 + 40)
            roi_h = min(img.height() - roi_y, circle.r() * 2 + 40)

            # 在新范围内进行蓝色识别
            blobs = img.find_blobs([self.blue_threshold], roi=(roi_x, roi_y, roi_w, roi_h), x_stride=5, y_stride=5, pixels_threshold=10)
        else :
            blobs=0

        if blobs:
            max_size = 0
            max_blob=blobs[0]
            for blob in blobs:#寻找最大
                if blob[2] * blob[3] > max_size:
                    max_blob = blob
                    max_size = blob[2] * blob[3]
            img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
            img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
            img.draw_string(max_blob[0], (max_blob[1]-10), "blue", color=(255,0,0))
            block_cx=max_blob[5]
            block_cy=max_blob[6]

            if self.move_status==0:#追踪颜色
                #************************运动机械臂**********************************
                if(abs(block_cx-self.mid_block_cx)>5):
                    if block_cx > self.mid_block_cx:
                        self.move_x+=1
                    else:
                        self.move_x-=1
                if(abs(block_cy-(self.mid_block_cy+20))>5):
                    if block_cy > (self.mid_block_cy+20) and self.move_z>1:
                        self.move_z-=1
                    else:
                        self.move_z+=1
                self.kinematic.kinematics_move(self.move_x,self.move_y,self.move_z,0)
                time.sleep_ms(10)

                if abs(self.move_z-100)>10 or abs(self.move_x-0)>10: #寻找到物块
                    self.mid_block_cnt += 1
                    self.grasp_cnt=0
                    if self.mid_block_cnt>5:#计数100次对准物块，防止误差
                        #************************运动机器人*********************************
                        if self.move_x > 10:#右转
                            self.mid_block_cnt=0
                            if self.adjust_position!=3:
                                self.adjust_position=3
                                self.robot_move_cmd.car_move(0,0,-300)
                        elif self.move_x < -10:#左转
                            self.mid_block_cnt=0
                            if self.adjust_position!=2:
                                self.adjust_position=2
                                self.robot_move_cmd.car_move(0,0,300)
                        elif self.move_z-100>10:#前进
                            self.mid_block_cnt=0
                            if self.adjust_position!=1:
                                self.adjust_position=1
                                self.robot_move_cmd.car_move(100,0,0)
                        elif self.move_z-100<-10:#后退
                            self.mid_block_cnt=0
                            if self.adjust_position!=4:
                                self.adjust_position=4
                                self.robot_move_cmd.car_move(-100,0,0)
                else:
                    self.mid_block_cnt=0
                    self.grasp_cnt += 1
                    if self.grasp_cnt>50:#计数到100说明物块已到中心位置
                        self.grasp_cnt=0
                        self.move_status=1
                    if self.adjust_position!=5:#调整完毕，停止
                        self.adjust_position=5
                        self.robot_move_cmd.car_move(0,0,0)
            elif self.move_status==1:#机械臂寻找物块
                self.move_status=0
                self.grasp_cnt=0
                l=math.sqrt(self.move_x*self.move_x+self.move_y*self.move_y)
                sin=self.move_y/l
                cos=self.move_x/l
                self.move_x=(l+111)*cos
                self.move_y=(l+111)*sin
                self.kinematic.send_str("#005P1100T100!")#机械爪张开
                time.sleep_ms(100)
                self.kinematic.kinematics_move(self.move_x,self.move_y,50,1000)#移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x,self.move_y,-25,1000)
                time.sleep_ms(1200)
                self.kinematic.send_str("#005P1400T500!")#机械爪闭合
                time.sleep_ms(500)
                self.kinematic.send_str("$DGT:3-4,1!")#放球在车上
                time.sleep_ms(1800)
                self.move_x=0#机械臂归位
                self.move_y=160
                self.move_z=100
                self.kinematic.kinematics_move(self.move_x,self.move_y,self.move_z,1000)
                time.sleep_ms(1200)
                self.uart.write("%\n")#发送手柄开启标志
                self.run_app_status=0
        else:
            if self.adjust_position!=5:
                self.adjust_position=5
                self.robot_move_cmd.car_move(0,0,0)



    def run_trace_grasp_box(self):#红盒追踪抓取
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy

        # 获取图像
        img = sensor.snapshot()
        blobs = img.find_blobs([self.red_threshold],x_stride=5, y_stride=5, pixels_threshold=10 )

        if blobs:
            max_size = 0
            max_blob=blobs[0]
            for blob in blobs:#寻找最大
                if blob[2] * blob[3] > max_size:
                    max_blob = blob
                    max_size = blob[2] * blob[3]
            img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
            img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
            img.draw_string(max_blob[0], (max_blob[1]-10), "red", color=(255,0,0))
            #print("中心X坐标",max_blob[5],"中心Y坐标",max_blob[6],"识别颜色类型","红色")
            block_cx=max_blob[5]
            block_cy=max_blob[6]


            if self.move_status==0:#追踪颜色
                #************************运动机械臂**********************************
                if(abs(block_cx-self.mid_block_cx)>5):
                    if block_cx > self.mid_block_cx:
                        self.move_x+=1
                    else:
                        self.move_x-=1
                if(abs(block_cy-(self.mid_block_cy+20))>5):
                    if block_cy > (self.mid_block_cy+20) and self.move_z>1:
                        self.move_z-=1
                    else:
                        self.move_z+=1
                self.kinematic.kinematics_move(self.move_x,self.move_y,self.move_z,0)
                time.sleep_ms(10)

                if abs(self.move_z-100)>10 or abs(self.move_x-0)>10: #寻找到物块
                    self.mid_block_cnt += 1
                    self.grasp_cnt=0
                    if self.mid_block_cnt>5:#计数100次对准物块，防止误差
                        #************************运动机器人*********************************
                        if self.move_x > 10:#右转
                            self.mid_block_cnt=0
                            if self.adjust_position!=3:
                                self.adjust_position=3
                                self.robot_move_cmd.car_move(0,0,-300)
                        elif self.move_x < -10:#左转
                            self.mid_block_cnt=0
                            if self.adjust_position!=2:
                                self.adjust_position=2
                                self.robot_move_cmd.car_move(0,0,300)
                        elif self.move_z-100>10:#前进
                            self.mid_block_cnt=0
                            if self.adjust_position!=1:
                                self.adjust_position=1
                                self.robot_move_cmd.car_move(100,0,0)
                        elif self.move_z-100<-10:#后退
                            self.mid_block_cnt=0
                            if self.adjust_position!=4:
                                self.adjust_position=4
                                self.robot_move_cmd.car_move(-100,0,0)
                else:
                    self.mid_block_cnt=0
                    self.grasp_cnt += 1
                    if self.grasp_cnt>50:#计数到50说明物块已到中心位置
                        self.grasp_cnt=0
                        self.move_status=1
                    if self.adjust_position!=5:#调整完毕，停止
                        self.adjust_position=5
                        self.robot_move_cmd.car_move(0,0,0)
            elif self.move_status==1:#机械臂寻找物块
                self.move_status=0
                self.grasp_cnt=0
                l=math.sqrt(self.move_x*self.move_x+self.move_y*self.move_y)
                sin=self.move_y/l
                cos=self.move_x/l
                self.move_x=(l+111)*cos
                self.move_y=(l+111)*sin
                self.kinematic.send_str("#005P900T100!")#机械爪张开
                time.sleep_ms(100)
                self.kinematic.kinematics_move(self.move_x,self.move_y,50,1000)#移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x,self.move_y,-25,1000)
                time.sleep_ms(1200)
                self.kinematic.send_str("#005P1400T100!")#机械爪闭合
                time.sleep_ms(1200)
                self.kinematic.send_str("$DGT:20-21,1!")#放球在车上
                time.sleep_ms(1800)
                self.move_x=0#机械臂归位
                self.move_y=160
                self.move_z=100
                self.kinematic.kinematics_move(self.move_x,self.move_y,self.move_z,1000)
                time.sleep_ms(1200)
                self.uart.write("%\n")#发送手柄开启标志
                self.run_app_status = 0
        else:
            if self.adjust_position!=5:
                self.adjust_position=5
                self.robot_move_cmd.car_move(0,0,0)
