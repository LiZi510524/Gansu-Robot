from pyb import UART, Pin,Timer
import sensor,time, pyb
import colorTrace


color_trace = colorTrace.ColorTrace()
# run_app_status=colorTrace.run_app_status

led = pyb.LED(3)

flag=1

uart = UART(3,115200)   #设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1 )

tim = Timer(4, freq=1000) # Frequency in Hz
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(0)

uart.write("#openmv reset!")

#调整中点，用于调整机械臂抓取物块中点
#如果机械臂抓取偏左，mid_block_cx减小，反之增加
#如果机械臂抓取偏前，mid_block_cy减小，反之增加
#如果机械臂抓取偏下或偏上，就调节2号舵机偏差
mid_block_cx=80.5
mid_block_cy=60.5

def beep():
    uart.write("$BEEP!\n")#发送蜂鸣器鸣叫指令
    led.on()            #亮灯
    time.sleep_ms(100)  #延时150ms
    led.off()           #暗灯
    time.sleep_ms(100)

beep()
color_trace.init(mid_block_cx,mid_block_cy)

while(True):
    if uart.any():#接收指令
        try:#用来判断串口数据异常
            string = uart.read()
            if string:
                string = string.decode()
                #print(string, colorTrace.run_app_status,string.find("#start_led!"))
                if string.find("#StartLed!") >= 0 :#开灯指令
                    led_dac.pulse_width_percent(100)
                    beep()
                elif string.find("#StopLed!") >= 0 :#关灯指令
                    led_dac.pulse_width_percent(0)
                    beep()
                elif string.find("#RunStop!") >= 0 :#停止所有运行并复位
                    color_trace.run_app_status=0
                    uart.write("$CAR:0,0,0!\n")#发送控制小车行动的指令
                    uart.write("$KMS:0,160,50,1000!\n")
                    led_dac.pulse_width_percent(0)
                    beep()
                elif string.find("ball") >= 0 :    #需要换上黑金大爪子,蓝球追踪抓取
                    uart.write("*\n")#发送手柄关闭标志
                    color_trace.run_app_status=11
                    color_trace.init(mid_block_cx,mid_block_cy)
                    beep()
                elif string.find("box") >= 0 :#需要换上黑金大爪子,红盒追踪抓取
                    uart.write("*\n")#发送手柄关闭标志
                    color_trace.run_app_status=15
                    color_trace.init(mid_block_cx,mid_block_cy)
                    beep()

        except Exception as e:#串口数据异常进入
            print('Error:', e)

    led_dac.pulse_width_percent(100)
    #run_app_status=15

#    if flag:
#        color_trace.run_app_status=15
#        flag=0

    if  color_trace.run_app_status==11:
        color_trace.run_trace_grasp()    #运行蓝球追踪抓取功能，需要换上黑金大爪子
        #uart.write("%\n")#发送手柄开启标志
    elif color_trace.run_app_status==15:
        color_trace.run_trace_grasp_box()#运行红盒追踪抓取功能，需要换上黑金大爪子
        #uart.write("%\n")#发送手柄开启标志



