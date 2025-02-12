from Robotic_Arm.rm_robot_interface import *
import random
import socket
import time
class claw_init:
    def __init__(self,ip,port):
        # 创建一个 TCP 服务器套接字
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 绑定到本地地址和端口
        self.client.connect((ip,port))
        print("机械臂连接",ip) 
        # self.set_claw_init()
        self.set_claw_voltage()
        self.set_modbus_mode()  
        self.set_Claw_N(50)
        self.threshould_low=0.040
        self.threshould_high=0.047
        self.claw_status = 0
    def send_cmd(self, cmd_6axis):
        self.client.send(cmd_6axis.encode('utf-8'))
        # Optional: Receive a response from the server
        # _ = client.recv(1024).decode(s)
        return True 
    def set_claw_voltage(self):
        point6_00 = '{"command":"set_tool_voltage","voltage_type":3}\r\n'
        _ = self.send_cmd(point6_00)
        print("设置夹爪端电源输出 24V")
    def set_modbus_mode(self):
        point6_00 = '{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout ":2}\r\n'
        _ = self.send_cmd(point6_00)
        print("配置通讯端口 ModbusRTU 模式")
    def set_claw_init(self):
        point6_00 = '{"command":"write_single_register","port":1,"address":256,"data":1, "device":1}\r\n'
        _ = self.send_cmd(point6_00) 
        print("执行初始化成功")
    def set_Claw_N(self,N):
        point6_00 = '{"command":"write_single_register","port":1,"address":257,"data"'+str(N)+'"device":1}\r\n'
        _ = self.send_cmd(point6_00)
        print(f"设置{N}% 力值 （写操作）") 
    def set_Claw_position(self,position):
        #调整夹爪的位置到设定值
        #范围100~1000
        point6_00 = '{"command":"write_single_register","port":1,"address":259,"data":'+str(position)+', "device":1}\r\n'
        _ = self.send_cmd(point6_00)
    def claw_threshold_judgment(self,value):
        """
        判断输入值与阈值的关系
        :param value: 输入值
        :param threshold_low && threshold_high: 高低阈值判断，夹爪开启关闭
        """
        if value > self.threshould_high:
            # print( f"{value} claw_status :open")
            self.set_Claw_position(1000)
        elif value <self.threshould_low:
            # print( f"{value} claw_status :close")
            self.set_Claw_position(500)
        self.check_claw_condition()


# 实例化RoboticArm类
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
claw = claw_init("192.168.1.18", 8080)

claw.set_Claw_position(random.randint(0, 1000))
# 创建机械臂连接，打印连接id
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
print(handle.id)

# 配置控制器RS485端口为RTU主站
print(arm.rm_set_modbus_mode(0,115200,2))



for i in range(200):
    #    通过控制器RS485端口读取数据，起始地址为10， 外设设备地址为2
    param = rm_peripheral_read_write_params_t(1,0x0201,1)
    
    print("读保持寄存器: ", arm.rm_read_holding_registers(param))

arm.rm_delete_robot_arm()