import sys
import os
# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import time
from Robotic_Arm.rm_robot_interface import *
import socket
import numpy as np
import matplotlib.pyplot as plt
# 定义机械臂连接信
IP_ADDRESS = "192.168.1.18"
PORT = 8080
CONNECTION_LEVEL = 3
THREAD_MODE = rm_thread_mode_e(2)


class MIArmController:
    def __init__(self, ip, port, level=3, mode=2):
        self.thread_mode = rm_thread_mode_e(mode)
        self.robot = RoboticArm(self.thread_mode)
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)
        
        if self.handle.id == -1:
            print("\nFailed to connect to the robot arm\n")
            exit(1)
        else:
            print(f"\nSuccessfully connected to the robot arm: {self.handle.id}\n")
            self.robot.rm_movej([0,0,0,0,0,0,0], v=20, r=0, connect=0, block=1)
        # isaacgym保存的npy文件路径
        file_path = '/home/shifei/code/isaacgyms/arm_dof_pos_cumulative.npy'
        self.joint_data = np.load(file_path)
        self.num_eposide = self.joint_data.shape[0]
        self.arm_joint_dofs = self.joint_data[:,0:7]
        self.arm_dof = self.arm_joint_dofs.shape[1] 
        self.Claw_joit_dofs = self.joint_data[:,8:9]
        #####夹爪类初始化########
        # 创建一个 TCP 服务器套接字
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 绑定到本地地址和端口
        self.client.connect((ip,port))
        print("机械臂连接",ip) 
        self.set_claw_init()
        self.set_claw_voltage()
        self.set_modbus_mode()  
        self.set_Claw_N(50)
        self.threshould_low=0.040
        self.threshould_high=0.047
        self.claw_status = 0
        self.robot.rm_set_modbus_mode(0,115200,2)
        self.pre_claw_condition=0#0为张开，1为闭合，初始化夹爪为张开
        self.now_claw_condition=0
        self.now_degree = []
    def send_cmd(self, cmd_6axis):
        self.client.send(cmd_6axis.encode('utf-8'))
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
        self.pre_claw_condition = 0 
        self.now_claw_condition = 0
        print("执行初始化成功")
    def set_Claw_N(self,N):
        point6_00 = '{"command":"write_single_register","port":1,"address":257,"data"'+str(N)+'"device":1}\r\n'
        _ = self.send_cmd(point6_00)
        print(f"设置{N}% 力值 （写操作）") 
    def set_Claw_position(self,position):
        """
        Args:
            调整夹爪的位置到设定值
            范围100~1000
        """
        point6_00 = '{"command":"write_single_register","port":1,"address":259,"data":'+str(position)+', "device":1}\r\n'
        _ = self.send_cmd(point6_00)
    def claw_threshold_judgment(self,value):
        """
        Args:
            判断输入值与阈值的关系
            param value: 输入值
            param threshold_low && threshold_high: 高低阈值判断，夹爪开启关闭
        """
        if value > self.threshould_high:
            # print( f"{value} claw_status :open")
            self.set_Claw_position(1000)
            self.now_claw_condition = 1
            if(self.now_claw_condition!=self.pre_claw_condition):
                self.check_claw_condition()
        elif value <self.threshould_low:
            # print( f"{value} claw_status :close")
            self.set_Claw_position(200)
            self.now_claw_condition = 0
            if(self.now_claw_condition!=self.pre_claw_condition):
                self.check_claw_condition()
        self.pre_claw_condition = self.now_claw_condition
    
    def check_claw_condition(self):
        """
        判断夹爪是否到位，不到位一直循环等待
        """
        param = rm_peripheral_read_write_params_t(1,0x0201,1)
        time.sleep(0.01)
        while True:
            self.claw_status = self.robot.rm_read_holding_registers(param)[1]
            print(self.claw_status)
            if self.claw_status!=0:
                break 
            else:
                print("the claw is reaching")
    def save_joint_state(self):
        
        joint_state = self.robot.rm_get_joint_degree()[1]
        
        self.now_degree.append(joint_state)
    def draw_joint_state(self):
        """
        Args:
            flag为True,开启绘图
        """
        #numpy类型
    
        output_folder = 'joint_state_images'
        os.makedirs(output_folder, exist_ok=True)
        self.arm_joint_dofs = self.arm_joint_dofs*180/3.14
        for dof_idx in range (self.arm_dof):
            plt.figure()
            self.now_degree = np.array(self.now_degree)
            plt.plot(self.now_degree[:,dof_idx], label="Real",linestyle="--", marker=".")
            plt.plot(self.arm_joint_dofs[:,dof_idx], label="target",linestyle="-.", marker=".")
            plt.legend()
            plt.title(f'Joint {dof_idx} Angle')
            plt.xlabel('Step')
            plt.ylabel('Angle')
            # plt.show()
            image_path = os.path.join(output_folder, f'joint_{dof_idx}_angle.png')
            plt.savefig(image_path)
                
def todu(x):
    return x *180/3.14

def main():
    try:
        # 创建机械臂对象
        #这里采用两个类单独控制机械臂和夹爪，MI_robot为机械臂 MI_claw为夹爪
        MI_robot = MIArmController("192.168.1.18", 8080, 3)
        MI_robot.set_Claw_position(1000)
        MI_robot.robot.rm_set_modbus_mode(0,115200,2)
        for dof_idx in range(MI_robot.num_eposide):
            # 提取当前关节的数量
            joint_angle = MI_robot.arm_joint_dofs[dof_idx,:].tolist() 
            #弧度转换为°
            joint_angle = list(map(todu, joint_angle))
            #夹爪状态获取并转换
            claw_status = MI_robot.Claw_joit_dofs[dof_idx,0].tolist()#两侧夹爪对称
            MI_robot.claw_threshold_judgment(claw_status)
            if(dof_idx==1):
                movej_result = MI_robot.robot.rm_movej(joint_angle, v=10, r=1, connect=0, block=1)
            movej_result = MI_robot.robot.rm_movej_canfd(joint_angle, follow=False)  
            # movej_result = MI_robot.robot.rm_movej_canfd(joint_angle, follow=True,trajectory_mode=2,radio=100)  

            time.sleep(0.02)
            # 执行关节运动
            if movej_result == 0:
                print(f"Joint movement succeeded.{dof_idx}")
                MI_robot.save_joint_state()
            else:
                print(f"Joint movement failed, Error code: {movej_result}")
            # 透传模式下需要等待一段时间
            if dof_idx==MI_robot.num_eposide-1:
                MI_robot.draw_joint_state()


        #重置为零位状态
        # joint_angle = [0,0,0,0,0,0,0]
        # MI_robot.robot.rm_movej(joint_angle, v=20, r=0, connect=0, block=1)



        # 断开机械臂连接
        disconnect_result = MI_robot.robot.rm_delete_robot_arm()
        if disconnect_result == 0:
            print("Successfully disconnected from the robot arm.")
        else:
            print("Failed to disconnect from the robot arm.")

    except Exception as e:
        print(f"An error occurred: {e}")
