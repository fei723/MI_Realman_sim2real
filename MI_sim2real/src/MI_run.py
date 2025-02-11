import sys
import os
# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import time
from Robotic_Arm.rm_robot_interface import *
import socket
import numpy as np

# 定义机械臂连接信
IP_ADDRESS = "192.168.1.18"
PORT = 8080
CONNECTION_LEVEL = 3
THREAD_MODE = rm_thread_mode_e(2)



class MIArmController:
    def __init__(self, ip, port, level=3, mode=2):
        """
        Initialize and connect to the robotic arm.

        Args:
            ip (str): IP address of the robot arm.
            port (int): Port number.
            level (int, optional): Connection level. Defaults to 3.
            mode (int, optional): Thread mode (0: single, 1: dual, 2: triple). Defaults to 2.
        """
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
        joint_data = np.load(file_path)
        self.num_eposide = joint_data.shape[0]
        self.arm_joint_dofs = joint_data[:,0:6]
        self.arm_dof = self.arm_joint_dofs.shape[1] 
        self.Claw_joit_dofs = joint_data[:,7:8]
        print(self.num_eposide)
        self.threshould_low=0.040
        self.threshould_high=0.047
    def disconnect(self):
        """
        Disconnect from the robot arm.

        Returns:
            None
        """
        handle = self.robot.rm_delete_robot_arm()
        if handle == 0:
            print("\nSuccessfully disconnected from the robot arm\n")
        else:
            print("\nFailed to disconnect from the robot arm\n")

    def movel(self, pose, v=20, r=0, connect=0, block=1):
        """
        Perform movel motion.

        Args:
            pose (list of float): End position [x, y, z, rx, ry, rz].
            v (float, optional): Speed of the motion. Defaults to 20.
            connect (int, optional): Trajectory connection flag. Defaults to 0.
            block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.
            r (float, optional): Blending radius. Defaults to 0.

        Returns:
            None
        """
        movel_result = self.robot.rm_movel(pose, v, r, connect, block)
        if movel_result == 0:
            print("\nmovel motion succeeded\n")
        else:
            print("\nmovel motion failed, Error code: ", movel_result, "\n")

    def movej_p(self, pose, v=20, r=0, connect=0, block=1):
        """
        Perform movej_p motion.

        Args:
            pose (list of float): Position [x, y, z, rx, ry, rz].
            v (float, optional): Speed of the motion. Defaults to 20.
            connect (int, optional): Trajectory connection flag. Defaults to 0.
            block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.
            r (float, optional): Blending radius. Defaults to 0.

        Returns:
            None
        """
        movej_p_result = self.robot.rm_movej_p(pose, v, r, connect, block)
        if movej_p_result == 0:
            print("\nmovej_p motion succeeded\n")
        else:
            print("\nmovej_p motion failed, Error code: ", movej_p_result, "\n")
    def set_Claw(self,flag_claw):
        #力控相关参数初始化
        speed = 30#夹爪运行速度初始化
        force = 50#无量纲，需要尝试
        block = True
        timeout = 0 #单位为秒设置为非阻塞模式，这样与透传模式形成配合
        if flag_claw==1:
            #此处使得夹爪闭合，采用力控方法
            print("\nGripper is gripping\n")
            gripper_result = self.robot.rm_set_gripper_pick_on(speed, force, block, timeout)
            if gripper_result == 0:
                print("\nGripper continuous force control gripping succeeded\n")
            else:
                print("\nGripper continuous force control gripping failed, Error code: ", gripper_result, "\n")
        elif flag_claw==0:
            #此处应为夹爪张开状态，采用力控方法
            gripper_result = self.robot.rm_set_gripper_release(speed, block, timeout)
            if gripper_result == 0:
                print("\nGripper release succeeded\n")
            else:
                print("\nGripper release failed, Error code: ", gripper_result, "\n")
    def claw_threshold_judgment(self,value):
        """
        判断输入值与阈值的关系
        :param value: 输入值
        :param threshold_low && threshold_high: 高低阈值判断，夹爪开启关闭
        """
        if value > self.threshould_high:
            print( f"{value} claw_status :open")
            self.set_Claw(0)
        elif value <self.threshould_low:
            print( f"{value} claw_status :close")
            self.set_Claw(1)



class claw_init:
    def __init__(self,ip,port):
        # 创建一个 TCP 服务器套接字
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 绑定到本地地址和端口
        self.client.connect((ip,port))
        print("机械臂连接",ip) 
        self.set_claw_voltage()
        self.set_modbus_mode()  
        self.set_Claw_N()
        self.write_single_register()
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
    def write_single_register(self):
        point6_00 = '{"command":"write_single_register","port":1,"address":256,"data":1, "device":1}\r\n'
        _ = self.send_cmd(point6_00) 
        print("执行初始化成功")
    def set_Claw_N(self):
        point6_00 = '{"command":"write_single_register","port":1,"address":257,"data":30, "device":1}\r\n'
        _ = self.send_cmd(point6_00)
        print("设置30% 力值 （写操作）") 
    def set_Claw_position(self,position):
        #调整夹爪的位置到设定值
        #范围100~1000
        point6_00 = '{"command":"write_single_register","port":1,"address":259,"data":'+str(position)+', "device":1}\r\n'
        _ = self.send_cmd(point6_00)
def todu(x):
    return x *180/3.14

def main():
    try:
        # 创建机械臂对象
        #这里采用两个类单独控制机械臂和夹爪，MI_robot为机械臂 MI_claw为夹爪
        MI_robot = MIArmController("192.168.1.18", 8080, 3)
        MI_claw = claw_init("192.168.1.18", 8080)
        MI_claw.set_Claw_position(200)
        for dof_idx in range(MI_robot.num_eposide):
            # 提取当前关节的数量
            joint_angle = MI_robot.arm_joint_dofs[dof_idx,:].tolist() 
            #弧度转换为°
            joint_angle = list(map(todu, joint_angle))
            #夹爪状态获取并转换
            claw_status = MI_robot.Claw_joit_dofs[dof_idx,0].tolist()#两侧夹爪对称
            print(claw_status)
            # MI_robot.claw_threshold_judgment(claw_status)

            if(dof_idx==1):
                movej_result = MI_robot.robot.rm_movej(joint_angle, v=10, r=1, connect=0, block=1)
                print("move1")
            # movej_result = robot.rm_movej(joint_angle, v=20, r=0, connect=0, block=1)
            movej_result = MI_robot.robot.rm_movej_canfd(joint_angle, follow=False)    
            # 简单的运动示例
            # 执行关节运动
            if movej_result == 0:
                print(f"Joint movement succeeded.{dof_idx}")
            else:
                print(f"Joint movement failed, Error code: {movej_result}")

            # 透传模式下需要等待一段时间
            time.sleep(0.01)
        joint_angle = [0,0,0,0,0,0,0]
        MI_robot.robot.rm_movej(joint_angle, v=20, r=0, connect=0, block=1)
        # 断开机械臂连接
        disconnect_result = MI_robot.robot.rm_delete_robot_arm()
        if disconnect_result == 0:
            print("Successfully disconnected from the robot arm.")
        else:
            print("Failed to disconnect from the robot arm.")

    except Exception as e:
        print(f"An error occurred: {e}")