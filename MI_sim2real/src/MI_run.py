import sys
import os
# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import time
from Robotic_Arm.rm_robot_interface import *

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
        self.Claw_dof = self.Claw_joit_dofs.shape[1]
        print(self.num_eposide)
    

        self.threshold_low=0.47
        self.threshold_high=0.40
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

    def set_gripper_pick_on(self, speed, force, block=True, timeout=30):
        """
        Perform continuous force-controlled gripping with the gripper.

        Args:
            speed (int): Speed of the gripper.
            force (int): Force applied by the gripper.
            block (bool, optional): Whether the function is blocking. Defaults to True.
            timeout (int, optional): Timeout duration. Defaults to 30.

        Returns:
            None
        """
        gripper_result = self.robot.rm_set_gripper_pick_on(speed, force, block, timeout)
        if gripper_result == 0:
            print("\nGripper continuous force control gripping succeeded\n")
        else:
            print("\nGripper continuous force control gripping failed, Error code: ", gripper_result, "\n")
        time.sleep(2)

    def set_gripper_release(self, speed, block=True, timeout=30):
        """
        Release the gripper.

        Args:
            speed (int): Speed of the gripper release.
            block (bool, optional): Whether the function is blocking. Defaults to True.
            timeout (int, optional): Timeout duration. Defaults to 30.

        Returns:
            None
        """
        gripper_result = self.robot.rm_set_gripper_release(speed, block, timeout)
        if gripper_result == 0:
            print("\nGripper release succeeded\n")
        else:
            print("\nGripper release failed, Error code: ", gripper_result, "\n")
        time.sleep(2)

    def set_lift_height(self, speed, height, block=True):
        """
        Set the lift height of the robot.

        Args:
            speed (int): Speed of the lift.
            height (int): Target height of the lift.
            block (bool, optional): Whether the function is blocking. Defaults to True.

        Returns:
            None
        """
        lift_result = self.robot.rm_set_lift_height(speed, height, block)
        if lift_result == 0:
            print("\nLift motion succeeded\n")
        else:
            print("\nLift motion failed, Error code: ", lift_result, "\n")
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
    def claw_threshold_judgment(self,value,):
        """
        判断输入值与阈值的关系
        :param value: 输入值
        :param threshold_low && threshold_high: 高低阈值判断，夹爪开启关闭
        """
        if value > self.threshould_high:
            print( f"{value} claw_status :open")
            self.set_Claw(0)
        elif value <self.threshold_low:
            print( f"{value} claw_status :close")
            self.set_Claw(1)


def todu(x):
    return x *180/3.14

def main():
    try:
        # 创建机械臂对象
        MI_robot = MIArmController("192.168.1.18", 8080, 3)
      
        for dof_idx in range(MI_robot.num_eposide):
            # 提取当前关节的数量
            joint_angle = MI_robot.arm_joint_dofs[dof_idx,:].tolist() 
            #弧度转换为°
            joint_angle = list(map(todu, joint_angle))
            #夹爪状态获取并转换
            claw_status = MI_robot.Claw_joit_dofs[dof_idx,1].tolist()#两侧夹爪对称
            MI_robot.claw_threshold_judgment(claw_status)

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