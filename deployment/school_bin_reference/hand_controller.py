from robotic_arm_package.robotic_arm import *
from roh_registers_v1 import *
import socket
import time
from enum import Enum
from utils import L_BYTE, H_BYTE

class OHand():
    def __init__(self, robot, delay_time = 1):
        self._robot = robot
        self.COM_PORT = 1
        self.ROH_ADDR = 2

        self.target_fingers = [
            ROH_FINGER_POS_TARGET0,
            ROH_FINGER_POS_TARGET1,
            ROH_FINGER_POS_TARGET2,
            ROH_FINGER_POS_TARGET3,
            ROH_FINGER_POS_TARGET4,
            ROH_FINGER_POS_TARGET5,
        ]
        self._delay_time = delay_time

    def finger_move(self, target, num_finger, hand_pos, delay):
        """
        控制 num_finger 个手指移动到 HAND_POS 所指示的位置（寄存器值）。
        target: 起始指令寄存器 (ROH_FINGER_POS_TARGET0, etc)
        num_finger: 写入的寄存器数量
        HAND_POS: 寄存器数据（长度 = 2 * num_finger）
        delay: 等待时间
        """
        self._robot.Write_Registers(
            self.COM_PORT,
            target,
            num_finger,
            hand_pos,
            self.ROH_ADDR,
            True
        )
        time.sleep(delay)
    
    def move_open_status(self):
        HAND_READY_OPEN_POS = [
            H_BYTE(0),     L_BYTE(0),  # 大拇指内翻
            H_BYTE(0),     L_BYTE(0), # 食指内翻
            H_BYTE(0),     L_BYTE(0), # 中指内翻
            H_BYTE(0),     L_BYTE(0), # 无名指内翻
            H_BYTE(0),     L_BYTE(0), # 小拇指内翻
            H_BYTE(0),     L_BYTE(0), #大拇指内折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_READY_OPEN_POS,
                         self._delay_time
                         )

class GraspHand(OHand):
    def __init__(self, robot, delay_time=1):
        super().__init__(robot, delay_time)
    
    def move_prepare_grasp_status(self):
        HAND_READY_PREPARE_POS = [
            H_BYTE(8000),     L_BYTE(8000),  # 大拇指内翻
            H_BYTE(8000),     L_BYTE(8000), # 食指内翻
            H_BYTE(8000),     L_BYTE(8000), # 中指内翻
            H_BYTE(8000),     L_BYTE(8000), # 无名指内翻
            H_BYTE(65535),     L_BYTE(65535), # 小拇指内翻
            H_BYTE(65535),     L_BYTE(65535), #大拇指内折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_READY_PREPARE_POS,
                         self._delay_time
                         )
    
    def move_prepare_grasp_capacitor_status(self):
        HAND_READY_GRASP_POS = [
            H_BYTE(10000),     L_BYTE(10000),  # 大拇指内翻
            H_BYTE(10000),     L_BYTE(10000), # 食指内翻
            H_BYTE(10000),     L_BYTE(10000), # 中指内翻
            H_BYTE(10000),     L_BYTE(10000), # 无名指内翻
            H_BYTE(10000),     L_BYTE(10000), # 小拇指内翻
            H_BYTE(65535),     L_BYTE(65535), #大拇指内折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_READY_GRASP_POS,
                         self._delay_time
                         )

    def move_grasp_capacitor_status(self):
        HAND_READY_PREPARE_POS = [
            H_BYTE(35000),     L_BYTE(35000),  # 大拇指内翻
            H_BYTE(35000),     L_BYTE(35000), # 食指内翻
            H_BYTE(35000),     L_BYTE(35000), # 中指内翻
            H_BYTE(35000),     L_BYTE(35000), # 无名指内翻
            H_BYTE(35000),     L_BYTE(35000), # 小拇指内翻
            H_BYTE(65535),     L_BYTE(65535), #大拇指内折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_READY_PREPARE_POS,
                         self._delay_time
                         )

    def move_grasp_battery_status(self):
        HAND_READY_GRASP_POS1 = [
            H_BYTE(30000),     L_BYTE(30000)  # 大拇指内翻
            ]     
        self.finger_move(self.target_fingers[0],
                         1,
                         HAND_READY_GRASP_POS1,
                         self._delay_time
                         )
        HAND_READY_PREPARE_POS = [
            H_BYTE(15000),     L_BYTE(15000),  # 大拇指内翻
            H_BYTE(15000),     L_BYTE(15000), # 食指内翻
            H_BYTE(15000),     L_BYTE(15000), # 中指内翻
            H_BYTE(15000),     L_BYTE(15000), # 无名指内翻
            H_BYTE(65535),     L_BYTE(65535), # 小拇指内翻
            H_BYTE(65535),     L_BYTE(65535), #大拇指内折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_READY_PREPARE_POS,
                         self._delay_time
                         )
        
        HAND_READY_GRASP_POS = [
            H_BYTE(45000),     L_BYTE(45000),  # 大拇指内翻
            H_BYTE(30000),     L_BYTE(30000), # 食指内翻
            H_BYTE(35000),     L_BYTE(35000), # 中指内翻
            H_BYTE(40000),     L_BYTE(40000), # 无名指内翻
            H_BYTE(65535),     L_BYTE(65535), # 小拇指内翻
            H_BYTE(65535), L_BYTE(65535), #大拇指内折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_READY_GRASP_POS,
                         self._delay_time
                         )  
        HAND_READY_GRASP_POS1 = [
            H_BYTE(40000),     L_BYTE(40000)  # 大拇指内翻
            ]     
        self.finger_move(self.target_fingers[1],
                         1,
                         HAND_READY_GRASP_POS1,
                         self._delay_time
                         )

    def move_hold_card_block_status(self):
        """持块手型：用于夹持黏贴卡片的块状物，基线来自 HAND_GRASP_BOTTLE_POS"""
        HAND_HOLD_CARD_BLOCK_POS = [
            H_BYTE(15000),     L_BYTE(15000),  # 大拇指内翻
            H_BYTE(30000),     L_BYTE(30000),  # 食指内翻
            H_BYTE(21000),     L_BYTE(21000),  # 中指内翻
            H_BYTE(19000),     L_BYTE(19000),  # 无名指内翻
            H_BYTE(16000),     L_BYTE(16000),  # 小拇指内翻
            H_BYTE(65535),     L_BYTE(65535),   # 大拇指折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_HOLD_CARD_BLOCK_POS,
                         self._delay_time
                         )

    def move_index_point_status(self):
        HAND_INDEX_POINT_POS = [
            H_BYTE(42000),     L_BYTE(42000),  # 大拇指内翻
            H_BYTE(0),         L_BYTE(0),      # 食指伸开
            H_BYTE(48000),     L_BYTE(48000),  # 中指握拳
            H_BYTE(45000),     L_BYTE(45000),  # 无名指握拳
            H_BYTE(42000),     L_BYTE(42000),  # 小拇指握拳
            H_BYTE(65535),     L_BYTE(65535),  # 大拇指折叠
            ]
        self.finger_move(self.target_fingers[0],
                         6,
                         HAND_INDEX_POINT_POS,
                         self._delay_time
                         )

        
class LeftGraspHand(GraspHand):
    def __init__(self, robot, delay_time=1):
        super().__init__(robot, delay_time)


class RightGraspHand(GraspHand):
    def __init__(self, robot, delay_time=1):
        super().__init__(robot, delay_time)