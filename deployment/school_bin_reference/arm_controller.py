from utils import L_BYTE, H_BYTE
from hand_controller import GraspHand
import copy
import time

class ArmController():
    def __init__(self, robot, arm_speed = 10):
        self._robot = robot
        self._arm_speed = arm_speed

    def Movel(self, target):
        return self._robot.Movel_Cmd(target, 
                             self._arm_speed, 
                             0, 
                             True)
    
    def Movej(self, target):
        return self._robot.Movej_P_Cmd(target,
                               self._arm_speed,
                                0, 
                                True)
    def MoveJoint(self, joints):
        return self._robot.Movej_Cmd(joints,
                                     self._arm_speed,
                                     0, 
                                     True)
    def SetLiftHeight(self, height, speed):
        return self._robot.Set_Lift_Height(height, speed, True)
        

class GraspArm(ArmController):
    def __init__(self, robot, hand : GraspHand, arm_speed=10):
        super().__init__(robot, arm_speed)
        self._hand = hand
        self._idle_position = None

    def _recover_to_idle(self):
        self._hand.move_open_status()
        self.MoveJoint(self._idle_position)

    def move_to_idle_position(self):
        print("--> move to idle position")
        self.Movej(self._idle_position)

    @property
    def idle_position(self):
        return self._idle_position
    
    @idle_position.setter
    def idle_position(self, new_position):
        self._idle_position = new_position

    def move_to_idle_position(self):
        print("--> move to idle position")
        self.MoveJoint(self._idle_position)

    def set_lift_height(self, height, speed):
        return self.SetLiftHeight(height, speed)

    def prepare_grasp_battery(self, from_pos):
        print("--> prepare grasp battery")
        self.Movej(from_pos)

        print("---> prepare finger positon")
        self._hand.move_prepare_grasp_status()

    def grasp_capacitor(self, from_pos, target_pos):
        print("capacitor to from_pos")
        self.Movej(from_pos)

        self._hand.move_prepare_grasp_capacitor_status()
        print("capacitor to end_pos")
        self.Movel(target_pos)

        self._hand.move_grasp_capacitor_status()
        quit_pos = copy.deepcopy(from_pos)
        quit_pos[2] = quit_pos[2] + (30 / 1000)
        self.Movel(quit_pos)
        

    def grasp_battery(self, from_pos, target_pos):
        """
        grasp_battery _summary_

        Args:
            from_pos (x, y, z, theta_x, theta_y, theta_z): postion of prepare grasp
            target_pos (x, y, z, theta_x, theta_y, theta_z): position of grasp object
        """
        print("---> prepare finger positon")
        self._hand.move_prepare_grasp_status()

        print("--> prepare grasp battery")
        prepare_pos = copy.deepcopy(from_pos)
        print(from_pos)
        print(target_pos)
        prepare_pos_status = False
        for i in range(30):
            print(f'try ++ {i} times')
            temp_pos = copy.deepcopy(from_pos)
            temp_pos[5] = temp_pos[5] + 0.01 * i
            for j in range(10):
                temp_pos_2 = copy.deepcopy(temp_pos)
                temp_pos_3 = copy.deepcopy(temp_pos)
                temp_pos_2[4] = temp_pos_2[4] + 0.01 * j
                temp_pos_3[4] = temp_pos_3[4] - 0.01 * j
                if self.Movej(temp_pos_2) == 0:
                    prepare_pos_status = True
                    prepare_pos = copy.deepcopy(temp_pos_2)
                    break
                if self.Movej(temp_pos_3) == 0:
                    prepare_pos_status = True
                    prepare_pos = copy.deepcopy(temp_pos_3)
                    break
            if prepare_pos_status is True:
                break
        
        if prepare_pos_status is False:
            for i in range(30):
                print(f'try -- {i} times')
                temp_pos = copy.deepcopy(from_pos)
                temp_pos[5] = temp_pos[5] - 0.01 * i
                for j in range(10):
                    temp_pos_2 = copy.deepcopy(temp_pos)
                    temp_pos_3 = copy.deepcopy(temp_pos)
                    temp_pos_2[4] = temp_pos_2[4] + 0.01 * j
                    temp_pos_3[4] = temp_pos_3[4] - 0.01 * j
                    if self.Movej(temp_pos_2) == 0:
                        prepare_pos_status = True
                        prepare_pos = copy.deepcopy(temp_pos_2)
                        break
                    if self.Movej(temp_pos_3) == 0:
                        prepare_pos_status = True
                        prepare_pos = copy.deepcopy(temp_pos_3)
                        break
                if prepare_pos_status is True:
                    break


        print("---> move to grasp position")
        grasp_pos = copy.deepcopy(target_pos)
        grasp_pos_status = False
        for i in range(30):
            print(f'try ++ {i} times')
            temp_pos = copy.deepcopy(target_pos)
            temp_pos[5] = temp_pos[5] + 0.01 * i
            for j in range(30):
                temp_pos_2 = copy.deepcopy(temp_pos)
                temp_pos_3 = copy.deepcopy(temp_pos)
                temp_pos_2[2] = temp_pos_2[2] + 0.0005 * j 
                temp_pos_3[2] = temp_pos_3[2] - 0.0005 * j
                if self.Movel(temp_pos_2) == 0:
                    grasp_pos_status = True
                    grasp_pos = copy.deepcopy(temp_pos_2)
                    break
                if self.Movel(temp_pos_3) == 0:
                    grasp_pos_status = True
                    grasp_pos = copy.deepcopy(temp_pos_3)
                    break
            if grasp_pos_status is True:
                    break
        
        if grasp_pos_status is False:
            for i in range(30):
                print(f'try -- {i} times')
                temp_pos = copy.deepcopy(target_pos)
                temp_pos[5] = temp_pos[5] - 0.01 * i
                for j in range(10):
                    temp_pos_2 = copy.deepcopy(temp_pos)
                    temp_pos_3 = copy.deepcopy(temp_pos)
                    temp_pos_2[2] = temp_pos_2[2] + 0.0005 * j 
                    temp_pos_3[2] = temp_pos_3[2] - 0.0005 * j
                    if self.Movel(temp_pos_2) == 0:
                        grasp_pos_status = True
                        grasp_pos = copy.deepcopy(temp_pos_2)
                        break
                    if self.Movel(temp_pos_3) == 0:
                        grasp_pos_status = True
                        grasp_pos = copy.deepcopy(temp_pos_3)
                        break
                if grasp_pos_status is True:
                        break

        # self.Movel(grasp_pos)

        print("---> grasp object")
        self._hand.move_grasp_battery_status()

        print("---> quit position")
        quit_pos = copy.deepcopy(from_pos)
        quit_pos[2] = quit_pos[2] + (60 / 1000)
        self.Movel(quit_pos)
    
    def palce_object_recycle_area(self, position):
        print("---> move to recycle position")
        self.Movej(position)
        self._hand.move_open_status()

    def scan_battery(self, target_pos):
        print("---> scan battery")
        self.Movej(target_pos)
    
    def move_to(self, position):
        self.Movej(position)

    def wave_hand(self, wave_pre_pos, wave_left_pos, wave_right_pos, repeat_times=3):
        print("--> wave_hand: move to idle")
        if self.MoveJoint(self._idle_position) != 0:
            return (False, 4)

        print("--> wave_hand: open hand")
        self._hand.move_open_status()

        print("--> wave_hand: move to wave_pre_pos")
        if self.Movej(wave_pre_pos) != 0:
            self._recover_to_idle()
            return (False, 4)

        for index in range(repeat_times):
            print(f"--> wave_hand: cycle {index + 1} left")
            if self.Movej(wave_left_pos) != 0:
                self._recover_to_idle()
                return (False, 4)

            print(f"--> wave_hand: cycle {index + 1} right")
            if self.Movej(wave_right_pos) != 0:
                self._recover_to_idle()
                return (False, 4)

        print("--> wave_hand: back to wave_pre_pos")
        if self.Movej(wave_pre_pos) != 0:
            self._recover_to_idle()
            return (False, 4)

        print("--> wave_hand: back to idle")
        if self.MoveJoint(self._idle_position) != 0:
            return (False, 4)

        return (True, 0)

    def point_direction(self, point_pos, hold_seconds=1):
        print("--> point_direction: move to idle")
        if self.MoveJoint(self._idle_position) != 0:
            return (False, 4)

        print("--> point_direction: switch to index point hand pose")
        self._hand.move_index_point_status()

        print("--> point_direction: move to target point position")
        if self.Movej(point_pos) != 0:
            self._recover_to_idle()
            return (False, 4)

        time.sleep(hold_seconds)

        print("--> point_direction: recover hand and arm")
        self._hand.move_open_status()
        if self.MoveJoint(self._idle_position) != 0:
            return (False, 4)

        return (True, 0)

    def one_move(self, target_pos, hold_seconds=1):
        print("--> one_move: move to idle")
        if self.MoveJoint(self._idle_position) != 0:
            return (False, 4)

        print("--> one_move: open hand")
        self._hand.move_open_status()

        print("--> one_move: move to target position")
        if self.Movej(target_pos) != 0:
            self._recover_to_idle()
            return (False, 4)

        time.sleep(hold_seconds)

        print("--> one_move: back to idle")
        if self.MoveJoint(self._idle_position) != 0:
            return (False, 4)

        return (True, 0)

    def tap_card(self, pickup_pre_pos, pickup_pos, reader_pos):
        """
        刷卡流程：取块 -> 刷卡 -> 放回 -> 回 idle
        返回 (success: bool, error_code: int)
        """
        # 1. 回 idle 作为统一起点
        print("--> tap_card: move to idle")
        self.MoveJoint(self._idle_position)

        # 2. 张手
        print("--> tap_card: open hand")
        self._hand.move_open_status()

        # 3. Movej 到取块预备位
        print("--> tap_card: move to pickup_pre_pos")
        ret = self.Movej(pickup_pre_pos)
        if ret != 0:
            self._hand.move_open_status()
            self.MoveJoint(self._idle_position)
            return (False, 3)

        # 4. Movel 到取块位
        print("--> tap_card: movel to pickup_pos")
        ret = self.Movel(pickup_pos)
        self
        if ret != 0:
            self._hand.move_open_status()
            self.MoveJoint(self._idle_position)
            return (False, 3)

        # 5. 持块手型夹住
        print("--> tap_card: hold card block")
        self._hand.move_hold_card_block_status()

        # 6. Movel 退回取块预备位
        print("--> tap_card: movel back to pickup_pre_pos")
        ret = self.Movel(pickup_pre_pos)
        if ret != 0:
            self._hand.move_open_status()
            self.MoveJoint(self._idle_position)
            return (False, 3)

        # 7. Movej 到刷卡机位
        print("--> tap_card: movej to reader_pos")
        ret = self.Movej(reader_pos)
        if ret != 0:
            self._hand.move_open_status()
            self.MoveJoint(self._idle_position)
            return (False, 4)

        # 8. 停顿 1 秒完成刷卡
        print("--> tap_card: pause 1s for card reading")
        time.sleep(1)

        # 9. Movej 回取块预备位
        print("--> tap_card: movej back to pickup_pre_pos")
        ret = self.Movej(pickup_pre_pos)
        if ret != 0:
            self._hand.move_open_status()
            self.MoveJoint(self._idle_position)
            return (False, 5)

        # 10. Movel 到取块位放回
        print("--> tap_card: movel to pickup_pos (return block)")
        ret = self.Movel(pickup_pos)
        if ret != 0:
            self._hand.move_open_status()
            self.MoveJoint(self._idle_position)
            return (False, 5)

        # 11. 张手释放块体
        print("--> tap_card: open hand (release block)")
        self._hand.move_open_status()

        # 12. Movel 退回取块预备位
        print("--> tap_card: movel back to pickup_pre_pos")
        self.Movel(pickup_pre_pos)

        # 13. 回 idle
        print("--> tap_card: move to idle")
        self.MoveJoint(self._idle_position)

        return (True, 0)      

class LeftGraspArm(GraspArm):
    def __init__(self, robot, hand, arm_speed=10):
        super().__init__(robot, hand, arm_speed)

class RightGraspArm(GraspArm):
    def __init__(self, robot, hand, arm_speed=10):
        super().__init__(robot, hand, arm_speed)