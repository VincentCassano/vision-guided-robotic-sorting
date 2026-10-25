#!/usr/bin/env python3

from robotic_arm_package.robotic_arm import *
from roh_registers_v1 import *
import socket
import time
from enum import Enum
from utils import wait_event_clear
import copy
import rospy
from hand_controller import LeftGraspHand, RightGraspHand
from arm_controller import LeftGraspArm, RightGraspArm
from grasp_demo.msg import DetectedObject, PickCommand, ReturnHome
from grasp_demo.srv import SetWorkMode, SetWorkModeResponse, QueryWorkMode, QueryWorkModeResponse
from grasp_demo.srv import StartWork, StartWorkResponse
from grasp_demo.srv import SuspendWork, SuspendWorkResponse
from grasp_demo.srv import GraspObject, GraspObjectResponse
from grasp_demo.srv import CheckTaskStatus, CheckTaskStatusResponse
from grasp_demo.srv import TapCard, TapCardResponse
from grasp_demo.srv import WaveHand, WaveHandResponse
from grasp_demo.srv import PointDirection, PointDirectionResponse
from grasp_demo.srv import OneMove, OneMoveResponse
import functools
import requests
import threading

class HANDSELECT(Enum):
    LEFT = 0
    RIGHT = 1

class WORKMODE(Enum):
    MANUAL = "manual"
    AUTO  = "auto"

class RobotRoutine:

    def __init__(self, arm_ip="169.254.128.18", hand_selected = 1):
        # -----------------------
        # Configuration constants
        # -----------------------
        self.ARM_IP = arm_ip

        self.COM_PORT = 1
        self.ROH_ADDR = 2

        self.HAND_SELECTED = hand_selected  # 选择左手(0) 或者右手(1)

        self.LOOP_TIME = 100   # 循环次数
        self.ARM_SPEED = 10    # 机械臂速度/%
        self.DELAY_TIME = 1  # 延时时间/s


        # --------------------------------------
        # 尝试和机械臂连接 (重复连接直到成功)
        # --------------------------------------
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.connect((self.ARM_IP, 8080))
                s.close()
                break
            except socket.error:
                print("connect time out, try again") #[-0.11308199912309647, -0.5092869997024536, -0.10766900330781937, -2.1510000228881836, 0.3199999928474426, -3.118000030517578]
            time.sleep(1)

        # 初始化机械臂对象并设置通信模式
        print("Init arm")
        self.robot = Arm(RM65, self.ARM_IP)
        self.robot.Close_Modbustcp_Mode()
        self.robot.Set_Modbus_Mode(1, 115200, 1, True)

    def current_pos(self):
        x = self.robot.Get_Current_Arm_State(4)[2][0] # Example values, replace with actual data
        y = self.robot.Get_Current_Arm_State(4)[2][1]
        z = self.robot.Get_Current_Arm_State(4)[2][2]
        theta_x = self.robot.Get_Current_Arm_State(4)[2][3]
        theta_y = self.robot.Get_Current_Arm_State(4)[2][4]
        theta_z =  self.robot.Get_Current_Arm_State(4)[2][5]
        return [x, y, z, theta_x, theta_y, theta_z]
    


class GlobalVar:

    Y_BIAS = 0.110
    X_BIAS = 0.115

    # 标定抓取电池位置
    ARM_GRASP_POS = [0.2977910041809082, -0.43435901403427124, -0.21806100010871887,
                      -2.4649999141693115, -0.8889999985694885, -2.509000062942505]
    ARM_GRASP_POS = [0.2724590003490448, -0.4211130142211914, -0.25222599506378174, -2.4570000171661377, -1.065999984741211, -2.3320000171661377]
    ARM_START_POS = copy.deepcopy(ARM_GRASP_POS)

    # 原扫描摄像头位置
    SCAN_BATTERY_POS = [[0.05982000008225441, -0.3046959936618805, -0.06598900258541107, 2.072999954223633, -0.6510000228881836, -1.9220000505447388],
                [0.05832799896597862, -0.30875998735427856, -0.11492700129747391, 1.74399995803833, -0.07699999958276749, -1.9019999504089355],
               [-0.0013139999937266111, -0.3148350119590759, -0.07608199864625931, 1.6619999408721924, -1.2790000438690186, -2.0739998817443848]]
    
    SCAN_BATTERY_RESTART_POS = [0.5446869730949402, -0.3363009989261627, 0.14712999761104584, -1.9700000286102295, -0.4860000014305115, -2.4179999828338623]

    # 正常电池回收位置
    RECYCLE_AREA = [0.45028701424598694, -0.418067991733551, -0.14466699957847595, -2.5940001010894775, -1.2289999723434448, -1.7489999532699585]
    # 红色电池回收位置
    RECYCLE_RED_AREA = [0.5475559830665588, -0.24783800542354584, -0.17733100056648254, -2.7190001010894775, -1.2419999837875366, -1.5700000524520874]

    RECYCLE_WHITE_AREA = [0.47132399678230286, -0.23525099456310272, -0.181878000497818, -2.736999988555908, -1.3209999799728394, -2.071000099182129]

    # laster battery reposition.
    LASTER_BATTERY_REPOSITION = [0.11203800141811371, -0.4014500081539154, 0.03451399877667427, 3.065999984741211, -1.1080000400543213, -2.434999942779541]

    # LEFT_IDLE_POSITION = [0.3083550035953522, -0.23879100382328033, 0.07954200357198715, 
    #                       -3.072999954223633, -0.8690000176429749, -2.3399999141693115]
    LEFT_IDLE_POSITION = [-46.527000427246094, -1.4700000286102295, -101.78500366210938, -26.367000579833984, -47.14699935913086, 113.95600128173828]
    RIGHT_IDLE_POSITION = [36.926998138427734, 12.840999603271484, 88.16300201416016, -139.6959991455078, -68.36599731445312, 5.72599983215332]

    # RIGHT_IDLE_POSITION = [-0.32850098609924316, -0.24685099720954895, 0.06782100349664688, 
    #                        -2.1440000534057617, 0.7009999752044678, -2.7100000381469727]

    CAPACITOR_RECYCLE_AREA = [-0.4885849952697754, -0.240665003657341, -0.17952899634838104, -1.8519999980926514, 0.8270000219345093, 2.8940000534057617]

    # HIGHT_POS = [0.48590099811553955, -0.45009198784828186, 0.3013249933719635, 2.7160000801086426, -1.2630000114440918, -0.9900000095367432]
    HIGHT_POS = [0.38515201210975647, -0.4414309859275818, -0.023086000233888626, -2.4130001068115234, -1.3769999742507935, -2.63100004196167]

    CAPACITOR_GRASP_POS = [-0.21278099715709686, -0.21299700438976288, -0.2844879925251007,
                            -1.8919999599456787, 1.1920000314712524, -2.7139999866485596]
    CAPACITOR_GRASP_POS = [-0.21326300501823425, -0.21911300718784332, -0.2667570114135742, 
                           -1.9919999837875366, 1.0679999589920044, -2.8440001010894775]
    CAPACITOR_HIGHT_POS = [-0.3137199878692627, -0.3843179941177368, -0.07731600105762482, -1.159999966621399, 0.8569999933242798, -2.437999963760376]

    CAPACITOR_START_POS = copy.deepcopy(CAPACITOR_GRASP_POS)

    # ---- 刷卡流程位姿（笛卡尔 [x, y, z, rx, ry, rz]），需人工标定 ----
    # 取块预备位（块体正上方安全高度）
    CARD_BLOCK_PICKUP_PRE_POS = [-0.5593429803848267, -0.06811100244522095, 0.030160000547766685, 2.2039999961853027, 1.2020000219345093, 0.2549999952316284]  # TODO: 人工标定
    # 取块位（贴近块体可夹持位置）
    CARD_BLOCK_PICKUP_POS = [-0.4996429979801178, -0.03753200173377991, -0.08006499707698822, -2.25600004196167, 1.4880000352859497, 1.8320000171661377]     # TODO: 人工标定
    # 刷卡机位（卡片贴近读卡器位置）
    CARD_READER_POS = [-0.5071250200271606, -0.32284098863601685, -0.060892000794410706, -1.5479999780654907, 1.4520000219345093, 2.5869998931884766]          # TODO: 人工标定

    # ---- 挥手动作位姿（笛卡尔 [x, y, z, rx, ry, rz]），需人工标定 ----
    LEFT_WAVE_PRE_POS = [-44.0, -3.0, -98.0, -24.0, -45.0, 108.0]      # TODO: 用实测标定值替换
    LEFT_WAVE_LEFT_POS = [-40.0, -1.0, -96.0, -20.0, -42.0, 102.0]     # TODO: 用实测标定值替换
    LEFT_WAVE_RIGHT_POS = [-48.0, -5.0, -100.0, -28.0, -48.0, 114.0]   # TODO: 用实测标定值替换
    RIGHT_WAVE_PRE_POS = [-0.5093550086021423, -0.13925500214099884, 0.5012850165367126, -0.13899999856948853, 0.028999999165534973, -2.9739999771118164]         # TODO: 用实测标定值替换
    RIGHT_WAVE_LEFT_POS = [-0.42410901188850403, -0.11367499828338623, 0.4782310128211975, -0.11100000143051147, -0.6159999966621399, -2.9130001068115234]       # TODO: 用实测标定值替换
    RIGHT_WAVE_RIGHT_POS = [-0.6166790127754211, -0.1681160032749176, 0.44252699613571167, -0.5419999957084656, 0.8539999723434448, 2.9089999198913574]      # TODO: 用实测标定值替换

    # ---- 方向指定动作位姿（笛卡尔 [x, y, z, rx, ry, rz]），需人工标定 ----
    LEFT_POINT_LEFT_DOWN_POS = [0.2116899937391281, -0.5242159962654114, -0.17169000208377838, -2.3350000381469727, -0.7409999966621399, -2.450000047683716]    # TODO: 用实测标定值替换
    LEFT_POINT_RIGHT_DOWN_POS = [-34.0, -5.0, -94.0, -29.0, -50.0, 108.0]  # TODO: 用实测标定值替换
    RIGHT_POINT_LEFT_DOWN_POS = [30.0, 6.0, 86.0, -146.0, -73.0, -2.0]     # TODO: 用实测标定值替换
    RIGHT_POINT_RIGHT_DOWN_POS = [-0.7131059765815735, -0.07133399695158005, 0.2500700056552887, 1.527999997138977, 1.5, -1.5230000019073486]    # TODO: 用实测标定值替换

    # One Move动作目标位姿映射，需人工标定
    RIGHT_HAND_BLOCK_POS = [-0.36864298582077026, -0.3682689964771271, 0.20793400704860687, 0.0949999988079071, 0.04100000113248825, 2.7230000495910645] 


    battery_color = "BLACK" # 电池颜色

    AUTO_MODE_THREAD = None

    WORK_MODE = WORKMODE.MANUAL
    HTTP_SERVER_URL = "http://127.0.0.1:5000"
    INSPECT_URL = HTTP_SERVER_URL + "/inspect"
    CLEAR_INSPECT_URL = HTTP_SERVER_URL + "/clearinspect"
    CURRENT_WORK_INDEX = 0
    PAUSE_AUTO_WORK = threading.Event()

    TASK_IS_DONE = False

def pick_command_callback(msg, left_arm, right_arm):
    pass

def return_to_home_callback(msg, left_arm, right_arm):
    left_arm.move_to_idle_position()
    right_arm.move_to_idle_position()

def handle_set_work_mode(req):
    work_mode = req.work_mode.lower()
    set_mode_success = False
    if work_mode == WORKMODE.AUTO.value:
        GlobalVar.WORK_MODE = WORKMODE.AUTO
        set_mode_success = True
    elif work_mode == WORKMODE.MANUAL.value:
        GlobalVar.WORK_MODE = WORKMODE.MANUAL
        set_mode_success = True
    return SetWorkModeResponse(set_mode_success, GlobalVar.WORK_MODE.value)
    
def handle_query_work_mode(req):
    return QueryWorkModeResponse(GlobalVar.WORK_MODE.value)

def handle_start_work(req, left_arm, right_arm):
    if GlobalVar.WORK_MODE == WORKMODE.AUTO:
        if GlobalVar.AUTO_MODE_THREAD is not None and \
            GlobalVar.AUTO_MODE_THREAD.is_alive() is True:
            return StartWorkResponse(True)
        else:
            GlobalVar.AUTO_MODE_THREAD = threading.Thread(target=start_auto_mode_work, 
                                                          args=(left_arm, right_arm),
                                                          daemon=True)
            GlobalVar.AUTO_MODE_THREAD.start()
    elif GlobalVar.WORK_MODE == WORKMODE.MANUAL:
        pass
    return StartWorkResponse(True)

def handle_pause_work(req):
    if GlobalVar.WORK_MODE == WORKMODE.MANUAL:
        return SuspendWorkResponse(True, GlobalVar.WORK_MODE.value)
    elif GlobalVar.WORK_MODE == WORKMODE.AUTO:
        if GlobalVar.AUTO_MODE_THREAD is not None and GlobalVar.AUTO_MODE_THREAD.is_alive():
            GlobalVar.PAUSE_AUTO_WORK.set()
            return SuspendWorkResponse(True, GlobalVar.WORK_MODE.value)
        else:
            return SuspendWorkResponse(False, GlobalVar.WORK_MODE.value)
        
def handle_check_task_status(req):
    if GlobalVar.WORK_MODE == WORKMODE.MANUAL:
        return CheckTaskStatusResponse(False, GlobalVar.WORK_MODE.value)
    elif GlobalVar.WORK_MODE == WORKMODE.AUTO:
        if GlobalVar.AUTO_MODE_THREAD is not None and GlobalVar.AUTO_MODE_THREAD.is_alive() is False:
            GlobalVar.AUTO_MODE_THREAD = None
            if GlobalVar.TASK_IS_DONE is True:
                GlobalVar.TASK_IS_DONE = False
                return CheckTaskStatusResponse(True, GlobalVar.WORK_MODE.value)
            else:
                return CheckTaskStatusResponse(False, GlobalVar.WORK_MODE.value)
        else:
            return CheckTaskStatusResponse(False, GlobalVar.WORK_MODE.value)
        
def handle_grasp_object(req, left_arm, right_arm):
    if GlobalVar.WORK_MODE == WORKMODE.AUTO:
        return GraspObjectResponse(False, 2)
    
    object_name = req.object_name
    supprort_object = ['battery', 'capacitor']
    if object_name not in supprort_object:
        return GraspObjectResponse(False, 1)
    
    if object_name == supprort_object[0]:
        grasp_battery(left_arm, right_arm)
    elif  object_name == supprort_object[1]:
        grasp_capacitor(left_arm, right_arm)
    return GraspObjectResponse(True, 0)
        

def handle_tap_card(req, right_arm: 'RightGraspArm'):
    if GlobalVar.WORK_MODE == WORKMODE.AUTO:
        return TapCardResponse(False, 1)

    # 检查位姿是否仍为占位值
    placeholder = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if (GlobalVar.CARD_BLOCK_PICKUP_PRE_POS == placeholder or
        GlobalVar.CARD_BLOCK_PICKUP_POS == placeholder or
        GlobalVar.CARD_READER_POS == placeholder):
        return TapCardResponse(False, 2)

    success, error_code = right_arm.tap_card(
        GlobalVar.CARD_BLOCK_PICKUP_PRE_POS,
        GlobalVar.CARD_BLOCK_PICKUP_POS,
        GlobalVar.CARD_READER_POS
    )
    return TapCardResponse(success, error_code)


def is_pose_configured(position):
    return (
        isinstance(position, list) and
        len(position) == 6 and
        any(value != 0.0 for value in position)
    )


def get_target_arm(hand, left_arm, right_arm):
    hand_name = hand.lower()
    if hand_name == "left":
        return left_arm
    if hand_name == "right":
        return right_arm
    return None


def get_wave_positions(hand):
    if hand == "left":
        return (
            GlobalVar.LEFT_WAVE_PRE_POS,
            GlobalVar.LEFT_WAVE_LEFT_POS,
            GlobalVar.LEFT_WAVE_RIGHT_POS,
        )
    if hand == "right":
        return (
            GlobalVar.RIGHT_WAVE_PRE_POS,
            GlobalVar.RIGHT_WAVE_LEFT_POS,
            GlobalVar.RIGHT_WAVE_RIGHT_POS,
        )
    return (None, None, None)


def get_point_direction_pose(hand, direction):
    if hand == "left" and direction == "left_down":
        return GlobalVar.LEFT_POINT_LEFT_DOWN_POS
    if hand == "left" and direction == "right_down":
        return GlobalVar.LEFT_POINT_RIGHT_DOWN_POS
    if hand == "right" and direction == "left_down":
        return GlobalVar.RIGHT_POINT_LEFT_DOWN_POS
    if hand == "right" and direction == "right_down":
        return GlobalVar.RIGHT_POINT_RIGHT_DOWN_POS
    return None


def get_one_move_target(action):
    action_map = {
        "hand_block": ("right", GlobalVar.RIGHT_HAND_BLOCK_POS),
        # "left_wave_left": ("left", GlobalVar.LEFT_WAVE_LEFT_POS)

    }
    return action_map.get(action)


def handle_wave_hand(req, left_arm, right_arm):
    if GlobalVar.WORK_MODE == WORKMODE.AUTO:
        return WaveHandResponse(False, 1)

    hand = req.hand.lower()
    target_arm = get_target_arm(hand, left_arm, right_arm)
    if target_arm is None:
        return WaveHandResponse(False, 2)

    wave_pre_pos, wave_left_pos, wave_right_pos = get_wave_positions(hand)
    if not all(is_pose_configured(position) for position in (wave_pre_pos, wave_left_pos, wave_right_pos)):
        return WaveHandResponse(False, 3)

    success, error_code = target_arm.wave_hand(wave_pre_pos, wave_left_pos, wave_right_pos, repeat_times=3)
    return WaveHandResponse(success, error_code)


def handle_point_direction(req, left_arm, right_arm):
    if GlobalVar.WORK_MODE == WORKMODE.AUTO:
        return PointDirectionResponse(False, 1)

    hand = req.hand.lower()
    direction = req.direction.lower()
    target_arm = get_target_arm(hand, left_arm, right_arm)
    if target_arm is None or direction not in ["left_down", "right_down"]:
        return PointDirectionResponse(False, 2)

    point_pos = get_point_direction_pose(hand, direction)
    if not is_pose_configured(point_pos):
        return PointDirectionResponse(False, 3)

    success, error_code = target_arm.point_direction(point_pos)
    return PointDirectionResponse(success, error_code)


def handle_one_move(req, left_arm, right_arm):
    if GlobalVar.WORK_MODE == WORKMODE.AUTO:
        return OneMoveResponse(False, 1)

    action = req.action.lower()
    one_move_target = get_one_move_target(action)
    if one_move_target is None:
        return OneMoveResponse(False, 2)

    hand, target_pos = one_move_target
    target_arm = get_target_arm(hand, left_arm, right_arm)
    if target_arm is None:
        return OneMoveResponse(False, 2)

    if not is_pose_configured(target_pos):
        return OneMoveResponse(False, 3)

    success, error_code = target_arm.one_move(target_pos)
    return OneMoveResponse(success, error_code)


def grasp_battery(left_arm, right_arm):
    x, y  = divmod(GlobalVar.CURRENT_WORK_INDEX, 3)
    start_position = copy.deepcopy(GlobalVar.ARM_GRASP_POS)
    start_position[2] = start_position[2] + 0.080
    target_position = copy.deepcopy(GlobalVar.ARM_GRASP_POS)

    start_position[0] = start_position[0] - x * GlobalVar.X_BIAS
    target_position[0] = target_position[0] -  x * GlobalVar.X_BIAS

    # start_position[1] = start_position[1] - y * GlobalVar.Y_BIAS
    # target_position[1] = target_position[1] - y * GlobalVar.Y_BIAS
    start_position[1] = start_position[1] + y * GlobalVar.Y_BIAS
    target_position[1] = target_position[1] + y * GlobalVar.Y_BIAS

    left_arm.grasp_battery(start_position, target_position)
    left_arm.move_to(GlobalVar.HIGHT_POS)
    print("MOVE_TO IS RUNNING !!!!!!!!!!!")
    for pos in GlobalVar.SCAN_BATTERY_POS:
        left_arm.scan_battery(pos)
        time.sleep(0.5)
        requests.post(GlobalVar.INSPECT_URL, json={})
        time.sleep(1.5)
    left_arm.move_to(GlobalVar.SCAN_BATTERY_RESTART_POS)
    # 检测电池颜色放在相对应的分拣框内
    if GlobalVar.battery_color == "WHITE":
        left_arm.palce_object_recycle_area(GlobalVar.RECYCLE_WHITE_AREA)
        # print("True" + GlobalVar.battery_color)

    elif GlobalVar.battery_color == "RED":
        left_arm.palce_object_recycle_area(GlobalVar.RECYCLE_RED_AREA)
        # print("True" + GlobalVar.battery_color)
    else:
        left_arm.palce_object_recycle_area(GlobalVar.RECYCLE_AREA)
        # print("True" + GlobalVar.battery_color)

    left_arm.move_to_idle_position()
    GlobalVar.CURRENT_WORK_INDEX = GlobalVar.CURRENT_WORK_INDEX + 1
    requests.post(GlobalVar.CLEAR_INSPECT_URL, {})

def grasp_capacitor(left_arm, right_arm):

    start_pos = copy.deepcopy(GlobalVar.CAPACITOR_START_POS)
    start_pos[2] = start_pos[2] + 0.100
    right_arm.grasp_capacitor(start_pos, GlobalVar.CAPACITOR_GRASP_POS)
    right_arm.move_to(GlobalVar.CAPACITOR_HIGHT_POS)
    right_arm.palce_object_recycle_area(GlobalVar.CAPACITOR_RECYCLE_AREA)
    right_arm.move_to_idle_position()

def start_auto_mode_work(left_arm, right_arm):

    GlobalVar.ARM_START_POS = copy.deepcopy(GlobalVar.ARM_GRASP_POS)
    GlobalVar.ARM_START_POS[2] = GlobalVar.ARM_START_POS[2] + 0.080
    # GlobalVar.CAPACITOR_GRASP_POS[2] = GlobalVar.CAPACITOR_GRASP_POS[2] - 0.080
    Y_BIAS =  GlobalVar.Y_BIAS
    X_BIAS = GlobalVar.X_BIAS
    RED_POS = [0, 2]
    WHITE_POS = [2, 1]
    for x in range(3):
        start_pos = copy.deepcopy(GlobalVar.ARM_START_POS)
        target_pos = copy.deepcopy(GlobalVar.ARM_GRASP_POS)
        start_pos[0] = start_pos[0] - x * X_BIAS
        target_pos[0] = target_pos[0] - x * X_BIAS
        for y in range(3):
            # The laster battery dont capture.
            if x == 2 and y == 2:
                break
            current_work_index = x * 3 + y
            if current_work_index < GlobalVar.CURRENT_WORK_INDEX:
                continue
            if current_work_index ==  3: # grasp capcaitor
                if GlobalVar.PAUSE_AUTO_WORK.is_set():
                    GlobalVar.PAUSE_AUTO_WORK.clear()
                    rospy.loginfo(f"Pause work:{GlobalVar.CURRENT_WORK_INDEX}")
                    return
                grasp_capacitor(left_arm, right_arm)
            temp_start_pos = copy.deepcopy(start_pos)
            temp_target_pos = copy.deepcopy(target_pos)
            # temp_start_pos[1] = temp_start_pos[1] - y * Y_BIAS
            # temp_target_pos[1] = temp_target_pos[1] - y * Y_BIAS
            temp_start_pos[1] = temp_start_pos[1] + y * Y_BIAS
            temp_target_pos[1] = temp_target_pos[1] + y * Y_BIAS
            if x == RED_POS[0] and y == RED_POS[1]:
                GlobalVar.battery_color = "RED"
            elif x == WHITE_POS[0] and y == WHITE_POS[1]:
                GlobalVar.battery_color = "WHITE"
            else:
                GlobalVar.battery_color = "BLACK"

            print(GlobalVar.battery_color)
            if GlobalVar.PAUSE_AUTO_WORK.is_set():
                GlobalVar.PAUSE_AUTO_WORK.clear()
                rospy.loginfo(f"Pause work:{GlobalVar.CURRENT_WORK_INDEX}")
                return
            left_arm.grasp_battery(temp_start_pos, temp_target_pos)
            left_arm.move_to(GlobalVar.HIGHT_POS)
            print("MOVE_TO IS RUNNING !!!!!!!!!!!")
            for pos in GlobalVar.SCAN_BATTERY_POS:
                left_arm.scan_battery(pos)
                # time.sleep(0.5)
                requests.post(GlobalVar.INSPECT_URL, json={})
                # time.sleep(1.5)
            left_arm.move_to(GlobalVar.SCAN_BATTERY_RESTART_POS)
            if GlobalVar.battery_color == "WHITE":
                left_arm.palce_object_recycle_area(GlobalVar.RECYCLE_WHITE_AREA)
                # print("True" + GlobalVar.battery_color)
            elif GlobalVar.battery_color == "RED":
                left_arm.palce_object_recycle_area(GlobalVar.RECYCLE_RED_AREA)
                # print("True" + GlobalVar.battery_color)
            else:
                left_arm.palce_object_recycle_area(GlobalVar.RECYCLE_AREA)
                # print("True" + GlobalVar.battery_color)
            left_arm.move_to_idle_position()
            GlobalVar.CURRENT_WORK_INDEX = GlobalVar.CURRENT_WORK_INDEX + 1
            requests.post(GlobalVar.CLEAR_INSPECT_URL, {})
            if GlobalVar.PAUSE_AUTO_WORK.is_set():
                GlobalVar.PAUSE_AUTO_WORK.clear()
                rospy.loginfo(f"Pause work:{GlobalVar.CURRENT_WORK_INDEX}")
                return
    GlobalVar.TASK_IS_DONE = True
    print("TASK is DONE is RUNNING !!!!")
    GlobalVar.CURRENT_WORK_INDEX = 0

# print(left_arm.current_pos())

if __name__ == "__main__":
    left_arm = RobotRoutine(arm_ip = "169.254.128.18", hand_selected = HANDSELECT.LEFT.value)
    right_arm = RobotRoutine(arm_ip="169.254.128.19",hand_selected=HANDSELECT.RIGHT.value)


    right_grasp_hand = RightGraspHand(robot=right_arm.robot)
    right_grasp_arm = RightGraspArm(robot=right_arm.robot,
                              hand=right_grasp_hand)


    left_grasp_hand = LeftGraspHand(robot=left_arm.robot)
    left_grasp_arm = LeftGraspArm(robot=left_arm.robot,
                              hand = left_grasp_hand)

    right_grasp_arm.idle_position = GlobalVar.RIGHT_IDLE_POSITION
    left_grasp_arm.idle_position = GlobalVar.LEFT_IDLE_POSITION

    right_grasp_arm.move_to_idle_position()
    right_grasp_hand.move_open_status()

    left_grasp_arm.move_to_idle_position()
    left_grasp_hand.move_open_status()

    rospy.init_node("battery_sorting_controller", anonymous=True)
    rospy.Subscriber("/pick_command", PickCommand, functools.partial(pick_command_callback, 
                                                                     left_arm = left_grasp_arm, 
                                                                     right_arm = right_grasp_arm))
    rospy.Subscriber("/reset_pose", ReturnHome, functools.partial(return_to_home_callback, left_arm=left_grasp_arm, right_arm=right_grasp_arm))
    
    set_work_mode_service = rospy.Service('/set_work_mode', SetWorkMode, handle_set_work_mode)
    query_work_mode_service = rospy.Service('/query_work_mode', QueryWorkMode, handle_query_work_mode)
    start_work = rospy.Service('/start_work', StartWork, functools.partial(handle_start_work,left_arm=left_grasp_arm, right_arm=right_grasp_arm))
    suspend_work = rospy.Service('/suspend_work', SuspendWork,handle_pause_work)
    grasp_object_work = rospy.Service('/grasp_object', GraspObject, functools.partial(handle_grasp_object, left_arm=left_grasp_arm, right_arm=right_grasp_arm))
    check_task_status = rospy.Service('/check_task_status', CheckTaskStatus, handle_check_task_status)
    tap_card_service = rospy.Service('/tap_card', TapCard, functools.partial(handle_tap_card, right_arm=right_grasp_arm))
    wave_hand_service = rospy.Service('/wave_hand', WaveHand, functools.partial(handle_wave_hand, left_arm=left_grasp_arm, right_arm=right_grasp_arm))
    point_direction_service = rospy.Service('/point_direction', PointDirection, functools.partial(handle_point_direction, left_arm=left_grasp_arm, right_arm=right_grasp_arm))
    one_move_service = rospy.Service('/one_move', OneMove, functools.partial(handle_one_move, left_arm=left_grasp_arm, right_arm=right_grasp_arm))
    rospy.spin()

