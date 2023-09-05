from __future__ import annotations
from typing import TYPE_CHECKING, Union, TypeVar, Generic
from tkinter import ttk
from custom_actions.action._move_motor import MoveMotor_Goal
from rclpy.qos import QoSProfile
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, Int16, Int8
from std_srvs.srv import Empty as EmptySrv, Trigger
from geometry_msgs.msg import PoseStamped, Pose
from custom_msgs.msg import Statusword, MotorPosition, DigitalInputs, Float64Array, BoolArray
from custom_srvs.srv import SetString, Tension, SerialSend
from custom_actions.action import ExecuteSequence, MoveMotor, Empty as EmptyAction, PlanTetherbot
from threading import Thread, Event, Lock
from time import sleep
from .interface_queue import InterfaceQueue
from .tkinter_objects import TkBoolLabel, TkArrayLabel, TkButton, TkStringLabel, TkEntry, TkCancelButton, \
    TkActionStatusLabel, TkFloatLabel, TkOptionMenu, TkFloatEntry, TkPoseLabelFrame, TkPoseEntryFrame

if TYPE_CHECKING:
    from .window import Window

MsgType = TypeVar('MsgType')

SrvType = TypeVar('SrvType')
ResType = TypeVar('ResType')
ReqType = TypeVar('ReqType')

ActionType = TypeVar('ActionType')
GoalType = TypeVar('GoalType')
ResultType = TypeVar('ResultType')
FeedbackType = TypeVar('FeedbackType')


class TkSafeActionClient(ActionClient):
    '''
    Calling ActionClient.destroy inside a callback function of tkinter leads to "Segmentation fault (core dumped)" error.
    This class works around the issue by destroying the action within a one-shot timer which is executed by the nodes executor outside the tkinter callback.
    '''

    def __init__(self, *args, **kwargs):

        super().__init__(*args, **kwargs)

        self._node: Node
        self.tk_safe_destroy_timer = self._node.create_timer(0.1, self.tk_safe_destroy_timer_callback)
        self.tk_safe_destroy_timer.cancel()

    def tk_safe_destroy(self):

        self.tk_safe_destroy_timer.reset()
        
    def tk_safe_destroy_timer_callback(self):

        self.tk_safe_destroy_timer.cancel()
        self.destroy()

    def destroy(self):

        self._node.destroy_timer(self.tk_safe_destroy_timer)

        return super().destroy()


class ROSInterface():

    def __init__(self, master: Window):

        self.master = master
        self.master.interfaces.append(self)

    @property
    def queue(self) -> InterfaceQueue:

        return self.master.queue

    def destroy(self):

        pass


class MsgInterface(ROSInterface, Generic[MsgType]):

    def __init__(self, master: Window = None, msg_type: type[MsgType] = None, msg_name: str = '', qos_profile: Union[QoSProfile, int] = 1):

        super().__init__(master)

        self.subscription = self.master.node.create_subscription(msg_type, msg_name, self.subscription_callback, qos_profile)
        self.msg = msg_type()

    def subscription_callback(self, msg):
    
        self.msg = msg
        self.enqueue_message()

    def enqueue_message(self):

        pass

    def destroy(self):

        self.master.node.destroy_subscription(self.subscription)

        return super().destroy()


class BoolMsgInterface(MsgInterface[Bool]):

    def __init__(self, label: TkBoolLabel = None, **kwargs):

        super().__init__(msg_type=Bool, **kwargs)
        self.label = label

    def enqueue_message(self):

        self.queue.put(self.label, self.msg.data)


class StringMsgInterface(MsgInterface[String]):

    def __init__(self, label: TkStringLabel, **kwargs):

        super().__init__(msg_type=String, **kwargs)

        self.label = label

    def enqueue_message(self):

        self.queue.put(self.label, self.msg.data)


class Float64MsgInterface(MsgInterface[Float64]):

    def __init__(self, label: TkFloatLabel, **kwargs):

        super().__init__(msg_type=Float64, **kwargs)

        self.label = label

    def enqueue_message(self):

        self.queue.put(self.label, self.msg.data)


class Int8MsgInterface(MsgInterface[Int8]):

    def __init__(self, label: TkFloatLabel, **kwargs):

        super().__init__(msg_type=Int8, **kwargs)

        self.label = label

    def enqueue_message(self):

        self.queue.put(self.label, self.msg.data)


class Int16MsgInterface(MsgInterface[Int16]):

    def __init__(self, label: TkFloatLabel, **kwargs):

        super().__init__(msg_type=Int16, **kwargs)

        self.label = label

    def enqueue_message(self):

        self.queue.put(self.label, self.msg.data)


class BoolArrayMsgInterface(MsgInterface[BoolArray]):

    def __init__(self, label: TkArrayLabel, **kwargs):

        super().__init__(msg_type=BoolArray, **kwargs)

        self.label=label

    def enqueue_message(self):
        
        self.queue.put(self.label, self.msg.data)


class Float64ArrayMsgInterface(MsgInterface[Float64Array]):

    def __init__(self, label: TkArrayLabel, **kwargs):

        super().__init__(msg_type=Float64Array, **kwargs)

        self.label=label

    def enqueue_message(self):
        
        self.queue.put(self.label, self.msg.data)


class PoseMsgInterface(MsgInterface[Pose]):

    def __init__(self, label_frame: TkPoseLabelFrame, **kwargs):

        super().__init__(msg_type=PoseStamped, **kwargs)

        self.label_frame = label_frame

    def enqueue_message(self):
        
        self.queue.put(self.label_frame.pos_label, [self.msg.position.x, self.msg.position.y, self.msg.position.z])
        self.queue.put(self.label_frame.rot_label, [self.msg.orientation.w, self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z])
 

class PoseStampedMsgInterface(MsgInterface[PoseStamped]):

    def __init__(self, label_frame: TkPoseLabelFrame, **kwargs):

        super().__init__(msg_type=PoseStamped, **kwargs)

        self.label_frame = label_frame

    def enqueue_message(self):
        
        self.queue.put(self.label_frame.pos_label, [self.msg.pose.position.x, self.msg.pose.position.y, self.msg.pose.position.z])
        self.queue.put(self.label_frame.rot_label, [self.msg.pose.orientation.w, self.msg.pose.orientation.x, self.msg.pose.orientation.y, self.msg.pose.orientation.z])


class DigitalInputsMsgInterface(MsgInterface[DigitalInputs]):

    def __init__(self, positive_limit_switch_label: TkBoolLabel, negative_limit_switch_label: TkBoolLabel, homing_switch_label: TkBoolLabel, **kwargs):

        super().__init__(msg_type=DigitalInputs, **kwargs)

        self.positive_limit_switch_label = positive_limit_switch_label
        self.negative_limit_switch_label = negative_limit_switch_label
        self.homing_switch_label = homing_switch_label

    def enqueue_message(self):
        
        self.queue.put(self.negative_limit_switch_label, self.msg.negative_limit_switch)
        self.queue.put(self.positive_limit_switch_label, self.msg.positive_limit_switch)
        self.queue.put(self.homing_switch_label, self.msg.homing_switch)

class MotorPositionInterface(MsgInterface[MotorPosition]):
    
    def __init__(self, target_position_label: TkFloatLabel, actual_position_label: TkFloatLabel, **kwargs):

        super().__init__(msg_type=MotorPosition, **kwargs)

        self.target_position_label = target_position_label
        self.actual_position_label = actual_position_label

    def enqueue_message(self):
        
        self.queue.put(self.target_position_label, self.msg.target_position)
        self.queue.put(self.actual_position_label, self.msg.actual_position)


class StatuswordMsgInterface(MsgInterface[Statusword]):

    def __init__(self, 
                 ready_to_switch_on_label: TkBoolLabel = None, 
                 switched_on_label: TkBoolLabel = None,
                 voltage_enabled_label: TkBoolLabel = None,
                 operation_enabled_label: TkBoolLabel = None,
                 target_reached_label: TkBoolLabel = None,
                 quick_stop_label: TkBoolLabel = None,
                 switch_on_disabled_label: TkBoolLabel = None,
                 setpoint_acknowledge_label: TkBoolLabel = None,
                 internal_limit_active_label: TkBoolLabel = None,
                 deviation_error_label: TkBoolLabel = None,
                 fault_label: TkBoolLabel = None,
                 warning_label: TkBoolLabel = None, **kwargs):
        
        super().__init__(msg_type=Statusword, **kwargs)

        self.ready_to_switch_on_label = ready_to_switch_on_label
        self.switched_on_label = switched_on_label
        self.voltage_enabled_label = voltage_enabled_label
        self.operation_enabled_label = operation_enabled_label
        self.target_reached_label = target_reached_label
        self.quick_stop_label = quick_stop_label
        self.switch_on_disabled_label = switch_on_disabled_label
        self.setpoint_acknowledge_label = setpoint_acknowledge_label
        self.internal_limit_active_label = internal_limit_active_label
        self.deviation_error_label = deviation_error_label
        self.fault_label = fault_label
        self.warning_label = warning_label

    def enqueue_message(self):

        self.queue.put(self.ready_to_switch_on_label, self.msg.ready_to_switch_on)
        self.queue.put(self.switched_on_label, self.msg.switched_on)
        self.queue.put(self.voltage_enabled_label, self.msg.voltage_enabled)
        self.queue.put(self.operation_enabled_label, self.msg.operation_enabled)
        self.queue.put(self.target_reached_label, self.msg.target_reached)
        self.queue.put(self.quick_stop_label, self.msg.quick_stop)
        self.queue.put(self.switch_on_disabled_label, self.msg.switch_on_disabled)
        self.queue.put(self.setpoint_acknowledge_label, self.msg.setpoint_acknowledge_or_speed_or_homing_attained)
        self.queue.put(self.internal_limit_active_label, self.msg.internal_limit_active)
        self.queue.put(self.deviation_error_label, self.msg.deviation_error)
        self.queue.put(self.fault_label, self.msg.fault)
        self.queue.put(self.warning_label, self.msg.warning)


class SrvInterface(ROSInterface, Generic[SrvType, ReqType, ResType]):

    def __init__(self, master: Window = None, srv_type: type[SrvType] = None, srv_name: str = '', button: TkButton = None):

        super().__init__(master)

        self.srv_type = srv_type
        self.srv_name = srv_name
        self.button = button
        self.stop_event = Event()
        self.thread: Thread = None
        self.lock = Lock()
        self.timeout = 5

        self.button.configure(command=self.button_callback)

    def get_request(self) -> ReqType:

        return None
    
    def enqueue_response(self, response: ResType):

        pass

    def button_callback(self):

        with self.lock:
            if self.thread is not None:
                if self.thread.is_alive():
                    self.stop_event.set()
                    self.thread.join()
            self.stop_event.clear()
            self.thread = Thread(target=self.execute_service, args=(self.get_request(),))
            self.thread.start()

    def execute_service(self, request):

        state = 0
        while True:
            # create client
            if state == 0:
                self.enqueue_response(None)
                start_time = self.master.node.get_clock().now().seconds_nanoseconds()[0]
                cli = self.master.node.create_client(self.srv_type, self.srv_name)
                state = 1
            # send goal
            elif state == 1:
                if cli.service_is_ready():
                    future = cli.call_async(request)
                    state = 2
                else:
                    state = 1
            # wait for result
            elif state == 2:
                if future.done():
                    self.enqueue_response(future.result())
                    state = 99
                else:
                    state = 2
            # clean up
            elif state == 99:
                self.master.node.destroy_client(cli)
                break
            # exit conditions
            if self.stop_event.is_set():
                state = 99
            if self.master.node.get_clock().now().seconds_nanoseconds()[0] - start_time > self.timeout:
                self.master.get_logger().error('Call of service ' + self.srv_name + ' timed out')
                state = 99
            sleep(0.1)
            
    def destroy(self):

        if self.thread is not None:
            if self.thread.is_alive():
                self.stop_event.set()
                self.thread.join()

        return super().destroy()


class EmptySrvInterface(SrvInterface[EmptySrv, EmptySrv.Request, EmptySrv.Response]):

    def __init__(self, **kwargs):

        super().__init__(srv_type=EmptySrv, **kwargs)

    def get_request(self) -> EmptySrv.Request:
        
        return EmptySrv.Request()


class TriggerSrvInterface(SrvInterface[Trigger, Trigger.Request, Trigger.Response]):

    def __init__(self, success_label: TkBoolLabel = None, message_label: TkStringLabel = None, **kwargs):
        
        super().__init__(srv_type=Trigger, **kwargs)

        self.success_label = success_label
        self.message_label = message_label

    def get_request(self) -> Trigger.Request:
        
        return Trigger.Request()
    
    def enqueue_response(self, response: Trigger.Response) -> None:
        
        if response is not None:
            self.queue.put(self.success_label, response.success)
            if self.message_label is not None:
                self.queue.put(self.message_label, response.message)
        else:
            self.queue.put(self.success_label, None)
            if self.message_label is not None:
                self.queue.put(self.message_label, '')


class SetStringSrvInterface(SrvInterface[SetString, SetString.Request, SetString.Response]):

    def __init__(self, entry: Union[TkEntry, TkOptionMenu] = None, **kwargs):

        super().__init__(srv_type=SetString, **kwargs) 
        
        self.entry = entry

    def get_request(self) -> SetString.Request:

        request = SetString.Request()
        request.data = self.entry.get_data()
        
        return request
    

class SerialSendSrvInterface(SrvInterface[SerialSend, SerialSend.Request, SerialSend.Response]):

    def __init__(self, entry: Union[TkEntry, TkOptionMenu] = None, success_label: TkBoolLabel = None, **kwargs):

        super().__init__(srv_type=SerialSend, **kwargs) 
        
        self.entry = entry
        self.success_label = success_label

    def get_request(self) -> SerialSend.Request:

        request = SerialSend.Request()
        request.msg = self.entry.get_data()

    def enqueue_response(self, response: SerialSend.Response) -> None:
        
        self.queue.put(self.success_label, response.success)
    

class TensionSrvInterface(SrvInterface[Tension, Tension.Request, Tension.Response]):

    def __init__(self, option_menu: TkOptionMenu = None, success_label: TkBoolLabel = None, tension_value: bool = False, **kwargs):

        super().__init__(srv_type=Tension, **kwargs)

        self.option_menu = option_menu
        self.success_label = success_label
        self.tension_value = tension_value

    def get_request(self) -> Tension.Request:

        request = Tension.Request()
        request.gripper_index = self.option_menu.get_index()
        request.value = self.tension_value

        return request
    
    def enqueue_response(self, response: Tension.Response) -> None:

        self.queue.put(self.success_label, response.success)
    

class ActionInterface(ROSInterface, Generic[ActionType, GoalType, ResultType, FeedbackType]):

    def __init__(self, master: Window = None, action_type: type[ActionType] = None, action_name: str = '', execute_button: TkButton = None, cancel_button: TkCancelButton = None, status_label: TkActionStatusLabel = None):

        super().__init__(master)

        self.action_type = action_type
        self.action_name = action_name
        self.status_label = status_label
        self.execute_button = execute_button
        self.cancel_button = cancel_button
        self.timeout = 5
        self.stop_event = Event()
        self.thread: Thread = None
        self.lock = Lock()

        self.execute_button.configure(command=self.execute_button_callback)
        self.cancel_button.configure(command=self.cancel_button_callback)

    def get_goal(self) -> GoalType:

        return None
    
    def enqueue_result(self, result: ResultType) -> None:

        pass

    def enqueue_feedback(self, feedback: FeedbackType) -> None:

        pass

    def enqueue_status(self, status: int) -> None:

        self.queue.put(self.status_label, status)

    def execute_button_callback(self):

        with self.lock:
            if self.thread is not None:
                if self.thread.is_alive():
                    self.stop_event.set()
                    self.thread.join()
            self.stop_event.clear()
            self.thread = Thread(target=self.execute_action, args=(self.get_goal(),))
            self.thread.start()

    def cancel_button_callback(self):

        with self.lock:
            self.stop_event.set()

    def feedback_callback(self, msg):

        self.enqueue_feedback(msg.feedback)

    def execute_action(self, goal):

        state = 0
        while True:
            # create client
            if state == 0:
                self.enqueue_result(None)
                start_time = self.master.node.get_clock().now().seconds_nanoseconds()[0]
                cli = TkSafeActionClient(self.master.node, self.action_type, self.action_name)
                goal_handle: ClientGoalHandle = None
                send_goal_future: Future = None
                get_result_future: Future = None
                cancel_goal_future: Future = None
                cancelling = False
                state = 1
            # send goal
            elif state == 1:
                if cli.server_is_ready():
                    send_goal_future = cli.send_goal_async(goal, feedback_callback=self.feedback_callback)
                    state = 2
                elif self.master.node.get_clock().now().seconds_nanoseconds()[0] - start_time > self.timeout:
                    self.master.get_logger().error('Send goal of action ' + self.action_name + ' timed out')
                    state = 96
                else:
                    state = 1
            # wait for send goal, get result
            elif state == 2:
                if send_goal_future.done():
                    goal_handle = send_goal_future.result()
                    get_result_future = goal_handle.get_result_async()
                    state = 3
                else:
                    state = 2
            # wait for get result
            elif state == 3:
                if get_result_future.done():
                    self.enqueue_result(get_result_future.result().result)
                    state = 99
                else:
                    state = 3
            # cancel
            elif state == 96:
                cancelling = True
                if send_goal_future is not None:
                    send_goal_future.cancel()
                if goal_handle is not None:
                    cancel_goal_future = goal_handle.cancel_goal_async()
                    state = 97
                else:
                    state = 98
            # wait for cancel
            elif state == 97:
                if cancel_goal_future.done():
                    state = 98
                else:
                    state = 97
            # do another iteration to update status
            elif state == 98:
                state = 99
            # clean up
            elif state == 99:
                cli.tk_safe_destroy()
                break
            # exit conditions
            if self.stop_event.is_set() and not cancelling:
                state = 96
            if goal_handle is not None:
                self.enqueue_status(goal_handle.status)
            sleep(0.1)
    
    def destroy(self):
        if self.thread is not None:
            if self.thread.is_alive():
                self.stop_event.set()
                self.thread.join()   
        return super().destroy()


class EmptyActionInterface(ActionInterface[EmptyAction, EmptyAction.Goal, EmptyAction.Result, EmptyAction.Feedback]):

    def __init__(self, **kwargs):

        super().__init__(action_type=EmptyAction, **kwargs)

    def get_goal(self) -> EmptyAction.Goal:

        return EmptyAction.Goal()


class ExecuteSequenceActionInterface(ActionInterface[ExecuteSequence, ExecuteSequence.Goal, ExecuteSequence.Result, ExecuteSequence.Feedback]):

    def __init__(self, feedback_label: TkStringLabel = None, **kwargs):
        
        super().__init__(action_type=ExecuteSequence, **kwargs)

        self.feedback_label = feedback_label

    def get_goal(self) -> ExecuteSequence.Goal:
        
        return ExecuteSequence.Goal()
    
    def enqueue_feedback(self, feedback: ExecuteSequence.Feedback) -> None:
        
        self.queue.put(self.feedback_label, str(feedback.current) + ' of ' + str(feedback.length))


class MoveMotorActionInterface(ActionInterface[MoveMotor, MoveMotor.Goal, MoveMotor.Result, MoveMotor.Feedback]):

    def __init__(self, 
                 mode_menu: TkOptionMenu = None, 
                 profile_acceleration_entry: TkFloatEntry = None, 
                 profile_deceleration_entry: TkFloatEntry = None,
                 target_position_entry: TkFloatEntry = None,
                 profile_velocity_entry: TkFloatEntry = None, **kwargs):
        
        super().__init__(action_type=MoveMotor, **kwargs)

        self.mode_menu = mode_menu
        self.profile_acceleration_entry = profile_acceleration_entry
        self.profile_deceleration_entry = profile_deceleration_entry
        self.target_position_entry = target_position_entry
        self.profile_velocity_entry = profile_velocity_entry

    def get_goal(self) -> MoveMotor_Goal:
        
        goal = MoveMotor.Goal()

        mode = self.mode_menu.get_data()

        if mode == 'Home':
            goal.mode = int(1)
        elif mode == 'Position (abs.)':
            goal.mode = int(0)
            goal.absolute_relative = False
            goal.profile_acceleration = self.profile_acceleration_entry.get_data()
            goal.profile_deceleration = self.profile_deceleration_entry.get_data()
            goal.profile_velocity = self.profile_velocity_entry.get_data()
            goal.target_position = self.target_position_entry.get_data()
        elif mode == 'Position (rel.)':
            goal.mode = int(0)
            goal.absolute_relative = True
            goal.profile_acceleration = self.profile_acceleration_entry.get_data()
            goal.profile_deceleration = self.profile_deceleration_entry.get_data()
            goal.profile_velocity = self.profile_velocity_entry.get_data()
            goal.target_position = self.target_position_entry.get_data()
        elif mode == 'Home*':
            goal.mode = int(3)
        elif mode == 'CSP':
            goal.mode = int(2)

        return goal

class PlanTetherbotActionInterface(ActionInterface[PlanTetherbot, PlanTetherbot.Goal, PlanTetherbot.Result, PlanTetherbot.Feedback]):

    def __init__(self, 
                 notebook: ttk.Notebook = None, 
                 feedback_label: TkStringLabel = None,
                 platform_pose_entry_frame: TkPoseEntryFrame = None, 
                 arm_pose_entry_frame: TkPoseEntryFrame = None,
                 pick_gripper_menu: TkOptionMenu = None,
                 place_hold_menu: TkOptionMenu = None,
                 pick_and_place_gripper_menu: TkOptionMenu = None,
                 pick_and_place_hold_menu: TkOptionMenu = None,
                 to_configuration_gripper_menu: TkOptionMenu = None,
                 global_hold_menus: list[TkOptionMenu] = None,
                 **kwargs):
        
        super().__init__(action_type=PlanTetherbot, **kwargs)

        self.notebook = notebook
        self.feedback_label = feedback_label
        self.platform_pose_entry_frame = platform_pose_entry_frame
        self.arm_pose_entry_frame = arm_pose_entry_frame
        self.pick_gripper_menu = pick_gripper_menu
        self.place_hold_menu = place_hold_menu
        self.pick_and_place_gripper_menu = pick_and_place_gripper_menu
        self.pick_and_place_hold_menu = pick_and_place_hold_menu
        self.to_configuration_gripper_menu = to_configuration_gripper_menu
        self.global_hold_menus = global_hold_menus

    def get_goal(self) -> PlanTetherbot.Goal:

        goal = PlanTetherbot.Goal()

        i = self.notebook.index(self.notebook.select())

        if i == 0: # platform to pose
            goal.mode = 0
            goal.goal_pose = self.pose2list(self.platform_pose_entry_frame.get_data())
        elif i == 1: # arm to pose
            goal.mode = 1
            goal.goal_pose = self.pose2list(self.arm_pose_entry_frame.get_data())
        elif i == 2: # pick gripper
            goal.mode = 2
            goal.gripper_index = self.pick_gripper_menu.get_index()
        elif i == 3: # place gripper
            goal.mode = 3
            goal.hold_index = self.place_hold_menu.get_index()
        elif i == 4: # pick and place gripper
            goal.mode = 4
            goal.gripper_index = self.pick_and_place_gripper_menu.get_index()
            goal.hold_index = self.pick_and_place_hold_menu.get_index()
        elif i == 5: # to configuration
            goal.mode = 5
            goal.gripper_index = self.to_configuration_gripper_menu.get_index()
        elif i == 6: # global path planning
            goal.mode = 6
            goal.goal_configuration = [menu.get_index() for menu in self.global_hold_menus]
        return goal

    def enqueue_feedback(self, feedback: PlanTetherbot.Feedback) -> None:

        self.queue.put(self.feedback_label, feedback.message)

    def pose2list(self, pose: Pose) -> list:

        return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]