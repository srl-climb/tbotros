from __future__ import annotations
import tkinter as tk
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, Empty as EmptyService
from custom_msgs.msg import Float64Array, BoolArray
from custom_srvs.srv import SetString, Tension
from custom_actions.action import Empty as EmptyAction
from geometry_msgs.msg import Pose, PoseStamped
from .window import Window
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App


class PlatformWindow(Window):

    def __init__(self, parent: App, title: str = 'Platform Window'):

        super().__init__(parent, title)

    def create_user_interface(self):

        self._gripper_names = [gripper.name for gripper in self.parent.tbot.grippers]

        self._actual_pose_node = self.create_subscriber_node(msg_type = PoseStamped, 
                                                             msg_name = self.parent.tbot.platform.name + '/platform_state_publisher/pose')
        self._target_pose_node = self.create_subscriber_node(msg_type = PoseStamped, 
                                                             msg_name = self.parent.tbot.platform.name + '/platform_controller/target_pose')
        self._joint_states_node = self.create_subscriber_node(msg_type = Float64Array, 
                                                              msg_name = self.parent.tbot.platform.name + '/platform_state_publisher/joint_states')
        self._control_enabled_node = self.create_subscriber_node(msg_type = Bool, 
                                                                 msg_name = self.parent.tbot.platform.name + '/platform_controller/control_enabled')
        self._transform_source_node = self.create_subscriber_node(msg_type = String,
                                                                  msg_name = self.parent.tbot.platform.name + '/platform_state_publisher/transform_source')
        self._tether_tension_node = self.create_subscriber_node(msg_type = BoolArray,
                                                                msg_name = self.parent.tbot.platform.name + '/platform_controller/tether_tension')
        self._enable_control_node = self.create_client_node(srv_type = EmptyService, enable_response = False,
                                                            srv_name = self.parent.tbot.platform.name + '/platform_controller/enable_control')
        self._disable_control_node = self.create_client_node(srv_type = EmptyService, enable_response = False, 
                                                             srv_name = self.parent.tbot.platform.name + '/platform_controller/disable_control')
        self._set_transform_source_node = self.create_client_node(srv_type = SetString, enable_response = False,
                                                                  srv_name = self.parent.tbot.platform.name + '/platform_state_publisher/set_transform_source')
        self._calibrate_zed_pose_node = self.create_client_node(srv_type = Trigger,
                                                                srv_name = self.parent.tbot.platform.name + '/platform_state_publisher/calibrate_zed_pose')
        self._tension_tethers_node = self.create_client_node(srv_type = Tension,
                                                                    srv_name = self.parent.tbot.platform.name + '/platform_controller/tension_gripper_tethers')
        self._calibrate_tether_lengths_node = self.create_action_client_node(action_type = EmptyAction,
                                                                             action_name = self.parent.tbot.platform.name + '/platform_controller/calibrate_tether_lengths')
        
        state_frame = self.create_label_frame(master = self, text = 'State')
        state_frame.grid(row = 0, column = 0)

        self._actual_pose_label_frame = self.create_pose_label_frame(master = state_frame, text = 'Actual Pose:')
        self._actual_pose_label_frame.grid(row = 0, column = 0, columnspan = 2)
        self._target_pose_label_frame = self.create_pose_label_frame(master = state_frame, text = 'Target Pose:')
        self._target_pose_label_frame.grid(row = 1, column = 0, columnspan = 2)

        label = self.create_label(master = state_frame, text = 'Joint States:')
        label.grid(row = 2, column = 0)
        self._joint_states_label = self.create_vector_label(length = self.parent.tbot.platform.m, digits = 3, master = state_frame)
        self._joint_states_label.grid(row = 2, column = 1)

        label = self.create_label(master = state_frame, text = 'Control Enabled:')
        label.grid(row = 3, column = 0)
        self._control_enabled_label = self.create_bool_label(master = state_frame)
        self._control_enabled_label.grid(row = 3, column = 1)

        label = self.create_label(master = state_frame, text = 'Transform Source:')
        label.grid(row = 4, column = 0)
        self._transform_source_label = self.create_string_label(master = state_frame)
        self._transform_source_label.grid(row = 4, column = 1)

        label = self.create_label(master = state_frame, text = 'Tether Tension:')
        label.grid(row = 5, column = 0)
        self._tether_tension_label = self.create_vector_label(master = state_frame, length = self.parent.tbot.platform.m, digits = 0)
        self._tether_tension_label.grid(row = 5, column = 1)

        service_frame = self.create_label_frame(master = self, text = 'Services')
        service_frame.grid(row = 1, column = 0)

        self._enable_control_button = self.create_button(master = service_frame, text = 'Enable Control', command = self.enable_control_button_callback)
        self._enable_control_button.grid(row = 0, column = 0, columnspan = 2)
        
        self._disable_control_button = self.create_button(master = service_frame, text = 'Disable Control', command = self.disable_control_button_callback)
        self._disable_control_button.grid(row = 1, column = 0, columnspan = 2)
        
        label = self.create_label(master = service_frame, text = 'Transform Source:')
        label.grid(row = 2, column = 0)
        self._set_transform_source_optionmenu = self.create_option_menu(master = service_frame, values = ['optitrack', 'zed', 'fwk'])
        self._set_transform_source_optionmenu.grid(row = 2, column = 1)
        self._set_transform_source_button = self.create_button(master = service_frame, text = 'Set Transform Source', command = self.set_transform_source_button_callback)
        self._set_transform_source_button.grid(row = 3, column = 0, columnspan = 2)

        label = self.create_label(master = service_frame, text = 'Gripper Name:')
        label.grid(row = 4, column = 0)
        self._tension_tethers_optionmenu = self.create_option_menu(master = service_frame, values = self._gripper_names)
        self._tension_tethers_optionmenu.grid(row = 4, column = 1)
        self._tension_tethers_button = self.create_button(master = service_frame, text = 'Tension Tethers', command = self.tension_tethers_button_callback)
        self._tension_tethers_button.grid(row = 5, column = 0, columnspan = 2)
        self._tension_tethers_button = self.create_button(master = service_frame, text = 'Untension Tethers', command = self.untension_tethers_button_callback)
        self._tension_tethers_button.grid(row = 6, column = 0, columnspan = 2)

        self._calibrate_zed_pose_button = self.create_button(master = service_frame, text = 'Calibrate ZED Pose')
        self._calibrate_zed_pose_button.grid(row = 7, column = 0, columnspan = 2)

        label = self.create_label(master = service_frame, text = 'Status:')
        label.grid(row = 8, column = 0)
        self._success_label = self.create_bool_label(master = service_frame)
        self._success_label.grid(row = 8, column = 1)

        action_frame = self.create_label_frame(master = self, text = 'Actions')
        action_frame.grid(row = 2, column = 0)

        button = self.create_button(master = action_frame, text = 'Calibrate Tether Lengths', command = self.calibrate_tether_lengths_button_callback)
        button.grid(row = 0, column = 0, columnspan = 2)
        button = self.create_cancel_button(master = action_frame, command = self.cancel_button_callback)
        button.grid(row = 1, column = 0, columnspan = 2)
        label = self.create_label(master = action_frame, text = 'Status:')
        label.grid(row = 2, column = 0)
        self._status_label = self.create_action_status_label(master = action_frame)
        self._status_label.grid(row = 2, column = 1)

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)

    def timer_callback(self):

        if not self._actual_pose_node.msg_queue.empty():
            msg: PoseStamped = self._actual_pose_node.msg_queue.get()
            self._actual_pose_label_frame.update_data(msg.pose)
        if not self._target_pose_node.msg_queue.empty():
            msg: PoseStamped = self._target_pose_node.msg_queue.get()
            self._target_pose_label_frame.update_data(msg.pose)
        if not self._joint_states_node.msg_queue.empty():
            msg: Float64Array = self._joint_states_node.msg_queue.get()
            self._joint_states_label.update_data(msg.data)
        if not self._control_enabled_node.msg_queue.empty():
            msg: Bool = self._control_enabled_node.msg_queue.get()
            self._control_enabled_label.update_data(msg.data)
        if not self._transform_source_node.msg_queue.empty():
            msg: String = self._transform_source_node.msg_queue.get()
            self._transform_source_label.update_data(msg.data)
        if not self._tether_tension_node.msg_queue.empty():
            msg: BoolArray = self._tether_tension_node.msg_queue.get()
            self._tether_tension_label.update_data(msg.data)
        if not self._tension_tethers_node.res_queue.empty():
            res: Tension.Response = self._tension_tethers_node.res_queue.get()
            self._success_label.update_data(res.success)
        if not self._calibrate_zed_pose_node.res_queue.empty():
            res: Trigger.Response = self._calibrate_zed_pose_node.res_queue.get()
            self._success_label.update_data(res.success)
        if not self._calibrate_tether_lengths_node.status_queue.empty():
            self._status_label.update_data(self._calibrate_tether_lengths_node.status_queue.get())
        
    def enable_control_button_callback(self):

        self._enable_control_node.req_queue.put(EmptyService.Request())

    def disable_control_button_callback(self):

        self._disable_control_node.req_queue.put(EmptyService.Request())

    def set_transform_source_button_callback(self):

        request = SetString.Request()
        request.data = self._set_transform_source_optionmenu.variable.get()

        self._set_transform_source_node.req_queue.put(request)

    def tension_tethers_button_callback(self):

        self._success_label.update_data('None')

        request = Tension.Request()
        request.gripper_index = self._gripper_names.index(self._tension_tethers_optionmenu.variable.get())
        request.value = True
        self._tension_tethers_node.req_queue.put(request)

    def untension_tethers_button_callback(self):

        self._success_label.update_data('None')

        request = Tension.Request()
        request.gripper_index = self._gripper_names.index(self._tension_tethers_optionmenu.variable.get())
        request.value = False
        self._tension_tethers_node.req_queue.put(request)

    def calibrate_zed_pose_button_callback(self):

        self._success_label.update_data('None')

        self._calibrate_zed_pose_node.req_queue.put(Trigger.Request())

    def calibrate_tether_lengths_button_callback(self):

        self._calibrate_tether_lengths_node.goal_queue.put(EmptyAction.Goal())

    def cancel_button_callback(self):

        self._calibrate_tether_lengths_node.cancel_event.set()

