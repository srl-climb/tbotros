from __future__ import annotations
import tkinter as tk
import numpy as np
import cv_bridge
from PIL import Image as PILImage, ImageTk as PILImageTk
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from custom_srvs.srv import CalibrateCameraTransform
from geometry_msgs.msg import PoseStamped
from .interface_nodes import ClientNode, SubscriptionNode
from .tkinter_objects import TkLabel, TkPoseLabelFrame, TkBoolLabel
from .window import Window
from .image_resize import image_resize
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tetherbot_gui import App

    
class ArucoDetectorWindow(Window):

    def __init__(self, parent: App, title: str = 'Aruco Detector Window'):
        
        self._camera_names = ['camera0', 'camera1', 'camera2', 'camera3', 'camera4']
        self._image_width = 420
        self._image_height = int(2160 * self._image_width / 3840)

        super().__init__(parent, title)

    def create_user_interface(self):

        self._cv_bridge = cv_bridge.CvBridge()

        self._video_nodes: list[SubscriptionNode] = []
        self._pose_nodes: list[SubscriptionNode] = []
        self._detected_nodes: list[SubscriptionNode] = []
        self._calibrate_nodes: list[ClientNode] = []
        self._image_labels: list[TkLabel] = []
        self._pose_labelframes: list[TkPoseLabelFrame] = []
        self._detected_labels: list[TkBoolLabel] = []
        self._success_labels: list[TkBoolLabel] = []

        i = 0 # colum
        j = 0 # row
        k = 0 # counter
        for name in self._camera_names:
            if i % 3 == 0:
                i = 0
                j = j+1

            self._video_nodes.append(
                self.create_subscriber_node(msg_name = name + '/aruco_detector/image_markers', msg_type = Image))
            self._pose_nodes.append(
                self.create_subscriber_node(msg_name = name + '/aruco_detector/marker_pose', msg_type = PoseStamped))
            self._detected_nodes.append(
                self.create_subscriber_node(msg_name = name + '/aruco_detector/detected', msg_type = Bool))
            self._calibrate_nodes.append(
                self.create_client_node(srv_name = name + '/aruco_detector/calibrate_camera_transform', srv_type = CalibrateCameraTransform))
            
            frame = self.create_label_frame(master = self, text = 'Camera ' + str(k) + ' (' + name + ')')
            frame.grid(column = i, row = j)

            state_frame = self.create_label_frame(master = frame, text = 'State')
            state_frame.grid(column = 0, row = 0)

            labelframe = self.create_pose_label_frame(master = state_frame)
            labelframe.grid(column = 0, row = 0, columnspan = 2)
            self._pose_labelframes.append(labelframe)

            label = self.create_label(master = state_frame, text = 'Detected: ')
            label.grid(column = 0, row = 1)
            label = self.create_bool_label(master = state_frame)
            label.grid(column = 1, row = 1)
            self._detected_labels.append(label)

            service_frame = self.create_label_frame(master = frame, text = 'Services')
            service_frame.grid(column = 1, row = 0)

            label = self.create_label(master = service_frame, text = 'Success: ')
            label.grid(column = 0, row = 1)
            label = self.create_bool_label(master = service_frame)
            label.grid(column = 1, row = 1)
            self._success_labels.append(label)

            command = lambda node = self._calibrate_nodes[-1], label = self._success_labels[-1]: self.calibrate_button_callback(node, label)
            button = self.create_button(master = service_frame, text = 'Calibrate Camera Transform', command = command)
            button.grid(row = 0, column = 0, columnspan = 2)

            video_frame = self.create_label_frame(master = frame, text = 'Image')
            video_frame.grid(column = 0, row = 1, columnspan = 2)
            label = self.create_label(master = video_frame)
            label.grid(row = 1, column = 0)
            self.draw_image(label)
            self._image_labels.append(label)

            i = i + 1
            k = k + 1

        self.create_timer(callback = self.timer_callback, timeout_ms = 100)
        self.create_timer(callback = self.video_timer, timeout_ms=100)

    def calibrate_button_callback(self, node: ClientNode, label: TkBoolLabel):

        node.req_queue.put(CalibrateCameraTransform.Request())
        label.update_data('None')

    def timer_callback(self):

        for i in range(len(self._camera_names)):
            if not self._detected_nodes[i].msg_queue.empty():
                self._detected_labels[i].update_data(self._detected_nodes[i].msg_queue.get().data)
            if not self._pose_nodes[i].msg_queue.empty():
                self._pose_labelframes[i].update_data(self._pose_nodes[i].msg_queue.get().pose)
            if not self._calibrate_nodes[i].res_queue.empty():
                self._success_labels[i].update_data(self._calibrate_nodes[i].res_queue.get().success)

    def video_timer(self):

        for i in range(len(self._camera_names)):
            if not self._video_nodes[i].msg_queue.empty():
                self.draw_image(self._image_labels[i], self._video_nodes[i].msg_queue.get())
    
    def draw_image(self, label: TkLabel, img: Image = None):
        
        if img is None:
            cv2img = np.zeros((self._image_height, self._image_width, 3), dtype = np.uint8)
        else:
            cv2img = self._cv_bridge.imgmsg_to_cv2(img)

        cv2img = image_resize(cv2img, width = self._image_width)
        tkimg  = PILImageTk.PhotoImage(image = PILImage.fromarray(cv2img))
        label.imgtk = tkimg
        label.configure(image = tkimg)


