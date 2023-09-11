from __future__ import annotations
import rclpy
import time
from rclpy.client import Client
from rclpy.action import ActionClient
from custom_actions.action import Empty as EmptyAction, MoveTetherbot
from custom_srvs.srv import SetString, Tension
from abc import ABC, abstractmethod
from tbotlib import TransformMatrix
from typing import TypeVar, Generic

ResType = TypeVar('ResType')
ReqType = TypeVar('ReqType')
GoalType = TypeVar('GoalType')

class TbHandler(ABC):

    # status values
    # Note: Values equivalent to ROS action status
    UNKNOWN = 0
    EXECUTING = 2
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6

    def __init__(self) -> None:
        
        super().__init__()

        self._status = self.UNKNOWN
        self._done = True

    @abstractmethod
    def execute(self) -> tuple[bool, int]:

        return self._done , self._status

    @abstractmethod
    def cancel(self) -> None:
        
        pass

    @abstractmethod
    def info(self) -> str:

        pass


class TbDelayHandler(TbHandler):

    def __init__(self, delay: float):

        super().__init__()

        self._delay = delay
        self._state = 0
        self._start_time = 0

    def execute(self) -> tuple[bool, int]:

        if self._state == 0:
            self._status = self.EXECUTING
            self._done = False
            self._start_time = time.time()
            self._state = 1
        elif self._state == 1:
            if time.time() - self._start_time >= self._delay:
                self._status = self.SUCCEEDED
                self._state = 99
            else:
                self._state = 1
        elif self._state == 99:
            self._state = 0
            self._done = True

        return self._done, self._status
    
    def cancel(self) -> None:

        pass

    def info(self) -> str:

        return 'delay ' + str(self._delay) + 's'


class TbServiceHandler(TbHandler, Generic[ReqType, ResType]):

    def __init__(self, client: Client):

        super().__init__()

        self._client = client
        self._future: rclpy.Future = None
        self._state = 0

    def get_request(self) -> ResType:

        pass

    def eval_response(self, response: ResType) -> int:

        return self.SUCCEEDED

    def execute(self) -> None:
        # internal state machine

        # wait for service, send request
        if self._state == 0:
            self._status = self.EXECUTING
            self._done = False
            if self._client.service_is_ready():
                self._future = self._client.call_async(self.get_request())
                self._state = 1
            else:
                self._state = 0

        # wait for results
        elif self._state == 1:
            if self._future.done():
                self._status = self.eval_response(self._future.result())
                self._state = 99
            else:
                self._state = 1

        #clean up and return result status
        elif self._state == 99:
            self._state = 0
            self._done = True
            self._send_future = None

        return self._done, self._status

    def cancel(self):
        if self._future:
            self._future.cancel()

    def info(self) -> str:

        return 'service ' + self._client.srv_name


class TbSetStringServiceHandler(TbServiceHandler[SetString.Request, SetString.Response]):

    def __init__(self, client: Client, data: str):

        super().__init__(client)

        self.data = data

    def get_request(self) -> SetString.Request:

        request = SetString.Request()
        request.data = self.data

        return request
    
    def eval_response(self, response: SetString.Response) -> int:

        return self.SUCCEEDED


class TbTensionServiceHandler(TbServiceHandler[Tension.Request, Tension.Response]):

    def __init__(self, client: Client, gripper_index: int, value: bool):

        super().__init__(client)

        self.value = value
        self.gripper_index = gripper_index

    def get_request(self) -> Tension.Request:
        
        request = Tension.Request()
        request.gripper_index = int(self.gripper_index)
        request.value = bool(self.value)

        return request
    
    def eval_response(self, response: Tension.Response) -> int:

        if response.success:
            return self.SUCCEEDED
        else:
            return self.ABORTED
    

class TbActionHandler(TbHandler, Generic[GoalType]):

    def __init__(self, client: ActionClient):

        super().__init__()

        self._client = client
        self._send_future: rclpy.Future = None
        self._state = 0

    def get_goal(self) -> GoalType:

        return

    def execute(self) -> int:
        # internal state machine

        # wait for server, send goal
        if self._state == 0:
            self._status = self.EXECUTING
            self._done = False
            if self._client.server_is_ready():
                self._send_future = self._client.send_goal_async(self.get_goal())
                self._state = 1
            else:
                self._state = 0

        # wait for goal to arrive
        elif self._state == 1:
            if self._send_future.done():
                self._state = 2
            else:
                self._state = 1

        # wait for action to finish
        elif self._state == 2:
            if self._send_future.result().status >= self.SUCCEEDED:  
                self._state = 99
            else:
                self._state = 2

        #clean up and return result status
        elif self._state == 99:
            self._status = self._send_future.result().status
            self._state = 0
            self._done = True
            self._send_future = None

        return self._done, self._status
            
    def cancel(self) -> None: 
        if self._send_future:
            if self._send_future.done():
                self._send_future.result().cancel_goal_async()
            self._send_future.cancel()

    def info(self) -> str:

        return 'action ' + self._client._action_name 

class TbMoveActionHandler(TbActionHandler[MoveTetherbot.Goal]):

    def __init__(self, client: ActionClient, poses: list[TransformMatrix]):
        super().__init__(client)

        self.poses = poses

    def get_goal(self) -> MoveTetherbot.Goal:

        goal_poses = []
        for pose in self.poses:
            goal_poses.extend(pose.r)
            goal_poses.extend(pose.q)

        goal = MoveTetherbot.Goal()
        goal.poses = goal_poses

        return goal

class TbEmptyActionHandler(TbActionHandler[EmptyAction.Goal]):

    def get_goal(self) -> EmptyAction.Goal:

        return EmptyAction.Goal()