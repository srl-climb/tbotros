from __future__ import annotations
import rclpy
from rclpy.client import Client
from rclpy.action import ActionClient
from custom_actions.action import Empty as EmptyAction, MoveTetherbot
from custom_srvs.srv import SetString
from abc import ABC, abstractmethod
from tbotlib import TransformMatrix


class TbHandler(ABC):

    def __init__(self) -> None:
        
        super().__init__()

        self.status = 0
        self._state = 0

    @abstractmethod
    def execute(self) -> None:
        # NOTE: Return value are standard ros status flags
        # 0: unknown
        # 2: executing
        # 4: succeeded
        # 5: canceled
        # 6: aborted
        pass

    @abstractmethod
    def cancel(self) -> None:
        
        pass

    @abstractmethod
    def info(self) -> str:

        pass


class TbServiceHandler(TbHandler):

    def __init__(self, client: Client):

        super().__init__()

        self._client = client
        self._request = None
        self._future: rclpy.Future = None
        self._state = 0

    def execute(self) -> None:
        # internal state machine

        # wait for service, send request
        if self._state == 0:
            self.status = 2
            if self._client.service_is_ready():
                self._future = self._client.call_async(self._request)
                self._state = 1
            else:
                self._state = 0

        # wait for results
        elif self._state == 1:
            if self._future.done():
                self._state = 99
            else:
                self._state = 1

        #clean up and return result status
        elif self._state == 99:
            self._state = 0
            self._send_future = None
            self.status = 4

    def cancel(self):
        if self._future:
            self._future.cancel()

    def info(self) -> str:

        return 'service ' + self._client.srv_name


class TbSetStringServiceHandler(TbServiceHandler):

    def __init__(self, client: Client, data: str):
        super().__init__(client)

        self._request = SetString.Request()
        self._request.data = data


class TbActionHandler(TbHandler):

    def __init__(self, client: ActionClient):

        super().__init__()

        self._client = client
        self._goal = None
        self._send_future: rclpy.Future = None
        self._state = 0

    def execute(self) -> int:
        # internal state machine

        # wait for server, send goal
        if self._state == 0:
            self.status = 2
            if self._client.server_is_ready():
                self._send_future = self._client.send_goal_async(self._goal)
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
            if self._send_future.result().status >= 4:  
                self._state = 99
            else:
                self._state = 2

        #clean up and return result status
        elif self._state == 99:
            self.status = self._send_future.result().status
            self._state = 0
            self._send_future = None
            

    def cancel(self) -> None: 
        if self._send_future:
            if self._send_future.done():
                self._send_future.result().cancel_goal_async()
            self._send_future.cancel()

    def info(self) -> str:

        return 'action ' + self._client._action_name 

class TbMoveActionHandler(TbActionHandler):

    def __init__(self, client: ActionClient, poses: list[TransformMatrix]):
        super().__init__(client)

        self._goal = MoveTetherbot.Goal()
        
        goal_poses = []
        for pose in poses:
            goal_poses.extend(pose.r)
            goal_poses.extend(pose.q)

        self._goal.poses = goal_poses
            

class TbEmptyActionHandler(TbActionHandler):

    def __init__(self, client: ActionClient):
        super().__init__(client)

        self._goal = EmptyAction.Goal()