from __future__ import annotations

import rclpy
import rclpy.time
from tbotlib import TbTetherbot
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from custom_msgs.msg import BoolArray
from builtin_interfaces.msg import Time
from .tetherbot_control_base_state_publisher_node import BaseStatePublisherNode

class VizualizationPublisherNode(BaseStatePublisherNode):

    def __init__(self):

        super().__init__(node_name = 'visualization_publisher')

        # tb arm object for forward kinematics
        self._tbot: TbTetherbot = TbTetherbot.load(self._config_file)
        
        # publishers
        self.tether_line_pub = self.create_publisher(MarkerArray, self.get_name() + '/tether_lines', 1)
        # subscribers
        self.create_subscription(BoolArray, '/' + self._tbot.platform.name + '/platform_controller/tether_tension', self.tether_tension_sub_callback, 1)
        # timers
        self.create_timer(0.1, self.timer_callback)

        self._tether_tension = []

    def timer_callback(self):

        time = self.get_clock().now().seconds_nanoseconds()

        try:
            lines = MarkerArray()
            for tether, tension in zip(self._tbot.tethers, self._tether_tension):
                p1 = Point()
                p2 = Point()

                transform: TransformStamped = self._tf_listener.buffer.lookup_transform(target_frame = 'map', 
                                                                                        source_frame = tether.anchorpoints[0].name,
                                                                                        time = rclpy.time.Time())
                p1.x = transform.transform.translation.x
                p1.y = transform.transform.translation.y
                p1.z = transform.transform.translation.z

                transform: TransformStamped = self._tf_listener.buffer.lookup_transform(target_frame = 'map', 
                                                                                        source_frame = tether.anchorpoints[1].name,
                                                                                        time = rclpy.time.Time())
                p2.x = transform.transform.translation.x
                p2.y = transform.transform.translation.y
                p2.z = transform.transform.translation.z


                line = Marker(type = Marker.LINE_LIST)
                line.ns = tether.name
                line.header.frame_id = 'map'
                line.header.stamp = Time(sec = time[0], nanosec = time[1])
                line.scale.x = 0.002
                line.scale.y = 0.002
                line.points.append(p1)
                line.points.append(p2)

                if tension:
                    line.color.g = 1.0
                    line.color.a = 1.0
                else:
                    line.color.r = 1.0
                    line.color.a = 1.0               
                
                lines.markers.append(line)

            self.tether_line_pub.publish(lines)
        
        except Exception as exc:
              self.get_logger().warn('Look up transform failed: ' + str(exc))

    def tether_tension_sub_callback(self, msg: BoolArray):

        self._tether_tension = msg.data


def main(args = None):

    rclpy.init(args = args)
    try:
        node = VizualizationPublisherNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
