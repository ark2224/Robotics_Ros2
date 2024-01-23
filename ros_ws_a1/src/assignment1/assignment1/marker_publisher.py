import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'markers', 10)
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        my_object = Marker()
        my_object.header.frame_id = "object_frame"
        my_object.header.stamp = self.get_clock().now().to_msg()
        my_object.ns = "a1"
        my_object.id = 0
        my_object.type = Marker.CYLINDER
        my_object.action = Marker.ADD
        my_object.pose.position.x = 0.0
        my_object.pose.position.y = 0.0
        my_object.pose.position.z = 0.0
        my_object.pose.orientation.x = 0.0
        my_object.pose.orientation.y = 0.0
        my_object.pose.orientation.z = 0.0
        my_object.pose.orientation.w = 1.0
        my_object.scale.x = 0.3
        my_object.scale.y = 0.3
        my_object.scale.z = 0.3
        my_object.color.r = 0.0
        my_object.color.g = 1.0
        my_object.color.b = 0.0
        my_object.color.a = 1.0
        my_object.lifetime.sec = 0
        my_object.lifetime.nanosec = 0
        self.publisher_.publish(my_object)
        
        robot = Marker()
        robot.header.frame_id = "robot_frame"
        robot.header.stamp = self.get_clock().now().to_msg()
        robot.ns = "a1"
        robot.id = 1;
        robot.type = Marker.CUBE
        robot.action = Marker.ADD
        robot.pose.position.x = 0.0
        robot.pose.position.y = 0.0
        robot.pose.position.z = 0.0
        robot.pose.orientation.x = 0.0
        robot.pose.orientation.y = 0.0
        robot.pose.orientation.z = 0.0
        robot.pose.orientation.w = 1.0
        robot.scale.x = 0.7
        robot.scale.y = 0.7
        robot.scale.z = 0.7
        robot.color.r = 0.0
        robot.color.g = 1.0
        robot.color.b = 0.0
        robot.color.a = 1.0
        robot.lifetime.sec = 0
        robot.lifetime.nanosec = 0
        self.publisher_.publish(robot)

        camera = Marker()
        camera.header.frame_id = "camera_frame";
        camera.header.stamp = self.get_clock().now().to_msg()
        camera.ns = "a1"
        camera.id = 2
        camera.type = Marker.ARROW
        camera.action = Marker.ADD
        camera.pose.position.x = 0.0
        camera.pose.position.y = 0.0
        camera.pose.position.z = 0.0
        camera.pose.orientation.x = 0.0
        camera.pose.orientation.y = 0.0
        camera.pose.orientation.z = 0.0
        camera.pose.orientation.w = 1.0
        camera.scale.x = 2.96
        camera.scale.y = 0.1
        camera.scale.z = 0.1
        camera.color.r = 0.0
        camera.color.g = 1.0
        camera.color.b = 0.0
        camera.color.a = 1.0
        camera.lifetime.sec = 0
        camera.lifetime.nanosec = 0
        self.publisher_.publish(camera)



def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    print("Publishing markers")
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
