import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('Image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        timer_period = 0.7# seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

        self.bridge = CvBridge()
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        print(ret)
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()

        

def main(args=None):
    rclpy.init(args=args)
    imagePublisher = ImagePublisher()
    rclpy.spin(imagePublisher)
    imagePublisher.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
#source /opt/ros/humble/setup.bash