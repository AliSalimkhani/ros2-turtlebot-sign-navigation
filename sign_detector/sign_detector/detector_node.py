import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String 


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.bridge = CvBridge()

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/command',
            10
        )

        self.get_logger().info("Sign detector node started")

        # One window only (for Raspberry Pi)
        cv2.namedWindow('Sign Detection', cv2.WINDOW_NORMAL)

        self.command = None

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize to lower resolution for speed
            frame = cv2.resize(frame, (320, 240))
            debug_frame = frame.copy()

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- Blue background detection ---
            blue_lower = np.array([100, 100, 20])
            blue_upper = np.array([125, 255, 100])
            mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

            # --- White arrow detection ---
            white_lower = np.array([0, 0, 80])
            white_upper = np.array([180, 30, 255])
            mask_white = cv2.inRange(hsv, white_lower, white_upper)

            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv,lower_red1,upper_red1)
            mask2 = cv2.inRange(hsv,lower_red2,upper_red2)
            mask_red = cv2.bitwise_or(mask1, mask2)

            


            # --- Find blue sign contours ---
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contour_red:
                if cv2.contourArea(cnt) < 400:
                    continue
                
                x , y , w, h = cv2.boundingRect(cnt)
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                self.command = "STOP"
                self.publisher.publish(String(data=self.command))
                cv2.putText(debug_frame, self.command , (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            for cnt in contours_blue :
                if cv2.contourArea(cnt) < 300:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                sign_center_x = int(x + w / 2)

                # Draw sign bounding box and center line
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.line(debug_frame, (sign_center_x, y), (sign_center_x, y + h), (0, 0, 255), 1)

                # Extract arrow region (white inside blue)
                roi_white = mask_white[y:y + h, x:x + w]
                arrow_contours, _ = cv2.findContours(roi_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for arrow_cnt in arrow_contours:
                    if cv2.contourArea(arrow_cnt) < 80:
                        continue

                    # Convert to global coordinates
                    arrow_cnt_global = arrow_cnt + np.array([[x, y]])

                    cv2.drawContours(debug_frame, [arrow_cnt_global], -1, (0, 255, 0), 2)

                    # Find leftmost and rightmost points (as ints)
                    leftmost_x = int(np.min(arrow_cnt_global[:, :, 0]))
                    rightmost_x = int(np.max(arrow_cnt_global[:, :, 0]))
                    leftmost_y = int(arrow_cnt_global[:, :, 1][arrow_cnt_global[:, :, 0].argmin()])
                    rightmost_y = int(arrow_cnt_global[:, :, 1][arrow_cnt_global[:, :, 0].argmax()])

                    leftmost = (leftmost_x, leftmost_y)
                    rightmost = (rightmost_x, rightmost_y)

                    # Compute distances from sign center
                    dist_left = abs(leftmost_x - sign_center_x)
                    dist_right = abs(rightmost_x - sign_center_x)

                    # Determine direction
                    if dist_left > dist_right:
                        direction = "LEFT"
                        color = (0, 255, 0)
                    else:
                        direction = "RIGHT"
                        color = (0, 0, 255)

                    self.command = direction

                    # Draw points and decision info
                    cv2.circle(debug_frame, leftmost, 3, (0, 255, 255), -1)
                    cv2.circle(debug_frame, rightmost, 3, (255, 255, 0), -1)
                    cv2.putText(debug_frame, f"TURN {direction}", (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    

                    self.publisher.publish(String(data=self.command))
            # --- Show result ---
            cv2.imshow('Sign Detection', debug_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
