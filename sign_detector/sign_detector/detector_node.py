#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.bridge = CvBridge()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw/esp',
            self.image_callback,
            qos
        )
        self.publisher = self.create_publisher(String, '/command', 10)

        cv2.namedWindow('Sign Detection', cv2.WINDOW_NORMAL)

        self.blue_lower = np.array([105, 120, 60])
        self.blue_upper = np.array([125, 255, 255])
        
        self.white_lower = np.array([0, 0, 160])
        self.white_upper = np.array([180, 40, 255])

        self.red_lower1 = np.array([0, 70, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 70, 50])
        self.red_upper2 = np.array([180, 255, 255])

        self.kernel = np.ones((3, 3), np.uint8)

        self.last_direction = None
        self.stable_count = 0

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (320, 240))
            debug_frame = frame.copy()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            
            mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
            mask_white = cv2.inRange(hsv, self.white_lower, self.white_upper)

            mask_r1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask_r2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask_red = cv2.bitwise_or(mask_r1, mask_r2)

            
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, self.kernel)

            mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel)
            mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self.kernel)

            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, self.kernel)

            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            
            for cnt in contours_red:
                area = cv2.contourArea(cnt)
                if area < 800:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect = w / float(h)

                if 0.75 < aspect < 1.3:
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    self.publisher.publish(String(data="STOP"))
                    cv2.putText(debug_frame, "STOP", (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            for cnt in contours_blue:
                area = cv2.contourArea(cnt)

                if area < 2000:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect = w / float(h)

                if aspect < 0.7 or aspect > 1.4:
                    continue

                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                roi_white = mask_white[y:y + h, x:x + w]
                arrow_contours, _ = cv2.findContours(roi_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if not arrow_contours:
                    continue

                arrow = max(arrow_contours, key=cv2.contourArea)
                white_area = cv2.contourArea(arrow)

                if white_area < 0.12 * area:
                    continue

                arrow_global = arrow + np.array([[x, y]])
                cv2.drawContours(debug_frame, [arrow_global], -1, (0, 255, 0), 2)


                M = cv2.moments(arrow_global)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])


                pts = arrow_global.reshape(-1, 2)
                dists = np.linalg.norm(pts - np.array([cx, cy]), axis=1)
                tip_pt = tuple(pts[np.argmax(dists)])

                if tip_pt[0] > cx:
                    direction = "LEFT"
                else:
                    direction = "RIGHT"

                if direction == self.last_direction:
                    self.stable_count += 1
                else:
                    self.last_direction = direction
                    self.stable_count = 1

                if self.stable_count >= 3:  
                    self.publisher.publish(String(data=f"TURN_{direction}"))

                cv2.putText(debug_frame, f"TURN {direction}", (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow('Sign Detection', debug_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed: {e}")


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
