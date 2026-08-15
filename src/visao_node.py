import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class VisaoNode:

    def __init__(self):
        rospy.init_node('visao_node', anonymous=False)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            '/camera/rgb/image_raw', Image, self.image_callback, queue_size=1
        )

        self.bloco_pub = rospy.Publisher('/bloco_detectado', Point, queue_size=1)
        
        self.area_minima = rospy.get_param('~area_minima', 300)

        rospy.loginfo("visao_node iniciado: procurando bloco vermelho...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("Erro ao converter imagem: " + str(e))
            return

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        altura, largura = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        limite_baixo1 = np.array([0, 120, 70])
        limite_alto1 = np.array([10, 255, 255])
        limite_baixo2 = np.array([170, 120, 70])
        limite_alto2 = np.array([180, 255, 255])

        mascara1 = cv2.inRange(hsv, limite_baixo1, limite_alto1)
        mascara2 = cv2.inRange(hsv, limite_baixo2, limite_alto2)
        mascara = cv2.bitwise_or(mascara1, mascara2)

        kernel = np.ones((5, 5), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        msg_saida = Point()
        msg_saida.z = 0.0

        if len(contornos) > 0:
            maior_contorno = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(maior_contorno)

            if area > self.area_minima:
                M = cv2.moments(maior_contorno)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    erro_x = (cx - largura / 2.0) / (largura / 2.0)

                    msg_saida.x = erro_x
                    msg_saida.y = area
                    msg_saida.z = 1.0

                    x, y, w, h = cv2.boundingRect(maior_contorno)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                    cv2.line(frame, (largura // 2, 0), (largura // 2, altura),
                              (255, 0, 0), 1)

        self.bloco_pub.publish(msg_saida)

        frame_display = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6)
        cv2.imshow("Camera do Robo", frame_display)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        VisaoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()