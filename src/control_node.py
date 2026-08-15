import rospy
from geometry_msgs.msg import Point, Twist

class ControlNode:

    def __init__(self):
        rospy.init_node('control_node', anonymous=False)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bloco_sub = rospy.Subscriber(
            '/bloco_detectado', Point, self.bloco_callback, queue_size=1
        )

        self.area_alvo = rospy.get_param('~area_alvo', 200000)
        self.estado = "PROCURANDO"

        rospy.loginfo("control_node iniciado.")

    def bloco_callback(self, msg):
        cmd = Twist()
        
        detectado = msg.z > 0.5
        erro_x = msg.x
        area = msg.y

        if self.estado == "PROCURANDO":
            if detectado:
                self.estado = "ALINHANDO"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.4 

        elif self.estado == "ALINHANDO":
            if detectado:
                if erro_x < -0.1:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -0.25 
                elif erro_x > 0.1:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.25 
                else:
                    self.estado = "SEGUINDO"
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
            else:
                self.estado = "PROCURANDO"

        elif self.estado == "SEGUINDO":
            if not detectado:
                self.estado = "PROCURANDO"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            elif area > self.area_alvo:
                self.estado = "PARADO"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                rospy.loginfo("Bloco alcançado!")
                
            elif erro_x < -0.25:
                self.estado = "ALINHANDO"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            elif erro_x > 0.25:
                self.estado = "ALINHANDO"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            else:
                # Sinal invertido para compensar a montagem do chassi
                cmd.linear.x = -0.2
                cmd.angular.z = 0.0

        elif self.estado == "PARADO":
            if not detectado:
                self.estado = "PROCURANDO"
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        ControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass