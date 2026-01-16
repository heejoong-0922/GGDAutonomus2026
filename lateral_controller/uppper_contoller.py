import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int16
import numpy as np
class Upper_controller : 

    def __init__(self):
        rospy.init_node('upper_controller', anonymous = True)
        #================== subscriber ======================
        rospy.Subscriber("/heading_error", Float32, self.heading_callback)
        rospy.Subscriber("/lateral_error", Float32, self.lateral_callback)

        #================== publisher =======================
        self.steering_pub = rospy.Publisher("/des_steer", Int16, queue_size = 1)
        self.speed_pub = rospy.Publisher("/motor_cmd_long",Int16,queue_size = 1)
        #====================================================
        self.velocity = 0                
        self.error_heading = 0  
        self.lateral_error = 0
        self.is_heading_error = False
        self.is_lateral_error = False

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            is_ready = self.is_heading_error and self.is_lateral_error

            if is_ready:

                self.velocity = 150                
                steering = self.stanley_lane()

                self.steering_pub.publish(steering)
                self.speed_pub.publish(self.velocity)

                print(f"steering:   {steering}")
                print(f"velocity:   {self.velocity}")
                print("=================================")

            else:
                 print('Error')

            rate.sleep()




    def heading_callback(self, msg):
        self.is_heading_error = True
        self.heading_error = msg.data

    def lateral_callback(self, msg):
        self.is_lateral_error = True
        self.lateral_error = msg.data

    def stanley_lane(self):

        #===================== stanley 파라미터 =================================   
            k_s = 1
            GAIN_pi_t = 1

        # ==================== x(t) from vision ========================================
            x_t =  self.lateral_error

        # ====================== pi(t) from vision =====================================
            pi_t = self.heading_error

            pi_t = (pi_t + 180) % (2 * 180) - 180 #정규화

        # ======================= 조향각 구하기 ======================================

            steering = GAIN_pi_t * pi_t + np.arctan(k_s * x_t / self.velocity) #속도는 100으로 고정

            rospy.logwarn(f'\n x_t : {x_t} \n pi_t : {pi_t} \n')

            steering = int(steering)
            
            return steering
    



if __name__ == '__main__':
    try:
        test_track = Upper_controller()
    except rospy.ROSInterruptException:
        pass