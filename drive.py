#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64,Int64, Bool

class BasicDriveNode:
    def __init__(self):
        rospy.init_node('basic_drive_node', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)#/commands/servo/position 
        self.pub_cmd_ang = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.rate = rospy.Rate(10)  # 주행 속도 조절을 위한 주기 (10Hz)
        self.speed_msg = Float64()
        self.angle_msg = Float64()

        ##variables
        ##variables_speed
    
        self.drive_speed = 1800
        self.static_speed = 2000
        self.dynamic_speed = 0
        self.child_speed = 1000
        self.rabacon_speed = 1000
        
        ##variables_ang
        self.drive_ang = 0.5
        self.dynamic_ang = 0.5
        self.stop_count = 0
        self.steer_ang = -1
        self.child_mark = 0
        self.obtacle_flag = 0
        self.parking = 0
        self.tunnel_steer = 0.5
        self.raba_steer = 0.5
        self.raba_mode=False
        self.stop_flag=False

        self.turn_right_flag = 0
        self.turn_left_flag = 0

        self.mark_count = 0
        self.dynamic_count = 0
        self.dynamic_time = None
        self.stop_count=0

    def drive_with_steering(self, speed, steering_angle):
        self.speed_msg.data = speed
        self.angle_msg.data = steering_angle
        self.pub_cmd_vel.publish(self.speed_msg.data)
        self.pub_cmd_ang.publish(self.angle_msg.data)

    def run(self):
        rospy.Subscriber("steering_angle", Float64, self.drive_callback)
        #rospy.Subscriber("status",Int64,self.status_CB)
        rospy.Subscriber("child",Int64,self.child_CB)
        rospy.Subscriber("objec_flag",Int64,self.object_CB)
        #rospy.Subscriber("L_or_R",bool,self.L_or_R_CB)
        rospy.Subscriber("stop", Bool, self.stop_CB)
        rospy.Subscriber("tunnel_steering", Float64, self.tunnel_CB)
        rospy.Subscriber("raba_steering", Float64, self.raba_CB)
        rospy.Subscriber("rabacon_mode", Bool, self.rmode_CB)
        while not rospy.is_shutdown():
            if self.raba_mode ==True:
                print("raba_on")
                self.drive_with_steering(1500, self.raba_steer)

            else:
                if self.obtacle_flag == 1:
                    print("dynamic")
                    self.drive_with_steering(0, self.steer_ang)
                    rospy.sleep(3)
                    self.dynamic_count+=1
                    print(self.dynamic_count)
        
                elif self.stop_flag and self.stop_count==0:
                    print("stop")
                    self.drive_with_steering(0, self.steer_ang)
                    rospy.sleep(3)
                    self.stop_count+=1

                elif self.parking == 1:  #parking mode
                    continue

                else:
                    if self.steer_ang == -1:  #tunnel
                        print("tunnel")
                        self.drive_with_steering(2000, self.tunnel_steer)                    


                    else:
                        if self.child_mark == 1:
                            self.drive_with_steering(1000,self.steer_ang)
                    
                        else:
                            self.drive_with_steering(1500,self.steer_ang)

                self.rate.sleep()

    
    def stop_CB(self, msg):
        self.stop_flag = msg.data

    def drive_callback(self, msg):
        self.steer_ang = msg.data

    def child_CB(self, msg):
        self.child_mark = msg.data
        if self.child_mark == 1:
            print("어린이 보호구역")
        else: pass

    def object_CB(self,msg):
        self.obtacle_flag = msg.data
        if self.obtacle_flag==1:
            print("object_find")
        else:
            pass      

    def tunnel_CB(self, msg):
        self.tunnel_steer = msg.data
        print(self.tunnel_steer)


    def raba_CB(self, msg):
        self.raba_steer = msg.data
        print(self.raba_steer)
    
    def rmode_CB(self, msg):
        self.raba_mode = msg.data
       


if __name__ == '__main__':
    try:
        node = BasicDriveNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
