#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy


from morai_msgs.msg import SetTrafficLight



class scen_config():
    rospy.init_node('scen_config', anonymous=True)

    arg = rospy.myargv(argv = sys.argv)
    traffic_control = True

    #publisher
    traffic_pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight,queue_size=1)
    #time var
    # self.count = 0
    rate = rospy.Rate(10) # 30hz
    print ("======== set traffic ON =================")
    while not rospy.is_shutdown():
        if traffic_control == True:
            ## traffic_control ######################
            set_traffic_data= SetTrafficLight()
            set_traffic_data.trafficLightStatus = 16 ##set greenlight 
            traffic_pub.publish(set_traffic_data)
            

        # if count/300==1 :
        #     global_path_pub.publish(self.global_path)
        #     count=0
        # self.count+=1

        rate.sleep()
    
if __name__ == '__main__':
    scen_config()
        

