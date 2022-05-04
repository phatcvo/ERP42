#! /usr/bin/env python

import rospy
import time
import rospkg
import numpy as np
from std_msgs.msg import Int32, Float32, Float64
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt

gear = 0
speed = 0
steer = 0
brake = 0
encoder = 0x00
steer_ref = []
steer_act = []
brake_ref = []
brake_act = []
def speed_callback(data):
    global speed
    speed = data.data
    # print("speed: ", speed)

def steer_callback(data):
    global steer
    steer = data.data * np.pi/180
    # ar_steer.append(steer)
    # steer_act.append(steer)
    # print("steer: ", steer)

def brake_callback(data):
    global brake
    brake = data.data
    # brake_act.append(brake)
    # print("brake: ", brake)

def gear_callback(data):
    global gear
    gear = data.data
    # print("gear: ", gear)

def encoder_callback(data):
    global encoder
    encoder = data.data
    # print("encoder: ", encoder)
    # print('gear:', gear, 'speed: ', speed, 'steer:', steer, 'brake', brake, 'encoder: ', encoder)


if __name__=='__main__':
    rospy.init_node('Control')
    # Publisher
    ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=10)
    ackermann=AckermannDriveStamped()

    # Subscriber
    rospy.Subscriber("ERP42_speed", Float32, speed_callback)
    rospy.Subscriber('ERP42_steer', Float32, steer_callback)
    rospy.Subscriber("ERP42_brake", Float32, brake_callback)
    rospy.Subscriber("ERP42_gear", Float32, gear_callback)
    rospy.Subscriber("ERP42_encoder", Float64, encoder_callback)

    a = np.array([28, 28, 28, 0, 0, 0, -28, -28, -28, 0, 0, 0])
    b = np.array([1, 1, 1, 10, 10, 10, 40, 40, 40, 200, 200, 200])
    j = 0
    fig, (ax1, ax2) = plt.subplots(2)
    while True:

        for i in range(len(a)):
            ackermann.drive.steering_angle = a[i]
            ackermann.drive.acceleration = 1
            ackermann.drive.jerk = b[i]
            ackermann_pub.publish(ackermann)
            time.sleep(1)
            steer_ref.append(a[i])
            steer_act.append(steer)
            brake_ref.append(b[i])
            brake_act.append(brake)
            print("===============")
            print(np.arange(len(steer_act)), steer_act)
            # plt.figure()


            ax1.plot(np.arange(len(steer_act)), steer_ref, 'r', np.arange(len(steer_act)), steer_act, 'b')

            ax2.plot(np.arange(len(steer_act)), brake_ref, 'r', np.arange(len(steer_act)), brake_act, 'b')

            # ax1.set_xlabel('Episode')
            ax1.set_ylabel('Steer [deg]')
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Brake [%]')
            ax1.legend(['Ref.', 'Act.'])
            ax2.legend(['Ref.', 'Act.'])
            # plt.plot(np.arange(len(steer_ref)), steer_ref, 'r', np.arange(len(steer_ref)), steer_act, 'b')
            # # plt.title('Episode via Reward')
            # plt.xlabel('steer')
            # plt.ylabel('Reward')
            # # plt.tight_layout()  # Function to make distance between figures
            # # plt.imshow(env.render(mode="rgb_array"))
        j = j + 1
        print(j)
        if j == 10:
            plt.show()
            break