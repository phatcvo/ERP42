#! /usr/bin/env python

import rospy
import time
import rospkg
import numpy as np
from std_msgs.msg import Int32, Float32, Float64
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
from scipy import signal


gear = 0
speed = 0
steer = 0
brake = 0
encoder = 0x00
steer_ref = []
steer_act = []
brake_ref = []
brake_act = []
steer_cmd = 0
er = []
prev_error = 0
sum_error = 0
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

t = np.linspace(0, 100, 1000, endpoint=False)
# Ref_steer = 20 * signal.square(0.05 * 2 * np.pi * t)#np.array([28, 28, 28, 0, 0, 0, -28, -28, -28, 0, 0, 0])
Ref_steer = 25 * np.sin(0.05 * 2 * np.pi * t)
b = 10 * signal.square(0.05 * 2 * np.pi * t)#np.array([1, 1, 1, 10, 10, 10, 40, 40, 40, 200, 200, 200])
j = 0
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


    fig, (ax1, ax2) = plt.subplots(2)
    # ======== sine
    KP = 0.172
    KI = 0.001
    KD = 0.00008
    # ======== square
    # KP = 0.07
    # KI = 0.0001
    # KD = 0.0005
    while True:

        for i in range(500):
            error = Ref_steer[i] - steer
            # ============ PID controller ==============================
            steer_cmd += error * KP + sum_error * KI + prev_error * KD
            # ==========================================================
            ackermann.drive.steering_angle =  steer_cmd
            ackermann.drive.acceleration = 1
            ackermann.drive.jerk = 1#b[i]
            ackermann_pub.publish(ackermann)

            steer_ref.append(Ref_steer[i])
            steer_act.append(steer)
            er.append(error)
            brake_ref.append(b)
            brake_act.append(brake)
            print("===============")
            print(i, steer_cmd)
            # plt.figure()


            ax1.plot(np.arange(len(steer_ref)), steer_ref, 'k--', np.arange(len(steer_act)), steer_act, 'b')
            ax2.plot(np.arange(len(steer_act)), er, 'r')

            # ax1.set_xlabel('Episode')
            ax1.set_ylabel('Steer [deg]')
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Error [deg]')
            ax1.legend(['Ref.', 'Act.'])
            ax2.legend(['PID', 'Proposed'])
            # plt.plot(t, Ref_steer)
            # # plt.title('Episode via Reward')
            # plt.xlabel('steer')
            # plt.ylabel('Reward')
            # # plt.tight_layout()  # Function to make distance between figures
            # # plt.imshow(env.render(mode="rgb_array"))
            time.sleep(0.1)
            prev_error = error
            sum_error += error

        plt.show()
        break
