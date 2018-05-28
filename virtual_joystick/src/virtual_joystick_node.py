#!/usr/bin/env python
import pygame
import time
import rospy
from sensor_msgs.msg import Joy
import os, sys



class VirtualJoystick(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.vjoy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()
        self.screen_size = 300
        self.speed_tang = 1.0
        self.speed_norm = 1.0
        # obtain vehicle name
        self.veh_name = rospy.get_param('~veh_name')
    
        # prepare pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_size,self.screen_size))
        self.prepare_dpad()
        # prepare ROS publisher
        self.pub_joystick = rospy.Publisher("/"+self.veh_name+"/joy", Joy, queue_size=1)

        # print the hint
        self.print_hint()

        # start the main loop
        self.loop()


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def loop(self):
        veh_standing = True
    
        while True:              
            # add dpad to screen
            self.screen.blit(dpad, (0,0))
        
            # prepare message
            msg = Joy()
            msg.header.seq = 0
            msg.header.stamp.secs = 0
            msg.header.stamp.nsecs = 0
            msg.header.frame_id = ''
            msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

            # obtain pressed keys
            keys = pygame.key.get_pressed()

            ### checking keys and executing actions ###
        
            # drive left
            if keys[pygame.K_LEFT]:
                self.screen.blit(dpad_l, (0,0))
                msg.axes[3] += self.speed_norm

            # drive right
            if keys[pygame.K_RIGHT]:
                self.screen.blit(dpad_r, (0,0))
                msg.axes[3] -= self.speed_norm

            # drive forward
            if keys[pygame.K_UP]:
                self.screen.blit(dpad_f, (0,0))
                msg.axes[1] += self.speed_tang

            # drive backwards
            if keys[pygame.K_DOWN]:
                self.screen.blit(dpad_b, (0,0))
                msg.axes[1] -= self.speed_tang

        

            # activate line-following aka autopilot
            if keys[pygame.K_a]:
                msg.buttons[7] = 1    

            # stop line-following
            if keys[pygame.K_s]:
                msg.buttons[6] = 1

            # toggle anti-instagram
            if keys[pygame.K_i]:
                msg.buttons[3] = 1

            ## key/action for quitting the program
            
            # check if top left [x] was hit
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()

            # quit program
            if keys[pygame.K_q]:
                pygame.quit()

            ### END CHECKING KEYS ###
            
            # refresh screen
            pygame.display.flip()

            # check for any input commands
            stands = (sum(map(abs, msg.axes)) == 0 and sum(map(abs, msg.buttons)) == 0)
            if not stands:
                veh_standing = False
            
            # publish message
            if not veh_standing:
                self.pub_joystick.publish(msg)

            # adjust veh_standing such that when vehicle stands still, at least
            # one last publishment was sent to the bot. That's why this adjustment
            # is made after the publishment of the message
            if stands:
                veh_standing = True
            
            time.sleep(0.03)

            # obtain next key list
            pygame.event.pump()


        # prepare size and rotations of dpad and dpad_pressed
    def prepare_dpad(self):
        global dpad, dpad_f, dpad_r, dpad_b, dpad_l
        file_dir = os.path.dirname(__file__)
        file_dir = (file_dir + "/") if  (file_dir) else ""
    
        dpad = pygame.image.load(file_dir + "../images/d-pad.png")
        dpad = pygame.transform.scale(dpad, (self.screen_size, self.screen_size))
        dpad_pressed = pygame.image.load(file_dir + "../images/d-pad-pressed.png")
        dpad_pressed = pygame.transform.scale(dpad_pressed, (self.screen_size, self.screen_size))
        dpad_f = dpad_pressed
        dpad_r = pygame.transform.rotate(dpad_pressed, 270)
        dpad_b = pygame.transform.rotate(dpad_pressed, 180)
        dpad_l = pygame.transform.rotate(dpad_pressed, 90)

    # Hint which is print at startup in console
    def print_hint(self):
        print("\n\n\n")
        print("Virtual Joystick for your Duckiebot")
        print("-----------------------------------")
        print("\n")
        print("[ARROW_KEYS]:    Use them to steer your bot")
        print("         [q]:    Quit the program")
        print("         [a]:    Start line-following aka. autopilot")
        print("         [s]:    Stop line-following")
        print("         [i]:    Toggle anti-instagram")
        print("\n")
        print("Questions? Contact Julien Kindle: jkindle@ethz.ch")
if __name__ == '__main__':
    # prepare ROS node
    rospy.init_node('virtual_joy',anonymous=False)
    virtual_joy = VirtualJoystick()
    rospy.spin()  
