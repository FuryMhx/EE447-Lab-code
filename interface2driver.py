#!/usr/bin/env python
import queue
import rospy
from freenove_base.msg import motor_msg, line_tracking_msg, \
                                servo_msg,adc_msg,buzzer_msg,led_msg
from freenove_base.srv import ultrasonic_srv
from driver4hardware import *


class interface2driver:
    def __init__(self,TOPIC):
        #======================
        # Subscriber 
        #======================
        # Motor initialize
        self.motor = Motor()
        self.motor.setMotorModel(0,0,0,0)
        self.motor_sub = rospy.Subscriber(TOPIC["motor_topic"],motor_msg,
                            self.motor_callback, queue_size=3)        
        # Servo initialize
        self.servo = Servo()
        self.servo_sub =  rospy.Subscriber(TOPIC["servo_topic"],servo_msg,
                            self.servo_callback, queue_size=3)

        # Buzzer initialize
        self.buzzer = Buzzer()
        self.buzzer_sub =  rospy.Subscriber(TOPIC["buzzer_topic"],buzzer_msg,
                            self.buzzer_callback, queue_size=3)
        
        # LED initialize (assignment)
        self.led = Led()
        self.led.rainbow
        self.led_sub = rospy.Subscriber(TOPIC["led_topic"],led_msg,
                           self.led_callback, queue_size=3)
        #======================
        # Publisher
        #======================
        # Line tracking sensor initialize
        self.line = Line_Tracking()
        self.line_tracking_pub = rospy.Publisher(TOPIC["line_tracking_topic"], 
                                    line_tracking_msg, queue_size=3)

        # ADC initialize
        self.adc = Adc()
        self.adc_pub =  rospy.Publisher(TOPIC["adc_topic"],adc_msg, queue_size=3) 

        #======================
        # service 
        #======================
        # Ultrasonic        
        self.ultrasonic = Ultrasonic()
        self.ultrasonic_run = rospy.Service(TOPIC["ultrasonic_topic"],ultrasonic_srv,
                                self.ultrasonic_callback)               

        #Set up rate for update publishers 
        # Update 30 times per second
        self.rate = rospy.Rate(30)

    # callback motors
    def motor_callback(self, msg):
        self.motor.setMotorModel(msg.left_Upper_Wheel,
                            msg.left_Lower_Wheel,
                            msg.right_Upper_Wheel,
                            msg.right_Lower_Wheel)

    # callback servo
    def servo_callback(self, msg):   
        self.servo.setServoPwm('0',msg.horizontal)
        self.servo.setServoPwm('1',msg.vertical)
   
    def led_callback(self,msg):
        self.rainbowCycle(self,strip, wait_ms=20, iterations=5)
    # callback for buzzer
    def buzzer_callback(self,msg):
        self.buzzer.run(msg.command)

    # callback for ultrasonic
    # if request is activate==1, run ultrasonic algorithm
    def ultrasonic_callback(self,srv):
        if srv.activate:
            return self.ultrasonic.get_distance()
    
    # regularly updating publishers
    def update(self):
        line_msg = line_tracking_msg()
        line_msg.left, line_msg.mid, line_msg.right = self.line.signal()
        self.line_tracking_pub.publish(line_msg)

        volt_msg = adc_msg()
        volt_msg.left=self.adc.recvADC(0)
        volt_msg.right=self.adc.recvADC(1)
        volt_msg.power=self.adc.recvADC(2)*3
        self.adc_pub.publish(volt_msg)
        self.rate.sleep()
    # This will be invoked before actual shutdown occurs
    # prevent motor keep running
    def end(self):
        self.motor.setMotorModel(0,0,0,0)


if __name__ == '__main__':
    print("start interface")
    rospy.init_node('interface2driver', anonymous=True)
    TOPIC ={}
    TOPIC["motor_topic"] = rospy.get_param("~motor_topic",'/car/hardware/motor')
    TOPIC["servo_topic"] = rospy.get_param("~servo_topic",'/car/hardware/servo')
    TOPIC["buzzer_topic"] = rospy.get_param("~buzzer_topic",'/car/hardware/buzzer')
    TOPIC["led_topic"] = rospy.get_param("~led_topic",'/car/hardware/led')
    TOPIC["line_tracking_topic"] = rospy.get_param("~line_tracking_topic",'/car/hardware/line_tracking')
    TOPIC["adc_topic"] = rospy.get_param("~adc_topic",'/car/hardware/adc')
    TOPIC["ultrasonic_topic"] = rospy.get_param("~ultrasonic_topic",'/car/hardware/ultrasonic')
    run = interface2driver(TOPIC)  
    # Keeping this node is activated
    while not rospy.is_shutdown():        
        run.update()        
    # prevent motor keep running
    run.end()   
    print("\nexit interface")
