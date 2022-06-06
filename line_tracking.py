#!/usr/bin/env python
import rospy
from freenove_base.msg import motor_msg,line_tracking_msg,servo_msg
from freenove_base.srv import ultrasonic_srv
import cv2
import numpy as np

class Line_Tracking:
    def __init__(self,TOPIC,display):
        self.stage = 0
        self.display = display
        self.TOPIC = TOPIC
        rospy.wait_for_service(self.TOPIC["ultrasonic_topic"])
        self.motor_pub = rospy.Publisher(self.TOPIC["motor_topic"], motor_msg, queue_size=1)
        self.servo_pub = rospy.Publisher(self.TOPIC["servo_topic"], servo_msg, queue_size=1)              
        self.line_tracking_sub = rospy.Subscriber(TOPIC["line_tracking_topic"], 
                                    line_tracking_msg, self.line_tracking_callback, queue_size=3) 
        self.LMR=0x00
 
        
        # Init camera                 
        self.image_w = 320
        self.image_h = 240
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.image_w)
        self.cap.set(4, self.image_h) 
        self.safe_zone_l = int(self.image_w/2-10)
        self.safe_zone_r = int(self.image_w/2 + 10)
        self.Horizontal_center = 90        
        self.center = int(self.image_w/2)           

        
        # Init variables
        self.signal = 1
        self.dis = 200
        self.left = 0
        self.right = 0
        self.distance = 0
        self.flag = 0
        self.vertical = 82
        self.horizontal = 90
        self.now = rospy.get_rostime().to_sec()
       
    def line_tracking_callback(self,msg):
                self.LMR=0x00
                
                # Get left,mid,right sensors statement
                if msg.left==True:
                    self.LMR=(self.LMR | 4)
                if msg.mid==True:
                    self.LMR=(self.LMR | 2)
                if msg.right==True:
                    self.LMR=(self.LMR | 1)

                    
    def line_tracking_strategy(self): 
            self.dis = self.get_distance()
            print("The distance is ",self.dis,self.LMR)
#             if self.dis > 5:
            if self.dis > 0:
                flag=1         
                # Motors' speed and direction judgment
                if self.LMR==2:
                    wheels_pose =[700,700,700,700]
                elif self.LMR==4:
                    wheels_pose =[-750,-750,1250,1250]
                elif self.LMR==6:
                    wheels_pose =[-1000,-1000,2000,2000]
                elif self.LMR==1:
                    wheels_pose =[1250,1250,-750,-750]
                elif self.LMR==3:
                    wheels_pose =[2000,2000,-1000,-1000]            
                elif self.LMR==7: 
                    wheels_pose =[0,0,0,0]
                else:
                    flag=0 
                    

                # Publish only if judgment is changed           
                if flag:
                    self.update(wheels_pose)
                
            else:
                 self.update_moto(-500,-500,-500,-500)
                 rospy.sleep(1.0)
                 self.update_moto(0,0,0,0)
                 rospy.sleep(0.5)
                 self.update_moto(-1250, -1250, 2500, 2500)
                 rospy.sleep(1.5)
                 self.update_moto(0,0,0,0)
                 rospy.sleep(0.5)                 
                 self.stage = 1
                 
    def update(self,wheels_pose):
        move = motor_msg()
        move.left_Upper_Wheel = wheels_pose[0]
        move.left_Lower_Wheel = wheels_pose[1]
        move.right_Upper_Wheel = wheels_pose[2]
        move.right_Lower_Wheel=wheels_pose[3]
        self.motor_pub.publish(move)
        
    
    def get_distance(self):
        # Push out request
        call = rospy.ServiceProxy(self.TOPIC["ultrasonic_topic"],ultrasonic_srv)     
        rusult=call(bool(1))
        # Pull disntace result
        return rusult.distance

    def update_moto(self,lu,ll,ru,rl):
        move = motor_msg()
        move.left_Upper_Wheel = lu
        move.left_Lower_Wheel =ll
        move.right_Upper_Wheel = ru
        move.right_Lower_Wheel= rl
        self.motor_pub.publish(move)
        
    def update_servo(self,horizontal,vertical=80):
        turning  = servo_msg()
        turning.horizontal = horizontal-30
        turning.vertical = vertical
        self.servo_pub.publish(turning)
     
    # Color filter 
    # This is a function for image processing 
    def color_plate(self,img):    
        # Image Smoothing techniques help in reducing the noise. 
        # Using a Gaussian Blur filter before edge detection aims 
        #   to reduce the level of noise in the image, which improves the result.     
        img = cv2.GaussianBlur(img,(5,5),0)
        cv2.imshow('GaussianBlur', img)
        
        # BGR2HSV Determine  color adjustment
        # HSV used to separate image luminance from color information.
        # HSV is closer to how humans perceive color. It has three components: hue, saturation, and value. 
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Only get Red in the image
        Rmin2 = np.array([165, 43, 46])
        Rmax2 = np.array([200, 255, 255])
        img_bin = cv2.inRange(hsv_img,Rmin2, Rmax2)
        cv2.imshow('Red objects', img_bin)
        # Use to perform erosion on the image 
        # In cases like noise removal, erosion is followed by GaussianBlur. 
        # Because, erosion removes small spots, but it also shrinks our object
        img_bin = cv2.erode(img_bin,None,iterations=2)
        cv2.imshow('Erosion', img_bin)
        # Binaryzation for image, red objects will be white and other color will all be black
        _, img_bin = cv2.threshold(img_bin, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)      
        cv2.imshow('Binaryzation', img_bin)    
        return img_bin
       
    def loop(self):  
        self.left = 0
        self.right = 0          
        
        # Read a image from camera
        success, img = self.cap.read()
        # Image processing for get red objects only 
        img_b=self.color_plate(img)
        # List of red blocks that found
        blocks = cv2.findContours(img_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # Find target
        if len(blocks)>0 :
            print("Find target")            
            # Find the object that have max area as target           
            block = max(blocks, key=cv2.contourArea)
            # Get the edges of the object
            rect = cv2.minAreaRect(block)            
            box = cv2.boxPoints(rect)    
            # Get area size
            area = cv2.contourArea(block)
            # Get the start point, wide and high of the object
            x, y, w, h = cv2.boundingRect(block)
            # Checking target area
            print("area: ",area)
            if area > 0 :
                # Visualized safe zone area
                img = cv2.rectangle(img,(self.safe_zone_l, 0),
                        (self.safe_zone_r, self.image_h),
                        (0, 255, 0), 2)
                # Drawing Contours in image for the object        
                cv2.drawContours(img, [np.int0(box)], -1, (0, 255, 255), 2)
                # Get the center point of the object
                self.Horizontal_center = int(w/2+x)
                self.vertical_center = int(y + h/2)     
                # Drawing the center of the object  
                img = cv2.circle(img, (self.Horizontal_center,self.vertical_center), 1, (0, 0, 0),3)
                
                # If target is outside of safe_zone area 
                # Turn servo only
                if self.Horizontal_center >= self.safe_zone_r:
                    self.horizontal +=1
                    self.left = 0
                    self.right = 0
                elif self.Horizontal_center < self.safe_zone_l:
                    self.horizontal -=1
                    self.left = 0
                    self.right = 0
                # If target is inside of safe_zone area 
                # Turn servo and motors 
                # Servo turn opposite direction to prevent overshoot 
                else:
                    if 90-self.horizontal>10:
                        self.horizontal +=1
                        self.left = -1200
                        self.right = 1200
                    elif 90-self.horizontal<-10:
                        self.horizontal -=1
                        self.left = 1200
                        self.right = -1200
                    else:
                        self.left = 1200
                        self.right = 1200

                # Update servo
                # In the range
                if self.horizontal<151 and self.horizontal>29:
                    self.update_servo(self.horizontal)
                # If target out of range, turn car to keep servo in the range 
                elif self.horizontal>150:
                    self.left = 800
                    self.right = -800
                    print("turn over limit")
                elif self.horizontal<30:
                    self.left = -800
                    self.right = 800
                    print("turn over limit")
       
        # Updata motors   
        self.update_moto(self.left,self.left,self.right,self.right) 
        # Updata image 
        if self.display == 1:
            cv2.imshow('block_detect', img)
    
        if self.LMR > 0:
            self.update_moto(1500,1500,-750,-750)
            rospy.sleep(1.0)
            self.update_moto(0,0,0,0)
            rospy.sleep(0.5)            
            self.stage = 0
            
    def end(self):
        self.update_moto(0,0,0,0)      

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    TOPIC ={}
    TOPIC["motor_topic"] = rospy.get_param("~motor_topic",'/car/hardware/motor')
    TOPIC["servo_topic"] = rospy.get_param("~servo_topic",'/car/hardware/servo')
    TOPIC["buzzer_topic"] = rospy.get_param("~buzzer_topic",'/car/hardware/buzzer')
    TOPIC["line_tracking_topic"] = rospy.get_param("~line_tracking_topic",'/car/hardware/line_tracking')
    TOPIC["adc_topic"] = rospy.get_param("~adc_topic",'/car/hardware/adc')
    TOPIC["ultrasonic_topic"] = rospy.get_param("~ultrasonic_topic",'/car/hardware/ultrasonic')

    # Keep running node run
    display_image = 1
    run_line = Line_Tracking(TOPIC,display_image)
     
    while not rospy.is_shutdown(): 
        if (run_line.stage == 0): 
            run_line.line_tracking_strategy()
        elif (run_line.stage == 1):
            run_line.loop()
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
    run_line.cap.release()
    cv2.destroyWindow('block_detect')
    print("exit tacking")
