import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
def forward_stop_at_blue():
    cap = cv2.VideoCapture(0)
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)
    GPIO.setup (24, GPIO.OUT)
    GPIO.setup (22, GPIO.OUT)
    PWML = GPIO.PWM (22,500)
    PWMR = GPIO.PWM (24,500)
    PWMR.start(0)
    PWML.start(0)
    while(1):
        ret, img = cap.read()
        ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        b1,g1,r1 = thresh[240,160]
        b2,g2,r2 = thresh[240,445]
        b3,g3,r3 = thresh[240,300]                                        #condition testing ->according to the current facing direction of the bot
        if b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0:
            PWML.ChangeDutyCycle(100)
            PWMR.ChangeDutyCycle(100)

        elif b1==255 and g1==255 and r1==255 and b2==0 and g2==0 and r2==0: #and b3==255 and g3==255 and r3==255:
            PWMR.ChangeDutyCycle(80)
            PWML.ChangeDutyCycle(100)

        elif b1==0 and g1==0 and r1==0 and b2==255 and g2==255 and r2==255: #and b3==255 and g3==255 and r3==255:
            PWMR.ChangeDutyCycle(100)
            PWML.ChangeDutyCycle(80)
        elif b3==255 and g3==0 and r3==0:
            PWMR.stop()
            PWMR.stop()
    return

def left_stop_at_black():
    cap = cv2.VideoCapture(0)
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)
    GPIO.setup (24, GPIO.OUT)
    GPIO.setup (22, GPIO.OUT)
    PWML = GPIO.PWM (22,500)
    PWMR = GPIO.PWM (24,500)
    PWMR.start(0)
    PWML.start(0)
    while(1):
        ret, img = cap.read()
        ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        b1,g1,r1 = thresh[240,160]
        b2,g2,r2 = thresh[240,445]
        b3,g3,r3 = thresh[240,300]
        PWMR.ChangeDutyCycle(100)
        PWML.ChangeDutyCycle(0)
        if b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0:
            PWMR.stop()
            PWML.stop()  
    return


def right_stop_at_black():
    cap = cv2.VideoCapture(0)
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)
    GPIO.setup (24, GPIO.OUT)
    GPIO.setup (22, GPIO.OUT)
    PWML = GPIO.PWM (22,500)
    PWMR = GPIO.PWM (24,500)
    PWMR.start(0)
    PWML.start(0)
    while(1):
        ret, img = cap.read()
        ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        b1,g1,r1 = thresh[240,160]
        b2,g2,r2 = thresh[240,445]
        b3,g3,r3 = thresh[240,300]
        PWMR.ChangeDutyCycle(0)
        PWML.ChangeDutyCycle(100)
        if b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0:
            PWMR.stop()
            PWML.stop() 

    return


def bot_movement(path_gp):
    d=1
    n = len(path_gp) 
    cap = cv2.VideoCapture(0)
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)                                    #Baudrate is choosen 9600
    GPIO.setup (24, GPIO.OUT)
    GPIO.setup (22, GPIO.OUT)
    
    PWML = GPIO.PWM (22,60)
    PWMR = GPIO.PWM (23,60)

    PWML.start(100)
    PWMR.start(100)

    for i in range(0,n-1):
        a1,b1=path_gp[i]                                 #a1,b1->coordinates of start point
        a2,b2=path_gp[i+1]                               #a2,b2->coordinates of next point
        

        if(a1>a2):                                          #condition testing ->in which direction the bot is supposed to move
            
            if d==1:
                c=0
                while(1):
                    ret, img = cap.read()
                    ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
                    b1,g1,r1 = thresh[240,160]
                    b2,g2,r2 = thresh[240,445]
                    b3,g3,r3 = thresh[240,300]                                        #condition testing ->according to the current facing direction of the bot
                    if b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0:
                        PWML.ChangeDutyCycle(100)
                        PWMR.ChangeDutyCycle(100)
                        c=c+1
                    else:
                        PWMR.ChangeDutyCycle(50)
                        PWML.ChangeDutyCycle(100)
                        if c>0:
                            PWMR.stop()
                            PWML.stop()
                            break
                ser.write("4")
                time.sleep(0.8)
                ser.write("8")                              #motion command is activated according to the time in next statement
                time.sleep(0.85)
                d=4
            elif d==2:
                ser.write("6")
                time.sleep(0.8)
                ser.write("8")                              
                time.sleep(0.85)
                d=4
            elif d==3:
                ser.write("4")
                time.sleep(1.5)
                ser.write("8")
                time.sleep(0.85)
                d=4
            elif d==4:
                ser.write("8")
                time.sleep(0.85)
                d=4
        
        elif (a1<a2):
            
            if d==1:
                ser.write("6")
                time.sleep(0.8)
                ser.write("8")
                time.sleep(0.85)
                d=3
            elif d==2:
                ser.write("4")
                time.sleep(0.8)
                ser.write("8")
                time.sleep(0.85)
                d=3
            elif d==3:
                ser.write("8")
                time.sleep(0.85)
                d=3
            elif d==4:
                ser.write("4")
                time.sleep(1.5)
                ser.write("8")
                time.sleep(0.85)
                d=3
        
        elif (b1<b2):
            
            if d==1:
                ser.write("4")
                time.sleep(1.5)
                ser.write("8")
                time.sleep(0.85)
                d=2
            elif d==2:
                ser.write("8")
                time.sleep(0.85)
                d=2
            elif d==3:
                ser.write("6")
                time.sleep(0.8)
                ser.write("8")
                time.sleep(0.85)
                d=2
            elif d==4:
                ser.write("4")
                time.sleep(0.8)
                ser.write("8")
                time.sleep(0.85)
                d=2
        
        elif (b1>b2):
            
            if d==1:
                ser.write("8")
                time.sleep(0.85)
                d=1
            elif d==2:
                ser.write("6")
                time.sleep(1.5)
                ser.write("8")
                time.sleep(0.85)
                d=1
            elif d==3:
                ser.write("4")
                time.sleep(0.8)
                ser.write("8")
                time.sleep(0.85)
                d=1
            elif d==4:
                ser.write("6")
                time.sleep(0.8)
                ser.write("8")
                time.sleep(0.85)
                d=1
        ser.write("5")
    ser.close()
    return d
