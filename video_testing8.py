import cv2
import time
import numpy as np
import RPi.GPIO as GPIO

def forward_stop_at_blue():
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)
    GPIO.setup (24, GPIO.OUT)
    GPIO.setup (22, GPIO.OUT)
    PWML = GPIO.PWM (22,60)
    PWMR = GPIO.PWM (24,60)
    PWMR.start(0)
    PWML.start(0)
    cap = cv2.VideoCapture(0)
    while(1):
        ret, img = cap.read()
        ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        b1,g1,r1 = thresh[240,160]
        b2,g2,r2 = thresh[240,445]
        b3,g3,r3 = thresh[240,300]
        counter=0
        #condition testing ->according to the current facing direction of the bot
        if b3==255 and g3==0 and r3==0:
            counter+=1
            if b1==255 and g1==0 and r1==0 and b2==255 and g2==0 and r2==0:
                PWML.ChangeDutyCycle(100)
                PWMR.ChangeDutyCycle(100)

            elif b1==0 and g1==0 and r1==0 and b2==255 and g2==0 and r2==0: #and b3==255 and g3==255 and r3==255:
                PWMR.ChangeDutyCycle(80)
                PWML.ChangeDutyCycle(100)

            elif b1==255 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0: #and b3==255 and g3==255 and r3==255:
                PWMR.ChangeDutyCycle(100)
                PWML.ChangeDutyCycle(80)
        elif b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0:
            PWML.ChangeDutyCycle(100)
            PWMR.ChangeDutyCycle(100)

        elif b1==255 and g1==255 and r1==255 and b2==0 and g2==0 and r2==0: #and b3==255 and g3==255 and r3==255:
            PWMR.ChangeDutyCycle(80)
            PWML.ChangeDutyCycle(100)

        elif b1==0 and g1==0 and r1==0 and b2==255 and g2==255 and r2==255: #and b3==255 and g3==255 and r3==255:
            PWMR.ChangeDutyCycle(100)
            PWML.ChangeDutyCycle(80)
        if counter>0:
            time.sleep(1.2)
            PWML.stop()
            PWMR.stop()
            break
    GPIO.cleanup()
    cap.release()
    return

def left_stop_at_black():
    cap = cv2.VideoCapture(0)
    time.sleep(2)
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)
    GPIO.setup (24, GPIO.OUT)
    GPIO.setup (27, GPIO.OUT)
    PWML1 = GPIO.PWM (24,100)
    PWMR = GPIO.PWM (27,100)
    PWML1.start(0)
    PWMR.start(0)
    counter = 0
    while(1):
        ret, img = cap.read()
        PWMR.ChangeDutyCycle(100)
        PWML1.ChangeDutyCycle(100)
        
        ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        b1,g1,r1 = thresh[240,100]
        if b1==255:
            counter+=1
            continue
        if counter>0:
            PWML1.stop()
            PWMR.stop()
            break
            
    GPIO.cleanup()
    cap.release()
    return


def right_stop_at_black():
    cap = cv2.VideoCapture(0)
    time.sleep(2)
    GPIO.setmode (GPIO.BCM)
    GPIO.setwarnings (False)
    GPIO.setup (23, GPIO.OUT)
    GPIO.setup (22, GPIO.OUT)
    PWML = GPIO.PWM (22,1)
    PWMR1 = GPIO.PWM (23,1)
    PWMR1.start(0)
    PWML.start(0)
    counter =0
    while(1):
        ret, img = cap.read()
        PWMR1.ChangeDutyCycle(100)
        PWML.ChangeDutyCycle(100)
        
        ret,thresh = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        b1,g1,r1 = thresh[240,500]
        if b1==255:
            counter+=1
            continue
        if counter>0:
            PWMR1.stop()
            PWML.stop()
            break
            
    GPIO.cleanup()
    cap.release()
    return


forward_stop_at_blue()
time.sleep(0.5)

right_stop_at_black()
time.sleep(0.5)

forward_stop_at_blue()
time.sleep(0.5)

left_stop_at_black()
time.sleep(0.5)

forward_stop_at_blue()
time.sleep(0.5)

right_stop_at_black()
time.sleep(0.5)

forward_stop_at_blue()
time.sleep(0.5)

left_stop_at_black()
time.sleep(0.5)

forward_stop_at_blue()
time.sleep(0.5)


