# -*- coding: cp1252 -*-
'''
*
* Team Id: eYRCPlus-CS#1393
* Author List: Ved Vasu Sharma, Arun Soni, Abheet Devasthale		
* Filename: skullcrusher.py
* Theme: eYRC-Plus: Courier Service
* Functions: forward_detect_green, forward_stop_at_blue, left_stop_at_black, right_stop_at_black,action_after_traffic
			 bot_movement, shortest_path, masking_colors, find_nearest_node, deliver_packages
* Global Variables: d: Starting direction of robot
					total_paths: list having information of node pair
					black_list_nodes: Signal with red color
					package_client.package_server_ip: ip address of the system on which simulation is required
'''

import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import package_client

####################################################
#global variables
													# d is used for direction throughout the program
d=1                     							# d can take values: 1-North 2-South 3-East 4-West
total_paths = []									# this is the path containing information of each package eg. centroid, colour, shape
black_list_nodes = []								# nodes where red signal has once encountered
package_client.package_server_ip = '192.168.10.13'	# ip address of the system on which simulation is required

img = cv2.imread('CS_Test_Image1.jpg')
img = cv2.resize(img,(775,775))
			
####################################################

#############################################################################################################################
#function for orienting bot in appropiate position before the run 
#############################################################################################################################

'''
* Function Name: forward_detect_green
* Input: NULL
* Output: d: direction -- detects green colour and the direction of the robot
* Description: - This function takes the starting image of the arena and decides its starting orientation
			   - Camera will focus on green node (given)
			   - Also moves the robot forward with required distance for the further run
* Example Call: forward_detect_green()
'''

def forward_detect_green():
	
	# Checking Starting orientaion
	
	cap = cv2.VideoCapture(0)
	ret, frame = cap.read()
	ret,thresh = cv2.threshold(img,130,255,cv2.THRESH_BINARY)
	b1,g1,r1 = thresh[400,5]
	b2,g2,r2 = thresh[400,635]

	if b1==0 and g1==0 and r1==0:
		d=3
	else:
		d=1
		
	# Moving forward to orient itself for further processing
	
	GPIO.setmode (GPIO.BCM)
	GPIO.setwarnings (False)
	GPIO.setup (24, GPIO.OUT)
	GPIO.setup (22, GPIO.OUT)
	PWML1 = GPIO.PWM (22,100)
	PWMR1 = GPIO.PWM (24,100)
	
	PWMR1.start(0)
	PWML1.start(0)
	
	PWMR1.ChangeDutyCycle(100)
	PWML1.ChangeDutyCycle(100)
	time.sleep(0.6)
	time.sleep(0.6)
	
	PWMR1.stop()
	PWML1.stop()
	
        cap.release()
        #GPIO.cleanup()
	return d

#############################################################################################################################
#function for orienting bot in appropiate position before the run 
#############################################################################################################################

def forward_stop_at_blue():

        cap = cv2.VideoCapture(0)
	GPIO.setmode (GPIO.BCM)
	GPIO.setwarnings (False)
	GPIO.setup (24, GPIO.OUT)
	GPIO.setup (22, GPIO.OUT)
	PWML2 = GPIO.PWM (22,100)
	PWMR2 = GPIO.PWM (24,100)
	PWMR2.start(0)
	PWML2.start(0)
	counter1=0
	counter2=0
	flag_traffic = 0

	while(1):
                counter2+=1
		ret, img = cap.read()
		ret,thresh = cv2.threshold(img,130,255,cv2.THRESH_BINARY)
		b1,g1,r1 = thresh[250,215]
		b2,g2,r2 = thresh[250,415]
		b3,g3,r3 = thresh[250,350]
		b6,g6,r6 = thresh[250,290]
		b4,g4,r4 = thresh[250,5]
		b5,g5,r5 = thresh[250,635]
		b7,g7,r7 = thresh[250,100]
		
		#condition testing ->according to the current facing direction of the bot
		if b3==0 and g3==0 and r3==255:
			flag_traffic+=1

		if (b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0) or (b1==255 and g1==0 and r1==0 and b2==255 and g2==0 and r2==0):
			PWML2.ChangeDutyCycle(100)
			PWMR2.ChangeDutyCycle(100)
		
		elif b3==255 and g3==255 and r3==255:
			if b1==255 and g1==255 and r1==255 and b2==255 and g2==255 and r2==255:
				if b4==0 and g4==0 and r4==0:
					PWMR2.ChangeDutyCycle(80)
					PWML2.ChangeDutyCycle(60)   
				elif b5==0 and g5==0 and r5==0:
					PWMR2.ChangeDutyCycle(60)
					PWML2.ChangeDutyCycle(80)
				else:
					PWMR2.ChangeDutyCycle(70)
					PWML2.ChangeDutyCycle(70)
										
			elif b1==255 and g1==255 and r1==255:
				PWMR2.ChangeDutyCycle(70)
				PWML2.ChangeDutyCycle(90)				
			elif b2==255 and g2==255 and r2==255:
				PWMR2.ChangeDutyCycle(90)
				PWML2.ChangeDutyCycle(70)
										
		elif (b1==255 and g1==255 and r1==255 and b2==0 and g2==0 and r2==0) or (b1==0 and g1==0 and r1==0 and b2==255 and g2==0 and r2==0) or (b1==0 and g1==0 and r1==0 and b2==0 and g2==0 and r2==255): #and b3==255 and g3==255 and r3==255:
			PWMR2.ChangeDutyCycle(70)
			PWML2.ChangeDutyCycle(90)

		elif (b1==0 and g1==0 and r1==0 and b2==255 and g2==255 and r2==255) or (b1==255 and g1==0 and r1==0 and b2==0 and g2==0 and r2==0) or (b1==0 and g1==0 and r1==255 and b2==0 and g2==0 and r2==0): #and b3==255 and g3==255 and r3==255:
			PWMR2.ChangeDutyCycle(90)
			PWML2.ChangeDutyCycle(70)

		if ((b3==0 and g3==0 and r3==255) or (b5==0 and g5==0 and r5==0) or (b4==0 and g4==0 and r4==0) or (b3==255 and g3==0 and r3==0) or (b3==0 and g3==255 and r3==0)): #or ((b3==255 and g3==0 and r3==0) and (b5==0 and g5==0 and r5==0)) or ((b4==0 and g4==0 and r4==0) and (b3==255 and g3==0 and r3==0)) or ((b3==0 and g3==0 and r3==255) and (b5==0 and g5==0 and r5==0)) or ((b4==0 and g4==0 and r4==0) and (b3==0 and g3==0 and r3==255)) or (b3==0 and g3==255 and r3==0 and b6==0 and g6==255 and r6==0):
                        if counter2>8:
                                counter1+=1
                                pt = time.clock()
		
		elif counter1>0:
			if (time.clock()-pt) > 0.45:
				PWML2.stop()
				PWMR2.stop()
				break
##		elif (b7==255 and g7==255 and r7==255 and b6==255 and g6==255 and r6==255 and b5==255 and g5==255 and r5==255 and b4==255 and g4==255 and r4==255 and b1==255 and g1==255 and r1==255 and b2==255 and g2==255 and r2==255):
##                        if counter2>6:
##                                PWML.stop()
##                                PWMR.stop()
##                                break                        
	
	cap.release()
	#GPIO.cleanup()
	return flag_traffic

################### Function for left motion of the robot ###################

def left_stop_at_black():
	cap = cv2.VideoCapture(0)
	GPIO.setmode (GPIO.BCM)
	GPIO.setwarnings (False)
	GPIO.setup (24, GPIO.OUT)
	GPIO.setup (27, GPIO.OUT)
	PWML3 = GPIO.PWM (24,100)
	PWMR3 = GPIO.PWM (27,100)
	PWML3.start(0)
	PWMR3.start(0)
	counter = 0
	while(1):
		ret, img = cap.read()
		ret,thresh = cv2.threshold(img,130,255,cv2.THRESH_BINARY)
		b1,g1,r1 = thresh[250,120]

		PWMR3.ChangeDutyCycle(90)
		PWML3.ChangeDutyCycle(90)
		
		if (b1>200 and g1>200 and r1>200):
                        counter+=1
		elif counter>0:
			PWML3.stop()
			PWMR3.stop()
			break

	cap.release()
	#GPIO.cleanup()
	return

################### Function for right motion of the robot ###################

def right_stop_at_black():
	cap = cv2.VideoCapture(0)
	GPIO.setmode (GPIO.BCM)
	GPIO.setwarnings (False)
	GPIO.setup (23, GPIO.OUT)
	GPIO.setup (22, GPIO.OUT)
	PWML4 = GPIO.PWM (22,100)
	PWMR4 = GPIO.PWM (23,100)
	PWMR4.start(0)
	PWML4.start(0)
	counter =0
	while(1):
		ret, img = cap.read()
		ret,thresh = cv2.threshold(img,130,255,cv2.THRESH_BINARY)
		b1,g1,r1 = thresh[250,520]

		PWMR4.ChangeDutyCycle(90)
		PWML4.ChangeDutyCycle(90)

		if (b1>200 and g1>200 and r1>200):
                        counter+=1
		elif counter>0:
			PWMR4.stop()
			PWML4.stop()
			break
		
	cap.release()
	#GPIO.cleanup()
	return

#############################################################################################################################
#function to display path in the required format
#############################################################################################################################

def display_label(path):
	'''
	* Function Name: display_label
	* Input: contours -- list of path node in pixel form of which the number is to be displayed
						 as per the format in the problem statement.
	* Output: required_display –- list nodes in required format
	* Example Call: display_label(path)
	* Example Output: required_display >>> [[1,2],[0,1],[2,1]]
	'''

	#this function converts the list containing nodes in the form of pixels into required format
	
	required_display = []                                       
        path = path[1:len(path)]
	for i in range (0,len(path)):
			cx = path[i][0]                    #getting x coordinates
			cy = path[i][1]                    #getting y coordinates
			y = (cy-8)/127                    #equation to convert : starting pixel is 12 and each node is at a distance of 125 pixel                
			x = (cx-8)/127

			node = (x,y)                        
			required_display.append(node)      #required node format is apended                    

        print required_display
	return

#############################################################################################################################
#function to take action when the traffic is encountered
#############################################################################################################################

'''
* Function Name: action_after traffic
* Input: previous_node: node last traversed before red node is encountered
		 current_node: node at which red in encountered
		 end: end of path currently traversed by bot
		 d: direction of bot  
* Output: d: direction after traversing alternating path
* Description: - black list the node so that it is not traversed again unless no other path is possible
			   - moves back to the previous_node
			   - initiate shortest path from previous_node to end
* Example Call: d = action_after_traffic()
'''

def action_after_traffic(previous_node, current_node, end,d):
	
	black_list_nodes.append(current_node)
	path = shortest_path(previous_node,end)

	white_nodes = 0

	for i in range(len(path)-1):
		cx,cy = path[i]
		b1,g1,r1 = img[cy,cx]
		if b1>200 and g1>200 and r1>200:
			white_nodes+=1

	if len(path) == 0:
		black_list_nodes[len(black_list_nodes)-1:len(black_list_nodes)] = []
		time.sleep(30)
		path = shortest_path(current_node,end)
		display_label(path)
		d = bot_movement(path,d)

	elif len(path)>=7 or white_nodes>0:
                time.sleep(30)
                path = shortest_path(current_node,end)
                display_label(path)
		d = bot_movement(path,d)
                
	
	else:
		if current_node[0] == 8:
			if d==1:
				right_stop_at_black()
				right_stop_at_black()

			elif d==2:
				left_stop_at_black()
				left_stop_at_black()

		elif current_node[0] == 770:
			if d==1:
				left_stop_at_black()
				left_stop_at_black()

			elif d==2:
				right_stop_at_black()
				right_stop_at_black()
		
		elif current_node[1] == 0:
			if d==3:
				left_stop_at_black()
				left_stop_at_black()

			elif d==4:
				right_stop_at_black()
				right_stop_at_black()	
		elif current_node[1] == 0:
			if d==3:
				right_stop_at_black()
				right_stop_at_black()

			elif d==4:
				left_stop_at_black()
				left_stop_at_black()
		else:
			left_stop_at_black()
			left_stop_at_black()
		
		flag_traffic = forward_stop_at_blue()
		
		if d==1:
			d=2
		elif d==2:
			d=1
		elif d==3:
			d=4
		elif d==4:
			d=3

                display_label(path)
		d = bot_movement(path,d)

	return d

#############################################################################################################################
#function to invoke proper motion function for motion depending upon the nodes to be traversed and the direction of the robot
#############################################################################################################################

'''
* Function Name: bot_movement
* Input: path: path to be traversed (from start to end)
		 d: current direction	
* Output: final direction 
* Description: - This function works on the difference between pixel values
			   - Motion commands are initiated depending upon the current direction
			   - If flag_traffic is set 1: action_after_traffic() is called
			   - appropiate direction is set after every step
* Example Call: d = bot_movement(path,d)
'''

def bot_movement(path,d):
	
	n = len(path)                                     #Baudrate is choosen 9600
	for i in range(0,n-1):
		a1,b1=path[i]                                 #a1,b1->coordinates of start point
		a2,b2=path[i+1]                               #a2,b2->coordinates of next point
		
		if(a1>a2):                                          #condition testing ->in which direction the bot is supposed to move
			if d==1:                                      #condition testing ->according to the current facing direction of the bot
				left_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=4
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==2:
				right_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=4
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==3:
				if b1==8 and b2==8:
					right_stop_at_black()
					right_stop_at_black()
				else:
					right_stop_at_black()
					right_stop_at_black()

				flag_traffic = forward_stop_at_blue()
				d=4
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==4:
				flag_traffic = forward_stop_at_blue()
				d=4
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
		
		elif (a1<a2):
			if d==1:
				right_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=3
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==2:
				left_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=3
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==3:
				flag_traffic = forward_stop_at_blue()
				d=3
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==4:
				if b1==8 and b2==8:
					left_stop_at_black()
					left_stop_at_black()
				else:
					right_stop_at_black()
					right_stop_at_black()
				
				flag_traffic = forward_stop_at_blue()
				d=3
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
		
		elif (b1<b2):
			if d==1:
				if a1==8 and a2==8:
					right_stop_at_black()
					right_stop_at_black()
				else:
					left_stop_at_black()
					left_stop_at_black()

				flag_traffic = forward_stop_at_blue()
				d=2
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==2:
				flag_traffic = forward_stop_at_blue()
				d=2
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==3:
				right_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=2
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==4:
				left_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=2
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
		
		elif (b1>b2):
			if d==1:
				flag_traffic = forward_stop_at_blue()
				d=1
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==2:
				if a1==8 and a2==8:
					left_stop_at_black()
					left_stop_at_black()
				else:
					right_stop_at_black()
					right_stop_at_black()
				
				flag_traffic = forward_stop_at_blue()
				d=1
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==3:
				left_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=1
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
			
			elif d==4:
				right_stop_at_black()
				flag_traffic = forward_stop_at_blue()
				d=1
				if flag_traffic > 1:
					d = action_after_traffic(path[i],path[i+1],path[n-1],d)
					break
	
	return d

#############################################################################################################################
#function to find the shortest path 
#############################################################################################################################

'''
* Function Name: shortest_path
* Input: start: start point of path, end: end point of path
* Output: path_gp[z2]: List containing list of nodes constituting shortest path
* Description: - This function uses Dijkstras Algorithm to find the shortest path
			   - Various filtering flags are used, eg. black_list_nodes, back traversing etc			  
* Example Call: path = shortest_path(start,end)
'''

def shortest_path(start,end):
	
	start = tuple(start)
	end = tuple(end)

	next_count = 1
	previous_count = 0
	counter = 1
	path_found_flag = 0

	path_gp = [[start]]

	#check contains all the nodes visited by function
	check = [start] 

	for z1 in range (0,20):                                 #loop for the maximum number path_length
		for z2 in range (previous_count,next_count):        #loop for the possible paths
			
			path1 = []                                        #path list for creating new paths in all four directions
			path2 = []
			path3 = []
			path4 = []


##Part A: Setting the new path to be visited in which next new node will be added
			for z3 in range(0,counter):
				path1.append(path_gp[z2][z3])
				path2.append(path_gp[z2][z3])
				path3.append(path_gp[z2][z3])
				path4.append(path_gp[z2][z3])

##Part B: Creating new nodes to be visited in all four directions            
			c1,c2 = check[z2]                       
			cxa = c1+127
			cya = c2
			layer1 = (cxa,cya)                              
			cxb = c1
			cyb = c2-127
			layer2 = (cxb,cyb)
			cxc = c1-127
			cyc = c2
			layer3 = (cxc,cyc)
			cxd = c1
			cyd = c2+127
			layer4 = (cxd,cyd)

##Part C: This part checks for redundancy of any element in the same path i.e loop should be formed in the path
			t1 = 0
			t2 = 0
			t3 = 0
			t4 = 0
			for z3 in range(0,next_count):                          
					if check[z3] == (cxa,cya):
						t1 = 1
					elif check[z3] == (cxb,cyb):
						t2 = 1
					elif check[z3] == (cxc,cyc):
						t3 = 1
					elif check[z3] == (cxd,cyd):
						t4 = 1;

			flag1 = 0
			flag2 = 0
			flag3 = 0
			flag4 = 0
			for z4 in range(len(black_list_nodes)):
				if tuple(black_list_nodes[z4]) == (cxa,cya):
					flag1 = 1
				if tuple(black_list_nodes[z4]) == (cxb,cyb):
					flag2 = 1
				if tuple(black_list_nodes[z4]) == (cxc,cyc):
					flag3 = 1
				if tuple(black_list_nodes[z4]) == (cxd,cyd):
					flag4 = 1


##Part D: Checking the node is eligible to be appended as new node                         
			if (cxa<775)&(cya<775):                         #for checking that the points should not be out of the plane
				b1,g1,r1 = img[cya,cxa-70]                  #for checking that the link between first and last node is black
				
				if (flag1!=1)&(t1!=1)&(cxa>0)&(cya>0)&(b1<40):         #conditions for accurate next path element

					path1.append(layer1)                       
					path_gp.append(path1)
					check.append(layer1)                        # t1!=0 : checking for loop formation
																# cxa>0 and cya>0 checking for node in the plane
																# b1 : checking that the link for black 
			if (cxb<775)&(cyb<775):            
				if cyb<700:
					b2,g2,r2 = img[cyb+70,cxb]

				if (flag2!=1)&(t2!=1)&(cxb>0)&(cyb>0)&(b2<40):
			   
					path2.append(layer2)                        # path2: current path this loop is working on
					path_gp.append(path2)                       # path_gp: all the paths formed
					check.append(layer2)                        # check: containing all the nodes

			if (cxc<775)&(cyc<775):        
				if cxc<700:                                     
					b3,g3,r3 = img[cyc,cxc+70]

				if (flag3!=1)&(t3!=1)&(cxc>0)&(cyc>0)&(b3<40):
					
					path3.append(layer3)
					path_gp.append(path3)
					check.append(layer3)


			if (cxd<775)&(cyd<775):    
				b4,g4,r4 = img[cyd-70,cxd]

				if (flag4!=1)&(t4!=1)&(cxd>0)&(cyd>0)&(b4<40):

					path4.append(layer4)
					path_gp.append(path4)
					check.append(layer4)

#Part E: Checking for the end point
			if (check[len(check)-1] == end)|(check[len(check)-2] == end)|(check[len(check)-3] == end)|(check[len(check)-4] == end):             #checking for the end point
				break
		if (check[len(check)-1] == end)|(check[len(check)-2] == end)|(check[len(check)-3] == end)|(check[len(check)-4] == end):
			path_found_flag = 1
			break

		previous_count = next_count
		next_count = len(check)                         #setting the counters
		counter = counter + 1

	if path_found_flag == 1:
		path_gp[z2].append(end)
	else:
		path_gp[z2] = []
	return path_gp[z2]


#############################################################################################################################
#function to make a list of all possible packages start and delivery points 
#############################################################################################################################

'''
* Function Name: masking_colours
* Input: img: test_image
		 blue1, green1, red1: parameter1 lower mask values
		 blue2, green2, red2: parameter2 upper mask values 
* Output: appends infromation in global list (total_paths)
* Description: - Image processing operations: Masking, threshing and contour detection
			   - Area matching for same shape 
			   - checking for pickup point or delivery point (for simulation)
			   - appends information in total_paths as seperate node 
* Example Call: masking_colors(img,blue1,green1,red1,blue2,green2,red2)
'''

def masking_colors(img,blue1,green1,red1,blue2,green2,red2):

	param1 = [blue1,green1,red1]                     ##B,G,R values higher and lower range
	param2 = [blue2,green2,red2]

	lower = np.array(param1)                                        #masking for a given color
	upper = np.array(param2)
	mask = cv2.inRange(img, lower, upper)
	
	ret,thresh = cv2.threshold(mask,127,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)         #contour detection
	
	for i in xrange(0,len(contours)):
		
		M = cv2.moments(contours[i])
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		
		if cy<125:
			for j in range(0,len(contours)):
				delivery_position_pair = []
				if (cv2.contourArea(contours[i])-cv2.contourArea(contours[j]))<80 and (cv2.contourArea(contours[i])-cv2.contourArea(contours[j]))>-80:

					M1= cv2.moments(contours[j])
					cx1 = int(M1['m10']/M1['m00'])
					cy1 = int(M1['m01']/M1['m00'])
					
					if cx==cx1 and cy==cy1:
						continue
					else:
						delivery_position_pair.append([cx,cy])
						delivery_position_pair.append([cx1,cy1])

						if blue2 == 255 and green2 == 255 and red2 == 50:
							if cv2.contourArea(contours[i])>400 and cv2.contourArea(contours[i])<600:
								delivery_position_pair.append('BT1')
								delivery_position_pair.append('BT0')
							
							elif cv2.contourArea(contours[i])>700 and cv2.contourArea(contours[i])<900:
								delivery_position_pair.append('BC1')
								delivery_position_pair.append('BC0')
							
							elif cv2.contourArea(contours[i])>900 and cv2.contourArea(contours[i])<1200:
								delivery_position_pair.append('BS1')
								delivery_position_pair.append('BS0')


						elif blue2 == 50 and green2 == 150 and red2 == 255 : 
							
							if cv2.contourArea(contours[i])>400 and cv2.contourArea(contours[i])<600:
								delivery_position_pair.append('OT1')
								delivery_position_pair.append('OT0')
							
							elif cv2.contourArea(contours[i])>700 and cv2.contourArea(contours[i])<900:
								delivery_position_pair.append('OC1')
								delivery_position_pair.append('OC0')
							
							elif cv2.contourArea(contours[i])>900 and cv2.contourArea(contours[i])<1200:
								delivery_position_pair.append('OS1')
								delivery_position_pair.append('OS0')

						elif blue2 == 50 and green2 == 255 and red2 == 50 : 
							if cv2.contourArea(contours[i])>400 and cv2.contourArea(contours[i])<600:
								delivery_position_pair.append('GT1')
								delivery_position_pair.append('GT0')
							
							elif cv2.contourArea(contours[i])>700 and cv2.contourArea(contours[i])<900:
								delivery_position_pair.append('GC1')
								delivery_position_pair.append('GC0')
							
							elif cv2.contourArea(contours[i])>900 and cv2.contourArea(contours[i])<1200:
								delivery_position_pair.append('GS1')
								delivery_position_pair.append('GS0')

						elif blue2 == 255 and green2 == 50 and red2 == 255 :
							if cv2.contourArea(contours[i])>400 and cv2.contourArea(contours[i])<600:
								delivery_position_pair.append('PT1')
								delivery_position_pair.append('PT0')
							
							elif cv2.contourArea(contours[i])>700 and cv2.contourArea(contours[i])<900:
								delivery_position_pair.append('PC1')
								delivery_position_pair.append('PC0')
							
							elif cv2.contourArea(contours[i])>900 and cv2.contourArea(contours[i])<1200:
								delivery_position_pair.append('PS1')
								delivery_position_pair.append('PS0')                                

					total_paths.append(delivery_position_pair) 

	return

########################################################################################################################
#Function to convert list of points into nearest delivery node
########################################################################################################################

'''
* Function Name: find_nearest_node
* Input: input_array: array obtained from masking_colours() function
* Output: input_array: sorted array of corresponding nodes to be traversed
* Description: - Converts information of centroid into nodes to be traversed
* Example Call: find_nearest_node(input_array)
'''

def find_nearest_node(input_array):

	next_array = []
	for i in xrange(len(input_array)):

########################################################## for warehouse company cells 
		if input_array[i][0][1]<=127:
			if input_array[i][0][0]<=127:
				input_array[i][0][0] = 135
				input_array[i][0][1] = 135
			elif input_array[i][0][0]>127 and input_array[i][0][0]<=262:
				input_array[i][0][0] = 262
				input_array[i][0][1] = 135
			elif input_array[i][0][0]>262 and input_array[i][0][0]<=389:
				input_array[i][0][0] = 389
				input_array[i][0][1] = 135
			elif input_array[i][0][0]>389 and input_array[i][0][0]<=516:
				input_array[i][0][0] = 516
				input_array[i][0][1] = 135
			elif input_array[i][0][0]>516 and input_array[i][0][0]<=643:
				input_array[i][0][0] = 643
				input_array[i][0][1] = 135
			elif input_array[i][0][0]>643 and input_array[i][0][0]<=770:
				input_array[i][0][0] = 770
				input_array[i][0][1] = 135

################################################################# for city delivery points
		if input_array[i][1][1]>127:
			smallest_dist = ((input_array[i][1][0]-8)*(input_array[i][1][0]-8))+((input_array[i][1][1]-8)*(input_array[i][1][1]-8))

			for k in range(8,897,127):
				for l in range(8,897,127):
					dist = ((input_array[i][1][0]-k)*(input_array[i][1][0]-k))+((input_array[i][1][1]-l)*(input_array[i][1][1]-l))

					if dist<smallest_dist:
						x=k
						y=l
						smallest_dist=dist

			input_array[i][1][0] = x
			input_array[i][1][1] = y
	
	input_array.sort()
	return input_array
	

#######################################################################################################################
# Funtion to effectively deliver all the packages in city zone
#######################################################################################################################

'''
* Function Name: deliver_pakages
* Input: node_set: nodes at which delivery is to be made
		 current_position: node at which bot is present
		 d = direction
* Output: current_position: position of the bot after traversal
		  d = direction of bot after traversal
* Description: - find path with least length out of the nodes present in node_set
			   - traverse the path and remove that point from list
			   - repeats till node_set is empty
* Example Call: deliver_packages(nodes_set,current_position,d)
'''

def deliver_packages(nodes_set,current_position,d):

	while(len(nodes_set) != 0):

		nearest_node = nodes_set[0][0]
		k=0
		nearest_node_len =  len(shortest_path(current_position,nodes_set[0][0]))
		
		for i in range(len(nodes_set)):
			
			length = len(shortest_path(current_position,nodes_set[i][0]))
			if length<nearest_node_len:
				nearest_node_len = length
				k=i
		
		nearest_node = nodes_set[k][0]      

		path = shortest_path(current_position,nearest_node)
		display_label(path)
		d = bot_movement(path,d)

		package_client.Message(nodes_set[k][1])
		
		nodes_set[k:k+1] = []
		current_position = path[len(path)-1]
	
	return current_position, d


#######################################################################################################################
# Main Program
#######################################################################################################################

img = cv2.imread('CS_Test_Image1.jpg')
img = cv2.resize(img,(775,775))

##cleaning any already existing GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(24,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
GPIO.cleanup()

#####################################################
#to on power led before the run at GPIO 21, pin40 raspberry-pi
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21,GPIO.OUT)
P = GPIO.PWM(21,100)
P.start(100)

#####################################################

masking_colors(img,240,240,0,255,255,50)             ## parameters of blue colour
masking_colors(img,0,100,240,50,150,255)             ## parameters of orange colour
masking_colors(img,0,240,0,50,255,50)                ## parameters of green colour
masking_colors(img,240,0,240,255,50,255)             ## parameters of pink colour

# #####################################################

input_array = find_nearest_node(total_paths)

d = forward_detect_green()

# #####################################################
# #package collection from the warehouse

path = shortest_path((8,770),input_array[0][0])
display_label(path)
d = bot_movement(path,d)

next_counter = 1
previous_counter = 0
current_position = input_array[0][0]

for i in range(len(input_array)):
	delivery_nodes = []
	package_client.Message(input_array[i][2])
	if next_counter == 4 or next_counter == 8 or next_counter == 12 or next_counter == len(input_array):

######################################################################################################
## Logic for traversal once 4 nodes are collected
		for j in xrange(previous_counter, next_counter):

			delivery_nodes.append((input_array[j][1], input_array[j][3]))
			previous_counter = next_counter

		current_position,d = deliver_packages(delivery_nodes,current_position,d)

######################################################################################################
## Going back to the warehouse zone after deliery
                if i < len(input_array)-1:
                                path = shortest_path(current_position, input_array[i][0])
                                display_label(path)
                                d = bot_movement(path,d)
                                current_position = input_array[i][0]

######################################################################################################
	
	if i+1<len(input_array):
		if input_array[i][0] != input_array[i+1][0]:
		
			path = shortest_path(current_position,input_array[i+1][0])
			current_position = path[len(path)-1]
			display_label(path)
			d = bot_movement(path,d)
	next_counter+=1

time.sleep(1)
package_client.Message('PS2')
package_client.Message('GS3')
package_client.Message('OS4')
package_client.Message('BS5')

print '***** END OF TASK *****'
GPIO.cleanup()
cv2.waitKey(0)
cv2.destroyAllWindows()


