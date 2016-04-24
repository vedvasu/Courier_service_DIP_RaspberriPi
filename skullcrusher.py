# -*- coding: utf-8 -*-
import cv2
import numpy as np


#############################################################################################################################
#function to find the shortest path 
#############################################################################################################################

def shortest_path(img,start,end):
    '''
    * Function Name: shortest_path
    * Input: img – Any one of the test images, start point, end point
    * Output: length – length of shortest path
              shortest_path – the shortest path as a list of coordinates of form (x,y)
    * Example Call: l, sp = shortest_path(img)
                    >>> l = length, sp = shortest_path 
    '''

    next_count=1
    previous_count=0
    counter=1
    path_gp=[[start]]

    #check contains all the nodes visited by function
    check=[start] 

    for z1 in range (0,20):                                 #loop for the maximum number path_length
        for z2 in range (previous_count,next_count):        #loop for the possible paths
            
            path1=[]                                        #path list for creating new paths in all four directions
            path2=[]
            path3=[]
            path4=[]


##Part A: Setting the new path to be visited in which next new node will be added
            for z3 in range(0,counter):
                path1.append(path_gp[z2][z3])
                path2.append(path_gp[z2][z3])
                path3.append(path_gp[z2][z3])
                path4.append(path_gp[z2][z3])

##Part B: Creating new nodes to be visited in all four directions            
            c1,c2 = check[z2]                       
            cxa=c1+127
            cya=c2
            layer1 = (cxa,cya)                              
            cxb=c1
            cyb=c2-127
            layer2 = (cxb,cyb)
            cxc=c1-127
            cyc=c2
            layer3 = (cxc,cyc)
            cxd=c1
            cyd=c2+127
            layer4 = (cxd,cyd)

##Part C: This part checks for redundancy of any element in the same path i.e loop should be formed in the path
            t1=0
            t2=0
            t3=0
            t4=0
            for z3 in range(0,next_count):                          
                    if check[z3]==(cxa,cya):
                        t1=1
                    elif check[z3]==(cxb,cyb):
                        t2=1
                    elif check[z3]==(cxc,cyc):
                        t3=1
                    elif check[z3]==(cxd,cyd):
                        t4=1;


##Part D: Checking the node is eligible to be appended as new node                         
            if (cxa<775)&(cya<775):                         #for checking that the points should not be out of the plane
                b1,g1,r1 = img[cya,cxa-70]                  #for checking that the link between first and last node is black
                
                if (t1!=1)&(cxa>0)&(cya>0)&(b1<40):         #conditions for accurate next path element

                    path1.append(layer1)                       
                    path_gp.append(path1)
                    check.append(layer1)                        # t1!=0 : checking for loop formation
                                                                # cxa>0 and cya>0 checking for node in the plane
                                                                # b1 : checking that the link for black 
            if (cxb<775)&(cyb<775):            
                if cyb<700:
                    b2,g2,r2 = img[cyb+70,cxb]

                if (t2!=1)&(cxb>0)&(cyb>0)&(b2<40):
               
                    path2.append(layer2)                        # path2: current path this loop is working on
                    path_gp.append(path2)                       # path_gp: all the paths formed
                    check.append(layer2)                        # check: containing all the nodes

            if (cxc<775)&(cyc<775):        
                if cxc<700:                                     
                    b3,g3,r3 = img[cyc,cxc+70]

                if (t3!=1)&(cxc>0)&(cyc>0)&(b3<40):
                    
                    path3.append(layer3)
                    path_gp.append(path3)
                    check.append(layer3)


            if (cxd<775)&(cyd<775):    
                b4,g4,r4 = img[cyd-70,cxd]

                if (t4!=1)&(cxd>0)&(cyd>0)&(b4<40):

                    path4.append(layer4)
                    path_gp.append(path4)
                    check.append(layer4)

#Part E: Checking for the end point
            if (layer1 == end)|(layer2 == end)|(layer3 == end)|(layer4 == end):             #checking for the end point
                break
        if (layer1 == end)|(layer2 == end)|(layer3 == end)|(layer4 == end):
            break

        previous_count = next_count
        next_count = len(check)                         #setting the counters
        counter = counter + 1

##Part F: Path displayed as per the display asked in problem statement


    excluding_start = path_gp[z2][1:len(path_gp[z2])]
    excluding_start.append(end)
    
    ###### for displaying path on image

    for i in range(0,length):
        a1,b1=excluding_start[i]
        cv2.circle(img,(a1,b1),2,(0,255,255),-1)

    cv2.imshow('Image',img)

    return excluding_start


#############################################################################################################################
#function to make a list of all possible packages start and delivery points 
#############################################################################################################################

total_paths = []

def masking_colors(img,blue1,green1,red1,blue2,green2,red2):
	
	'''
	* Function Name: masking_colors
    * Input: img – Any one of the test images, range of color to be masked
    * Output: append in global list of start and end point for pair of packages
    * Example Call: masking_colour(img,50,50,50,255,255,255)
	'''
	
	param1 = [blue1,green1,red1]                     ##B,G,R values higher and lower range
	param2 = [blue2,green2,red2]

	lower = np.array(param1)										#masking for a given color
	upper = np.array(param2)
	mask = cv2.inRange(img, lower, upper)
	
	ret,thresh = cv2.threshold(mask,127,255,0)
	cv2.imshow('thresh',thresh)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)         #contour detection
	
	for i in xrange(0,len(contours)):
		
		M = cv2.moments(contours[i])
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		
		if cy<125:
			for j in range(0,len(contours)):
				delivery_position_pair = []
				if (cv2.contourArea(contours[i])-cv2.contourArea(contours[j]))<40 and (cv2.contourArea(contours[i])-cv2.contourArea(contours[j]))>-40:

					M1= cv2.moments(contours[j])
					cx1 = int(M1['m10']/M1['m00'])
					cy1 = int(M1['m01']/M1['m00'])
					
					if cx==cx1 and cy==cy1:
						continue
					else:
						delivery_position_pair.append([cx,cy])
						delivery_position_pair.append([cx1,cy1])

						total_paths.append(delivery_position_pair) 

	return


########################################################################################################################
#Function to convert list of points into nearest delivery node
########################################################################################################################

def find_nearest_node(input_array):
	# print input_array[1]
	# print input_array[1][1]
	# print input_array[1][1][1]
	next_array = []
	for i in xrange(len(input_array)):
		#print input_array[i][0]
		# input_array[i]=list(input_array[i][0])
		#print smallest_dist
		#print input_array[i][1][1]
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

		if input_array[i][1][1]>127:
			smallest_dist = ((input_array[i][1][0]-8)*(input_array[i][1][0]-8))+((input_array[i][1][1]-8)*(input_array[i][1][1]-8))
			#print smallest_dist
			for k in range(8,897,127):
				for l in range(8,897,127):
					dist = ((input_array[i][1][0]-k)*(input_array[i][1][0]-k))+((input_array[i][1][1]-l)*(input_array[i][1][1]-l))
					#print input_array[i][1][0],input_array[i][1][1],dist
					if dist<smallest_dist:
						x=k
						y=l
						smallest_dist=dist
						#print "ininininininininiinininiin",input_array[i][1][0],input_array[i][1][1],smallest_dist
			input_array[i][1][0] = x
			input_array[i][1][1] = y
			# tuple(list_array[i][0])
	#for i in xrange() 
	#input_array.sort()
	return input_array



#######################################################################################################################

#######################################################################################################################

img = cv2.imread('CS_Test_Image1.jpg')
img = cv2.resize(img,(775,775))
cv2.imwrite('RESIZED.jpg',img)

#####################################################

masking_colors(img,240,240,0,255,255,50)             ## parameters of blue colour
masking_colors(img,0,100,240,50,150,255)             ## parameters of orange colour
masking_colors(img,0,240,0,50,255,50)             	 ## parameters of green colour
masking_colors(img,240,0,240,255,50,255)             ## parameters of pink colour

print total_paths
print 

#####################################################

input_array = find_nearest_node(total_paths)

print input_array
print
#####################################################

for i in range(len(input_array)):
	cv2.circle(img,(input_array[i][0][0],input_array[i][0][1]),2,(255,255,0),-1)
	cv2.circle(img,(input_array[i][1][0],input_array[i][1][1]),2,(255,255,0),-1)

 

cv2.imshow('Image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()