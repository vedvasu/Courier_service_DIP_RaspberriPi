import cv2
import numpy as np


total_paths = []
def masking_colors(img,blue1,green1,red1,blue2,green2,red2):

	param1 = [blue1,green1,red1]                     ##B,G,R values higher and lower range
	param2 = [blue2,green2,red2]

	lower = np.array(param1)
	upper = np.array(param2)
	mask = cv2.inRange(img, lower, upper)
	#img= cv2.bitwise_and(img, img, mask=mask)
	#cv2.imshow('img',img)
	cv2.imshow('mask',mask)
	cv2.imwrite('Mask.jpg',mask)
	
	#gray = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(mask,127,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	
	
	for i in xrange(0,len(contours)):
		print len(contours)
		print 'Area_first',cv2.contourArea(contours[i])
		cv2.drawContours(img,contours,-1,(0,0,255),5)

		M = cv2.moments(contours[i])
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		#print "Centroid = ", cx, ", ", cy
		
		if cy<125:
			cv2.circle(img,(cx,cy), 2, (0,0,0), -1)
			for j in range(0,len(contours)):
				delivery_position_pair = []
				print 'Area',cv2.contourArea(contours[i])-cv2.contourArea(contours[j])
				if (cv2.contourArea(contours[i])-cv2.contourArea(contours[j]))<80 and (cv2.contourArea(contours[i])-cv2.contourArea(contours[j]))>-80:

					M1= cv2.moments(contours[j])
					cx1 = int(M1['m10']/M1['m00'])
					cy1 = int(M1['m01']/M1['m00'])
					print "                        Centroid = ", cx1, ", ", cy1
					if cx==cx1 and cy==cy1:
						continue
					else:
						delivery_position_pair.append((cx,cy))
						delivery_position_pair.append((cx1,cy1))

						total_paths.append(delivery_position_pair)


	#print total_paths
	cv2.imshow('img',mask)
	return

img = cv2.imread('CS_Test_Image2.jpg')
img = cv2.resize(img,(775,775))

masking_colors(img,240,240,0,255,255,50)             ## parameters of blue colour
print 
print
print total_paths
print 
print
masking_colors(img,0,100,240,50,150,255)             ## parameters of orange colour
print
print
print total_paths
print
print
masking_colors(img,0,240,0,50,255,50)             ## parameters of green colour
print
print
print total_paths
print
print
masking_colors(img,240,0,240,255,50,255)             ## parameters of pink colour
print
print
print total_paths
print
print


cv2.imshow('Sample',img)

cv2.waitKey(0)
cv2.destroyAllWindows()