# -*- coding: cp1252 -*-
import numpy as np
import cv2

#####################################################################
## More Functions
#####################################################################

def shortest_path(start,end):
    
    start = tuple(start)
    end = tuple(end)
    
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
            if (check[len(check)-1] == end)|(check[len(check)-2] == end)|(check[len(check)-3] == end)|(check[len(check)-4] == end):             #checking for the end point
                break
        if (check[len(check)-1] == end)|(check[len(check)-2] == end)|(check[len(check)-3] == end)|(check[len(check)-4] == end):
            break

        previous_count = next_count
        next_count = len(check)                         #setting the counters
        counter = counter + 1

##Part F: Path displayed as per the display asked in problem statement


    #excluding_start = path_gp[z2][1:len(path_gp[z2])]
    path_gp[z2].append(end)
    length = len(path_gp[z2])
    ###### for displaying path on image

    image1 = img.copy()
    for i in range(0,length):
        a1,b1=path_gp[z2][i]
        cv2.circle(image1,(a1,b1),2,(0,255,255),-1)

    cv2.imshow('Image',image1)

    return path_gp[z2]

#####################################################################
## Main Function
#####################################################################
if __name__ == "__main__":

## Experiment 2
    img = cv2.imread('CS_Test_Image2.jpg')
    img = cv2.resize(img,(775,775))
    print "Experiment 2 Output:"
    start = [8,770]
    end = [389,516]
    sp = shortest_path(start,end)
    print "Shortest Path:\n", sp
    print    

cv2.waitKey(0)
cv2.destroyAllWindows()
