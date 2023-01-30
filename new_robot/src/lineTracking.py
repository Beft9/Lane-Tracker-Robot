#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sklearn.cluster import KMeans

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                self.command = 1 # 0 for lane track; 1 for left lane, 2 for right lane, 3 for sit lane

                self.bottom_y_point = 610
                self.upper_y_point = 470

                self.linear_vel = 0.2
                self.max_angular_vel = 0.6

                self.slope_provided = False

                self.twist = Twist()

        def image_distance_calc(x1,y1,x2,y2, x3,y3,x4,y4):
                (y4-y2)**2 + (x4-x2)

        def calculate_line_slope(self, index): # 0 for left line; 1 for right line
                if(index == 0):
                        return (self.upper_y_point - self.bottom_y_point)* 1.0 / (self.x1_2 - self.x1_1)
                else: 
                        return (self.upper_y_point - self.bottom_y_point) / (self.x2_2 - self.x2_1)
        
        def track_lane(self, w):
                err = ((self.x1_1 + self.x2_1) / 2) - w/2
                # print("err:",err)
                self.publish_vel(err)
                
        
        def change_lane(self, w):
                # print("Diff:",  self.x2_1-self.x1_1)
                # print("right lane:", self.x2_1)
                print("Left lane:", self.x1_1)

                if(self.calculate_line_slope(0) < -1.15):
                        self.slope_provided = True
                if(self.slope_provided and self.x2_1 > 770 and self.x2_1 < 820):
                        self.slope_provided = False
                        print("Change Lane Finished")
                        self.command = 3
                        return
                err = self.x1_1 - 100 - w/2 # focus on 100 px left of left line bottom point
                # print("err:",err)
                if(self.x1_1 < 0):
                        self.publish_vel(0)
                else:
                        self.publish_vel(err)

        def sit_lane(self, w):
                
                print("left line tracking")
                err = self.x1_1 + 300 - w/2 # focus on 200 px left of right line bottom point
                self.publish_vel(err)
                print("left lane slope:",self.calculate_line_slope(0)) 
                if (abs(err) < 4 ):
                        print("Robot sitted the lane!")
                        self.command = 0

        
        def publish_vel(self, err):
                self.twist.linear.x = self.linear_vel
                angular_vel = -float(err) / 250
                if(angular_vel > self.max_angular_vel):
                        angular_vel = self.max_angular_vel
                elif(angular_vel < -self.max_angular_vel):
                        angular_vel = -self.max_angular_vel
                self.twist.angular.z = angular_vel
                self.cmd_vel_pub.publish(self.twist)



        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                h, w, d = image.shape
                search_top = 3*h/4
                search_bot = 3*h/4 + 20
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                dst = cv2.Canny(image, 50, 200, None, 3)
                linesP = cv2.HoughLinesP(dst, 1, 3.14 / 180, 50, None, 50, 10)

                # print(linesP)
                my_lines = []
                if linesP is not None:
                    for i in range(0, len(linesP)):
                        #
                        l = linesP[i][0]
                        egim = (l[3]*1.0 - l[1]) / (l[2]*1.0 -l[0])
                        
                        if(abs(egim) > 0.3):
                            cv2.line(image, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
                            my_lines.append(linesP[i])

                if(len(my_lines) < 2):
                        return
                my_lines = numpy.array(my_lines)[:,0],
                my_lines = my_lines[0] 
                
                diffs = my_lines[:,2:] - my_lines[:,:2] 
                               
                kmeans = KMeans(n_clusters=2, n_init=30, max_iter=600).fit(diffs)

                list1 = []
                list2 = []
                for i in range(len(kmeans.labels_)):
                        if(kmeans.labels_[i] == 0):
                                list1.append(my_lines[i])
                        else:
                                list2.append(my_lines[i])
                list1 = numpy.array(list1)
                list2 = numpy.array(list2)

                # print(kmeans)

                # print(kmeans.labels_)
                
                slopes1 = numpy.array(list1[:,2:] - list1[:,:2],dtype=numpy.float) 
                # print("slopes1:")
                slopes1 = slopes1[:,1] / slopes1[:,0]
                median1_index = numpy.argsort(slopes1)[len(slopes1)//2]
                # print("median1:",median1_index)

                slopes2 = numpy.array(list2[:,2:] - list2[:,:2],dtype=numpy.float) 
                # print("slopes2:")
                slopes2 = slopes2[:,1] / slopes2[:,0]
                median2_index = numpy.argsort(slopes2)[len(slopes2)//2]
                # print("median1:",median2_index)
                

                #line1 = numpy.mean(list1, axis=0, dtype=numpy.int32)
                #line2 = numpy.mean(list2, axis=0, dtype=numpy.int32)
                line1 = list1[median1_index]
                line2 = list2[median2_index]



                # print(line1)
                # print(line2)
                # (0, 610)
                egim1 = (line1[3]-line1[1])*1.0/(line1[2]-line1[0]) 
                # print(line1[3]-line1[1])
                # print(line1[2]-line1[0])
                # dogru denklemi: y = egim*x + c
                # egim*

                


                c1 = line1[1] - line1[0]*egim1
                self.x1_1 = int(( self.bottom_y_point -c1 ) / egim1 ) 
                self.x1_2 = int(( self.upper_y_point -c1 ) / egim1 )


                egim2 = (line2[3]-line2[1])*1.0/(line2[2]-line2[0] )

                c2 = line2[1] - line2[0]*egim2
                self.x2_1 = int( (self.bottom_y_point - c2) / egim2 )
                self.x2_2 = int( (self.upper_y_point - c2) / egim2 )
                
                # print("egim2:",egim2)
                if(self.x2_1 < self.x1_1):      # check if tghe left line is x1, if it is not change 
                        temp1 = self.x2_1
                        temp2 = self.x2_2
                        
                        self.x2_1 = self.x1_1
                        self.x2_2 = self.x1_2

                        self.x1_1 = temp1
                        self.x1_2 = temp2


                #cv2.circle(image, (0, 610 ), 20, (0,255,255), -1)

                cv2.circle(image, (self.x1_1, self.bottom_y_point ), 20, (255,255,255), -1)
                cv2.circle(image, (self.x1_2, self.upper_y_point ), 20, (255,255,255), -1)
                cv2.circle(image, (self.x2_1, self.bottom_y_point ), 20, (0,255,255), -1)
                cv2.circle(image, (self.x2_2, self.upper_y_point ), 20, (0,255,255), -1)

                center = image.shape[1]


                cv2.line(image, (line1[0], line1[1]), (line1[2], line1[3]), (255,255,255), 3, cv2.LINE_AA)
                cv2.line(image, (line2[0], line2[1]), (line2[2], line2[3]), (255,255,255), 3, cv2.LINE_AA)

                cv2.line(image, (self.x1_1, self.bottom_y_point), (self.x1_2, self.upper_y_point), (255,255,0), 3, cv2.LINE_AA)
                cv2.line(image, (self.x2_1, self.bottom_y_point), (self.x2_2, self.upper_y_point), (255,255,0), 3, cv2.LINE_AA)
                
                # print("left line Slope:", self.calculate_line_slope(0))
                # M = cv2.moments(mask)
                # if M['m00'] > 0:
                #         cx = int(M['m10']/M['m00'])
                #         cy = int(M['m01']/M['m00'])
                cv2.circle(image, ((self.x1_1 + self.x2_1) / 2, self.bottom_y_point), 20, (0,0,255), -1)
#The proportional controller is implemented in the following four lines which
#is reposible of linear scaling of an error to drive the control output.
                if(self.command == 0):
                        self.track_lane(w)
                elif(self.command == 1):
                        self.change_lane(w)
                elif(self.command == 3):
                        self.sit_lane(w)
                # err = cx - 620 #w/2
                # print("cx:",cx)
                        
                cv2.imshow("window", image)
                cv2.waitKey(3)


rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
