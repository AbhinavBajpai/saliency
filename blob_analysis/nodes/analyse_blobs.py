#!/usr/bin/env python
""" 
Blob Contour Analysis - extracts blobs from images and creates signatures for each blob
Conrad Spiteri - c.spiteri@surrey.ac.uk
Said Al-Milli - s.al-milli@surrey.ac.uk
"""

# Required ROS libraries
import roslib; roslib.load_manifest('blob_analysis')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Required algorithm libraries
import cv, os, heapq, sys, hashlib, math
import MultImg as Mi

# Global Variables
itter = 0
cv_image_1 = None
cv_image_2 = None

# Setting showImages as true will display images of every object detected and waits for any key to be pressed
showImages = True

# Maximum number of objects to capture from image
maxObj = 8

class blob_analyser:
    
    def __init__(self):
        #cv.NamedWindow("Result", 1)
        #cv.NamedWindow("AdaptiveThresholding", 2)
        self.bridge = CvBridge()
        rospy.init_node('blob_analyser', anonymous=True)
        self.image_pub = rospy.Publisher("image_topic_1",Image)
        self.image_sub = rospy.Subscriber("cam_image",Image,self.image_callback)

    def image_callback(self,img):
        rospy.loginfo("Image received... [Seq No = %i]",img.header.seq)
        global itter, cv_image_1, cv_image_2
        
        if (itter == 0) :
            try:
                cv_image_2 = self.bridge.imgmsg_to_cv(img, "mono8")
            except CvBridgeError, e:
                print e
        
        try:
            cv_image_1 = cv_image_2
            cv_image_2 = self.bridge.imgmsg_to_cv(img, "mono8")
        except CvBridgeError, e:
            print e

        self.analyse_blobs()
                
        rospy.sleep(1.0)
        itter += 1

        try:
           self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image_1, "mono8"))
        except CvBridgeError, e:
            print e
            
    def doDisplayImages(self,loc):
        rospy.loginfo("displaying results...")
        img = Mi.combineImages( cv_image_1,  cv_image_2)
        for g in range(len(loc)):
            cv.Line(img, (loc[g][0][0],loc[g][0][1]), (loc[g][1][0],loc[g][1][1]+ cv_image_1.height), (255,255,255), thickness=5, lineType=8, shift=0) 
        return img
    
    def analyse_blobs(self):
        global itter
        rospy.loginfo("analysing blobs...")                                          # File name for data storage
        destination = self.doAdaptiveThresholding(cv_image_1, 321, 10)                 # Thresholds the image using adaptive thresholding with a gaussian window size of 321 and a subtraction constant of 10
        #imageMask = doErodeDilate(destination, ["Dilate","Erode","Erode","Dilate"])# Smooth the blobs and remove noise - computationaly expensive. Replaced by doExtractLargestBlobs
        blobs, objectCount = self.doBlobDetect(destination)                          # Index all blobs in the image
        blobImages = self.doExtractLargestBlobs(blobs, objectCount)                  # Create individual images for the largest k number of objects: k = maxObj
        signatures, objLocation = self.doObjectAnalysis(blobImages)                               # Create object signatures based on moments
        
        destination2 = self.doAdaptiveThresholding(cv_image_2, 321, 10)                 # Thresholds the image using adaptive thresholding with a gaussian window size of 321 and a subtraction constant of 10
        #imageMask = doErodeDilate(destination, ["Dilate","Erode","Erode","Dilate"])# Smooth the blobs and remove noise - computationaly expensive. Replaced by doExtractLargestBlobs
        blobs2, objectCount2 = self.doBlobDetect(destination2)                          # Index all blobs in the image
        blobImages2 = self.doExtractLargestBlobs(blobs2, objectCount2)                  # Create individual images for the largest k number of objects: k = maxObj
        signatures2, objLocation2 = self.doObjectAnalysis(blobImages2)                               # Create object signatures based on moments
        
        matches = self.doCompareBlobs(signatures,signatures2)
        matchIndex = []
        matchLocation = []
        for z in range(len(matches)):
            if min(matches[z]) <= 0.1 and min(matches[z]) > 0:
                matchIndex.append([z,matches[z].index(min(matches[z]))])
                matchLocation.append([objLocation[z],objLocation2[matches[z].index(min(matches[z]))]])
        print "Matches - ", matchIndex; print
        print "Location - ", matchLocation; print
        
        finalImg = self.doDisplayImages(matchLocation)
    
        cv.ShowImage("Result", finalImg)
        cv.WaitKey(1)

    def doAdaptiveThresholding(self,img,WinSize,cVal):
        
        destination = cv.CreateImage((img.width,img.height), cv.IPL_DEPTH_8U, 1)
        cv.AdaptiveThreshold(img, destination,255,cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C, cv.CV_THRESH_BINARY,WinSize,cVal)
        cv.ShowImage("AdaptiveThresholding", destination)
        cv.WaitKey(1)
        return destination
    
    def doBlobDetect(self,img):
        objectCount = 1
        connectivity = 8; flags = connectivity; flags |= cv.CV_FLOODFILL_FIXED_RANGE
        for h in range(img.height):
                for w in range(img.width):
                    pixVal = cv.Get2D(img, h, w)
                    if (pixVal[0] == 0):
                        cv.FloodFill(img, (w,h), (objectCount, objectCount, objectCount), 1, 1, flags)
                        objectCount += 1
        return img, objectCount
    
    def doErodeDilate(self,src, mode):
        for curMode in mode:
            if curMode == "Erode":
                cv.Erode(src, src)
            elif curMode == "Dilate":
                cv.Dilate(src, src)
        return src
    
    def doExtractLargestBlobs(self,imgOrig, objectCount):
        global cv_image_1
        rospy.loginfo("extracting largest blobs...")
        objectIndex = []
        objectArea = []
        imageElements = []
        img = cv_image_1
        #cv.CvtColor(imgOrig ,img, cv.CV_GRAY2RGB)
        for i in range(1, objectCount):
            zm=0.0
            for h in range(img.height):
                for w in range(img.width):
                    pixVal = cv.Get2D(img, h, w)
                    if (pixVal[0] == i):
                        zm=zm+1
            objectArea.append([int(zm)])
        largestObjects = heapq.nlargest(maxObj, objectArea)
        for s in largestObjects:
            objectIndex.append(objectArea.index(s)+1)
        for e in range(len(objectIndex)):
            imageElements.append([])
            imageElements[e] = cv.CreateImage((img.width,img.height), cv.IPL_DEPTH_8U, 1)
            cv.Zero(imageElements[e])
            for h in range(img.height):
                for w in range(img.width):
                    pixVal = cv.Get2D(img, h, w)
                    if (pixVal[0] == objectIndex[e]):
                        cv.Set2D(imageElements[e], h, w, 255)
        return imageElements
     
    def doObjectAnalysis(self,img):
        rospy.loginfo("object analysis...")
        objectLocation = []
        contourArray = []
        for curObj in range(len(img)):
            objectLocation.append([])
            mat=cv.GetMat(img[curObj])
            contourArray.append(cv.FindContours(mat,cv.CreateMemStorage(),cv.CV_RETR_EXTERNAL,cv.CV_CHAIN_APPROX_SIMPLE))
            cv.DrawContours(mat,contourArray[-1],255,(255,0,0),-1)
            moments = cv.Moments(contourArray[-1],True)
            huMoments = cv.GetHuMoments(moments)
            # Next line = Full list of moments to make up the object signature. The following line uses only scale and orientation invariant moments - Comment out one of them
    #        objectSignature[curObj] = [moments.m00, moments.m10, moments.m01, moments.m20, moments.m11, moments.m02, moments.m30, moments.m21, moments.m12, moments.m03, moments.mu20, moments.mu11, moments.mu02, moments.mu30, moments.mu21, moments.mu12, moments.mu03,huMoments[0],huMoments[1],huMoments[2],huMoments[3],huMoments[4],huMoments[5],huMoments[6]]
            if moments.m00 != 0:
                objectLocation[curObj] = [int(moments.m10/moments.m00),int(moments.m01/moments.m00)]
            else:
                objectLocation[curObj] = ['E','E']
                        
        return contourArray, objectLocation
    
    
    def doCompareBlobs(self,img1,img2):
        rospy.loginfo("comparing blobs...")
        c1 = []
        matches = []
        for imgOne in range(len(img1)):
            for imgTwo in range(len(img2)):
                temp = cv.MatchShapes(img1[imgOne], img2[imgTwo], cv.CV_CONTOURS_MATCH_I1)
                if temp == 0:
                    temp = 2.0
                c1.append(temp)
            matches.append(c1)
            c1 = []
        return matches
       


            
def main(args):
    ba = blob_analyser()        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()
  

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
