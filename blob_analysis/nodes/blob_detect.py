#!/usr/bin/env python
""" 
Blob Contour Analysis - extracts blobs from images and creates signatures for each blob
Conrad Spiteri - c.spiteri@surrey.ac.uk
Said Al-Milli - s.al-milli@surrey.ac.uk
"""

# Required ROS libraries
import roslib; roslib.load_manifest('blob_analysis')
import rospy

# Required algorithm libraries
import cv, os, heapq, sys, hashlib, math
from operator import attrgetter
from optparse import OptionParser
import MultImg as Mi

#\/\/\/\/\/ Operational parameters \/\/\/\/\/

# Location of image files
filePath = "/home/said/workspace/ros_sb/blob_analysis/sample_images/"

# List of path contents
fileList = os.listdir(filePath)

# Setting showImages as true will display images of every object detected and waits for any key to be pressed
showImages = True

# Maximum number of objects to capture from image
maxObj = 8

#---------------------------------------------------------------------------------------------------------#
def doAdaptiveThresholding(im,WinSize,cVal):
    
    destination = cv.CreateImage((im.width,im.height), cv.IPL_DEPTH_8U, 1)
    cv.AdaptiveThreshold(im, destination,255,cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C, cv.CV_THRESH_BINARY,WinSize,cVal)
    return destination

def doBlobDetect(im):
    objectCount = 1
    connectivity = 8; flags = connectivity; flags |= cv.CV_FLOODFILL_FIXED_RANGE
    for h in range(im.height):
            for w in range(im.width):
                pixVal = cv.Get2D(im, h, w)
                if (pixVal[0] == 0):
                    cv.FloodFill(im, (w,h), (objectCount, objectCount, objectCount), 1, 1, flags)
                    objectCount += 1
    return im, objectCount

def doErodeDilate(src, mode):
    for curMode in mode:
        if curMode == "Erode":
            cv.Erode(src, src)
        elif curMode == "Dilate":
            cv.Dilate(src, src)
    return src

def doExtractLargestBlobs(imOrig, objectCount):
    objectIndex = []
    objectArea = []
    imageElements = []
    im = cv.LoadImageM(imageToLoad, cv.CV_LOAD_IMAGE_COLOR)
    cv.CvtColor(imOrig ,im, cv.CV_GRAY2RGB)
    for i in range(1, objectCount):
        zm=0.0
        for h in range(im.height):
            for w in range(im.width):
                pixVal = cv.Get2D(im, h, w)
                if (pixVal[0] == i):
                    zm=zm+1
        objectArea.append([int(zm)])
    largestObjects = heapq.nlargest(maxObj, objectArea)
    for s in largestObjects:
        objectIndex.append(objectArea.index(s)+1)
    for e in range(len(objectIndex)):
        imageElements.append([])
        imageElements[e] = cv.CreateImage((im.width,im.height), cv.IPL_DEPTH_8U, 1)
        cv.Zero(imageElements[e])
        for h in range(im.height):
            for w in range(im.width):
                pixVal = cv.Get2D(im, h, w)
                if (pixVal[0] == objectIndex[e]):
                    cv.Set2D(imageElements[e], h, w, 255)
    return imageElements


def doObjectAnalysis(im):
    objectLocation = []
    contourArray = []
    for curObj in range(len(im)):
        objectLocation.append([])
        mat=cv.GetMat(im[curObj])
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


def doCompareBlobs(im1,im2):
    c1 = []
    matches = []
    for imOne in range(len(im1)):
        for imTwo in range(len(im2)):
            temp = cv.MatchShapes(im1[imOne], im2[imTwo], cv.CV_CONTOURS_MATCH_I1)
            if temp == 0:
                temp = 2.0
            c1.append(temp)
        matches.append(c1)
        c1 = []
    return matches


def doDisplayImages(loc):
    im = Mi.combineImages( curImage,  curImage2)
    for g in range(len(loc)):
        cv.Line(im, (loc[g][0][0],loc[g][0][1]), (loc[g][1][0],loc[g][1][1]+curImage.height), (0,0,255), thickness=1, lineType=8, shift=0) 
    return im


#------------------------------------------------------------- Main Routine ---------------------------------------------------------------------

if __name__ == '__main__':
    
    rospy.init_node('blob_detect', anonymous=True) 
 
    parser = OptionParser(usage = "usage: %prog [image] [show] \n\n[image] -- an image name in .jpg or .bmp format or an image number from list of images sorted alphanumerically in default folder. \n\n[show] -- a switch (1 or 0) indicating the preference whether to show resultant images")
    (options, args) = parser.parse_args()
    if len(args) >= 1:
        if args[0] == "help" or args[0] == "?":
            parser.print_help()
            sys.exit(1)
        elif args[0].isdigit():
            curFile = int(args[0])                                          # Image number from list of images sorted alphanumerically
            imageToLoad = filePath + fileList[curFile]
            if imageToLoad[-4:] != ".jpg" and imageToLoad[-4:] != ".bmp":   # Check that the file has  a .jpg or .bmpextention
                print "\n------------------------"
                print "\nIncorrect file extension", imageToLoad
                print "\n------------------------\n"
                parser.print_help()
                sys.exit(1)
            else:
                print "Processing Image ", fileList[curFile]
        else:
            imageToLoad  = args[0]
            imageToLoad2 = args[1]
            curFile = 0
            if imageToLoad[-4:] != ".jpg" and imageToLoad[-4:] != ".bmp":   # Check that the file has  a .jpg or .bmpextention
                print "\n------------------------"
                print "\nIncorrect file extension", imageToLoad
                print "\n------------------------\n"
                parser.print_help()
                sys.exit(1)
            else:
                print "Processing Image ", imageToLoad
        if len(args) >= 2:
            if args[2] == "1":
                showImages = True
                print "Show images enabled. Press any key to dismiss images."
    else:
        curFile = 115
        curFile2 = 120
        print "Processing Image ", fileList[curFile], " and ", fileList[curFile2]; print
        imageToLoad = filePath + fileList[curFile]
        imageToLoad2 = filePath + fileList[curFile2]
        if imageToLoad[-4:] != ".jpg" and imageToLoad[-4:] != ".bmp":       # Check that the file has  a .jpg or .bmpextention
                print "\n------------------------"
                print "\nImage load failed - ", imageToLoad
                print "\n------------------------\n"
                parser.print_help()
                sys.exit(1)

#-------------------------------------------------------Where the actual code begins-------------------------------------------------------
    
    f = open('data.txt', 'w')                                               # File name for data storage
    curImage = cv.LoadImageM(imageToLoad, cv.CV_LOAD_IMAGE_GRAYSCALE)       # Load image in Grey scale
    destination = doAdaptiveThresholding(curImage, 321, 10)                 # Thresholds the image using adaptive thresholding with a gaussian window size of 321 and a subtraction constant of 10
    #imageMask = doErodeDilate(destination, ["Dilate","Erode","Erode","Dilate"])# Smooth the blobs and remove noise - computationaly expensive. Replaced by doExtractLargestBlobs
    blobs, objectCount = doBlobDetect(destination)                          # Index all blobs in the image
    blobImages = doExtractLargestBlobs(blobs, objectCount)                  # Create individual images for the largest k number of objects: k = maxObj
    signatures, objLocation = doObjectAnalysis(blobImages)                               # Create object signatures based on moments
    
    curImage2 = cv.LoadImageM(imageToLoad2, cv.CV_LOAD_IMAGE_GRAYSCALE)       # Load image in Grey scale
    destination2 = doAdaptiveThresholding(curImage2, 321, 10)                 # Thresholds the image using adaptive thresholding with a gaussian window size of 321 and a subtraction constant of 10
    #imageMask = doErodeDilate(destination, ["Dilate","Erode","Erode","Dilate"])# Smooth the blobs and remove noise - computationaly expensive. Replaced by doExtractLargestBlobs
    blobs2, objectCount2 = doBlobDetect(destination2)                          # Index all blobs in the image
    blobImages2 = doExtractLargestBlobs(blobs2, objectCount2)                  # Create individual images for the largest k number of objects: k = maxObj
    signatures2, objLocation2 = doObjectAnalysis(blobImages2)                               # Create object signatures based on moments
    
    matches = doCompareBlobs(signatures,signatures2)
    matchIndex = []
    matchLocation = []
    for z in range(len(matches)):
        if min(matches[z]) <= 0.1 and min(matches[z]) > 0:
            matchIndex.append([z,matches[z].index(min(matches[z]))])
            matchLocation.append([objLocation[z],objLocation2[matches[z].index(min(matches[z]))]])
    print "Matches - ", matchIndex; print
    print "Location - ", matchLocation; print
#    for p in matches:
#        print p
#        print
    finalImg = doDisplayImages(matchLocation)

    for k in range(len(signatures)):
        f.write(str(curFile) + "," + str(k) + "," + str(signatures[k])+"\n")# Write data to file *********** GENERATE ROS TOPIC DATA FROM HERE ***********
        # print k, signatures[k], hashlib.sha224(str(signatures[k])).hexdigest()   
    if showImages == True:                                                      # "k" is the object index within an image
        for c in range(len(matchIndex)):                                        # "signatures" is the signature data of an object with the above index
            cv.ShowImage("1_"+str(matchIndex[c][0]), blobImages[matchIndex[c][0]])
        for c in range(len(matchIndex)):                                        # "signatures" is the signature data of an object with the above index
            cv.ShowImage("2_"+str(matchIndex[c][1]), blobImages2[matchIndex[c][1]])
        cv.ShowImage("Result", finalImg)
        xx = cv.WaitKey(0)
        for c in range(len(matchIndex)):
            cv.DestroyWindow("1_"+str(matchIndex[c][0]))
        for c in range(len(matchIndex)):                                        # "signatures" is the signature data of an object with the above index
            cv.DestroyWindow("2_"+str(matchIndex[c][1]))
        cv.DestroyWindow("Result")
    else:
        print
    f.close
print "Done."
print
