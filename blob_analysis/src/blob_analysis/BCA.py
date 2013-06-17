# BCA.py Blob Contour Analysis - extracts blobs
#    from images and creates signatures for each blob
#
# Conrad Spiteri - c.spiteri@surrey.ac.uk
# Last updated 14/09/2012

# Required Libraries
import cv, os, heapq, sys
from optparse import OptionParser

#\/\/\/\/\/ Operational parameters \/\/\/\/\/

# Location of image files
filePath = "/home/said/workspace/workstation/sscrovers_vision/sample_training_images"

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
    objectSignature = []
    for curObj in range(len(im)):
        objectSignature.append([])
        mat=cv.GetMat(im[curObj])
        contours = cv.FindContours(mat,cv.CreateMemStorage(),cv.CV_RETR_EXTERNAL,cv.CV_CHAIN_APPROX_SIMPLE)
        cv.DrawContours(mat,contours,255,(255,0,0),-1)
        moments = cv.Moments(contours,True)
        huMoments = cv.GetHuMoments(moments)
        # Next line = Full list of moments to make up the object signature. The following line uses only scale and orientation invariant moments - Comment out one of them
#        objectSignature[curObj] = [moments.m00, moments.m10, moments.m01, moments.m20, moments.m11, moments.m02, moments.m30, moments.m21, moments.m12, moments.m03, moments.mu20, moments.mu11, moments.mu02, moments.mu30, moments.mu21, moments.mu12, moments.mu03,huMoments[0],huMoments[1],huMoments[2],huMoments[3],huMoments[4],huMoments[5],huMoments[6]]
        objectSignature[curObj] = [int(moments.m10/moments.m00),int(moments.m01/moments.m00),huMoments[0],huMoments[1],huMoments[2],huMoments[3],huMoments[4],huMoments[5],huMoments[6]]
    return objectSignature



#------------------------------------------------------------- Main Routine ---------------------------------------------------------------------

if __name__ == '__main__':
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
            imageToLoad = args[0]
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
            if args[1] == "1":
                showImages = True
                print "Show images enabled. Press any key to dismiss images."
    else:
        curFile = 31                                                        
        print "Processing Image ", fileList[curFile]
        imageToLoad = filePath + fileList[curFile]
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
    signatures = doObjectAnalysis(blobImages)                               # Create object signatures based on moments
    for k in range(len(signatures)):
        f.write(str(curFile) + "," + str(k) + "," + str(signatures[k])+"\n")# Write data to file *********** GENERATE ROS TOPIC DATA FROM HERE ***********
#       print curFile, k, signatures[k]                                         # "curFile" is simply an image number and can be ommited from the output
    if showImages == True:                                                      # "k" is the object index within an image
        for c in range(len(blobImages)):                                        # "signatures" is the signature data of an object with the above index
            cv.ShowImage(str(c), blobImages[c])
        xx = cv.WaitKey(0)
        for c in range(len(blobImages)):
            cv.DestroyWindow(str(c))
    else:
        print
    f.close
print "Done."
print
