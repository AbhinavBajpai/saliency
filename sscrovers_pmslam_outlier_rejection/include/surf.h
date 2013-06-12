
// This code was taken from OpenCV C++ examples and modified for PMSLAM

#include "opencv/cv.h"
#include "SURFPoint.h"

#include <vector>
using namespace std;

double compareSURFDescriptors( const float* d1, float* d2, double best, int length );

int naiveNearestNeighbor( const float* vec, int laplacian,
                      const vector<SURFPoint>* SURFDatabase );

void findPairs( const vector<SURFPoint>* SURFDatabase,
           const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs );

void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
           const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs );

/* a rough implementation for object location */
int locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                    const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                    const CvPoint src_corners[4], CvPoint dst_corners[4] );

void fillArray(float arr[], const vector<SURFPoint> *SURFDatabase, int index);
