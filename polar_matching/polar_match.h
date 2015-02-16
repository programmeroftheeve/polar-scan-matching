/*                                                                                                                         **************************************************************************
                          polar_match.h  - Matching laser scans in polar coord system
                             -------------------
    begin                : Tue Nov 9 2004
    version              : 0.3    
    copyright            : (C) 2004-2010 by Albert Diosi and Lindsay Kleeman
    email                : albert.diosi@gmail.com
    comments             : - range units are cm; angle units are radians or degrees
                           - the laser is on the robot's Y axis
                           - in scan projections, occluded ref scanpoints are not removed!
                           - TODO: Investigate why is checking the range difference necessary for 
                            the Hokuyo UTM and not for the rest.
 ***************************************************************************/
/****************************************************************************
Copyright (c) 2004-2010, Albert Diosi and Lindsay Kleeman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of the copyright holders may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/*
First of all you need to select the laser range finder you have by
setting "#define PM_LASER "
to PM_SICK_LMS200, PM_HOKUYO_URG_04LX. 
*/

#ifndef _POLAR_MATCH_
#define _POLAR_MATCH_

#include <stdio.h>

//TODO: get the result generation to work again:
//#define PM_GENERATE_RESULTS //If left uncommented scanmatching results are
                     //saved into the "results"directory if two required programs are present.

//----------------- L A S E R    S P E C I F I C    P A R A M E T E R S------------

// STEP 1) Define a name for your laser range finder here (if it hasn't been defined yet):
#define PM_PSD_SCANNER      0
#define PM_HOKUYO_URG_04LX  1
#define PM_SICK_LMS200      2
#define PM_HOKUYO_UTM_30LX  3

// STEP 2) Set the type your laser range finder here:
//  #define PM_LASER PM_SICK_LMS200
//  #define PM_LASER PM_HOKUYO_URG_04LX
//  #define PM_LASER PM_PSD_SCANNER
   #define PM_LASER PM_HOKUYO_UTM_30LX

// STEP 3) Add your laser range finder's parameters here if it is a different model (use centimeters)

#if PM_LASER ==  PM_PSD_SCANNER
  #define PM_LASER_NAME       "PSD_Scanner" ///< The name of the laser range finder. 
  #define PM_L_POINTS         200 ///< Maximum number of points in a scan.
  #define PM_FOV              360 ///< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        400 ///< [cm] Maximum valid laser range .
  #define PM_MIN_VALID_POINTS 100 ///< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    50  ///< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 ///< Threshold for angle variation between points to determine if scan was taken of a corridor.
#elif PM_LASER ==  PM_HOKUYO_URG_04LX
  #define PM_LASER_NAME       "Hokuyo URG-04LX" ///< The name of the laser range finder. 
  #define PM_L_POINTS         682 ///< Maximum number of points in a scan.
  #define PM_FOV              240 ///< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        530 ///< [cm] Maximum valid laser range.
  #define PM_MIN_VALID_POINTS 200 ///< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    80  ///< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 ///< Threshold for angle variation between points to determine if scan was taken of a corridor.
#elif PM_LASER ==  PM_SICK_LMS200
  #define PM_LASER_NAME       "Sick LMS" ///< The name of the laser range finder. 
  #define PM_L_POINTS         181  ///< Maximum number of points in a scan.
  #define PM_FOV              180  ///< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        1000 ///< [cm] Maximum valid laser range.
  #define PM_MIN_VALID_POINTS 40   ///< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    20   ///< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 ///< Threshold for angle variation between points to determine if scan was taken of a corridor.
#elif PM_LASER ==  PM_HOKUYO_UTM_30LX
  #define PM_LASER_NAME       "HOKUYO UTM-30LX" ///< The name of the laser range finder. 
  #define PM_L_POINTS         1081  ///< Maximum number of points in a scan.
  #define PM_FOV              270  ///< Field of view of the laser range finder in degrees.
  #define PM_MAX_RANGE        700  ///< [cm] Maximum valid laser range. (3000 for this sensor.)
  #define PM_MIN_VALID_POINTS 300   ///< Minimum number of valid points for scan matching.
  #define PM_SEARCH_WINDOW    200   ///< Half window size which is searched for correct orientation.
  #define PM_CORRIDOR_THRESHOLD 25.0 ///< Threshold for angle variation between points to determine if scan was taken of a corridor.
#endif

// STEP 4) Set the time registration delay (the time between your time stamps and the 
// time the scan was taken) and the distance of your laser from the odometry center 
// in the forward direction.
#define PM_TIME_DELAY       0 ///<[s] Time delay (time registration error) in the laser measurements. Use 0.02 for SLAMbot.
#define PM_LASER_Y          0 ///<@brief [cm] Y coordinate of the laser on the robot.
                              ///<
                              ///<Set to 0 for ground thruth, simulation, mapping_with_matching.
                              ///<Set to 0 when in doubt and handle the coordinate transforms yourself. 
                              ///<Set it to 31.3 for Slambot.
                              ///<The Y axis points in the forward motion direction of a differential drive robot. 
#define PM_MIN_RANGE        10.0f ///< [cm] Minimum valid laser range for the reprojected current scan.
#define PM_SEG_MAX_DIST     20.0 ///< The distance between points to break a segment. 
#define PM_WEIGHTING_FACTOR 70*70 ///< Parameter used for weighting associated points in the position calculation of PSM. Try setting it to 30*30 for laser odometry.
#define PM_CHANGE_WEIGHT_ITER 10 ///< The number of iterations after which the weighting factor is reduced to weight down outliers.

#define PM_TYPE             float ///< The variable type used in calculations. Change it to double for higher accuracy and lower speed.

#define PM_MAX_ERROR        100  ///< [cm] Maximum distance between associated points used in pose estimation. Try setting it to 30 for laser odometry.
#define PM_STOP_COND        0.4  ///< If the pose change (|dx|+|dy|+|dth|) is smaller than this PSM scan matching stops.
#define PM_MAX_ITER         30   ///< Maximum number of iterations for PSM.
#define PM_MAX_ITER_ICP     60   ///< Maximum number of iterations for ICP
#define PM_STOP_COND_ICP    0.1  ///< Stopping condition for ICP. The pose change has to be smaller than this.

#define PM_MIN_STD_XY          20.0 ///<[cm] The minimum match result standard deviation in X or Y direction. Used in covariance estimation.
#define PM_MIN_STD_ORIENTATION 4.0  ///<[degrees] The minimum standard deviation of the orientation match. Used in covariance estimation.
#define PM_MATCH_ERROR_OFFSET  5.0  ///<[cm] Offset subtracted from average range residual when scaling the covariance matrix.

#define PM_ODO  -1 ///< Show results with odometry only in mapping_with_matching().
#define PM_PSM   1 ///< Polar scan matching - matching bearing association rule.
#define PM_ICP   3 ///< Scan matching with iterative closest point association rule.

#define PM_TIME_FILE "results/iterations.txt" ///< When generating results, timing is data is saved into this file from the matching algorithms.  i|time|ax|ay|ath[deg] is saved. Does not work in the moment.

// Description of range reading errors. Each range measurement may be tagged with one of these:
#define PM_RANGE     1  ///< Measurement tag: range reading is longer than PM_MAX_RANGE.
#define PM_MOVING    2  ///< Measurement tag: range reading corresponds to a moving point. Not implemented yet.
#define PM_MIXED     4  ///< Measurement tag: range reading is a mixed pixel.
#define PM_OCCLUDED  8  ///< Measurement tag: range reading is occluded.
#define PM_EMPTY     16 ///< Measurement tag: no measurment (between 2 segments there is no interpolation!)

extern PM_TYPE   pm_fi[PM_L_POINTS];///< Contains precomputed range bearings.
extern PM_TYPE   pm_si[PM_L_POINTS];///< Contains the sinus of each bearing.
extern PM_TYPE   pm_co[PM_L_POINTS];///< Contains the cosinus of each bearing.
extern const PM_TYPE   PM_D2R; ///< Conversion factor for converting degrees to radians.
extern const PM_TYPE   PM_R2D; ///< Conversion factor for converting radians to degrees.

/** @brief Structure describing a laser scan.

The robot pose (rx,ry,th) describes the odometry center if PM_LASER_Y 
does not equal 0, or the laser center if PM_LASER_Y=0. <br>

In a laser scan, the middle laser bearing coincides with the laser coordinate frame's Y axis.

TODO: Consider using doubles for rx,ry in large environments.
*/
struct PMScan
{
  double   t;    ///<[s] Time when scan was taken.
  PM_TYPE  rx;   ///<[cm] Robot odometry X coordinate.
  PM_TYPE  ry;   ///<[cm] Robot odometry Y coordinate.
  PM_TYPE  th;   ///<[rad] Robot orientation.
  PM_TYPE  r[PM_L_POINTS];///<[cm] Laser range readings. 0 or negative ranges denote invalid readings.
  PM_TYPE  x[PM_L_POINTS];///<[cm] Laser reading X coordinates in Cartesian coordinates.
  PM_TYPE  y[PM_L_POINTS];///<[cm] Laser reading Y coordinates in Cartesian coordinates.
  int      bad[PM_L_POINTS];///< @brief Tag describing the validity of a range measurement.
                            ///< 0 if OK; sources of invalidity - out of range reading;
                            ///< reading belongs to moving object (not implemented); occlusion; mixed pixel.
  int      seg[PM_L_POINTS];///< Describes which segment the range reading belongs to.
};

void pm_init(const  char* filename=NULL, FILE **fin=NULL);
int  pm_readScan(FILE *fin, PMScan *ls);
void pm_save_scan(PMScan *act,const char *filename);

void pm_preprocessScan(PMScan *ls);

PM_TYPE pm_psm( const PMScan *lsr,PMScan *lsa);
PM_TYPE pm_icp( const PMScan *lsr,PMScan *lsa);

void pm_plotScanAt(const PMScan *ls, PM_TYPE x,PM_TYPE y,PM_TYPE th,const char *col, double diameter = 2.0, bool connect_lines = false);
void pm_plotScan(PMScan *ls, const char *col,double diameter = 2.0, bool connect_lines = false);
void pm_show_segmentation(const PMScan *ls);
void pm_plotScan4Thesis(PMScan *lsr,PMScan *lsa);
void pm_plotTime4Thesis(PM_TYPE xt, PM_TYPE yt, PM_TYPE tht,int *iter=NULL,double *time=NULL);

bool    pm_is_corridor(PMScan *act);
PM_TYPE pm_error_index(PMScan *lsr,PMScan *lsa);
PM_TYPE pm_error_index2 ( PMScan *ref,PMScan *cur, int* associatedPoints=NULL );
PM_TYPE pm_corridor_angle(PMScan *act);
void    pm_cov_est(PM_TYPE err, double *c11,double *c12, double *c22, double *c33,
                   bool corridor=false, PM_TYPE corr_angle=0);

void  pm_unit_test(int matching_alg = PM_PSM, bool interactive=true);
#endif
