/***************************************************************************
             polar_match.cpp  - Matching laser scans in polar coord system
                             -------------------
begin           : Tue Nov 9 2004
version         : 0.3
copyright       : (C) 2004-2010 by Albert Diosi and Lindsay Kleeman
email           : albert.diosi@gmail.com
comments        : - range units are cm; angle units are radians
                  - don't forget to set the optimization switch -O2!
                  - if you change PM_TYPE to double change fabsf->fabs, floorf->floor, ...
changed:        2007-2010 - major changes.
                05/03/2007- add interpolation to icp
                04/11/2005- bug fixed in pm_is_corridor. Bug pointed out by Alan Zhang
                03/08/2005- simple implementation of IDC added not working yet - remove!
                26/05/2005- projection filter of ICP fixed
                7/12/2004 - dx,dy estimation interleaved with dth estimation
                            seems to work OK
                3/12/2004 - iterative range least squares seems to work (for
                           dx,dy only), though it needs to be thoroughly tested
                
TODO: - Comment colours used when GR is defined.      
      - Optimize the orientation search. Perform adaptive subsampling 
        on the reference scan.
      - Check if calculations overflow when using float for lasers 
        with long range and lot of points: Hokuyo UTM-30LX.
                         
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

#include <iostream>
//#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __linux__
#include <time.h>
#endif

#include "polar_match.h"
#include "draw.h"


using namespace std;

//#define GR //STEP 5) When defined graphical debugging of PSM is enabled: each projection, orientation search and translation estimation iteration is shown.

PM_TYPE         pm_fi[PM_L_POINTS];//contains precomputed angles (0-180)
PM_TYPE         pm_si[PM_L_POINTS];//contains sinus of angles
PM_TYPE         pm_co[PM_L_POINTS];//contains cos of angles
const PM_TYPE   PM_D2R = M_PI/180.0; // degrees to rad
const PM_TYPE   PM_R2D = 180.0/M_PI; // rad to degrees
const PM_TYPE   PM_FI_MIN = M_PI/2.0 - PM_FOV*PM_D2R/2.0;//[rad] bearing from which laser scans start
const PM_TYPE   PM_FI_MAX = M_PI/2.0 + PM_FOV*PM_D2R/2.0;//[rad] bearing at which laser scans end
const PM_TYPE   PM_DFI    = PM_FOV*PM_D2R/ ( PM_L_POINTS + 1.0 );//[rad] angular resolution of laser scans

void pm_scan_project(const PMScan *act, PM_TYPE *new_r, int *new_bad);
PM_TYPE pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad);
PM_TYPE pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy);

/** @brief Returns thread runtime under Linux in milliseconds.

@return [ms] Thread run time.
*/
double pm_msec(void)
{
#ifdef __linux__
  timespec tp;
  double time;  
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tp);
  time = tp.tv_sec*1000.0 + ((double)tp.tv_nsec)/1000000.0;
  return time;
#else
  return -1.0;
#endif  
}

/** @brief Normalize angle.

Normalize angle to be within [-pi,pi).

TODO: Make more efficient. 
@param a The angle to be normalized.
*/
inline PM_TYPE norm_a ( PM_TYPE a )
{
  int m;
  m = (int) ( a / ( 2.0*M_PI ) );
  a = a - (PM_TYPE)m * M_PI;
  if ( a < (-M_PI) )
    a += 2.0*M_PI;
  if ( a >= M_PI )
    a -= 2.0*M_PI;
  return ( a );
}

/** @brief Initialises internal variables and opens a log file.

If filename is present, then it opens the scanfile and reads out
the first line. The file pointer is then set to the corresponding file.
Before performing any scan matching call this fuction once to initialize
important global variables.

Upon failure an exception is thrown.

@param filename The name of the optional logfile to be used.
@param fin The optional opened file pointer is returned here.
*/
void pm_init ( const char* filename, FILE **fin )
{
  printf("Initializing polar scan matching for: %s\n",PM_LASER_NAME);
  size_t len=2000;
  char *line;
  line = new char[len];
  if(fin != NULL)
  {
    *fin = NULL;
  }

  if(filename != NULL)
  {
    *fin=fopen ( filename,"rt" );
    if ( *fin==NULL )
    {
      cerr <<"pm_init: unable to open file:"<<filename<<endl;
      throw 1;
    }
    getline ( &line,&len,*fin );
    cout <<line<<endl;
    delete[] line;
  }
  
  for ( int i=0;i<PM_L_POINTS;i++ )
  {
    pm_fi[i] = ( ( float ) i ) *PM_DFI + PM_FI_MIN;
    pm_si[i] = sinf ( pm_fi[i] );
    pm_co[i] = cosf ( pm_fi[i] );
  }
}//pm_init



/** @brief Reads one scan from file @a fin and stores it in @a ls.

Assumed data format: <br>
time[s] x[m] y[m] theta[rad]<br>
r0[cm]<br>
r1[cm]<br>
.<br>
.<br>
.<br>
@param fin Pointer to the file opened with pm_init().
@param ls The read laser scan is returned here.
@return Returns 0 on success, -1 if there are no more scans.
*/
int pm_readScan ( FILE *fin, PMScan *ls )
{
  int n=0;
  n+=fscanf ( fin,"%lf %f %f %f\n",& ( ls->t ),& ( ls->rx ),& ( ls->ry ),& ( ls->th ) );
  ls->t     -=PM_TIME_DELAY;
  ls->rx    *=100.0;//convert into cm
  ls->ry    *=100.0;
  ls->th    = norm_a ( ls->th-M_PI/2.0 );// subtract 90 degrees - due to the coordinate frame definition for SLAMbot

  for ( int i=0; i<PM_L_POINTS; i++ )
  {
    n+=fscanf ( fin,"%f\n",& ( ls->r[i] ) );
    ls->x[i] = ( ls->r[i] ) *pm_co[i];
    ls->y[i] = ( ls->r[i] ) *pm_si[i];
    ls->bad[i] = 0;
    //because the logs are of double resolution, have to discard every second one...
    if(i!=(PM_L_POINTS-1))
    {
      PM_TYPE dummy;
      fscanf ( fin,"%f\n",& dummy);
    }
  }
  if ( n!= ( PM_L_POINTS+4 ) )
    return -1;
  return 0;
}

/** @brief Plots scan @a ls at robot/laser pose @a x, @a y, @a th.

The scan is shifted by PM_LASER_Y.
@param ls The laser scan to be plotted. The scan's pose is ignored.
@param x The X coordinate of the pose where the scan should be drawn.
@param y The Y coordinate of the pose where the scan should be drawn.
@param th The orientation coordinate of the pose where the scan should be drawn.
@param col The colour of the scan as "red", "green"... See @a dr_COLORS for more.
@param diameter [cm] The drawn diameter of the measured points.
@param connect_lines If true, measured point are connected with lines.
*/
void pm_plotScanAt(const PMScan *ls, PM_TYPE x,PM_TYPE y,PM_TYPE th,const char *col, double diameter, bool connect_lines)
{
  float xx,yy,xw,yw,last_xw=0,last_yw=0;
  float si_th = sinf ( th );
  float co_th = cosf ( th );
  char *color;
  char  orig_col[1000];

  strcpy(orig_col,col);

  dr_circle ( x,y,10.0,col );
  dr_line ( x,y,x+10.0*cosf ( th+M_PI/2.0 ), y+10.0*sinf ( th+M_PI/2.0 ),col );
  for ( int i=0;i<PM_L_POINTS;i+=1 )
  {
    if ( ls->r[i]>PM_MAX_RANGE )
      continue;
    xx  = ls->r[i]*pm_co[i];
    yy  = ls->r[i]*pm_si[i]  + PM_LASER_Y;
    xw = xx*co_th - yy*si_th  + x;
    yw = xx*si_th + yy*co_th  + y;
    if ( ls->bad[i]==0 )
    {
      color = orig_col;
      dr_circle ( xw,yw,diameter,color );
    }
    else //bad points are drawn green
    {
      color = orig_col;//"green";
      dr_circle ( xw,yw,diameter,color );
    }
    if(connect_lines && i!=0)
    {
       dr_line(xw,yw,last_xw,last_yw,color);
    }
    last_xw = xw;
    last_yw = yw;
  }
}

/** @brief Plots scan @a ls.

The scan is shifted by PM_LASER_Y.
@param ls The laser scan to be plotted at the scan's pose.
@param col The colour of the scan as "red", "green"... See @a dr_COLORS for more.
@param diameter [cm] The drawn diameter of the measured points.
@param connect_lines If true, measured points are connected with lines.
*/
void pm_plotScan ( PMScan *ls, const char *col,double diameter, bool connect_lines)
{
  pm_plotScanAt(ls,ls->rx,ls->ry,ls->th,col,diameter,connect_lines);
}


/** @brief Filters the laser ranges with a median filter.

The job of this median filter is to remove chair and table 
legs which are likely to change position with time.

The median filter helps to get rid of spurious data.
If the median filter's window is 5, then 3 points need be 
close to each other to surrive the filtering. Chair legs taking
1 or 2 range readings will be removed.

Do not use this function when fitting lines to laser scans!

Median filter will round up corners.

x,y coordinates of points are not upadted.
@param ls Laser scan to be filtered.
*/
void pm_median_filter ( PMScan *ls )
{
  const int HALF_WINDOW  = 2;//2 to left 2 to right
  const int WINDOW = 2*HALF_WINDOW+1;
  PM_TYPE   r[WINDOW];
  PM_TYPE   w;

  int i,j,k,l;

  for ( i=0;i<PM_L_POINTS;i++ )
  {
    k=0;
    for ( j=i-HALF_WINDOW;j<=i+HALF_WINDOW;j++ )
    {
      l = ( ( j>=0 ) ?j:0 );
      r[k]=ls->r[ ( ( l < PM_L_POINTS ) ?l: ( PM_L_POINTS-1 ) ) ];
      k++;
    }
    //bubble sort r
    for ( j= ( WINDOW-1 );j>0;j-- )
      for ( k=0;k<j;k++ )
        if ( r[k]>r[k+1] ) // wrong order? - swap them
        {
          w=r[k];
          r[k]=r[k+1];
          r[k+1] = w;
        }
    ls->r[i] = r[HALF_WINDOW];//choose the middle point
  }
}


/** @brief Segments scanpoints into groups based on range discontinuities.

By segmenting scans into groups of disconnected sets of points, one can
prevent falsely interpolating points into the free space between disconnected
objects during scan projection.

Segment number 0 is reserved to segments containing only 1 point.

Far away points (r > PM_MAX_RANGE), gaps between groups of 
points - divide segments. The gap between extrapolated point and
current point has to be large as well to prevent corridor walls to
be segmented into separate points.
*/
void pm_segment_scan ( PMScan *ls )
{
  const PM_TYPE   MAX_DIST = PM_SEG_MAX_DIST;//max range diff between conseq. points in a seg
  PM_TYPE   dr;
  int       seg_cnt = 0;
  int       i,cnt;
  bool      break_seg;

  seg_cnt = 1;

  //init:
  if ( fabsf ( ls->r[0]-ls->r[1] ) < MAX_DIST ) //are they in the same segment?
  {
    ls->seg[0] = seg_cnt;
    ls->seg[1] = seg_cnt;
    cnt        = 2;    //2 points in the segment
  }
  else
  {
    ls->seg[0] = 0; //point is a segment in itself
    ls->seg[1] = seg_cnt;
    cnt        = 1;
  }

  for ( i=2;i<PM_L_POINTS;i++ )
  {
    //segment breaking conditions: - bad point;
    break_seg = false;
    if ( ls->bad[i] )
    {
      break_seg = true;
      ls->seg[i] = 0;
    }
    else
    {
      dr = ls->r[i]- ( 2.0*ls->r[i-1] - ls->r[i-2] );//extrapolate & calc difference
      //Don't break a segment if the distance between points is small
      //or the distance beween the extrapolated point and current point is small.
      if ( fabsf ( ls->r[i]-ls->r[i-1] ) < MAX_DIST || 
         ( ( ls->seg[i-1]==ls->seg[i-2] ) && fabsf ( dr ) <MAX_DIST ) )
      {
        //not breaking the segment
        cnt++;
        ls->seg[i] = seg_cnt;
      }
      else
        break_seg = true;
    }//if ls->bad
    
    if ( break_seg ) // breaking the segment?
    {
      if ( cnt==1 )
      {
        //check first if the last three are not on a line by coincidence
        dr = ls->r[i]- ( 2.0*ls->r[i-1]-ls->r[i-2] );
        if ( ls->seg[i-2] == 0 && ls->bad[i] == 0 && ls->bad[i-1] == 0
                && ls->bad[i-2] == 0 && fabsf ( dr ) <MAX_DIST )
        {
          ls->seg[i]   = seg_cnt;
          ls->seg[i-1] = seg_cnt;
          ls->seg[i-2] = seg_cnt;
          cnt = 3;
        }//if ls->
        else
        {
          ls->seg[i-1] = 0;
          //what if ls[i] is a bad point? - it could be the start of a new
          //segment if the next point is a good point and is close enough!
          //in that case it doesn't really matters
          ls->seg[i] = seg_cnt;//the current point is a new segment
          cnt = 1;
        }
      }//if cnt ==1
      else
      {
        seg_cnt++;
        ls->seg[i] = seg_cnt;
        cnt = 1;
      }//else if cnt
    }//if break seg
  }//for
}//pm_segment_scan

/** @brief Tags point further than a given distance PM_MAX_RANGE.

Far away points get tagged as @a PM_RANGE.
@param ls The scan searched for far points.
*/
void pm_find_far_points ( PMScan *ls )
{
  for ( int i=0;i<PM_L_POINTS;i++ )
  {
    if ( ls->r[i]>PM_MAX_RANGE )
      ls->bad[i] |= PM_RANGE;
  }
}

/** @brief Shows segmentation results by plotting segments with different colours.
@param ls The scan to be plotted.
*/
void pm_show_segmentation ( const PMScan *ls )
{
  float x,y,xw,yw,last_xw=0,last_yw=0;
  float si_th = sinf ( ls->th );
  float co_th = cosf ( ls->th );
  char *color;
  const char *col = "red";

  dr_circle ( ls->rx,ls->ry,10.0,col );
  dr_line ( ls->rx,ls->ry,ls->rx+10.0*cosf ( ls->th+M_PI/2.0 ),
            ls->ry+10.0*sinf ( ls->th+M_PI/2.0 ),col );
  dr_line ( 0,-100,180,-100,"black" );
  for ( int i=0;i<PM_L_POINTS;i++ )
  {
    x = ls->r[i]*pm_co[i];
    y = ls->r[i]*pm_si[i]+PM_LASER_Y;
    xw = x*co_th -y*si_th + ls->rx;
    yw = x*si_th +y*co_th + ls->ry;
    color = dr_COLORS[abs ( ls->seg[i] ) % ( dr_COLORS_CNT-1 ) ];
    if ( ls->seg[i]==0 )
      color = dr_COLORS[dr_COLORS_CNT-1];
    dr_circle ( pm_fi[i]*PM_R2D,ls->r[i]/10.0-100.0,1,color );
    dr_circle ( xw,yw,2,color );
    if ( i!=0 && ls->seg[i]!=0 && ls->seg[i]==ls->seg[i-1] )
    {
      dr_line ( xw,yw,last_xw,last_yw,color );
      dr_line ( pm_fi[i]*PM_R2D,ls->r[i]/10.0-100.0,
                pm_fi[i-1]*PM_R2D,ls->r[i-1]/10.0-100.0,color );
    }
    else
    {
      char str[1000];
      sprintf ( str,"%i",ls->seg[i] );
      dr_text ( xw,yw,1,1,str,col );
      dr_text ( pm_fi[i]*PM_R2D,ls->r[i]/10.0-100.0,1,1,str,col );
    }
    last_xw = xw;
    last_yw = yw;
  }
  char str[1000];
  sprintf ( str,"%.3lf",ls->t );
  dr_text ( ls->rx,ls->ry,1,1,str,col );
}

/** @brief Prepares a scan for scan matching.

Filters the scan using median filter, finds far away points and segments the scan.
@param ls The scan to be preprocessed.
*/
void pm_preprocessScan(PMScan *ls)
{
  pm_median_filter(ls);
  pm_find_far_points(ls);
  pm_segment_scan(ls);
}

/** @brief Guesses if a scan was taken on a corridor.

Scan matching results on corridors are often inaccurate in the 
along-corridor direction. By detecting scans with a
corridor-like appearance one can tailor a covariance matrix to incorporate 
the along-corridor uncertainty of the scan matching result.

Calculates the variance of angles between neighbouring 
Cartesian points as the "corridorness" criterion. On corridoors, 
the majority of vectors connecting neighbouring points are aligned 
along one line. Thus corridors generate a sharp peak in an angle histogram.
 
To solve the problem caused at the 180-0 degree transition point
the calculations are repeated after a 30 degree shift. This is a hack
which can be fixed easily.

The scan is assumed be pre-processed (segmentation and median filtering).

An exeption is thrown if there is less than 1 valid point.

TODO: Remove the double calculation of the variance.
@param act The scan which is examined for being corridor-like.
@return True if @a act seems to be taken of a corridor.
*/
bool pm_is_corridor ( PMScan *act )
{
  PM_TYPE fi1=0,fi2=0,fi3=0;
  PM_TYPE sxx=0,sx=0,std1,std2;
  PM_TYPE n=0;

  for ( int i=0;i< ( PM_L_POINTS-1 );i++ )
  {
    if ( act->seg[i]==act->seg[i+1] && act->seg[i]!=0 && !act->bad[i] ) //are they in the same segment?
    {
      PM_TYPE x,y,x1,y1,fi;
      x  = act->r[i]*pm_co[i];
      y  = act->r[i]*pm_si[i];
      x1 = act->r[i+1]*pm_co[i+1];
      y1 = act->r[i+1]*pm_si[i+1];
      fi = atan2f ( y1-y, x1-x ) * PM_R2D;

      if ( fi<0 )   //want angles from 0 to 180
        fi+=180.0;
      if ( i==0 ) //init
      {
        fi1 = fi;fi2 = fi;fi3 = fi;
      }
      //median filtering with a window size of 3
      fi3=fi2;
      fi2=fi1;
      fi1=fi;
      // pick out the median value
      if ( fi1<=fi2 && fi2<=fi3 )
        fi = fi2;
      if ( fi2<=fi3 && fi3<=fi1 )
        fi = fi3;
      if ( fi3<=fi1 && fi1<=fi2 )
        fi = fi1; 

      sx  += fi;
      sxx += fi*fi;
      n   += 1.0;      
    }//if
  }
  if ( n>1 )
  {
    PM_TYPE var = ( sxx-sx*sx/n ) / ( n-1.0 );
    if(var > 0)
    {
      std1 = sqrtf ( var );
    }
    else
    {
      std1 = 0;    
    }
  }
  else
  {
    cerr<<"pm_is_corridor: ERROR n<=1"<<endl;
    throw 3;
  }

  //rotate by 30 degrees and repeat the calculations to handle cases where the corridor is aligned with the discontinuity in the angle representation. 
  sxx = 0;n=0;sx=0;
  for ( int i=0;i< ( PM_L_POINTS-1 );i++ )
  {
    if ( act->seg[i]==act->seg[i+1] && act->seg[i]!=0 && !act->bad[i] ) //are they in the same segment?
    {
      PM_TYPE x,y,x1,y1,fi;
      x  = act->r[i]*cosf ( pm_fi[i]+M_PI/5.0 );
      y  = act->r[i]*sinf ( pm_fi[i]+M_PI/5.0 );
      x1 = act->r[i+1]*cosf ( pm_fi[i+1]+M_PI/5.0 );
      y1 = act->r[i+1]*sinf ( pm_fi[i+1]+M_PI/5.0 );
      fi = atan2f ( y1-y,x1-x ) *PM_R2D;

      if ( fi<0 )   //want angles from 0 to 180
        fi+=180.0;
      if ( i==0 ) //init
      {
        fi1 = fi;fi2 = fi;fi3 = fi;
      }
      fi1=fi;
      fi2=fi1;
      fi3=fi2;
      // pick out the median value
      if ( fi1<=fi2 && fi2<=fi3 )
        fi = fi2;
      if ( fi2<=fi3 && fi3<=fi1 )
        fi = fi3;
      if ( fi3<=fi1 && fi1<=fi2 )
        fi = fi1;

      sx+=fi;
      sxx+=fi*fi;
      n+=1.0;
    }//if
  }

  if ( n>1 )
  {
    PM_TYPE var = ( sxx-sx*sx/n ) / ( n-1.0 );
    if(var > 0)
    {
      std2 = sqrtf ( var );
    }
    else
    {
      std2 = 0;    
    }
  }
  else
  {
    cerr<<"pm_is_corridor: ERROR n<=1"<<endl;
    throw 2;
  }

//  cout <<" std "<<std1<<" "<<std2<<endl;
  PM_TYPE st;
  if ( std1 < std2 )
    st = std1;
  else
    st = std2;
  if ( st < PM_CORRIDOR_THRESHOLD)
  {
    //cout <<"corridor"<<endl;
    return true;
  }
  else
  {
    //cout<<"room"<<endl;
    return false;
  }
}//bool pm_is_corridor(PMScan *act)


/** @brief Calculates an error index expressing the quality of a match.

This function assesses how well is the current scan aligned with the
reference scan. This function has to be called after a scan has been 
matched. The current scan's pose has to be expressed in the reference 
scan's coordinate system.

The current scan is compared to the reference scan and vice versa, 
then the maximum is taken. The comparisson is performed by calculating
the average closest point distance. Far away points are ignored in the process.
The number of non-far away points have to be larger than a threshold.

This function is computationally very expensive and takes a conservative
guess on the match quality. 

TODO: Improve the accuracy of the estimate. Speed it up. Perhaps
should use the error output from scan matching instead. A proper
test is necessary.

@param lsr The reference scan.
@param lra The current scan. 
@return The average minimum Euclidean distance.
*/
PM_TYPE pm_error_index ( PMScan *lsr,PMScan *lsa )
{
  int     i,j;
  PM_TYPE rx[PM_L_POINTS],ry[PM_L_POINTS],ax[PM_L_POINTS],ay[PM_L_POINTS];
  PM_TYPE x,y;
  PM_TYPE d,dmin,dsum,dx,dy;
  PM_TYPE dsum1;
  int     n,n1,rn=0,an=0;
  const   PM_TYPE HUGE_ERROR  = 1000000;//Error returned when there aren't corresponding points.
  const   PM_TYPE DISTANCE_TRESHOLD = PM_MAX_RANGE / 2; //Maximum allowed distance between corresponding points.
  const   int     MIN_POINTS = PM_MIN_VALID_POINTS;

  lsa->th = norm_a ( lsa->th );
  PM_TYPE co = cosf ( lsa->th );
  PM_TYPE si = sinf ( lsa->th );

  // Calculate the cartesian coordinates of scan points in the the overlap region of 
  // both scans.
  
  int index360 = ( (PM_L_POINTS-1) * 360)/ PM_FOV;// number of range readings if scan was 360 degrees
  int indexIncrement = (lsa->th * PM_R2D * (PM_L_POINTS-1)) / PM_FOV;// the current scans orientation expressed
                        //in indices into the range array
  for ( i=0;i<PM_L_POINTS;i++ )
  {
    //recalculate reference scan points in cartesian corrdinates and remove those which are not in the
    //field of view of the current scan
    if ( !lsr->bad[i] )
    {
      //calculate the index of current scan point corresponding to i-th ref. scan point:
      int j = (i - indexIncrement + index360) % index360;     
      assert( (i - indexIncrement + index360) >=0);
      
      //check if the corresponding current scan point is within the field of view of the sensor:
      if( (j >= 0) && (j < PM_L_POINTS))
      {
        rx[rn]   = lsr->r[i]*pm_co[i];
        ry[rn] = lsr->r[i]*pm_si[i];      
        //dr_circle(rx[rn],ry[rn],5,"blue");
        rn++;
      }      
    }
  }
    
  //do the same for the current scan, and transform it into the current scan frame:        
  for ( i=0;i<PM_L_POINTS;i++ )
  {
    if ( !lsa->bad[i] )
    {      
      int j = (i + indexIncrement + index360) % index360;     
      assert( (i + indexIncrement + index360) >=0);
      
      if( (j >= 0) && (j < PM_L_POINTS))
      {    
        x = lsa->r[i]*pm_co[i];
        y = lsa->r[i]*pm_si[i];
        //transform into ref. scan frame:
        ax[an] = x*co-y*si+lsa->rx;
        ay[an] = x*si+y*co+lsa->ry;
        //dr_circle(ax[an],ay[an],5,"brown");
        an++;
      }
    }//if
  }//for i

  //Now go through each current scan point and find the closest ref. scan point
  dsum = 0;n=0;
  for ( i=0;i<an;i++ )
  {
    dmin = HUGE_ERROR;
    for ( j=0;j<rn;j++ )
    {
      dx = rx[j]-ax[i];
      dy = ry[j]-ay[i];
      d = sqrtf ( dx*dx+dy*dy );
      if ( d<dmin )
        dmin = d;
    }//for j
    if ( dmin < DISTANCE_TRESHOLD )
    {
      n++;
      dsum+=dmin;
    }
  }//for i

  if ( n>0 )
  {
    dsum1 = dsum/ ( PM_TYPE ) n;
    n1    = n;
  }
  else
    return   HUGE_ERROR;

  //now checking the reference scan agains the current
  dsum = 0;n=0;
  for ( i=0;i<rn;i++ )
  {
    dmin = HUGE_ERROR;
    for ( j=0;j<an;j++ )
    {
      dx = rx[i]-ax[j];
      dy = ry[i]-ay[j];
      d = sqrtf ( dx*dx+dy*dy );
      if ( d<dmin )
        dmin = d;
    }//for j
    if ( dmin < DISTANCE_TRESHOLD )
    {
      n++;
      dsum+=dmin;
    }
  }//for i

  if ( n>0 )
  {
    dsum = dsum/ ( PM_TYPE ) n;
  }
  else
    return HUGE_ERROR;

//  cout<<"pm_error_index: "<<n1<<" "<<dsum1<<" "<<n<<" "<<dsum<<endl;

  if ( n1>MIN_POINTS && n>MIN_POINTS )
  {
    if ( dsum1>dsum )
      return dsum1; //return the larger one
    else
      return dsum;
  }
  return HUGE_ERROR;
}

/** @brief More quickly calculates an error index expressing the quality of a match.

This function assesses how well is the current scan aligned with the
reference scan. This function has to be called after a scan has been 
matched. The current scan's pose has to be expressed in the reference 
scan's coordinate system.

The current scan is compared to the reference scan by projecting the
current scan where the reference scan was taken and calculating the
average range residuals.

@param lsr The reference scan.
@param lra The current scan. 
@return The average minimum Euclidean distance.
*/
PM_TYPE pm_error_index2 ( PMScan *ref,PMScan *cur, int* associatedPoints )
{

  PMScan    cur2;//copies of current and reference scans
  PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
  PM_TYPE   t13,t23,LASER_Y = PM_LASER_Y;
  PM_TYPE   new_r[PM_L_POINTS];//interpolated r at measurement bearings
  int       new_bad[PM_L_POINTS];//bad flags of the interpolated range readings 
  PM_TYPE   avg_err = 100000000.0;

  rx =  ref->rx; ry = ref->ry; rth = ref->th;
  ax =  cur->rx;  ay = cur->ry;  ath = cur->th;

  // Transformation of the current scan laser scanner coordinates into the reference
  // laser scanner's coordinate system:
  t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
  t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

  cur2 = *cur;
  cur2.rx = t13; cur2.ry = t23; cur2.th = ath-rth;
 
  //from now on act.rx,.. express the laser's position in the reference frame
  pm_scan_project( &cur2,  new_r, new_bad );

  PM_TYPE  e = 0;
  int n = 0;
  for ( int i=0;i < PM_L_POINTS;i++ ) //searching through the current points
  {
    PM_TYPE delta = fabsf ( new_r[i] - ref->r[i] );
    if ( !new_bad[i] && !ref->bad[i] && delta < PM_MAX_ERROR / 2.0)
    {
      e += delta;
      n++;
    }
  }//for i

  if ( n > 0 )
    avg_err = e/n;
  if(associatedPoints != NULL)
  {
    *associatedPoints = n;
  } 
  return avg_err;
}

/** @brief Determines the orientation of a corridor.

TODO: NEEDS A REWRITE - TOO MESSY.

Assuming the scan was taken of a corridor, determines the
orientation of the corridor by finding the maximum in a
180 degree wide angle histogram. The input into the 
histogram are angles of lines created by connecting
neighbouring points.

Normally, this function is used on the reference scan and 
not the current scan.

@param act The scan of which oriention is to be determined. 
@return The orientation of the corridor.
*/
PM_TYPE pm_corridor_angle ( PMScan *act )
{
  PM_TYPE fi;
  int   n=0,j,i;
  PM_TYPE ang[PM_L_POINTS];
  int hist[180];//180 degree angle histogram. at 2 deg; 0,2,4....

  for ( i=0;i<180;i++ )
    hist[i]=0;

  for ( i=0;i< ( PM_L_POINTS-1 );i++ )
  {
    if ( act->seg[i]==act->seg[i+1] && act->seg[i]!=0 && !act->bad[i] ) //are they in the same segment?
    {
      PM_TYPE x,y,x1,y1,fi;
      x  = act->r[i]*pm_co[i];
      y  = act->r[i]*pm_si[i];
      
      x1 = act->r[i+1]*pm_co[i+1];
      y1 = act->r[i+1]*pm_si[i+1];
      fi = atan2f ( y1-y,x1-x ) *PM_R2D;//angle in degrees

      //Want angles from 0 to 360
      if ( fi<0 )   
        fi+=180.0;
      ang[n] = fi;
      n++;

      j = ( 2* ( int ) floor ( fi/2.0+0.5 ) ) % 180;//index into the angle hist.
      hist[j%180]++;
    }//if
  }

  //find the maximum
  int imax=-1,max = 0;
  for ( i=0;i<180;i+=2 )
    if ( hist[i]>max )
    {
      max   = hist[i];
      imax  = i;
    }//if
  if ( max ==0 )
  {
    cerr <<"pm_corridor_angle: ERROR no maximum"<<endl;
    throw 1;
  }

  PM_TYPE m=0;
  int cnt=0,d;

  for ( i=0;i<n;i++ )
  {
    fi = ang[i];
    j = ( ( int ) floor ( fi+0.5 ) ) %180;//index into the angle hist.
    d = abs ( j-imax );
    const int TRESHOLD=5;
    if ( (d<90 && d%90<TRESHOLD) || (d>=90 && ( 90-d%90 )) <TRESHOLD )
    {
      //watch out average is tricky with angles! around 180 and 0
      cnt++;
      if ( imax<10 && j>170 )
        m+= 180.0-fi;
      else
        if ( imax>170 && j<10 )
          m+= 180.0+fi;
        else
          m+= fi;
    }//i
  }
  m = m/ ( PM_TYPE ) cnt;
  return m*PM_D2R;
}


/** @brief Estimates the covariance matrix of a match.

Estimates elements (c11,c12,c22,c33) of a match result
covariance matrix of the following structure:<br>
[c11 c12 0.0]<br>
[c12 c22 0.0]<br>
[0.0 0.0 c33]<br>

This function scales a base covariance matrix by an error index:<br>
C = C0(PM_MIN_STD_XY, PM_MIN_STD_ORIENTATION) * (err - PM_MATCH_ERROR_OFFSET).<br>
For corridors it scales and rotates a covariance matrix stretched 
in the direction of the corridor:<br>
C = Rot(C0, corr_angle) * (err - PM_MATCH_ERROR_OFFSET).<br>
If (err - PM_MATCH_ERROR_OFFSET) < 1 then C = C0.

@param err The error index of the match. 
@param c11,c12,c22,c33 The estimated covariance matrix elements.
@param corridor If true, a special estimate with large along-corridor-error is used.
@param corr_angle The orientation of the corridor. Call @a pm_corridor_angle on the refernce scan to get this value.
*/
void pm_cov_est ( PM_TYPE err, double *c11,double *c12, double *c22, double *c33,
                  bool corridor, PM_TYPE corr_angle )
{
#define SQ(x) ((x)*(x))
  const double cov_x  = SQ ( PM_MIN_STD_XY ); 
  const double cov_y  = SQ ( PM_MIN_STD_XY );
  const double cov_th = SQ ( PM_MIN_STD_ORIENTATION*M_PI/180.0 );
  //for corridors
  const double cov_along   = SQ ( PM_MIN_STD_XY*20.0 );   // cm
  const double cov_across  = SQ ( PM_MIN_STD_XY*1.5 ); // cm

  err = err - PM_MATCH_ERROR_OFFSET;
  if ( err < 1.0 )
    err = 1.0;
  if ( corridor ) //Was the current scan taken on a corridor?
  {
    double co = cosf ( corr_angle );
    double si = sinf ( corr_angle );
    *c11 = err* ( SQ ( co ) *cov_along+SQ ( si ) *cov_across );
    *c12 = err* ( -co*si* ( -cov_along+cov_across ) );
    *c22 = err* ( SQ ( si ) *cov_along+SQ ( co ) *cov_across );
    *c33 = err*cov_th;
  }
  else
  {
    *c12 = 0;//The covariance ellipse is a circle.
    *c11 = err*cov_x;
    *c22 = err*cov_y;
    *c33 = err*cov_th;
  }//else
}//pm_cov_est


/** @brief Match two laser scans using polar scan matching. 

Minimizes the sum of square range residuals through changing lsa->rx, lsa->ry, lsa->th.
The error is minimized by iterating a translation estimation step followed by an
orientation search step.

PSM was not explicitly designed for laser scan matching based odometry where scans with small
pose difference are matched with each other without any prior pose information. However when 
using PSM for this purpose, reduce the values of PM_MAX_ERROR, PM_WEIGHTING_FACTOR to 
reflect the small inter-scan motion. Also by reducing the value of PM_STOP_COND,
larger matching accuracy can be achieved. The currently implemented error estimation 
functions are not useful for laser odometry error estimation.

Limitations: due to the nature of the association rule divergence in a slow rate
may be experienced in rooms where there are not many features to constrain
the solution in all directions. This can occur for examples in corridor-like environments 
including rooms where the room directly in front of the laser is outside of 
the range of the laser range finder.

@param lsr The reference scan.
@param lra The current scan. 
*/
PM_TYPE pm_psm ( const PMScan *lsr,PMScan *lsa )
{
  PMScan    act, ref;//copies of current and reference scans
  PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
  PM_TYPE   t13,t23,LASER_Y = PM_LASER_Y;
  PM_TYPE   new_r[PM_L_POINTS];//interpolated r at measurement bearings
  int       new_bad[PM_L_POINTS];//bad flags of the interpolated range readings
  PM_TYPE   C = PM_WEIGHTING_FACTOR;//weighting factor; see dudek00
  int       iter,small_corr_cnt=0;
  PM_TYPE   dx=0,dy=0,dth=0;//match error, current scan corrections
  PM_TYPE   avg_err = 100000000.0;

#ifdef  PM_GENERATE_RESULTS
  double start_tick, dead_tick,end_tick,end_tick2;
  FILE *f;
  f = fopen ( PM_TIME_FILE,"w" );
  dead_tick = 0;
  start_tick =pm_msec();
#endif

  act = *lsa;
  ref = *lsr;

  rx =  ref.rx; ry = ref.ry; rth = ref.th;
  ax =  act.rx; ay = act.ry; ath = act.th;

  //transformation of the current scan laser scanner coordinates into the reference
  //laser scanner's coordinate system:
  t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
  t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

  ref.rx = 0;   ref.ry = 0;   ref.th = 0;
  act.rx = t13; act.ry = t23; act.th = ath-rth;

  ax = act.rx; ay = act.ry; ath = act.th;
  //from now on act.rx,.. express the laser's position in the reference frame

  iter = -1;
  while ( ++iter < PM_MAX_ITER && small_corr_cnt < 3 ) //Has to be a few small corrections before stopping.
  {
    if ( ( fabsf ( dx ) +fabsf ( dy ) +fabsf ( dth ) ) < PM_STOP_COND )
      small_corr_cnt++;
    else
      small_corr_cnt=0;

#ifdef  PM_GENERATE_RESULTS
    end_tick =pm_msec();
    fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
               end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
    end_tick2 =pm_msec();
    dead_tick += end_tick2- end_tick;
#endif


#ifdef GR
    dr_erase();
    dr_circle ( ax,ay,5.0,"green" );
    dr_line ( 0,-100,200,-100,"black" );
    dr_line ( 0,-200,200,-200,"black" );
    for(int i=0;i<PM_L_POINTS;i++)
    {
      dr_circle ( ref.r[i]*pm_co[i],ref.r[i]*pm_si[i],4.0,"black" );
      dr_circle ( pm_fi[i]*PM_R2D,ref.r[i]/10.0-100,1,"black" );
    }
#endif

    act.rx = ax;act.ry = ay;act.th = ath;
    pm_scan_project(&act,  new_r, new_bad);

    //---------------ORIENTATION SEARCH-----------------------------------
    //search for angle correction using crosscorrelation, perform it every second step
    if ( iter%2 == 0 )
    {
       dth = pm_orientation_search(&ref, new_r, new_bad);
       ath += dth;
       continue;
    }

    //------------------------------------------translation-------------
    //reduce C with time to consider only the best matches
    if ( iter == PM_CHANGE_WEIGHT_ITER )
      C = C/50.0; // weigh far points even less.
#ifdef GR
    for(int i=0;i<PM_L_POINTS;i++)
    {
        dr_circle ( pm_fi[i]*PM_R2D,ref.r[i]/10.0-200,1,"black" );
    }
#endif

    avg_err = pm_translation_estimation(&ref, new_r, new_bad, C, &dx, &dy);
    ax += dx;
    ay += dy;

#ifdef GR
    cout <<"iter "<<iter<<" "<<ax<<" "<<ay<<" "<<ath*PM_R2D<<" "<<dx<<" "<<dy<<endl;
    dr_zoom();
#endif
  }//while iter

#ifdef  PM_GENERATE_RESULTS
  end_tick =pm_msec();
  fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
             end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
  fclose ( f );
#endif
  //dr_zoom();
  //cout <<"Iterations: "<<iter<<endl;
  lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
  return ( avg_err);
}//pm_psm



/** @brief Calculate the distance of a point from a line section. 

Calculates the distance of the point (x3,y3) from a line defined by (x1,y1)
and (x2,y2). Returns the distance to the line or -1 if the
projection of (x3,y3) falls outside the line segment defined by (x1,y1)
and (x2,y2). The projection of (x3,y3) onto the line is also returned in x,y.
This function is used in ICP.
@param x1,y1 The start point of the line section.
@param x2,y2 The end point of the line section.
@param x3,y3 The point of which distance it sought.
@param x,y (x3,y3) projected onto the line segment is returned here.
@return The distance from the line or -1 if the projection falls outside of the line segment.
*/
PM_TYPE point_line_distance ( PM_TYPE x1, PM_TYPE y1, PM_TYPE x2, PM_TYPE y2,
                              PM_TYPE x3, PM_TYPE y3,PM_TYPE *x, PM_TYPE *y )
{
  PM_TYPE ax,ay,t1,D;
  ax = x2-x1;
  ay = y2-y1;
  D =  sqrtf ( ax*ax+ay*ay );
  if ( D < 0.0001 )
  {
    cerr <<"point_line_distance: unexpected D:" << D <<endl;
    return -1;
  }
  t1 =  - ( -ax*x3 + ay*y1 + ax*x1 - ay*y3 ) / ( ax*ax+ay*ay );
  if ( t1<0 || t1>1 )   // Projection falls outside the line segment?
  {
    return -1;
  }
  *x = x1+t1*ax;
  *y = y1+t1*ay;
  return ( sqrtf ( ( x3-*x ) * ( x3-*x ) + ( y3-*y ) * ( y3-*y ) ) );//distance of line to p.
}//  point_line_distance


/** @brief Matches two laser scans using the iterative closest point method.

Minimizes least square error of points through changing lsa->rx, lsa->ry, lsa->th
by using ICP. It interpolates associated 
points. Only the best 80% of points are used in the pose calculation.
Scan projection is done at each iteration.

For maintanence reasons changed scan projection to that of psm.
*/
PM_TYPE pm_icp (  const PMScan *lsr,PMScan *lsa )
{
#define INTERPOLATE_ICP  //comment out if no interpolation of ref. scan points iS  necessary
  PMScan    act,  ref;//copies of current and reference scans
  PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
  PM_TYPE   t13,t23,LASER_Y = PM_LASER_Y;
  int       new_bad[PM_L_POINTS];//bad flags of the projected current scan range readings
  PM_TYPE   new_r[PM_L_POINTS];//ranges of current scan projected into ref. frame for occlusion check
  PM_TYPE   nx[PM_L_POINTS];//current scanpoints in ref coord system
  PM_TYPE   ny[PM_L_POINTS];//current scanpoints in ref coord system
  int       index[PM_L_POINTS][2];//match indices current,refernce
  PM_TYPE   dist[PM_L_POINTS];// distance for the matches
  int       n = 0;//number of valid points
  int       iter,i,j,small_corr_cnt=0,k,imax;
  int       window       = PM_SEARCH_WINDOW;//+- width of search for correct orientation
  PM_TYPE   abs_err=0,dx=0,dy=0,dth=0;//match error, current scan corrections
  PM_TYPE   co,si;

#ifdef  PM_GENERATE_RESULTS
  double start_tick, dead_tick,end_tick,end_tick2;
  FILE *f;
  f = fopen ( PM_TIME_FILE,"w" );
  dead_tick = 0;
  start_tick =pm_msec();
#endif

  act = *lsa;
  ref = *lsr;

  rx =  ref.rx; ry = ref.ry; rth = ref.th;
  ax =  act.rx; ay = act.ry; ath = act.th;

  //transformation of current scan laser scanner coordinates into reference
  //laser scanner coordinates
  t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
  t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

  ref.rx = 0;   ref.ry = 0;   ref.th = 0;
  act.rx = t13; act.ry = t23; act.th = ath-rth;

  ax = act.rx; ay = act.ry; ath = act.th;
  //from now on act.rx,.. express the lasers position in the ref frame
  
  //intializing x,y of act and ref
  for ( i=0;i<PM_L_POINTS;i++ )
  {
    ref.x[i] = ref.r[i]*pm_co[i];
    ref.y[i] = ref.r[i]*pm_si[i];

    act.x[i] = act.r[i]*pm_co[i];
    act.y[i] = act.r[i]*pm_si[i];
  }//for i

  iter = -1;
  while ( ++iter<PM_MAX_ITER_ICP && small_corr_cnt<3 ) //have to be a few small corrections before stop
  {

    if ( ( fabsf ( dx ) +fabsf ( dy ) +fabsf ( dth ) *PM_R2D ) <PM_STOP_COND_ICP )
      small_corr_cnt++;
    else
      small_corr_cnt=0;

#ifdef  PM_GENERATE_RESULTS
    end_tick =pm_msec();
    fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
               end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
    end_tick2 =pm_msec();
    dead_tick += end_tick2- end_tick;
#endif

#ifdef GR
    dr_erase();
    dr_circle ( ax,ay,5.0,"green" );
    dr_line ( 0,-100,200,-100,"black" );
    dr_line ( 0,-200,200,-200,"black" );
#endif
        
    //Scan projection
    act.rx = ax;act.ry = ay;act.th = ath;
    pm_scan_project(&act,  new_r, new_bad);
        
    // transformation the cartesian coordinates of the points:
    co = cosf ( ath );
    si = sinf ( ath );    
    for ( i=0;i<PM_L_POINTS;i++ )
    {
      nx[i]     = act.x[i]*co - act.y[i]*si + ax;
      ny[i]     = act.x[i]*si + act.y[i]*co + ay;
#ifdef GR
      if ( ref.bad[i] )
        dr_circle ( ref.x[i],ref.y[i],4,"yellow" );
       else
        dr_circle ( ref.x[i],ref.y[i],4,"black" );
      if ( new_bad[i] )
        dr_circle ( nx[i],ny[i],4,"green" );
      else
        dr_circle ( nx[i],ny[i],4,"blue" );
#endif
    }
//    dr_zoom();
#ifdef GR
    cout <<"interpolated ranges. press enter"<<endl;
/*    for ( i=0;i<PM_L_POINTS;i++ )
      dr_circle ( new_r[i]*pm_co[i],new_r[i]*pm_si[i],6,"red" );*/
    dr_zoom();
#endif

    //Correspondence search: go through the points of the current
    //scan and find the closest point in the reference scan 
    //lying withing a search interval. 
    n=0;
    PM_TYPE d,min_d;
    int min_idx;

    for ( i=0;i<PM_L_POINTS;i++ )
    {
      min_d = 1000000;
      min_idx = -1;
      if ( !new_bad[i] )
      {
        int imin,imax;
        imin = i-window ;
        if ( imin<0 )
          imin =0;
        imax = i+window ;
        if ( imax>PM_L_POINTS )
          imax =PM_L_POINTS;
          
        for ( j=imin;j<imax;j++ )
        {
          if ( !ref.bad[j] )
          {
            d =  SQ ( nx[i]-ref.x[j] ) + SQ ( ny[i]-ref.y[j] );//square distance
            if ( d<min_d )
            {
              min_d  = d;
              min_idx = j;
            }
          }
        }//for
        if ( min_idx>=0 && sqrtf ( min_d ) <PM_MAX_ERROR ) // was there any match closer than 1m?
        {
          index[n][0] = i;
          index[n][1] = min_idx;
          dist[n] = sqrtf ( min_d );
          n++;
#ifdef GR
          dr_line ( nx[i],ny[i],ref.x[min_idx],ref.y[min_idx],"blue" );
#endif
        }
      }//if
    }//for
//    dr_zoom();

    if ( n<PM_MIN_VALID_POINTS )
    {
      cerr <<"pm_icp: ERROR not enough points"<<endl;
#ifdef  PM_GENERATE_RESULTS
      fclose ( f );
#endif
      throw 1;
    }

    //sort the matches with bubble sort
    //put the largest 20 percent to the end
    imax = ( int ) ( ( double ) n*0.2 );
    for ( i=0;i<imax;i++ )
      for ( j=1;j< ( n-i );j++ )
      {
        if ( dist[j]<dist[j-1] ) //are they in the wrong order?
        {
          //swap them
          k             = index[j][0];
          index[j][0]   = index[j-1][0];
          index[j-1][0] = k;

          k             = index[j][1];
          index[j][1]   = index[j-1][1];
          index[j-1][1] = k;

          d             = dist[j];
          dist[j]       = dist[j-1];
          dist[j-1]     = d;
        }
      }//for j

#ifdef INTERPOLATE_ICP
    //------------------------INTERPOLATION---------------------------
    //comment out if not necessary
    PM_TYPE ix[PM_L_POINTS],iy[PM_L_POINTS];//interp. ref. points.
    //replace nx,xy with their interpolated... where suitable
    {

      PM_TYPE d0,d1,d2;
      PM_TYPE minx1,miny1,minx2,miny2;
      int max_i = n-imax;
      for ( i=0;i<max_i;i++ )
      {

        //d1 = point_line_distance(1, 2,2,3, 2,2, &minx1, &miny1);  //debug

#ifdef GR
        dr_circle ( nx[index[i][0]],ny[index[i][0]],1.0,"brown" );
        dr_circle ( ref.x[index[i][1]],ref.y[index[i][1]],1.0,"brown" );
        dr_circle ( ref.x[index[i][1]-1],ref.y[index[i][1]-1],1.0,"brown" );
        dr_circle ( ref.x[index[i][1]+1],ref.y[index[i][1]+1],1.0,"brown" );
#endif
        d1=-1;d2=-1;
        if ( index[i][1]>0 ) //not associated to the first point?
        {
          d1 = point_line_distance ( ref.x[index[i][1]-1], ref.y[index[i][1]-1],
                                     ref.x[index[i][1]],   ref.y[index[i][1]],
                                     nx[index[i][0]],      ny[index[i][0]],
                                     &minx1, &miny1 );
        }

        if ( index[i][1]< ( PM_L_POINTS-1 ) ) //not associated to the last point?
        {
          d2 = point_line_distance ( ref.x[index[i][1]],  ref.y[index[i][1]],
                                     ref.x[index[i][1]+1],ref.y[index[i][1]+1],
                                     nx[index[i][0]],     ny[index[i][0]],
                                     &minx2, &miny2 );
        }

        ix[index[i][1]] = ref.x[index[i][1]];
        iy[index[i][1]] = ref.y[index[i][1]];
        d0 = sqrtf ( SQ ( ref.x[index[i][1]]-nx[index[i][0]] ) + SQ ( ref.y[index[i][1]]-ny[index[i][0]] ) );

        //is the first point closer?
        if ( d1>0 && d1<d0 )
        {
          ix[index[i][1]] = minx1;
          iy[index[i][1]] = miny1;
          d0 = d1;
        }

        //is the second point closer?
        if ( d2>0 && d2<d0 )
        {
          ix[index[i][1]] = minx2;
          iy[index[i][1]] = miny2;
        }
#ifdef GR
        dr_line ( nx[index[i][0]],ny[index[i][0]],ix[index[i][1]],iy[index[i][1]],"green" );
#endif
      }//for
    }
#endif


    //pose estimation
    //------------------------------------------translation-------------

    // do the weighted linear regression on the linearized ...
    // include angle as well
    //computation of the new dx1,dy1,dtheta1
    PM_TYPE sxx=0,sxy=0,syx=0,syy=0;
    PM_TYPE meanpx,meanpy,meanppx,meanppy;
    meanpx = 0;meanpy = 0;
    meanppx= 0;meanppy= 0;

    abs_err=0;
    imax = n-imax;
    for ( i=0;i<imax;i++ )
    {
      //weight calculation
      // do the cartesian calculations....
      meanpx +=  nx[index[i][0]];
      meanpy +=  ny[index[i][0]];

#ifdef INTERPOLATE_ICP
      meanppx +=  ix[index[i][1]];
      meanppy +=  iy[index[i][1]];
#else
      meanppx +=  ref.x[index[i][1]];
      meanppy +=  ref.y[index[i][1]];
#endif

#ifdef GR
      dr_line ( nx[index[i][0]],ny[index[i][0]],ref.x[index[i][1]],ref.y[index[i][1]],"red" );
#endif
    }//for
    meanpx /= imax;
    meanpy /= imax;

    meanppx /= imax;
    meanppy /= imax;

    for ( int i=0;i<imax;i++ )
    {
#ifdef INTERPOLATE_ICP
      sxx += ( nx[index[i][0]] - meanpx ) * ( ix[index[i][1]] - meanppx );
      sxy += ( nx[index[i][0]] - meanpx ) * ( iy[index[i][1]] - meanppy );
      syx += ( ny[index[i][0]] - meanpy ) * ( ix[index[i][1]] - meanppx );
      syy += ( ny[index[i][0]] - meanpy ) * ( iy[index[i][1]] - meanppy );
#else
      sxx += ( nx[index[i][0]] - meanpx ) * ( ref.x[index[i][1]] - meanppx );
      sxy += ( nx[index[i][0]] - meanpx ) * ( ref.y[index[i][1]] - meanppy );
      syx += ( ny[index[i][0]] - meanpy ) * ( ref.x[index[i][1]] - meanppx );
      syy += ( ny[index[i][0]] - meanpy ) * ( ref.y[index[i][1]] - meanppy );
#endif
    }
    //computation of the resulting translation and rotation
    //for method closest point match
    dth = atan2f ( sxy-syx,sxx+syy );
    dx  = meanppx - ax - ( cosf ( dth ) * ( meanpx- ax ) - sinf ( dth ) * ( meanpy - ay ) );
    dy  = meanppy - ay - ( sinf ( dth ) * ( meanpx- ax ) + cosf ( dth ) * ( meanpy - ay ) );

    ax += dx;
    ay += dy;
    ath+= dth;
    ath = norm_a ( ath );

//    //for SIMULATION iteration results..
//    cout <<iter<<"     "<<ax<<"    "<<ay<<"    "<<ath*PM_R2D<<" ;"<<endl;
#ifdef GR
    cout <<"iter "<<iter<<" "<<ax<<" "<<ay<<" "<<ath*PM_R2D<<" "<<dx<<" "<<dy<<endl;
//      if(iter==0)
    dr_zoom();
    usleep ( 10000 );

#endif

  }//for iter
  //cout <<iter<<endl;
#ifdef  PM_GENERATE_RESULTS
  end_tick =pm_msec();
  fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
             end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
  fclose ( f );
#endif

  lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
  return ( abs_err/n );
}//pm_icp

/** @brief Saves scan in a text file.

Saves the scan in the format of:<br>
range[0] bad[0] segment[0] <br>
range[1] bad[1] segment[1] <br>
...<br>
The aim is to enable quick scan loading in Octave using the
"scan=load(filename)"; command
@param act The scan to be saved.
@param filenam The name of the file the scan is saved under.
*/
void pm_save_scan ( PMScan *act,const char *filename )
{
  FILE *f;
  f=fopen ( filename,"w" );
  for ( int i=0;i<PM_L_POINTS;i++ )
    fprintf ( f,"%f %i %i\n",act->r[i],act->bad[i],act->seg[i] );
  fclose ( f );
}

/** @brief Plots current and reference scan in the way scans appeared in my thesis.

@param lsr Reference scan.
@param lsa Current scan.
*/
void pm_plotScan4Thesis ( PMScan *lsr,PMScan *lsa )
{
  float x,y,xw,yw,last_xw=0,last_yw=0;
  float si_th;
  float co_th;
  char *col;
  int i;
  PMScan *ls;

  dr_erase();
  //reference scan
  ls  = lsr;
  col = ( char * ) "black";
  dr_circle ( ls->rx,ls->ry,10.0,col );
  dr_line ( ls->rx,ls->ry,ls->rx+10.0*cosf ( ls->th+M_PI/2.0 ),
            ls->ry+10.0*sinf ( ls->th+M_PI/2.0 ),col );
  for ( i=0;i<PM_L_POINTS;i++ )
  {
    x  = ls->r[i]*pm_co[i];
    y  = ls->r[i]*pm_si[i];
    xw = x;//x*co_th - y*si_th  + ls->rx;
    yw = y;//x*si_th + y*co_th  + ls->ry;

    if ( ls->r[i]<PM_MAX_RANGE )
    {
      dr_circle ( xw,yw,4,col );
      if ( i>0 && ls->r[i-1]<PM_MAX_RANGE )
        dr_line ( xw,yw,last_xw,last_yw,col );
    }
    last_xw = xw;
    last_yw = yw;
  }

  //current scan
  ls  = lsa;
  si_th = sinf ( ls->th );
  co_th = cosf ( ls->th );
  col = ( char * ) "blue";
  dr_circle ( ls->rx,ls->ry,10.0,col );
  dr_line ( ls->rx,ls->ry,ls->rx+10.0*cosf ( ls->th+M_PI/2.0 ),
            ls->ry+10.0*sinf ( ls->th+M_PI/2.0 ),col );

  for ( i=0;i<PM_L_POINTS;i++ )
  {
    x  = ls->r[i]*pm_co[i];
    y  = ls->r[i]*pm_si[i];
    xw = x*co_th - y*si_th  + ls->rx;
    yw = x*si_th + y*co_th  + ls->ry;

    if ( ls->r[i]<PM_MAX_RANGE )
    {
      dr_circle ( xw,yw,4,col );
      if ( i>0 && ls->r[i-1]<PM_MAX_RANGE )
        dr_line ( xw,yw,last_xw,last_yw,col );
    }
    last_xw = xw;
    last_yw = yw;
  }
  dr_fit();
}

/** @brief Generates a convergence speed plot from previously saved result. 

If you want to see the convergence speed, then uncomment the definition
of PM_GENERATE_RESULTS, recompile, match a scan and run this function.

Only works if PM_GENERATE_RESULTS was enabled in the previous match as only
then are results saved in a file which is read here. 
@param xt The true X coordinate where the solution should converge.
@param yt The true Y coordinate where the solution should converge.
@param tht [degrees] The true orientation where the solution should converge.
*/
void pm_plotTime4Thesis ( PM_TYPE xt, PM_TYPE yt, PM_TYPE tht,int *iter,double *time )
{
  FILE *f;
  int cnt =5;
  int i=-1;
  double t=-1,xx,yy,th;
  double x,y1,y2,y3,lx=-1,ly1=-1,ly2=-1,ly3=-1;
  const char *c1="red",*c2="black",*c3 = "blue";
  char s[1000];

  f=fopen ( PM_TIME_FILE,"r" );
  if ( f==NULL )
  {
    cerr <<"pm_plotTime4Thesis: Error: could not open file "<<PM_TIME_FILE<<endl;
    throw 1;
  }

  dr_erase();
  while ( cnt==5 )
  {
    cnt=fscanf ( f,"%i %lf %lf %lf %lf\n",&i,&t,&xx,&yy,&th );
    x  = t;
    y1 = fabsf ( xx-xt );
    y2 = fabsf ( yy-yt );
    y3 = fabsf ( th-tht );
    if ( i%10 )
      dr_line ( x,0.4,x,-0.4,"black" );
    else
    {
      dr_line ( x,2,x,-2,"black" );
      sprintf ( s,"%02i",i );
      dr_text ( x,2,1,1,s,"black" );
    }
    dr_marker ( x,y1,1,c1 );
    dr_marker ( x,y2,2,c2 );
    dr_marker ( x,y3,3,c3 );
    if ( lx>0 )
    {
      dr_line ( lx,ly1,x,y1,c1 );
      dr_line ( lx,ly2,x,y2,c2 );
      dr_line ( lx,ly3,x,y3,c3 );
    }
    lx = x;
    ly1 = y1;
    ly2 = y2;
    ly3 = y3;

  }//whi
  dr_fit();
  if ( iter!=NULL && time!=NULL )
  {
    *iter  = i;*time = t;
  }
}//void pm_plotTime4Thesis(void)

/** @brief Performs scan projection.

This function enables the comparisson of two scans.
It projects the current (active) scan @a act into the reference scans @a ref
coordinate frame,  using the current scan's pose. As the reference scan
is assumed to be at the origin, its coordinates don't need to be passed along.
Returns in new_r the interpolated range readinds r at the reference scan's
measurement bearings. Returns in new_bad bad flags of the interpolated range 
readings, where occluded readings are tagged.

@param act The current scan.
@param new_r Array of the projected range readings (has to have the correct size).
@param new_bad Information about the validity of the interpolated range readings is returned here.
*/
void pm_scan_project(const PMScan *act,  PM_TYPE   *new_r,  int *new_bad)
{
    PM_TYPE   r[PM_L_POINTS],fi[PM_L_POINTS];//current scan in ref. coord. syst.
    PM_TYPE   x,y;
    int       i;
    PM_TYPE   delta;

    // convert range readings into the reference frame
    // this can be speeded up, by connecting it with the interpolation
    for ( i=0;i<PM_L_POINTS;i++ )
    {
      delta   = act->th + pm_fi[i];
      x       = act->r[i]*cosf ( delta ) + act->rx;
      y       = act->r[i]*sinf ( delta ) + act->ry;
      r[i]    = sqrtf ( x*x+y*y );
      fi[i]   = atan2f ( y,x );
      //handle discontinuity at pi (Angle goes from -pi/1 to 3pi/2 for 360deg. scans)
      if(x<0 && y<0)
        fi[i] += 2.0*M_PI;
      new_r[i]  = 10000;//initialize big interpolated r;
      new_bad[i]= PM_EMPTY;//for interpolated r;
      
#ifdef GR
//      dr_circle ( ref.r[i]*pm_co[i],ref.r[i]*pm_si[i],4.0,"black" );
      dr_circle ( x,y,4.0,"red" );
//      dr_circle ( pm_fi[i]*PM_R2D,ref.r[i]/10.0-100,1,"black" );
      dr_circle ( fi[i]*PM_R2D,r[i]/10.0-100,1,"red" );      
#endif
    }//for i

    //------------------------INTERPOLATION------------------------
    //calculate/interpolate the associations to the ref scan points
    //algorithm ignores crosings at the beginning and end points to make it faster
    for ( i=1;i<PM_L_POINTS;i++ )
    {
      //i points to the angles in the current scan

      // i and i-1 has to be in the same segment, both shouldn't be bad
      // and they should be larger than the minimum angle      
      if ( act->seg[i] != 0 && act->seg[i] == act->seg[i-1] && !act->bad[i] && !act->bad[i-1] ) /* && fi[i]>PM_FI_MIN && fi[i-1]>PM_FI_MIN*/
      {
        //calculation of the "whole" parts of the angles
        int j0,j1;
        PM_TYPE r0,r1,a0,a1;
        bool occluded;
        //This is a crude hack to fix a serious bug here!!!! 
        //At the -pi pi boundary it failed by interpolating throught the whole scan.
        //The affected 360 scans, or Hokuyo scans where the matched scans had
        //more than 60degree orientation difference. 
        if( fabsf(fi[i]-fi[i-1]) >= M_PI ) ///TODO: replace this hack with proper fix where we don't loose points.
          continue;
          
        if ( fi[i]>fi[i-1] ) //are the points visible?
        {
          //visible
          occluded = false;
          a0  = fi[i-1];
          a1  = fi[i];
          j0  =  (int) ceil ( ( fi[i-1] - PM_FI_MIN ) /PM_DFI );
          j1  =  (int) floor ( ( fi[i] - PM_FI_MIN ) /PM_DFI );
          r0  = r[i-1];
          r1  = r[i];
        }
        else
        {
          //invisible - still have to calculate to filter out points which
          occluded = true; //are covered up by these!
          //flip the points-> easier to program
          a0  = fi[i];
          a1  = fi[i-1];
          j0  =  (int) ceil ( ( fi[i] - PM_FI_MIN ) /PM_DFI );
          j1  =  (int) floor ( ( fi[i-1] - PM_FI_MIN ) /PM_DFI );
          r0  = r[i];
          r1  = r[i-1];
        }
        //here j0 is always smaller than j1!

        //interpolate for all the measurement bearings beween j0 and j1
        while ( j0<=j1 ) //if at least one measurement point difference, then ...
        {
          PM_TYPE ri = ( r1-r0 ) / ( a1-a0 ) * ( ( ( PM_TYPE ) j0*PM_DFI+PM_FI_MIN )-a0 ) +r0;

          //if j0 -> falls into the measurement range and ri is shorter
          //than the current range then overwrite it
          if ( j0>=0 && j0<PM_L_POINTS && new_r[j0]>ri )
          {
            new_r[j0]    = ri;//overwrite the previous reading
            new_bad[j0] &=~PM_EMPTY;//clear the empty flag
            if ( occluded ) //check if it was occluded
              new_bad[j0] = new_bad[j0]|PM_OCCLUDED;//set the occluded flag
            else
              new_bad[j0] = new_bad[j0]&~PM_OCCLUDED;
            //the new range reading also has to inherit the other flags
            new_bad[j0] |= act->bad[i];//superfluos - since act.bad[i] was checked for 0
            new_bad[j0] |= act->bad[i-1];//superfluos - since act.bad[i-1] was checked for 0
            ///TODO: Uncomment this? (or leave it as it is a local scan matching approach anyway)
            //if(ri>PM_MAX_RANGE)        //uncomment this later
            //  new_bad[fi0] |= PM_RANGE;
            #ifdef GR
            dr_circle ( pm_fi[j0]*PM_R2D,new_r[j0]/10.0-100,1,"yellow" );
            #endif
            //dr_zoom();
          }
          j0++;//check the next measurement angle!
        }//while
      }//if act
    }//for i

    #ifdef GR
      //show the interpolated measurements:
      for ( i=0;i<PM_L_POINTS;i++ )
      {
        double x,y;
        x = new_r[i]*pm_co[i];
        y = new_r[i]*pm_si[i];
        dr_circle ( x,y,3,"yellow" );
        dr_circle ( fi[i]*PM_R2D,new_r[i]/10.0-100,1,"yellow" );
      }
      dr_zoom();
    #endif
}//pm_scan_project


/** @brief Performs one iteration of orientation alignment of current scan.

Function estimating the orientation of the current scan represented with range readings
@a new_r tagged with flags @a new_bad with respect to the reference scan @a ref.

This function exploits that if the current and reference scan are taken at the same 
position, an orientation change of the current scan results in a left or right shift 
of the scan ranges.

This function estimates the orientation by finding that shift which minimizes the
difference between the current and ref. scan. The orientation estimate is then
refined using interpolation by fitting a parabole to the maximum and its 
neighbours and finding the maximum. 

@param ref The reference scan.
@param new_r The interpolated ranges of the current scan.
@param new_bad The tags corresponding to the new_r.
@return Returns the rotation of @new_bad in radians which minimize the sum of absolute range residuals.
 */
PM_TYPE pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad)
{
      int       i;
      int       window = PM_SEARCH_WINDOW;//20;//+- width of search for correct orientation
      PM_TYPE   dth = 0.0;//current scan corrections
      //pm_fi,ref.r - reference points
      PM_TYPE e, err[PM_L_POINTS]; // the error rating
      PM_TYPE beta[PM_L_POINTS];// angle corresponding to err
      const PM_TYPE LARGE_NUMBER = 10000;
      PM_TYPE n;
      int k=0;

      for ( int di=-window;di<=window;di++ )
      {
        n=0;e=0;

        int min_i,max_i;
        if ( di<=0 )
          {min_i = -di;max_i=PM_L_POINTS;}
        else
          {min_i = 0;max_i=PM_L_POINTS-di;}

        ///TODO: speed up by unrolling the loop, replace if with multiplication with 0 or 1/
        /// use sse2 instructions...
        for ( i=min_i;i<max_i;i++ ) //searching through the current points
        {
          PM_TYPE delta = fabsf ( new_r[i]-ref->r[i+di] );
          #if PM_LASER == PM_HOKUYO_UTM_30LX
            //checking delta < PM_MAX_ERROR helps to choose the correct local minimum for the UTM.
            //Without it the solution may be pulled in the wrong direction. Don't remove it.
            ///TODO: Find out when is it useful to check if delta < PM_MAX_ERROR - why only for the UTM...
          if ( !new_bad[i] && !ref->bad[i+di]  && delta < PM_MAX_ERROR)
          #else
          if ( !new_bad[i] && !ref->bad[i+di] )
          #endif
          { 
            e += delta;
            n++;
          }
        }//for i

        if ( n > 0 )
          err[k]  = e/n;//don't forget to correct with n!
        else
          err[k]  = LARGE_NUMBER;
        beta[k] = di;
        k++;
      }//for dfi

#ifdef GR
      FILE *fo;
      fo = fopen ( "angles.txt","w" );
      for ( i = 0; i < k; i++ )
      {
        dr_circle ( beta[i],err[i],1.0,"blue" );
        fprintf ( fo,"%f %f\n",beta[i],err[i] );
      }
      fclose ( fo );
#endif

      //now search for the global minimum
      //later I can make it more robust
      //assumption: monomodal error function!
      PM_TYPE emin = LARGE_NUMBER*10.0;
      int   imin=-1;
      for ( i = 0; i < k; i++ )
      {
        if ( err[i] < emin )
        {
          emin = err[i];
          imin = i;
        }
      }
            
      if ( err[imin]>=LARGE_NUMBER )
      {
        cerr <<"Polar Match: orientation search failed" <<err[imin]<<endl;
        throw 1;
      }
      dth = beta[imin]*PM_DFI;

      //interpolation
      if ( imin >= 1 && imin < ( k-1 ) ) //is it not on the extreme?
      {
        //lets try interpolation
        PM_TYPE D = err[imin-1]+err[imin+1]-2.0*err[imin];
        PM_TYPE d = LARGE_NUMBER;
        if ( fabsf ( D ) >0.01 && err[imin-1]>err[imin] && err[imin+1]>err[imin] )
          d= ( err[imin-1]-err[imin+1] ) /D/2.0;
//        cout <<"ORIENTATION REFINEMENT "<<d<<endl;
        if ( fabsf ( d ) < 1.0 )
          dth+=d*PM_DFI;
      }//if

#ifdef GR
      cout <<"angle correction[deg]: "<<dth*PM_R2D<<endl;
      dr_zoom();
#endif
     return(dth);
}//pm_orientation_search


/** @brief Estimate the postion of the current scan with respect to a reference scan.

@param ref The reference scan.
@param new_r The interpolated ranges of the current scan.
@param new_bad The tags corresponding to the new_r.
@param C Weighting factor for range residuals.
@param dx Estimated position increment X coordinate is returned here.
@param dy Estimated position increment Y coordinate is returned here.
@return Returns the average range residual.
*/
PM_TYPE pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy)
{
    // do the weighted linear regression on the linearized ...
    // include angle as well
    int i;
    PM_TYPE hi1, hi2,hwi1,hwi2, hw1=0,hw2=0,hwh11=0;
    PM_TYPE hwh12=0,hwh21=0,hwh22=0,w;
    PM_TYPE dr;
    PM_TYPE abs_err = 0;
    int     n = 0;
    for ( i=0;i<PM_L_POINTS;i++ )
    {
      dr = ref->r[i]-new_r[i];
      abs_err += fabsf ( dr );
      //weight calculation
      if ( ref->bad[i]==0 && new_bad[i]==0 && new_r[i]<PM_MAX_RANGE && new_r[i]>PM_MIN_RANGE && fabsf ( dr ) <PM_MAX_ERROR )
      {
//        cout <<i<<" "<<dr<<";"<<endl;
        //weighting according to DUDEK00
        w = C/ ( dr*dr+C );
        n++;

        //proper calculations of the jacobian
        hi1 = pm_co[i];//xx/new_r[i];//this the correct
        hi2 = pm_si[i];//yy/new_r[i];

        hwi1 = hi1*w;
        hwi2 = hi2*w;

        //par = (H^t*W*H)^-1*H^t*W*dr
        hw1 += hwi1*dr;//H^t*W*dr
        hw2 += hwi2*dr;

        //H^t*W*H
        hwh11 += hwi1*hi1;
        hwh12 += hwi1*hi2;
//        hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
        hwh22 += hwi2*hi2;

#ifdef GR
        //deb
        dr_circle ( pm_fi[i]*PM_R2D,new_r[i]/10.0-200,1,"red" );
        dr_circle ( pm_fi[i]*PM_R2D,dr/10.0-200,1,"blue" );
//          cout <<i<<"\t"<<ref.r[i]<<"\t"<<new_r[i]<<"\t"<<dr<<"\t"<<w<<" "<<act.r[index[i]]
//          <<" "<<pm_fi[index[i]]<<";"<<endl;
        {
          double x,y;
          x = pm_fi[i]*PM_R2D;
          y = new_r[i]/10.0-200;
          dr_line ( x,y,x+hwi1*dr,y+hwi2*dr,"red" );
          x = new_r[i]*pm_co[i];
          y = new_r[i]*pm_si[i];
          dr_line ( x,y,x+hwi1*dr,y+hwi2*dr,"red" );
        }
#endif

      }//if
    }//for i
    if ( n<PM_MIN_VALID_POINTS ) //are there enough points?
    {
      cerr <<"pm_translation_estimation: ERROR not enough points ("<<n<<")"<<endl;
      #ifdef GR
      dr_zoom();
      #endif
      throw 1;//not enough points
    }

    //calculation of inverse
    PM_TYPE D;//determinant
    PM_TYPE inv11,inv21,inv12,inv22;//inverse matrix

    D = hwh11*hwh22-hwh12*hwh21;
    if ( D<0.001 )
    {
      cerr <<"pm_linearized_match: ERROR determinant to small! "<<D<<endl;
      throw 1;
    }
    inv11 =  hwh22/D;
    inv12 = -hwh12/D;
    inv21 = -hwh12/D;
    inv22 =  hwh11/D;

    *dx = inv11*hw1+inv12*hw2;
    *dy = inv21*hw1+inv22*hw2;
    return(abs_err/n);
}//pm_translation_estimation


/** @brief Takes a scan in a simulated room.

The simulated room is just a rectangle.
@param xl  X coordinate of where the scan is taken.
@param yl  Y coordinate of where the scan is taken.
@param thl Orientation with which the scan is taken.
@param ls  The laser range finder pose is read out from here, and the  
@param wallDistLeft [cm] The distance of the left and right wall from the centre. Optional.
@param wallDistFront [cm] The distance of the front and back wall from the centre. Optional.
*/
void pm_take_simulated_scan(const PM_TYPE xl, const PM_TYPE yl, const PM_TYPE thl, PMScan *ls,
                            PM_TYPE wallDistLeft = 150.0, PM_TYPE wallDistFront = 200.0)
{
  if(PM_LASER_Y!= 0)
  {
    cerr <<"simulated_room: ERROR PM_LASER_Y is not 0!!!"<<endl;
    throw 4;
  }

  //The definintion of the walls. Has to be convex.
  const int N = 4;  
  PM_TYPE a[N]= {0, M_PI/2.0, M_PI, -M_PI/2.0};
  PM_TYPE d[N]= {wallDistLeft, wallDistFront, wallDistLeft, wallDistFront};
    
  PM_TYPE r,rmin,D,fi;
  
  ls->rx  = xl;
  ls->ry  = yl;
  ls->th = thl;
    
  //Take the scan  
  const PM_TYPE LARGE_NUMBER = 100000;
  for(int i = 0; i < PM_L_POINTS; i++)
  {
    rmin = LARGE_NUMBER;
    fi = pm_fi[i];
    for(int j = 0; j < N; j++)
    {
      D = cosf(thl+fi)*cosf(a[j]) + sinf(thl+fi)*sinf(a[j]);
      if( fabsf(D) > 0.001 )
      {
        r = ( d[j] - xl*cosf(a[j]) - yl*sinf(a[j]) ) / D;
        if( r > 0 && r < rmin )
          rmin = r;
      }
    }//for j
    ls->r[i]=rmin;    
    ls->seg[i]=0;//initialization
    ls->bad[i]=0;//initialization
  }//for i
}

/** @brief Performs unit tests on scan matching.

It will assert when compiled in debug mode.
Currently the test are very basic (too high level).
Should add more tests with time.

@param matching_alg Specify the scan matching algorithm to be tested (PM_PSM, PM_ICP).
@param interactive If true, graphically display results.
*/
void  pm_unit_test(int matching_alg, bool interactive)
{
  const PM_TYPE eps = 0.5; //allowed error in X or Y coordinate
  const PM_TYPE epsTh = 0.5*PM_D2R;//allowed orientation error

  char* matcherName;
  if(matching_alg == PM_PSM)
    matcherName = (char*)"PSM";
  else
    matcherName = (char*)"ICP";

  printf("Running scan matching unit test for %s. Compiled for: %s\n",matcherName, PM_LASER_NAME);

  pm_init();

  if(interactive)
  {
    dr_init(600,600,-1000, -1000, 1000, 1000);
  }

  PMScan lsc;//Current scan
  PMScan lsr;//Reference scan.
  float xc,yc,thc; 
  float xr,yr,thr;
  int test = -1;  
    
  xr=0.0; yr=0.0; thr=0.0; 
  pm_take_simulated_scan(xr, yr, thr, &lsr);
  pm_preprocessScan( &lsr );
  
  
  xc=0.0; yc=0.0; thc=0.0; test++; 
  pm_take_simulated_scan(xc, yc, thc, &lsc);  
  lsc.rx=0.0; lsc.ry=0.0; lsc.th=0.0;  
  pm_preprocessScan( &lsc );
      
  if(interactive)
  {
    dr_erase();
    pm_plotScan(&lsr,"black",2.0,true);
    pm_plotScan(&lsc,"blue",2.0,true);  
    dr_fit();
    dr_zoom();
  }
            
  if(matching_alg == PM_PSM)
    pm_psm(&lsr, &lsc);
  else
    pm_icp(&lsr, &lsc);
  
  printf("Test%i:init. pose:(%.1f,%.1f,%.1f); position error:%.2f[cm] orient. error:%.2f[deg]\n",
    test,xc,yc,thc*PM_R2D,sqrtf(SQ(xc - lsc.rx)+SQ(yc - lsc.ry)),(thc- lsc.th)*PM_R2D);    
  if(interactive)
  {
    pm_plotScan(&lsc,"red",2.0,true);
    dr_zoom();
  }    
  assert(fabsf(xc - lsc.rx) <= eps);
  assert(fabsf(yc - lsc.ry) <= eps);
  assert(fabsf(thc - lsc.th) <= epsTh);
        
  xc=0; yc=0; thc=0.1;test++;
  pm_take_simulated_scan(xc, yc, thc, &lsc);  
  lsc.rx=0.0; lsc.ry=0.0; lsc.th=0.0;  
  pm_preprocessScan( &lsc );
  if(matching_alg == PM_PSM)
    pm_psm(&lsr, &lsc);
  else
    pm_icp(&lsr, &lsc);
    
  printf("Test%i:init. pose:(%.1f,%.1f,%.1f); position error:%.2f[cm] orient. error:%.2f[deg]\n",
    test,xc,yc,thc*PM_R2D,sqrtf(SQ(xc - lsc.rx)+SQ(yc - lsc.ry)),(thc- lsc.th)*PM_R2D);    
  if(interactive)
  {
    dr_erase();
    pm_plotScan(&lsr,"black",2.0,true);
    pm_plotScanAt(&lsc,xc,yc,thc,"blue",2.0,true);  
    pm_plotScan(&lsc,"red",2.0,true);
    dr_fit();
    dr_zoom();
  }          
  assert(fabsf(xc - lsc.rx) <= eps);
  assert(fabsf(yc - lsc.ry) <= eps);
  assert(fabsf(thc- lsc.th) <= epsTh);
  
  xc=10.0; yc=0; thc=0.0;  test++;
  pm_take_simulated_scan(xc, yc, thc, &lsc);  
  lsc.rx=0.0; lsc.ry=0.0; lsc.th=0.0;  
  pm_preprocessScan( &lsc );
  if(matching_alg == PM_PSM)
    pm_psm(&lsr, &lsc);
  else
    pm_icp(&lsr, &lsc);
    
  printf("Test%i:init. pose:(%.1f,%.1f,%.1f); position error:%.2f[cm] orient. error:%.2f[deg]\n",
    test,xc,yc,thc*PM_R2D,sqrtf(SQ(xc - lsc.rx)+SQ(yc - lsc.ry)),(thc- lsc.th)*PM_R2D);    
  if(interactive)
  {
    dr_erase();
    pm_plotScan(&lsr,"black",2.0,true);
    pm_plotScanAt(&lsc,xc,yc,thc,"blue",2.0,true);  
    pm_plotScan(&lsc,"red",2.0,true);
    dr_fit();
    dr_zoom();
  }
  assert(fabsf(xc - lsc.rx) <= eps);
  assert(fabsf(yc - lsc.ry) <= eps);
  assert(fabsf(thc- lsc.th) <= epsTh);

  xc=0.0; yc=10.0; thc=0.0;test++;
  pm_take_simulated_scan(xc, yc, thc, &lsc);  
  lsc.rx=0.0; lsc.ry=0.0; lsc.th=0.0;  
  pm_preprocessScan( &lsc );
  if(matching_alg == PM_PSM)
    pm_psm(&lsr, &lsc);
  else
    pm_icp(&lsr, &lsc);
    
  printf("Test%i:init. pose:(%.1f,%.1f,%.1f); position error:%.2f[cm] orient. error:%.2f[deg]\n",
    test,xc,yc,thc*PM_R2D,sqrtf(SQ(xc - lsc.rx)+SQ(yc - lsc.ry)),(thc- lsc.th)*PM_R2D);    
  if(interactive)
  {    
    dr_erase();
    pm_plotScan(&lsr,"black",2.0,true);
    pm_plotScanAt(&lsc,xc,yc,thc,"blue",2.0,true);  
    pm_plotScan(&lsc,"red",2.0,true);
    dr_fit();
    dr_zoom();
  }
    
  assert(fabsf(xc - lsc.rx) <= eps);
  assert(fabsf(yc - lsc.ry) <= eps);
  assert(fabsf(thc- lsc.th) <= epsTh);
  
  assert( pm_is_corridor(&lsc) == false);
  
  //========== Test corridor detection ============================
  
  xr=0.0; yr=0.0; thr=0.0; test++;
  pm_take_simulated_scan(xr, yr, thr, &lsr, 200.0, 1.5*PM_MAX_RANGE);
  pm_preprocessScan( &lsr );
  
  xc=10.0; yc=0.0; thc=0.1;  
  pm_take_simulated_scan(xc, yc, thc, &lsc, 200.0, 1.5*PM_MAX_RANGE);  
  lsc.rx=0.0; lsc.ry=0.0; lsc.th=0.0;  
  pm_preprocessScan( &lsc );
      
  if(interactive)
  {
    dr_erase();
    pm_plotScan(&lsr,"black",2.0,true);
    pm_plotScan(&lsc,"blue",2.0,true);  
    dr_fit();
    dr_zoom();
  }
            
  if(matching_alg == PM_PSM)
    pm_psm(&lsr, &lsc);
  else
    pm_icp(&lsr, &lsc);
  
  printf("Test%i:init. pose:(%.1f,%.1f,%.1f); position error:%.2f[cm] orient. error:%.2f[deg]\n",
    test,xc,yc,thc*PM_R2D,sqrtf(SQ(xc - lsc.rx)+SQ(yc - lsc.ry)),(thc- lsc.th)*PM_R2D);
    
  assert(fabsf(xc - lsc.rx) <= eps);
  assert(fabsf(thc - lsc.th) <= epsTh);
    
  bool corridor = pm_is_corridor(&lsc);
  assert (corridor);
  PM_TYPE error = pm_error_index(&lsr, &lsc);  
  PM_TYPE angle = pm_corridor_angle(&lsr);
            
  double c11,c12, c22, c33;
  pm_cov_est(error, &c11,&c12, &c22, &c33,corridor, angle);
  
  printf("Error index:%.1f,  angle:%.1f, cov: %.1f,%.1f,%.1f,%.1f\n", error, angle*PM_R2D,sqrt(c11),
    c12/sqrt(c11)/sqrt(c22),sqrt(c22),sqrt(c33)*PM_R2D);
  if(interactive)
  {   
    pm_plotScan(&lsc,"red",2.0,true);
    dr_cov_ellipse(0,0,c11,c12,c22,"black");
    dr_fit();
    dr_zoom();
  }
         
  printf("Unit test......................PASSED\n");  
}
