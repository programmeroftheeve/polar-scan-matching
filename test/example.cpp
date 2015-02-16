/***************************************************************************
                         example.cpp  -  exmaple file for scan matching in 
                         polar coordinates sourcecode.
                           -------------------
   begin                : Tue dec 4 2007
   version              : 0.1   
   copyright            : (C) 2007-2010 by Albert Diosi 
   email                : albert.diosi@gmail.com      
***************************************************************************/
/****************************************************************************
Copyright (c) 2007-2010, Albert Diosi
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of the copyright holders may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h> 
#include <assert.h>
#include <polar_match.h>

#include "draw.h" //for showing matching results on the screen

using namespace std;

//STEP1: write your own function which can read 
//your scan log files
int read_scan2(FILE *fin, PMScan *ls);//reads psd sensor logs

int read_scan4(FILE *fin, PMScan *ls);//hokuyo one line per scan

int read_scan5(FILE *fin, PMScan *ls);//hokuyo UTM one line per scan, [m], no pose or time

void match_psd(void);

int laser_odometry_example(int argc, char *argv[]);

int laser_odometry_example_improved(int argc, char *argv[]);


/** @brief Main function calling scan matching demos for each laser scanner.
*/
int main(int argc, char *argv[])
{
  switch(PM_LASER)
  {
    case PM_HOKUYO_URG_04LX: 
      laser_odometry_example(argc,argv);
      break;
    case PM_HOKUYO_UTM_30LX: 
      laser_odometry_example(argc,argv);
      //laser_odometry_example_improved(argc,argv);
      break;
    case PM_PSD_SCANNER:
      match_psd();
      break;
    case PM_SICK_LMS200:
      laser_odometry_example(argc,argv);
      break;
    default:
      printf("Main: no action chosen.\n");
      break;
  }  
  return 0;
  printf("Finished.\n");
}//main


/** @brief Pose estimation by matching a sequence of laser scans.

This is an example for scan matching without using odometry
if you use odometry, then you'll likely get a better results.
As each consecutive pair of scan is matched, the pose error
will accumulate quickly.
*/
int laser_odometry_example(int argc, char *argv[])
{
  bool pairwise = false;
  if(PM_LASER_Y != 0)  
  {
    cerr <<"ERROR PM_LASER_Y  not 0 for mapping"<<endl;
    exit(-1);
  }
  
  if(argc > 4)
  {
    printf("usage: log_file_name [start_point] [p]\n");
    printf("Scans preceeding start_point are ignored. When p present, pairwise match results are shown.\n");
    return(-1);
  }
  
  char* fname;
  if(argc>1)
  {
    fname = argv[1];
  }
  else
  {
    switch(PM_LASER)
    {
      case PM_HOKUYO_URG_04LX: 
        fname = (char *)"data/hokuyo_urg_04lx_ug1_example.txt";
        break;
      case PM_HOKUYO_UTM_30LX: 
        fname = (char *)"data/hokuyo_utm_30lx_from_Mohsen_Akbaritabar.txt";
        break;
      case PM_SICK_LMS200:
        fname = (char *)"data/sick_lms200.log";
        break;
      default:
        exit(-1);
        break;
    }      
  }
  
  FILE *fin;
  
  //open the log file, initialize some internal variables 
  pm_init(fname,&fin);  
  
  int startPoint = 0;
  if(argc == 3 || argc == 4 )
  {
    assert(sscanf(argv[2],"%i",&startPoint)==1);
  }
  
  if(argc == 4)
  {
    assert(*argv[3]=='p');
    pairwise = true;
  }  
  
  //initialize the graphical display
  dr_init(600,600,-400, -200, 700, 1200); 
  
  PMScan ls,ls_last;
  
  double rx=0,ry=0,th=0;//robot starts at 0,0,0
  
  bool bFirst = true;
  int cnt = 0;
  int error = 0;
  while(!error)
  {      
    switch(PM_LASER)
    {
      case PM_HOKUYO_URG_04LX: 
          error = read_scan4(fin,&ls);
        break;
      case PM_HOKUYO_UTM_30LX: 
          error = read_scan5(fin,&ls);
        break;
      case PM_SICK_LMS200:
          error = read_scan4(fin,&ls);
        break;
      default:
        exit(-1);
        break;
    }      
    if(error)
    {
      printf("laser_odometry_example: reading scan failed.\n");
      continue;
    }
  
    cnt++;
    if(cnt < startPoint)
    {
      continue;
    }              
    
    cout <<endl<<cnt<<" t: "<<ls.t<< " ";
    
    //preprocess the scan...
    pm_preprocessScan(&ls);
        
    if(bFirst)    
    {
      bFirst = false;
      ls_last = ls;
    }
    else
    {
      //match against the last scan
      try{
        //match the last agains the current
        ls.rx = 0;ls.ry=0;ls.th=0;//matching without prior info
        ls_last.rx = 0;ls_last.ry=0;ls_last.th=0;//matching without prior info
        
        pm_psm(&ls_last,&ls);        
              
        //convert the match result into global pose
        double xx,yy,tth;
              
        xx = ls.rx*cos(th) - ls.ry*sin(th) + rx;
        yy = ls.rx*sin(th) + ls.ry*cos(th) + ry;
        tth= th + ls.th;

        rx = xx;
        ry = yy;
        th = tth;
              
        if(pairwise) //show only the results of the pairwise matches?
        {
          dr_erase();
          pm_plotScan(&ls_last,"black");
          pm_plotScan(&ls,"green");
                    
          bool  corridor = pm_is_corridor(&ls);
          PM_TYPE angle;
          if(corridor)
          {
            angle = pm_corridor_angle(&ls_last);
            cout <<" corridor"<<endl;
          }
          else
          {
            angle = 0;
          }
          PM_TYPE err_idx = pm_error_index(&ls_last,&ls);
          cout <<" err: "<<err_idx;
          
          double c11,c12, c22, c33;
          pm_cov_est(err_idx, &c11,&c12, &c22, &c33,corridor, angle);
          cout <<endl<<"Error index: "<<err_idx<<" angle "<<angle*PM_R2D<<endl;
          cout <<"COV: "<<sqrt(c11)<<" "<<c12/sqrt(c11)/sqrt(c22)<<" "<<sqrt(c22)<<" deg "<<sqrt(c33)*PM_R2D<<endl;
          dr_cov_ellipse(0,0,c11,c12,c22,"black");
          dr_fit();
          dr_zoom();                    
        }
        else
        {  
          ls.rx = xx; ls.ry = yy; ls.th = tth;           
          //if(cnt%30==0)
          pm_plotScanAt(&ls,rx,ry,th,"red",1.0);
          //dr_zoom();
/*          if(cnt%500==0)
          {
            dr_zoom();
            //dr_erase();
          }*/
        }        
                      
        ls.rx = 0;ls.ry=0;ls.th=0;     
        ls_last = ls;                
      }catch(int err)
      {
        cerr <<"error_caught: main"<<endl;
      };
    }    
  }//while
    
  fclose(fin);
  printf("\nFinished.\n");
  dr_zoom();
  return(0);
}//main


/** @brief Pose estimation by matching a sequence of laser scans. Reference scan is only switched when needed.

This is an example for scan matching without using odometry
if you use odometry, then you'll likely get a better results.
In this function the reference scan is changed only when the
match error increases or the number of matched points decreases.
*/
int laser_odometry_example_improved(int argc, char *argv[])
{
  bool pairwise = false;
  if(PM_LASER_Y != 0)  
  {
    cerr <<"ERROR PM_LASER_Y  not 0 for mapping"<<endl;
    exit(-1);
  }
  
  if(argc > 4)
  {
    printf("usage: log_file_name [start_point] [p]\n");
    printf("Scans preceeding start_point are ignored. When p present, pairwise match results are shown.\n");
    return(-1);
  }
  
  char* fname;
  if(argc>1)
  {
    fname = argv[1];
  }
  else
  {
    switch(PM_LASER)
    {
      case PM_HOKUYO_URG_04LX: 
        fname = (char *)"data/hokuyo_urg_04lx_ug1_example.txt";
        break;
      case PM_HOKUYO_UTM_30LX: 
        fname = (char *)"../../data/UTM_1.txt";
        break;
      case PM_SICK_LMS200:
        fname = (char *)"data/sick_lms200.log";
        break;
      default:
        exit(-1);
        break;
    }
  }
  
  FILE *fin;
  
  //open the log file, initialize some internal variables 
  pm_init(fname,&fin);  
  
  int startPoint = 0;
  if(argc == 3 || argc == 4 )
  {
    assert(sscanf(argv[2],"%i",&startPoint)==1);
  }
  
  if(argc == 4)
  {
    assert(*argv[3]=='p');
    pairwise = true;
  }  
  
  //initialize the graphical display
  dr_init(600,600,-400, -200, 700, 1200); 
  
  PMScan ls,ls_last, ls_ref;
  
  double rx=0,ry=0,th=0;//robot starts at 0,0,0
  double xx_last=0,yy_last=0,tth_last=0;  
  
  
  bool bFirst = true;
  int cnt = 0;
  int error = 0;
  
  while(!error)
  {      
  
    switch(PM_LASER)
    {
      case PM_HOKUYO_URG_04LX: 
          error = read_scan4(fin,&ls);
        break;
      case PM_HOKUYO_UTM_30LX: 
          error = read_scan5(fin,&ls);
        break;
      case PM_SICK_LMS200:
          error = read_scan4(fin,&ls);
        break;
      default:
        exit(-1);
        break;
    }      
    if(error)
    {
      printf("laser_odometry_example: reading scan failed.\n");
      continue;
    }
  
    cnt++;
    if(cnt < startPoint)
    {
      continue;
    }      
        
    
    cout <<endl<<cnt<<" t: "<<ls.t<< " ";
    
    //preprocess the scan...
    pm_preprocessScan(&ls);
        
    if(bFirst)    
    {
      bFirst = false;
      ls.rx = 0; ls.ry = 0;ls.th = 0;
      ls_last = ls;
      ls_ref  = ls;
    }
    else
    {      
      //matching using the previous result as prior info
      ls.rx = ls_last.rx;
      ls.ry = ls_last.ry;
      ls.th = ls_last.th;
      //matching without prior info
      ls_ref.rx = 0;ls_ref.ry=0;ls_ref.th=0;
        
      bool matchFailed = false;
      try
      {
        pm_psm(&ls_ref,&ls);
      }catch(int err)
      {
        cerr << "Error caught: failed match." << endl;
        matchFailed = true;
      }
      
      PM_TYPE err_idx = pm_error_index2(&ls_last,&ls);
      cout <<" err: "<<err_idx;
      
      bool refSwitched = false;
      if(matchFailed || err_idx > 5.0)
      {
        refSwitched = true;
        printf(" Switching reference scan.\n");
        ls_ref = ls_last;        
        ls.rx = 0;ls.ry = 0; ls.th = 0;
        ls_ref.rx = 0;ls_ref.ry = 0; ls_ref.th = 0;
        pm_psm( &ls_ref, &ls );
        rx = xx_last;
        ry = yy_last;
        th = tth_last;
        ls_ref = ls_last;
      }
                            
      //convert the match result into global pose
      double xx,yy,tth;
            
      xx = ls.rx*cos(th) - ls.ry*sin(th) + rx;
      yy = ls.rx*sin(th) + ls.ry*cos(th) + ry;
      tth= th + ls.th;
            
      ls.rx = xx; ls.ry = yy; ls.th = tth;           
      //if(cnt%30==0)
      if(refSwitched || cnt == 2)
        pm_plotScanAt(&ls,rx,ry,th,"red",1.0);
      
/*      if(cnt%500==0)
      {
        dr_zoom();
        //dr_erase();
      } */
      ls.rx = 0;ls.ry=0;ls.th=0;     
      ls_last = ls; 
      xx_last = xx;
      yy_last = yy;
      tth_last = tth;
    }    
  }//while
    
  fclose(fin);
  printf("\nFinished.\n");
  dr_zoom();
  return(0);
}//main


/** @brief Reads out a scan from a file storing scans from the PSD scanner.
*/
int read_scan2(FILE *fin, PMScan *ls)
{
  int n=0;  
  //Time nor pose are available from the log
  ls->rx = 0;
  ls->ry = 0;
  ls->t  =-1;
  ls->th = 0;

  for(int i=0; i<PM_L_POINTS; i++)
  {
    float fi;
    n+=fscanf(fin,"%f %f ",&fi,&(ls->r[i]));
    ls->x[i] = (ls->r[i])*pm_co[i];
    ls->y[i] = (ls->r[i])*pm_si[i];
    ls->bad[i] = 0;
    if(ls->r[i]<10)
    {
      ls ->r[i] = 10000;//set it to a value larger than the max. valid reading
    }
  }
  if(n!=(2*PM_L_POINTS))
    return -1;
  return 0;    
}

/** @brief Matches two low resolution, low-quality scans.
*/
void match_psd(void)
{
  assert(PM_LASER == PM_PSD_SCANNER);
  
  dr_init(600,600,-500, -500, 500, 500); 

  PMScan ls,ls_last;
   
  FILE* fin;  
  pm_init("data/bath_1.scan",&fin);    
  read_scan2(fin,&ls_last);
  fclose(fin);
  
  pm_preprocessScan(&ls_last);

  pm_init("data/bath_2.scan",&fin);  
  read_scan2(fin,&ls);
  fclose(fin);
  
  pm_preprocessScan(&ls);

  printf("Scan 1\n");
  pm_plotScan(&ls,"red");
  dr_fit();
  dr_zoom();

  printf("Scan 1 and 2\n");
  pm_plotScan(&ls_last,"black");
  dr_fit();
  dr_zoom();
    
  pm_psm(&ls_last,&ls);
      
  printf("Scan 1 and 2 matched\n");
  dr_erase();
  pm_plotScan(&ls_last,"black");
  pm_plotScan(&ls,"blue"); 
  dr_zoom();
}


/** @brief Reads out a series of ranges in mm corresponding to a scan.

Reads Hokuyo URG-04LX-UG1 data organized in one scan per line format: <br>
time[ms] x[cm] y[cm] th[deg] r44[mm] ... r725[mm]
@param fin The opened input file.
@param ls The read scan is stored here.
*/
int read_scan4(FILE *fin, PMScan *ls)
{
  int n=0;
  
  int time;
  n += fscanf(fin,"%i %f %f %f ", &time, &ls->rx, &ls->ry, &ls->th);
  
  ls->t = (float)time/1000.0f;
  ls->th    = ls->th * PM_D2R;

  for(int i=0; i<PM_L_POINTS; i++)
  {
    n+=fscanf(fin,"%f ",&(ls->r[i]));
    ls->r[i] = ls->r[i] /10.0;//convert mm to cm.
    ls->x[i] = (ls->r[i])*pm_co[i];
    ls->y[i] = (ls->r[i])*pm_si[i];
    ls->bad[i] = 0;
    //in the urg log there are small readings - however, PSM 
    //treats large readings as bad not the 0 readings
    if(ls->r[i]<10)
    {
      ls ->r[i] = 10000;//set it to a value larger than the max. valid reading
    }
  }
  if(n!=(PM_L_POINTS+4))
    return -1;
  return 0;    
}


/** @brief Reads out a series of ranges in m, corresponding to a scan.

Hokuyo UTM one line per scan, [m], no pose or time
Reads Hokuyo UTM-30LX data organized in one scan per line format: <br>
r0[mm] ... r1080[m]
@param fin The opened input file.
@param ls The read scan is stored here.
*/
int read_scan5(FILE *fin, PMScan *ls)
{
  int n=0;
  
  ls->rx = 0;
  ls->ry = 0;
  ls->th = 0;  
  ls->t = -1.0;
  
  for(int i=0; i<PM_L_POINTS; i++)
  {
    n+=fscanf(fin,"%f ",&(ls->r[i]));
    ls->r[i] = ls->r[i] *100.0;//convert mm to cm.
    ls->x[i] = (ls->r[i])*pm_co[i];
    ls->y[i] = (ls->r[i])*pm_si[i];
    ls->bad[i] = 0;
    
    if(ls->r[i]<10)
    {
      ls ->r[i] = 10000;//set it to a value larger than the max. valid reading
    }
  }
  if(n!=(PM_L_POINTS))
    return -1;
  return 0;    
}
