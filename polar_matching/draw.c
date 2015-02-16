/***************************************************************************
                          draw.c  -  simplistic drawing module using X11
                             -------------------
    begin                : Sun Dec 21 2003
    version              : 0.2    
    copyright            : (C) 2003-2010 by Albert Diosi and Lindsay Kleeman 
    email                : albert.diosi@gmail.com
    TODO:                The allocated colours should be freed. Colour search/allocation takes too much time;
                         Fix bug in which the colour changes when zooming.
                         Fix bug: grid sometimes appears and sometimes does not.
    change:             -1/08/2010 Adding switch to disable graphics.
                        -19/12/2009 Changed to react to keys while the graphical window is
                           active and to usel left mouse click to pan and the wheel to zoom.
                        -14/03/2005 rewriting to use only X, instead of libplot
                            however, markers are not implemented! assumes equal
                            pixel size! saving of window as png added -slow though
                            added markers(circle,triangle and cross)
 ***************************************************************************/
/****************************************************************************
Copyright (c) 2003-2010, Albert Diosi and Lindsay Kleeman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of the copyright holders may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/
#ifdef __linux__
  #define DR_ENABLE //Comment out this line when graphics is not needed.
  #include <unistd.h> 
#endif

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifdef DR_ENABLE
  #include <X11/Xlib.h>
  #include <X11/Xutil.h>
#endif  

/*markers are not scaled*/
#define DR_MARKER_CIRCLE   1 ///< Constant used with dr_marker() to draw a small circle.
#define DR_MARKER_TRIANGLE 2 ///< Constant used with dr_marker() to draw a small triange.
#define DR_MARKER_CROSS    3 ///< Constant used with dr_marker() to draw a small cross.

/* Program wide globals */

/// Array of colors one can use for drawing
const char* dr_COLORS[] = {"red","green","blue","gray","yellow","magenta","pink","lightgray","brown"};
const int   dr_COLORS_CNT = 9;///< The number of colours available for drawing.



#ifdef DR_ENABLE
//------------Internal definitions and functions----------------
int handle;
#define DR_MARKER_SIZE   3 ///<[pixels] The size markers are drawn at.
#define MAX_OBJECTS 400000 ///<Maximum nuber of lines+markers+texts.

#define LINE    1
#define MARKER  2
#define TEXT    3
#define CIRCLE  4

/*Module wide globals*/
Display       *dr_Display;
int           dr_Screen;
int           dr_Depth;
unsigned long dr_BlackPixel;
unsigned long dr_WhitePixel;
Window        dr_Window;//window id
GC            dr_GC;
Colormap      dr_Colormap;

struct Axis{
  double xmin,ymin,xmax,ymax;
} axis; //the area represented buy the window
double dr_xm,dr_ym;//screen size in pixels - has to be updatet
double dr_kx,dr_ky;// kx=dr_xm/(axis.xmax-axis.xmin)

//transforms x coordinates from the
int inline dr_dispx(double x)
{
  return (int)((x-axis.xmin)*dr_kx);
}
int inline dr_dispy(double y)
{
  return (int)(dr_ym-(y-axis.ymin)*dr_ky);
}

struct Line{ int t;  double x1,y1,x2,y2;  char color[100];  };
struct Circle{int t;  double x,y,r;  char color[100];  };
struct Marker{int t;double x,y;char color[100];int type;};
struct Text {int t;double x,y;char text[1000];int hz_al,vr_al;char color[100];};
//t is the type

union Obj {struct Line l;struct Marker m;struct Text t;struct Circle c;};

union Obj obj[MAX_OBJECTS];//stores lines, texts ...
int  obj_nmb;//number of objects stored

int dr_axes_equal=1;

//making dx equal to dy
//dr_xm, dr_ym i.e. the window size must be updated prior calling this function
void axisEqual(double *xmin,double* ymin,double* xmax,double *ymax)
{
  double dx=(*xmax-*xmin)/2.0;
  double dy=(*ymax-*ymin)/2.0;
  double cx=(*xmax+*xmin)/2.0;
  double cy=(*ymax+*ymin)/2.0;

  XWindowAttributes win_attr;
  XGetWindowAttributes(dr_Display, dr_Window, &win_attr);
  dr_xm = win_attr.width;
  dr_ym = win_attr.height;
  if(dr_axes_equal)
  {
    if(dx>dy)
    {
      *ymax = cy + (dx*dr_ym)/dr_xm;
      *ymin = cy - (dx*dr_ym)/dr_xm;
    }
    else
    {
      *xmax = cx + (dy*dr_xm)/dr_ym;
      *xmin = cx - (dy*dr_xm)/dr_ym;
    }
  }
  dr_kx = dr_xm/(axis.xmax-axis.xmin);
  dr_ky = dr_ym/(axis.ymax-axis.ymin);
}

void draw_grid(void)
{
  int     n,nx,ny;//how many intervals in each direction
  int     i;
  double  dx = (axis.xmax-axis.xmin);
  double  dy = (axis.ymax-axis.ymin);
  double  dinc,dincx,dincy,d;
  char s[1000];
  char *str,*strx, *stry;

  d   = dx;
  dinc= 1;
  n   = d/1.0;
  str = (char *)"1 cm";
  if(n>30)
  {  
    n=d/10.0;
    dinc=10;
    str = (char *)"10 cm";
    if(n>30)
    {
      n=d/100.0;
      dinc=100;
      str = (char *)"1 m";
      if(n>30)
      {
        n=d/100.0;
        dinc=1000;
        str = (char *)"10 m";
      }
    }
  }
  dincx = dinc;
  nx = n;
  strx = str;

  if(!dr_axes_equal)
  {
      d   = dy;
      dinc= 1;
      n   = d/1.0;
      str = (char *)"1 cm";
      if(n>30)
      {
        n=d/10.0;
        dinc=10;
        str = (char *)"10 cm";
        if(n>30)
        {
          n=d/100.0;
          dinc=100;
          str = (char *)"1 m";
          if(n>30)
          {
            n=d/100.0;
            dinc=1000;
            str = (char *)"10 m";
          }
        }
      }
  }
  dincy = dinc;
  ny = n;
  stry = str;
    
  //choose color for the mesh                
  XColor col;
//  XParseColor(dr_Display, dr_Colormap, "gray40", &col); //the original
  XParseColor(dr_Display, dr_Colormap, "black", &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);

  double xstart = (int)(axis.xmin/dincx)*dincx;
  double ystart = (int)(axis.ymin/dincy)*dincy;
  for(i=0;i<=nx;i++)
  {
    XDrawLine(dr_Display,dr_Window,dr_GC,dr_dispx(xstart+i*dincx),
              dr_dispy(axis.ymin),dr_dispx(xstart+i*dincx),dr_dispy(axis.ymax));
    //putting the numbers to the axes
    sprintf(s,"%.0f",xstart+i*dincx);
    XDrawString(dr_Display, dr_Window, dr_GC, dr_dispx(xstart+i*dincx),
      dr_dispy(axis.ymin), s,strlen(s));
  }
  for(i=0;i<=ny;i++)
  {
    XDrawLine(dr_Display,dr_Window,dr_GC,
        dr_dispx(axis.xmin),dr_dispy(ystart+i*dincy),
        dr_dispx(axis.xmax),dr_dispy(ystart+i*dincy));//horizontal
    //putting the numbers to the axes

    sprintf(s,"%.0f",ystart+i*dincy);
    XDrawString(dr_Display, dr_Window, dr_GC, dr_dispx(axis.xmin),
      dr_dispy(ystart+i*dincy), s,strlen(s));
  }
  //drawing the main axes
  XParseColor(dr_Display, dr_Colormap, /*"gray8"*/"black", &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);
  
  if(axis.xmin<=0 && axis.xmax>=0)
    XDrawLine(dr_Display,dr_Window,dr_GC,dr_dispx(0),dr_dispy(axis.ymin),
        dr_dispx(0),dr_dispy(axis.ymax));
  if(axis.ymin<=0 && axis.ymax>=0)
    XDrawLine(dr_Display,dr_Window,dr_GC,dr_dispx(axis.xmin),dr_dispy(0),
        dr_dispx(axis.xmax),dr_dispy(0));
}

int createGC(Window theNewWindow,GC *theNewGC)
{
  XGCValues theGCValues;
  *theNewGC = XCreateGC(dr_Display,theNewWindow,(unsigned long) 0,&theGCValues);
  if(*theNewGC == 0)
          return 0;
  else{
          XSetForeground(dr_Display,*theNewGC,dr_WhitePixel);
          XSetBackground(dr_Display,*theNewGC,dr_BlackPixel);
          return 1;
  }
}

Window OpenWindow(int x, int y, int width, int height, int flag,GC *theNewGC)
{
  XSetWindowAttributes theWindowAttributes;
  unsigned long theWindowMask;
  XSizeHints theSizeHints;
  XWMHints theWMHints;
  Window theNewWindow;
    
  /*Setting the attributes*/
  theWindowAttributes.border_pixel = BlackPixel(dr_Display,dr_Screen);
  theWindowAttributes.background_pixel = WhitePixel(dr_Display,dr_Screen);
  theWindowAttributes.override_redirect = False;
    
  theWindowMask = CWBackPixel|CWBorderPixel|CWOverrideRedirect;
    
  #define DR_BORDER_WIDTH 2

  theNewWindow = XCreateWindow( dr_Display,
                  RootWindow(dr_Display,dr_Screen),
                  x,y,width,height,
                  DR_BORDER_WIDTH,dr_Depth,
                  InputOutput,
                  CopyFromParent,
                  theWindowMask,
                  &theWindowAttributes);
  
  
  theWMHints.initial_state = NormalState;
  theWMHints.flags = StateHint;
    
  XSetWMHints(dr_Display,theNewWindow,&theWMHints);
  
  theSizeHints.flags = PPosition | PSize;
  theSizeHints.x = x;
  theSizeHints.y = y;
  theSizeHints.width = width;
  theSizeHints.height = height;
    
  XSetNormalHints(dr_Display,theNewWindow,&theSizeHints);
    
  if( createGC(theNewWindow,theNewGC) == 0){
          XDestroyWindow(dr_Display,theNewWindow);
          return( (Window) 0);
          }
  
  
  XMapWindow(dr_Display,theNewWindow);
  XFlush(dr_Display);
  
  return theNewWindow;  
}
#endif

/** @brief Initialize the drawing module.

@param screen_y_max [pixel] The height of the drawing screen in pixels.
@param screen_x_max [pixel] The width of the drawing screen in pixels.
@param xmin Minimum x coordinate of the drawing area in drawing units.
@param ymin Minimum y coordinate of the drawing area in drawing units.
@param xmax Maximum x coordinate of the drawing area in drawing units.
@param ymax Maximum y coordinate of the drawing area in drawing units.
*/
void dr_init(int screen_x_max,int screen_y_max,double xmin, double ymin, double xmax, double ymax)
{
#ifdef DR_ENABLE
  dr_Display = XOpenDisplay(NULL);
  
  dr_Screen = DefaultScreen(dr_Display);
  dr_Depth = DefaultDepth(dr_Display,dr_Screen);
  dr_BlackPixel = WhitePixel(dr_Display,dr_Screen);
  dr_WhitePixel = BlackPixel(dr_Display,dr_Screen);
  dr_Colormap  = DefaultColormap(dr_Display,dr_Screen);
  
  dr_Window = OpenWindow(0,0,screen_x_max, screen_y_max,0,&dr_GC);
  
  XSelectInput(dr_Display, dr_Window, ExposureMask|ButtonPressMask|KeyPressMask);
  
  axis.xmin=xmin; axis.xmax=xmax; axis.ymin=ymin; axis.ymax=ymax;
  axisEqual(&axis.xmin, &axis.ymin, &axis.xmax, &axis.ymax);
  
  draw_grid();
  obj_nmb=0;
  
  printf("Draw library: use mouse wheel to zoom, click with left button to pan.\n");
//    printf("screen %i Window %i\n",dr_Screen,dr_Window);
#else
  printf("Draw library: disabled. To enable it uncomment #define DR_ENABLE in draw.c\n");
#endif
}

/** @brief Close the drawing window.
*/
void dr_close(void)
{
#ifdef DR_ENABLE
    XDestroyWindow(dr_Display,dr_Window);
#endif    
}

/** @brief Plot a line between [x1,y1] and [x2,y2] using @a color.
@param x1 x coordinate of the first point.
@param y1 y coordinate of the first point.
@param x2 x coordinate of the second point.
@param y2 y coordinate of the second point.
@param color The color of the line as "red", "green",... see dr_COLORS.
*/
void dr_line(double x1, double y1, double x2, double y2,const char *color)
{
#ifdef DR_ENABLE
  XColor col;
  XParseColor(dr_Display, dr_Colormap, color, &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);
  
  XDrawLine(dr_Display,dr_Window,dr_GC,
                  dr_dispx(x1),dr_dispy(y1),
                  dr_dispx(x2),dr_dispy(y2));
  XFlush(dr_Display);
                  
  //struct Line{ int t;  double x1,y1,x2,y2;  char* color;  };

  obj[obj_nmb].l.t = LINE;
  obj[obj_nmb].l.x1 = x1;
  obj[obj_nmb].l.y1 = y1;
  obj[obj_nmb].l.x2 = x2;
  obj[obj_nmb].l.y2 = y2;
  strcpy(obj[obj_nmb].l.color,color);
  obj_nmb++;
  if(obj_nmb>=MAX_OBJECTS)
  {
      fprintf (stderr, "draw.c: Max number of stored objects reached");
      obj_nmb--;
  }
#endif  
}

/** @brief Plot a circle of radius @a r at [x,y] using @a color.

@param  x X coordinate of the center of the circle.
@param  y Y coordinate of the center of the circle.
@param  r Radius of the circle.
@param color The color of the circle as "red", "green",... see dr_COLORS.
*/
void dr_circle(double x, double y, double r,const char *color)
{ 
#ifdef DR_ENABLE
  XColor col;

  XParseColor(dr_Display, dr_Colormap, color, &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);

  if( (2.0*r*dr_kx)>2 &&  (2.0*r*dr_ky)>2 ) // check if larger than 2 pixels
    XDrawArc(dr_Display, dr_Window, dr_GC,dr_dispx(x-r),
             dr_dispy(y+r), 2.0*r*dr_kx, 2.0*r*dr_ky, 0, 23040);
  else
    XDrawPoint(dr_Display, dr_Window, dr_GC,dr_dispx(x-r),dr_dispy(y+r));
  
  XFlush(dr_Display);
    
  obj[obj_nmb].c.t = CIRCLE;
  obj[obj_nmb].c.x = x;
  obj[obj_nmb].c.y = y;
  obj[obj_nmb].c.r = r;
  strcpy(obj[obj_nmb].c.color,color);
  obj_nmb++;
  if(obj_nmb>=MAX_OBJECTS)
  {
      fprintf (stderr, "draw.c: Max number of stored objects reached");
      obj_nmb--;
  }
#endif  
}

/** @brief Plot a covariance ellipse to depict uncertainty.

The covariance ellipse is drawn by breaking it up 
into line segments.

Acknowledgement: the core equations are from Lindsay's kzoom.

@param  x X coordinate of the mean of the distribution. 
@param  y Y coordinate of the mean of the distribution. 
@param  cxx The variance of the X coordinate.
@param  cxy The XY covariance.
@param  cyy The variance of the Y coordinate.
@param color The color of the ellipse as "red", "green",... see dr_COLORS.
*/
void dr_cov_ellipse(double x, double y, double cxx,double cxy,double cyy,const char *color)
{
#ifdef DR_ENABLE
  double angle_major_rad, major_stdev, minor_stdev;
  XColor col;

  XParseColor(dr_Display, dr_Colormap, color, &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);

  double discrim, lambda;
  // first find largest eigenvalue
  discrim = (cxx- cyy)*(cxx- cyy)+4.0*cxy*cxy;
  if (discrim > 0)  // rounding errors can result in < 0
    discrim = sqrt(discrim)/2.0;
  else (discrim = 0.0);

  lambda = (cxx+cyy)/2.0 + discrim;
  if (cxy != 0)
    angle_major_rad = atan2(cxx-lambda, -cxy);
  else angle_major_rad = 0.0;
  major_stdev = sqrt(lambda);
  lambda = (cxx+cyy)/2.0 - discrim;
  if (lambda > 0)
    minor_stdev = sqrt(lambda);
  else
    minor_stdev = 0.0;

  const int ANGLE_DIVISIONS = 12;
  const double angle_inc = M_PI*2.0/(ANGLE_DIVISIONS-1.0);
  const double c = cos(angle_major_rad), s = sin(angle_major_rad);
  int i;

  float last_x = major_stdev*c+x, last_y=major_stdev*s+y;
  for (i=1; i<ANGLE_DIVISIONS; i++)
  {
    double cosa = cos(i*angle_inc), sina = sin(i*angle_inc);
    double xp = major_stdev*cosa*c-minor_stdev*sina*s + x;
    double yp = major_stdev*cosa*s+minor_stdev*sina*c + y;

    XDrawLine(dr_Display,dr_Window,dr_GC,
                  dr_dispx(last_x),dr_dispy(last_y),
                  dr_dispx(xp),dr_dispy(yp));
    
  //struct Line{ int t;  double x1,y1,x2,y2;  char* color;  };

    obj[obj_nmb].l.t = LINE;
    obj[obj_nmb].l.x1 = last_x;
    obj[obj_nmb].l.y1 = last_y;
    obj[obj_nmb].l.x2 = xp;
    obj[obj_nmb].l.y2 = yp;
    strcpy(obj[obj_nmb].l.color,color);
    obj_nmb++;
    if(obj_nmb>=MAX_OBJECTS)
    {
        fprintf (stderr, "draw.c: Max number of stored objects reached");
        obj_nmb--;
    }
    last_x = xp; last_y = yp;
  } // for
  XFlush(dr_Display);
#endif  
}

/** @brief Place a marker to (x,y).
Possible markers are: DR_MARKER_CIRCLE, DR_MARKER_TRIANGLE, DR_MARKER_CROSS.
@param  x X coordinate of the marker.
@param  y Y coordinate of the marker.
@param  type The type of the marker. One of DR_MARKER_CIRCLE, DR_MARKER_TRIANGLE, DR_MARKER_CROSS. 
@param color The color of the marker as "red", "green",... see dr_COLORS.
*/
void dr_marker(double x, double y, int type,const char *color)
{
#ifdef DR_ENABLE
  obj[obj_nmb].m.t = MARKER;
  obj[obj_nmb].m.x = x;
  obj[obj_nmb].m.y = y;
  obj[obj_nmb].m.type = type;
  strcpy(obj[obj_nmb].m.color,color);
  obj_nmb++;
  if(obj_nmb>=MAX_OBJECTS)
  {
      fprintf (stderr, "draw.c: Max number of stored objects reached");
      obj_nmb--;
  }
#endif  
}

/** @brief Place a text to (x,y).
@param  x X coordinate of the text.
@param  y Y coordinate of the text.
@param  hz_al Horizontal alignment. - CURRENTLY NOT USED.
@param  vr_al Horizontal alignment. - CURRENTLY NOT USED.
@param  text The text to be shown.
@param color The color of the marker as "red", "green",... see dr_COLORS.
*/
void dr_text(double x, double y, int hz_al,int vr_al,const char *text, const char* color)
{
#ifdef DR_ENABLE
  XColor col;
  XParseColor(dr_Display, dr_Colormap, color, &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);

  XDrawString(dr_Display, dr_Window, dr_GC,dr_dispx(x),
  dr_dispy(y), text,strlen(text));

//struct Text {int t;double x,y;char *text;int hz_al,vr_al;char color[100];};
  obj[obj_nmb].t.t = TEXT ;
  obj[obj_nmb].t.x = x;
  obj[obj_nmb].t.y = y;
  obj[obj_nmb].t.hz_al = hz_al ;
  obj[obj_nmb].t.vr_al = vr_al;
  strcpy(obj[obj_nmb].t.color,color);
  strcpy(obj[obj_nmb].t.text,text);
  obj_nmb++;
  if(obj_nmb>=MAX_OBJECTS)
  {
      fprintf (stderr, "draw.c: Max number of stored objects reached");
      obj_nmb--;
  }
  XFlush(dr_Display);
#endif  
}


/** @brief Temporary code to output text to screen coordinates.
@param  px [Pixel] X coordinate of the text.
@param  py [Pixel] Y coordinate of the text.
@param  text The text to be shown.
@param color The color of the marker as "red", "green",... see dr_COLORS.
*/
void dr_text_pix(int px, int py,const char *text, const char* color)
{
#ifdef DR_ENABLE
  XColor col;
  XParseColor(dr_Display, dr_Colormap, color, &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);

  XDrawString(dr_Display, dr_Window, dr_GC,px,py,text,strlen(text));

  XFlush(dr_Display);
#endif  
}


/** @brief Redraw the screen content.
*/
void dr_replot(void)
{
#ifdef DR_ENABLE
  int i;
  XColor col;
  double x,y,x1,x2,x3,y1,y2,y3;

  char* lastColour = "white";
  XParseColor(dr_Display, dr_Colormap, lastColour, &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);
  XFillRectangle(dr_Display, dr_Window, dr_GC, 0, 0, (int)dr_xm, (int)dr_ym);

  draw_grid();
  for(i=0;i<obj_nmb;i++)
  {
    if( strcmp(obj[i].l.color, lastColour) )
    {
      //Searching for colour is expensive - change it later. Don't forget to deallocate. 
      XParseColor(dr_Display, dr_Colormap, obj[i].l.color, &col);
      XAllocColor(dr_Display, dr_Colormap, &col);
      XSetForeground(dr_Display, dr_GC, col.pixel);
      lastColour = obj[i].l.color;
    }
    
    switch(obj[i].l.t){
      case LINE:
      
          XDrawLine(dr_Display,dr_Window,dr_GC,
                  dr_dispx(obj[i].l.x1),dr_dispy(obj[i].l.y1),
                  dr_dispx(obj[i].l.x2),dr_dispy(obj[i].l.y2));

        break;
      case MARKER:
          switch(obj[i].m.type)
          {
            case DR_MARKER_CIRCLE:            
               XDrawArc(dr_Display, dr_Window, dr_GC,dr_dispx(obj[i].m.x)-DR_MARKER_SIZE,
                 dr_dispy(obj[i].m.y)-DR_MARKER_SIZE, 2.0*DR_MARKER_SIZE, 2.0*DR_MARKER_SIZE, 0, 23040);            
              break;
            case DR_MARKER_TRIANGLE:
              x = dr_dispx(obj[i].m.x);
              y = dr_dispy(obj[i].m.y);
              x1 = x;
              y1 = y-DR_MARKER_SIZE;
              x2 = x-DR_MARKER_SIZE;
              y2 = y+DR_MARKER_SIZE;
              x3 = x+DR_MARKER_SIZE;
              y3 = y+DR_MARKER_SIZE;
              XDrawLine(dr_Display,dr_Window,dr_GC,x1,y1,x2,y2);
              XDrawLine(dr_Display,dr_Window,dr_GC,x2,y2,x3,y3);
              XDrawLine(dr_Display,dr_Window,dr_GC,x3,y3,x1,y1);
              break;            
            case DR_MARKER_CROSS:
              x = dr_dispx(obj[i].m.x);
              y = dr_dispy(obj[i].m.y);
              XDrawLine(dr_Display,dr_Window,dr_GC,x,y-DR_MARKER_SIZE,x,y+DR_MARKER_SIZE);
              XDrawLine(dr_Display,dr_Window,dr_GC,x-DR_MARKER_SIZE,y,x+DR_MARKER_SIZE,y);
              break;            
          }
        break;
      case TEXT:
          XDrawString(dr_Display, dr_Window, dr_GC,dr_dispx(obj[i].t.x),
          dr_dispy(obj[i].t.y), obj[i].t.text,strlen(obj[i].t.text));
        break;
      case CIRCLE:
          if(axis.xmin<obj[i].c.x && axis.xmax>obj[i].c.x
          && axis.ymin<obj[i].c.y && axis.ymax>obj[i].c.y)
          {
             XParseColor(dr_Display, dr_Colormap, obj[i].c.color, &col);
             XAllocColor(dr_Display, dr_Colormap, &col);
             XSetForeground(dr_Display, dr_GC, col.pixel);

             if(obj[i].c.r*dr_ky>1)
             {
               XDrawArc(dr_Display, dr_Window, dr_GC,dr_dispx(obj[i].c.x-obj[i].c.r),
               dr_dispy(obj[i].c.y+obj[i].c.r), 2.0*obj[i].c.r*dr_kx, 2.0*obj[i].c.r*dr_ky, 0, 23040);
             }
             else
               XDrawPoint(dr_Display, dr_Window, dr_GC,dr_dispx(obj[i].c.x),dr_dispy(obj[i].c.y) );
             
          }
        break;
    }
  }
  XFlush(dr_Display);
  #endif
}


/** @brief Scale the shown area.
Prior rescaling, the area is normalized to keep the aspect ratio.
@param  xmin Minimum X coordinate of the shown area.
@param  ymin Minimum Y coordinate of the shown area.
@param  xmax Maximum X coordinate of the shown area.
@param  ymax Maximum X coordinate of the shown area.
*/
void dr_scale(double xmin, double ymin, double xmax, double ymax)
{
#ifdef DR_ENABLE
  axisEqual(&xmin, &ymin, &xmax, &ymax);
  axis.xmin=xmin;axis.xmax=xmax;axis.ymin=ymin;axis.ymax=ymax;
  dr_replot();
#endif  
}
/** @brief Clear the screen and erase the drawn objects from memory.
*/
void dr_erase(void)
{
#ifdef DR_ENABLE
  XColor col;

  XParseColor(dr_Display, dr_Colormap, "white", &col);
  XAllocColor(dr_Display, dr_Colormap, &col);
  XSetForeground(dr_Display, dr_GC, col.pixel);
  XFillRectangle(dr_Display, dr_Window, dr_GC, 0, 0, (int)dr_xm, (int)dr_ym);
  obj_nmb=0;
  draw_grid();  
#endif  
}

/** @brief Interactive zooming until enter, space or 'q' is pressed.

Use mouse or numeric keypad to pan/zoom. On the numeric keypad 9,3 will 
zoom in/out, 2,8 will move the drawing up/down, 4,6 will move left/right.
Push r to redraw and c to reset to initial zoom level and view position.

When using the mouse, left click on figure top/bottom will move the drawing
down/up, clicking on the left or right side of the figure will move the
drawing right or left. Moving the wheel will zoom in and out.
*/
void dr_zoom(void)
{
#ifdef DR_ENABLE
  int ch=0;
  double dx,dy,cx,cy;
  struct Axis oldax = axis;
    
  // xwindow event handling learned from:
  // http://www.math.msu.su/~vvb/2course/Borisenko/CppProjects/GWindow/xintro.html
  KeySym  win_key;
  XEvent  event;   
  const int buff_len = 1024;
  char    buff[buff_len];//stores keypresses
  
  while(ch!='q')
  {         
      ch = 0;
      XNextEvent(dr_Display, &event);            
      if ( event.type == KeyPress &&  XLookupString(&event.xkey,buff,buff_len,&win_key,0) == 1) 
      {
        ch = buff[0];
      }
      if (event.type == ButtonPress) 
      {      
        //printf("You pressed a button at (%i,%i) %i\n",event.xbutton.x,event.xbutton.y, event.xbutton.button);
        switch (event.xbutton.button)
        {
          case 5: ch = '3'; break;//zoom out - roll up
          case 4: ch = '9'; break;//zoom in - roll down
          case 1:  //left button
                  if(event.xbutton.y < dr_ym/3)
                      ch = '8'; //up
                  if(event.xbutton.y > (2*dr_ym)/3) 
                      ch = '2';//down
                  if(event.xbutton.x < dr_xm/3)
                      ch = '4';//left
                  if(event.xbutton.x > (2*dr_xm)/3) 
                      ch = '6';//right
          break;
        } 
      }                
                      
      switch(ch)
      {
        case '6': // right
              dx = axis.xmax-axis.xmin;
              axis.xmax +=  dx/4.0;
              axis.xmin +=  dx/4.0;
                break;
        case '4': //left 
              dx = axis.xmax-axis.xmin;
              axis.xmax -=  dx/4.0;
              axis.xmin -=  dx/4.0;
                break;
        case '2': //down
              dy = axis.ymax-axis.ymin;
              axis.ymax -=  dy/4.0;
              axis.ymin -=  dy/4.0;
                break;
        case '8': // up
              dy = axis.ymax-axis.ymin;
              axis.ymax +=  dy/4.0;
              axis.ymin +=  dy/4.0;
                break;
        case '9':
              dx = axis.xmax-axis.xmin;
              dy = axis.ymax-axis.ymin;
              cx = (axis.xmax+axis.xmin)/2.0;
              cy = (axis.ymax+axis.ymin)/2.0;
              axis.xmin = cx-dx/3.6;
              axis.xmax = cx+dx/3.6;
              axis.ymin = cy-dy/3.6;
              axis.ymax = cy+dy/3.6;
            break;
        case '3':
              dx = axis.xmax-axis.xmin;
              dy = axis.ymax-axis.ymin;
              cx = (axis.xmax+axis.xmin)/2.0;
              cy = (axis.ymax+axis.ymin)/2.0;
        
              axis.xmin = cx-dx*0.7;
              axis.xmax = cx+dx*0.7;
              axis.ymin = cy-dy*0.7;
              axis.ymax = cy+dy*0.7;
            break;
        case 'c' : axis=oldax;
          break;
        case ' ':case 'q': case 10: // abort
                  ch ='q';
                break;
      } // switch
      if(ch!='q' && ch != 0)
      {
        axisEqual(&axis.xmin, &axis.ymin, &axis.xmax, &axis.ymax);
        dr_replot();
      }
   }//while
#endif   
}//dr_zoom

/** @brief Scale all drawn objects to fit the screen.
*/
void dr_fit(void)
{
#ifdef DR_ENABLE
  int i;
  axis.xmin = 1000000;
  axis.xmax = -10000000;
  axis.ymin = 1000000;
  axis.ymax = -10000000;
    
  for(i=0;i<obj_nmb;i++)
  {
    switch(obj[i].l.t){
      case LINE:
          if(obj[i].l.x1 <axis.xmin)
            axis.xmin = obj[i].l.x1;
          if(obj[i].l.x1 >axis.xmax)
            axis.xmax = obj[i].l.x1;
          if(obj[i].l.y1 <axis.ymin)
            axis.ymin = obj[i].l.y1;
          if(obj[i].l.y1 >axis.ymax)
            axis.ymax = obj[i].l.y1;
            
          if(obj[i].l.x2 <axis.xmin)
            axis.xmin = obj[i].l.x2;
          if(obj[i].l.x2 >axis.xmax)
            axis.xmax = obj[i].l.x2;
          if(obj[i].l.y2 <axis.ymin)
            axis.ymin = obj[i].l.y2;
          if(obj[i].l.y2 >axis.ymax)
            axis.ymax = obj[i].l.y2;
         break;
      case MARKER:
          if(obj[i].m.x <axis.xmin)
            axis.xmin = obj[i].m.x;
          if(obj[i].m.x >axis.xmax)
            axis.xmax = obj[i].m.x;
          if(obj[i].m.y <axis.ymin)
            axis.ymin = obj[i].m.y;
          if(obj[i].m.y >axis.ymax)
            axis.ymax = obj[i].m.y;
        break;
      case TEXT:
          if(obj[i].t.x <axis.xmin)
            axis.xmin = obj[i].t.x;
          if(obj[i].t.x >axis.xmax)
            axis.xmax = obj[i].t.x;
          if(obj[i].t.y <axis.ymin)
            axis.ymin = obj[i].t.y;
          if(obj[i].t.y >axis.ymax)
            axis.ymax = obj[i].t.y;
        break;
      case CIRCLE:
          if(obj[i].c.x <axis.xmin)
            axis.xmin = obj[i].c.x;
          if(obj[i].c.x >axis.xmax)
            axis.xmax = obj[i].c.x;
          if(obj[i].c.y <axis.ymin)
            axis.ymin = obj[i].c.y;
          if(obj[i].c.y >axis.ymax)
            axis.ymax = obj[i].c.y;
        break;
    }//switch
  }//for
  double dx,dy;
  dx= axis.xmax-axis.xmin;
  dy= axis.ymax-axis.ymin;
  axis.xmin -= dx/20.0;
  axis.xmax += dx/20.0;
  axis.ymin -= dy/20.0;
  axis.ymax += dy/20.0;
  
  axisEqual(&axis.xmin, &axis.ymin, &axis.xmax, &axis.ymax);
  dr_replot();
#endif
}

/** @brief Save the screen into a PNG image (needs extra packages).
@param filename The name of the file under which the image is saved.
*/
void dr_save(const char *filename)//save screen into a file
{
#ifdef DR_ENABLE
  char s[1000];
//  sprintf(s," xwd -id %i -out %s -nobdrs -silent ",dr_Window,filename);
  sprintf(s," xwd -id %i -nobdrs -silent | xwdtopnm | pnmtopng > %s",(int)dr_Window,filename);
  if(system(s)==-1)
   printf("dr_save: ERROR could not save image %s (check if xwd,xwdtopnm and pnmtopng are present)\n",filename);
#endif   
}

/** @brief Enable or disable equal drawing unit per pixel ratio for both axis.
@param equal 1 - enabled, 0 - disabled
*/
void dr_equal(int equal)
{
#ifdef DR_ENABLE  
  dr_axes_equal = equal;
#endif     
}
