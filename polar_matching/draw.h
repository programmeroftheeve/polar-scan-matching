/***************************************************************************
                          draw.h  -  simplistic drawing module
                             -------------------
    begin                : Sun Dec 21 2003
    version              : 0.1
    copyright            : (C) 2003 by Albert Diosi and Lindsay Kleeman
    email                : albert.diosi@gmail.com
    change:               -14/03/2005 rewriting to use only X, instead of libplot
                          -25/01/2005 dr_cov_ellipse added
                          -8/11/2004  dr_zoom added
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


#ifndef _DRAW_
#define _DRAW_
extern "C" char* dr_COLORS[];
extern "C" int   dr_COLORS_CNT;

extern "C" void dr_init(int screen_x_max,int screen_y_max,double xmin, double ymin, double xmax, double ymax);
extern "C" void dr_close(void);
extern "C" void dr_line(double x1, double y1, double x2, double y2,const char *color);
extern "C" void dr_marker(double x, double y, int type,const char *color);
extern "C" void dr_text(double x, double y, int hz_al,int vr_al,const char *text, const char* color);
extern "C" void dr_text_pix(int px, int py,const char *text, const char* color);
extern "C" void dr_scale(double xmin, double ymin, double xmax, double ymax);
extern "C" void dr_replot(void);
extern "C" void dr_erase(void);
extern "C" void dr_circle(double x, double y, double r,const char *color);
extern "C" void dr_cov_ellipse(double x, double y, double c11,double c12,double c22,const char *color);
extern "C" void dr_zoom(void);
extern "C" void dr_fit(void);
extern "C" void dr_save(const char *filename);
extern "C" void dr_equal(int equal);
#endif


