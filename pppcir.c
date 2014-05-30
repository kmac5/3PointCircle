/*
** pppcir.c
**
** Originally from rmrice@eskimo.com on the COMP.GRAPHICS.ALGORITHMS forum
**
** Contents: Routine for 3 point circle with supporting routines
**    to calculate v2 line intersection and v2 distance.
**
** The code is self contained except for a call to the standard library
** function 'sqrt'. 
** 
** If you intend to interface this code with another program note that 
** all position and vector parameters of the three functions (ppp_circle, 
** line_intersect and v2_dist) are passed as pointers to structures. The
** radius parameter is also a pointer; 
**
*/
/*
#include <math.h>

#define false 0
#define true (!false)

typedef struct V2_DIST_VECT
{
 double x;
 double y;
} v2_dist_vect;

typedef v2_dist_vect v2_vect;
typedef v2_dist_vect v2_pos;
*/
#include "pppcir.h"

/*
** Function v2_dist -- Find the distance between 2 points in 2D space
**
** Inputs:
**  a  pointer to point on first line
**  b  pointer to point on second line
**
** Return value: double
**  calculated distance
*/
static double v2_dist(v2_pos *a, v2_pos *b)
{
 double dx, dy;

 dx = (a->x - b->x);
 dy = (a->y - b->y);
 return (sqrt((dx * dx) + (dy * dy)));
}

/*
** Function line_intersect -- Find the intersection of 2 lines in 2D space
**
** Inputs:
**  p1  pointer to point on first line
**  p2  pointer to point on second line
**  d1  pointer to slope vector of first line
**  d2  pointer to slope vector of second line
**  ip  pointer to storage for intersection position values
**
** Return value: int
**  true *ip contents valid -- intersection found
**  false *ip contents undefined -- intersection NOT found
**    should only happen when passed two parallel lines
*/
static short line_intersect(v2_pos *p1, v2_pos *p2,
       v2_vect *d1, v2_vect *d2, v2_pos *ip)
{
 double m1, m2;
 int stat = true;

 if(d1->x != 0.0) m1 = d1->y / d1->x;
 if(d2->x != 0.0) m2 = d2->y / d2->x;

 if((d1->x != 0.0) && (d2->x != 0.0)) {
  double temp_x, m, b1, b2;

  m = m1 - m2;
  if (fabs(m) != 0.0){ 
   b1 = p1->y - m1 * p1->x;
   b2 = p2->y - m2 * p2->x;
   temp_x = (b2 - b1) / m;
   if((m1 == 0.0) || (m2 == 0.0)) {
    ip->x = temp_x;
    ip->y = ((b2 * m1) - (b1 * m2)) / m;
   } else {
    m1 = 1.0 / m1;
    m2 = 1.0 / m2;
    m = (m1 - m2);
    if (m != 0.0) {
     ip->x = temp_x;
     b1 = p1->x -  m1 * p1->y;
     b2 = p2->x -  m2 * p2->y;
     ip->y = (b2 - b1) / m;
    } else stat = false; /* Error: should never happen? */
   }
  } else stat = false; /* Error: parallel non-vertical lines */
 } else if(d1->x != 0.0) {
  ip->x = p2->x;
  m2 = p1->y - (p1->x * m1);
  ip->y =  (m1 * p2->x) + m2;
 } else if(d2->x != 0.0) {
  ip->x = p1->x;
  m1 = p2->y - (p2->x * m2);
  ip->y =  (m2 * p1->x) + m1;
 } else stat = false;   /* Error: parallel vertical lines */ return stat;
}

/*
** Function ppp_circle -- Find circle passing through 3 given points
**
** Inputs:
**  p1  pointer to first given point
**      p2      pointer to second given point
**      p3      pointer to third given point
**  center pointer to storage for circle center position values
**  radius pointer to storage for circle radius value
**
** Return value: int
**  true  *center and *radius values valid -- circle was found
**  false *center and *radius values undefined -- circle was NOT
**    found should only happen when passed 3 points on a
**    straight line
**
** For the ppp_circle function, if all three points are different the  
** results are as expected, but note that . . .
** 
**     if p1 and p2 are the same or p1 and p3 are the same the circle 
** will be horizontally centered on p1 and pass thru all three points, 
**
** and if p2 and p3 are the same then the center is calculated to
** be the midpoint of the line between them and p1.
*/
int ppp_circle(v2_pos *p1, v2_pos *p2, v2_pos *p3,
      v2_pos *center, double *radius)
{
 v2_vect d1, d2;
 v2_pos mp1, mp2;
 int have_center = true;

 /* calculate perpendicular slope vectors */
 d1.y = p2->x - p1->x;
 d2.y = p3->x - p1->x;
 d1.x = p1->y - p2->y;
 d2.x = p1->y - p3->y;

 /* calculate midpoint position values */
 mp1.x = (p1->x + p2->x) * 0.5;
 mp1.y = (p1->y + p2->y) * 0.5;
 mp2.x = (p1->x + p3->x) * 0.5;
 mp2.y = (p1->y + p3->y) * 0.5;

 if ((mp1.x == mp2.x) && (mp1.y == mp2.y)) {
  *center = mp1;
 } else have_center = line_intersect(&mp1, &mp2, &d1, &d2, center);

 if (have_center) *radius = v2_dist(center, p1);
 return (have_center);
}
