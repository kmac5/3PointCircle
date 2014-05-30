/*
** pppcir.h
*/

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


static double v2_dist(v2_pos *a, v2_pos *b);

static short line_intersect(v2_pos *p1, v2_pos *p2, v2_vect *d1, v2_vect *d2, v2_pos *ip);

int ppp_circle(v2_pos *p1, v2_pos *p2, v2_pos *p3, v2_pos *center, double *radius);
