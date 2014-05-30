/*
======================================================================
3PointCircle.c

A tool to generate a circle from any three arbitrary points.
Contains some code modified from the LW SDK.

Next version should be interactive. 

Kevin MacPhail	08/19/03
====================================================================== */

#include <lwserver.h>
#include <lwcmdseq.h>
#include <lwmodeler.h>
#include <lwxpanel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <lwmodlib.h>
#include <lwcomlib.h>
#include <com_math.h>
#include <com_vecmatquat.h>
#include "pppcir.h"

/* some matrix shorthand */
#define m00  m[ 0 ][ 0 ]
#define m01  m[ 0 ][ 1 ]
#define m02  m[ 0 ][ 2 ]
#define m03  m[ 0 ][ 3 ]
#define m10  m[ 1 ][ 0 ]
#define m11  m[ 1 ][ 1 ]
#define m12  m[ 1 ][ 2 ]
#define m13  m[ 1 ][ 3 ]
#define m20  m[ 2 ][ 0 ]
#define m21  m[ 2 ][ 1 ]
#define m22  m[ 2 ][ 2 ]
#define m23  m[ 2 ][ 3 ]
#define m30  m[ 3 ][ 0 ]
#define m31  m[ 3 ][ 1 ]
#define m32  m[ 3 ][ 2 ]
#define m33  m[ 3 ][ 3 ]

#define a0 point[ 0 ][ 0 ]
#define a1 point[ 0 ][ 1 ]
#define b0 point[ 1 ][ 0 ]
#define b1 point[ 1 ][ 1 ]
#define c0 point[ 2 ][ 0 ]
#define c1 point[ 2 ][ 1 ]

typedef struct st_PointStack{
   MeshEditOp *edit;
   int pointCount;
   double **pointArray;
} PointStack;

static EDError KMPointEnum( PointStack *pointcircle, const EDPointInfo *pointInfo );
static EDError KMPolyEnum( PointStack *pointcircle, const EDPolygonInfo *polyInfo );
void KM_D4Transform( LWDMatrix4 m, double d1, double d2, double d3 );
int KM_D4Rotate( LWDMatrix4 m, char axis, double theta );
void LWMAT_transpose4( LWDMatrix4 n, LWDMatrix4 m );

/*
======================================================================
LWXPanelFuncs

Create the interface panel.
====================================================================== */
int get_user( LWXPanelFuncs *xpanf, int *sides)
{
   LWXPanelID panel;
   int ok = 0;

   enum { ID_SIDES = 0x8001, };

   LWXPanelControl ctl[] = {
	  { ID_SIDES, "Number of Sides", "integer" },
      { 0 }
   };
   LWXPanelDataDesc cdata[] = {
	  { ID_SIDES, "Number of Sides", "integer" },
      { 0 }
   };
   LWXPanelHint hint[] = {
	   XpLABEL( 0, "3PointCircle v1.1.0" ),
	   XpEND
   };

   panel = xpanf->create( LWXP_FORM, ctl );
   if ( !panel ) return 0;

   xpanf->describe( panel, cdata, NULL, NULL );
   xpanf->hint( panel, 0, hint );
   xpanf->formSet( panel, ID_SIDES, sides );

   ok = xpanf->post( panel );

   if ( ok ) {
       int *i;
	   
	   i = xpanf->formGet( panel, ID_SIDES );
	   *sides = *i;
   }

   xpanf->destroy( panel );
   return ok;
}

/*
======================================================================
Activate()

The plug-in activation function.
====================================================================== */
XCALL_( int )
Activate( long version, GlobalFunc *global, LWModCommand *local,
   void *serverData )
{
	char cmd[ 128 ];
	LWStateQueryFuncs *query;
	LWMessageFuncs *msg;
	LWXPanelFuncs *xpanf;
	ModData *md;
	int ok = 0;
	int nmode;   
	int sides = 32;
	int pointEnum = 0;
	int polyEnum = 0;
	int i, j;
	double dangle = 0.0;
	double radius[3] = {0.0, 0.0, 0.0};
	double center[3] = {0.0, 0.0, 0.0};
	PointStack pinfo;
	PointStack npinfo;
	PointStack pointcircle;
	LWDVector temp;
	LWDVector unitX = { 1.0, 0.0, 0.0 };
	LWDVector unitY = { 0.0, 1.0, 0.0 };
	LWDVector unitZ = { 0.0, 0.0, 1.0 };
	LWDMatrix4 pointTranslate;
	LWDMatrix4 pointTranslateT;
	LWDMatrix4 pointRotateX;
	LWDMatrix4 pointRotateY;
	LWDMatrix4 pointRotateZ;
	LWDMatrix4 pointWork1;
	LWDMatrix4 pointWork2;
	LWDMatrix4 pointXYZT;
	LWPntID *cpntid;
	v2_pos v2_points[3];
	v2_pos v2_center;
	int lastLayer = 0;
	const char *layers;
	char *fgLayers, *bgLayers, *allLayers;
	char setLayer[20];
	char *token, *temptoken;
	char seps[] = " ";


	//////////////////////////////////
	// Initialize vectors and matrices
	//////////////////////////////////
	pinfo.pointCount = 0;
	npinfo.pointCount = 0;
	pointcircle.pointCount = 0;
	LWMAT_didentity4( pointTranslate );
	LWMAT_didentity4( pointRotateX );
	LWMAT_didentity4( pointRotateY );
	LWMAT_didentity4( pointRotateZ );
			
	/////////////////////
	// Initialize Globals
	/////////////////////
	if ( version != LWMODCOMMAND_VERSION ) return AFUNC_BADVERSION;

	query = global( LWSTATEQUERYFUNCS_GLOBAL, GFUSE_TRANSIENT );
	if ( !query ) return AFUNC_BADGLOBAL;

	xpanf = global( LWXPANELFUNCS_GLOBAL, GFUSE_TRANSIENT );
	if ( !xpanf ) return AFUNC_BADGLOBAL;

	msg = global (LWMESSAGEFUNCS_GLOBAL, GFUSE_TRANSIENT);
	if ( !msg ) return AFUNC_BADGLOBAL;
	
	nmode = query->mode(LWM_MODE_SELECTION);

	//////////////////////////////////////////////
	// Fail if user is not in point selection mode
	//////////////////////////////////////////////
	if ( md = csInit( global, local )) {

		csMeshBegin( 0, 0, OPSEL_USER );

		if (nmode == 1) {

			pointEnum = 3;
			polyEnum = mePolyCount( OPLYR_SELECT, EDCOUNT_SELECT );

			//////////////////////////////////////////
			// Fail if more than 1 polygon is selected
			//////////////////////////////////////////
			if ( polyEnum != 1 ) {
				msg->error("Please select only 1 polygon.", NULL);
				csMeshDone( EDERR_NONE, 0 );
				return AFUNC_OK;
			}
		}
		else if (nmode == 0) {

			pointEnum = mePointCount( OPLYR_SELECT, EDCOUNT_SELECT );

			//////////////////////////////////////////////////////
			// Fail if there are not exactly three points selected
			//////////////////////////////////////////////////////
			if ( pointEnum != 3 ) {
				msg->error("Please select 3 points.", NULL);
				csMeshDone( EDERR_NONE, 0 );
				return AFUNC_OK;
			}
		}
		else {
			msg->error("Please use a point or polygon selection and try again.", NULL);
			csMeshDone( EDERR_NONE, 0 );
			return AFUNC_OK;
		}

		//////////////////////////////
		// Initialize the point arrays
		//////////////////////////////
		pinfo.pointArray = (double **)malloc(pointEnum * sizeof(double *));
		npinfo.pointArray = (double **)malloc(pointEnum * sizeof(double *));

		for (i=0; i<pointEnum; i++) {
			pinfo.pointArray[i] = (double *)malloc( 3 * sizeof(double));
			npinfo.pointArray[i] = (double *)malloc( 3 * sizeof(double));
			for (j=0; j<3; j++) {
				pinfo.pointArray[i][j] = 0.0;
				npinfo.pointArray[i][j] = 0.0;
			}
		}

		if (nmode == 1) {
			if ( ( mePolyScan((EDPolyScanFunc *)KMPolyEnum, &pinfo, OPLYR_SELECT ) ) ) {
				msg->error("Please select a polygon with 3 vertices.", NULL);
				csMeshDone( EDERR_NONE, 0 );
				return AFUNC_OK;
			}
		}

		else {
			mePointScan((EDPointScanFunc *)KMPointEnum, &pinfo, OPLYR_SELECT);
		}

		csMeshDone( EDERR_NONE, 0 );
	}

	// Get input from XPanel
	ok = get_user( xpanf, &sides );
	if (!ok) {
		return AFUNC_OK;
	}

	//////////////////////
	// Deselect all points
	//////////////////////
	sprintf( cmd, "SEL_POINT SET VOLUME <%g %g %g> <%g %g %g>", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
	local->evaluate( local->data, cmd );

	/////////////////////////////////////////////
	// Find the FG, BG, and ALL layers
	// Keep FG and BG so we can be nice and reset
	// the users selection after processing
	/////////////////////////////////////////////
	layers = query->layerList( OPLYR_FG, NULL );
	fgLayers = (char *)malloc(strlen(layers)+1);
	strcpy(fgLayers,layers);
	layers = query->layerList( OPLYR_BG, NULL );
	bgLayers = (char *)malloc(strlen(layers)+1);
	strcpy(bgLayers,layers);
	layers = query->layerList( OPLYR_NONEMPTY, NULL );
	allLayers = (char *)malloc(strlen(layers)+1);
	strcpy(allLayers,layers);

	// Find the last empty FG layer
    token = strtok( allLayers, seps );
	while( token != NULL )
	{
		// Copy token value to temptoken
		temptoken = (char *)malloc(strlen(token)+1);
		strcpy(temptoken,token);

		// Get next token
		token = strtok( NULL, seps );

		// Set lastLayer to the highest layer found
		if (atoi(temptoken) > lastLayer) {
			lastLayer = atoi(temptoken);
		}
	}

	//////////////////////////////////
	// Generate the translation matrix
	//////////////////////////////////
	KM_D4Transform( pointTranslate, -pinfo.pointArray[0][0], -pinfo.pointArray[0][1], -pinfo.pointArray[0][2] );

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// Translate the positions of the three selected points relative to the first point to the origin
	/////////////////////////////////////////////////////////////////////////////////////////////////
	for (i=0; i<3; i++) {
        LWMAT_dtransformp ( pinfo.pointArray[i], pointTranslate, npinfo.pointArray[i] );
	}

	//////////////////////////
	// Rotate about the Z-Axis
	//////////////////////////
	VCPY ( temp, npinfo.pointArray[1] );
	temp[2] = 0.0; // Use only the X and Y components
	dangle = LWVEC_dangle( temp, unitX );

	if (temp[1] < 0) {dangle*=(-1);}

	if ( KM_D4Rotate( pointRotateZ, 'Z' , dangle ) ) {
		return AFUNC_BADLOCAL;
	}
	
	for (i=1; i<3; i++) {
		VCPY ( temp, npinfo.pointArray[i] );
        LWMAT_dtransformp ( temp, pointRotateZ, npinfo.pointArray[i] );
	}

	//////////////////////////
	// Rotate about the Y-Axis
	//////////////////////////
	VCPY ( temp, npinfo.pointArray[1] );
	temp[1] = 0.0; // Use only the X and Z components
	dangle = LWVEC_dangle( temp, unitX );

	if (temp[2] < 0) {dangle*=(-1);}

	if ( KM_D4Rotate( pointRotateY, 'Y', dangle ) ) {
		return AFUNC_BADLOCAL;
	}

	for (i=1; i<3; i++) {
		VCPY ( temp, npinfo.pointArray[i] );
        LWMAT_dtransformp ( temp, pointRotateY, npinfo.pointArray[i] );
	}

	//////////////////////////
	// Rotate about the X-Axis
	//////////////////////////
	VCPY ( temp, npinfo.pointArray[2] );
	temp[0] = 0.0; // Use only the Y and Z components
	dangle = LWVEC_dangle( temp, unitY );

	if (temp[2] < 0) {dangle*=(-1);}

	if ( KM_D4Rotate( pointRotateX, 'X', dangle ) ) {
		return AFUNC_BADLOCAL;
	}

	VCPY ( temp, npinfo.pointArray[2] );
	LWMAT_dtransformp ( temp, pointRotateX, npinfo.pointArray[2] );
    
	///////////////////////////////////////
	//Calculate the Center Point and Radius
	///////////////////////////////////////
	for (i=0; i<3; i++) {
		v2_points[i].x = npinfo.pointArray[i][0];
		v2_points[i].y = npinfo.pointArray[i][1];
	}

	v2_center.x = center[0];
	v2_center.y = center[1];

	if ( !ppp_circle(&v2_points[0], &v2_points[1], &v2_points[2], &v2_center, radius) ) {
        msg->error("Cannot calculate center point.", "Points may be co-linear.");
		return AFUNC_OK;
	}

	center[0] = v2_center.x;
	center[1] = v2_center.y;
	radius[2] = radius[1] = radius[0];

	///////////////////////////////////
	// setLayer is the next empty layer
	///////////////////////////////////
	lastLayer++;
	_itoa(lastLayer, setLayer, 10);

	sprintf( cmd, "SETLAYER \"%s\"", setLayer );
	local->evaluate( local->data, cmd );

	/////////////////////////////////
	//Draw the circle
	/////////////////////////////////
	csMakeDisc( radius, 0.0, 0.0, "Z", sides, 1, center );
    
	////////////////////////////////////
	// Select all points of new geometry
	////////////////////////////////////
	sprintf( cmd, "SEL_POINT SET VOLUME <%g %g %g> <%g %g %g>", 1000.0, 1000.0, 1000.0, -1000.0, -1000.0, -1000.0 );
	local->evaluate( local->data, cmd );

	/////////////////////////////////////////////////////
	// Place the coordinates of all new points in an array
	//////////////////////////////////////////////////////
	if ( md = csInit( global, local )) {

		csMeshBegin( 0, 0, OPSEL_USER );

		pointEnum = mePointCount( OPLYR_SELECT, EDCOUNT_SELECT );

		pointcircle.pointArray = (double **)malloc(pointEnum * sizeof(double *));

		for (i=0; i<pointEnum; i++) {
			pointcircle.pointArray[i] = (double *)malloc( 3 * sizeof(double));
			for (j=0; j<3; j++) {
				pointcircle.pointArray[i][j] = 0.0;
			}
		}
		
		mePointScan((EDPointScanFunc *)KMPointEnum, &pointcircle, OPLYR_SELECT);

		csMeshDone( EDERR_NONE, 0 );
	}

	////////////////////////////////////////////////
	// Generate the Transpose matrix pointRotateXYZT
	////////////////////////////////////////////////

	LWMAT_dcopym4( pointTranslateT, pointTranslate );
	pointTranslateT[3][0] = (pointTranslate[3][0] * (-1.0));
	pointTranslateT[3][1] = (pointTranslate[3][1] * (-1.0));
	pointTranslateT[3][2] = (pointTranslate[3][2] * (-1.0));

	LWMAT_transpose4( pointWork1, pointRotateX );
	LWMAT_dcopym4( pointRotateX, pointWork1 );

	LWMAT_transpose4( pointWork1, pointRotateY );
	LWMAT_dcopym4( pointRotateY, pointWork1 );

	LWMAT_transpose4( pointWork1, pointRotateZ );
	LWMAT_dcopym4( pointRotateZ, pointWork1 );

	LWMAT_dmatmul4  ( pointRotateX, pointRotateY, pointWork1 );
	LWMAT_dmatmul4  ( pointWork1, pointRotateZ, pointWork2 );
	LWMAT_dmatmul4  ( pointWork2, pointTranslateT, pointXYZT );

	for (i=0; i<pointEnum; i++) {
		VCPY ( temp, pointcircle.pointArray[i] );
        LWMAT_dtransformp ( temp, pointXYZT, pointcircle.pointArray[i] );
	}

	cpntid = (LWPntID *)malloc(pointEnum * sizeof(LWPntID));

	///////////////////////////////
	// Delete the selected geometry
	///////////////////////////////
	csDelete();
	
	/////////////////////////////////
	//Draw the new polygon
	/////////////////////////////////
	csMeshBegin( 0, 0, OPSEL_USER );
	for (i=0; i<pointEnum; i++) {
        cpntid[i] = meAddPoint( pointcircle.pointArray[i] );
	}
	meAddFace ( NULL, pointEnum, cpntid);
	csMeshDone( EDERR_NONE, 0 );
	
	/////////////////////////////////////////////////////////
	// Return layers to original selections plus newest layer
	/////////////////////////////////////////////////////////
	sprintf( cmd, "SETALAYER \"%s %s\"", fgLayers, setLayer);	
	local->evaluate( local->data, cmd );
	sprintf( cmd, "SETBLAYER \"%s\"", bgLayers);	
	local->evaluate( local->data, cmd );

	//////////////////////////////////////////////////////
	// If initial selection mode was polygons return to it
	//////////////////////////////////////////////////////
	if (nmode) {
		sprintf( cmd, "SEL_POLYGON CLEAR VOLINCL <%g %g %g> <%g %g %g>", 1000.0, 1000.0, 1000.0, -1000.0, -1000.0, -1000.0 );
		local->evaluate( local->data, cmd );
	}

	//////////////
	// free memory
	//////////////
	free(fgLayers);
	fgLayers = NULL;
	free(bgLayers);
	bgLayers = NULL;
	free(allLayers);
	allLayers = NULL;
	free(pointcircle.pointArray);
	pointcircle.pointArray = NULL;

	//////
	//Done
	//////
	return AFUNC_OK;
}


/*
======================================================================
KMPointEnum()

The callback passed to the MeshEditOp polyScan() function.  For each
point selected, add its position to the point array.
======================================================================*/

XCALL_( static EDError )
KMPointEnum( PointStack *pointcircle, const EDPointInfo *pointInfo ) {

	int i;

	if ( ( pointInfo->flags & EDDF_SELECT ) != EDDF_SELECT ) return EDERR_NONE;

	for (i=0; i<3; i++){
		pointcircle->pointArray[ pointcircle->pointCount ][ i ] = pointInfo->position[ i ];
	}
    
	pointcircle->pointCount++;

	return EDERR_NONE;
}

/*
======================================================================
KMPolyEnum()

The callback passed to the MeshEditOp polyScan() function.  For each
point selected, add its position to the point array.
======================================================================*/

XCALL_( static EDError )
KMPolyEnum( PointStack *pointcircle, const EDPolygonInfo *polyInfo ) {

	int i,j;
	EDPointInfo *pointInfo;

	if ( ( polyInfo->flags & EDDF_SELECT ) != EDDF_SELECT ) return EDERR_NONE;

	if ( polyInfo->numPnts != 3) return EDERR_BADARGS;

	pointcircle->pointCount = 3;

	for (i=0; i<3; i++){
			for (j=0; j<3; j++){
				pointInfo = mePointInfo( polyInfo->points[i] );
				pointcircle->pointArray[ i ][ j ] = pointInfo->position[ j ];
			}
	}
    
	return EDERR_NONE;
}

/*
======================================================================
KM_D4Transform()

Helper function to generate translation matrices.

Translate

| 1  0  0  0 |
| 0  1  0  0 |
| 0  0  1  0 |
| dx dy dz 1 |
====================================================================== */
void KM_D4Transform( LWDMatrix4 m, double a4, double b4, double c4 )
{
   m30 = a4;
   m31 = b4;
   m32 = c4;
}

/*
======================================================================
KM_D4Rotate()

Helper function to generate rotation matrices.

LWDMatrix4 m: pointer to a 4x4 matrix
char axis: the X, Y, or Z axis of rotation
double theta: the angle of rotation in degrees

Rotate about X axis

| 0 0   0    0 |
| 0 cos -sin 0 |
| 0 sin cos  0 |
| 0 0   0    0 |


Rotate about Y axis

| cos 0 -sin 0 |
| 0   0 0    0 |
| sin 0 cos  0 |
| 0   0 0    0 |


Rotate about Z axis

| cos -sin 0 0 |
| sin cos  0 0 |
| 0   0    0 0 |
| 0   0    0 0 |

====================================================================== */

int KM_D4Rotate( LWDMatrix4 m, char axis, double theta) {

	int i;
	double t1, t2, t3, t4;
	double t[4] = {0.0, 0.0, 0.0, 0.0};

	t1 = t2 = t3 = t4 = 0.0;
	
	switch (axis) {

		case 'x':
		case 'X':
			{			
			t[0] = cos(theta) ; t[1] = -sin(theta);
			t[2] = sin(theta); t[3] = cos(theta);
			for (i=0; i<4; i++) {
				if (( t[i] < 0.00000001 )&&( t[i] > -0.00000001 )) {t[i] = 0.0;}
			}
			m11 = t[0]; m12 = t[1]; m21 = t[2]; m22 = t[3];
			break;
			}
		case 'y':
		case 'Y':
			{			
			t[0] = cos(theta) ; t[1] = -sin(theta);
			t[2] = sin(theta); t[3] = cos(theta);
			for (i=0; i<4; i++) {
				if (( t[i] < 0.00000001 )&&( t[i] > -0.00000001 )) {t[i] = 0.0;}
			}
			m00 = t[0]; m02 = t[1]; m20 = t[2]; m22 = t[3];
			break;
			}
		case 'z':
		case 'Z':
			{			
			t[0] = cos(theta) ; t[1] = -sin(theta);
			t[2] = sin(theta); t[3] = cos(theta);
			for (i=0; i<4; i++) {
				if (( t[i] < 0.00000001 )&&( t[i] > -0.00000001 )) {t[i] = 0.0;}
			}
			m00 = t[0]; m01 = t[1]; m10 = t[2]; m11 = t[3];
			break;
			}
		default:
			return 1; // Error!
	}
	return 0;
}

/*
======================================================================
LWMAT_transpose4()

Helper function to generate transpose matrices.
====================================================================== */
void LWMAT_transpose4( LWDMatrix4 n, LWDMatrix4 m ) {
	LWMAT_dinitm4( n, m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33 );
}

/*
======================================================================
Server record declarations
====================================================================== */

ServerRecord ServerDesc[] = {
   { LWMODCOMMAND_CLASS, "3PointCircle", Activate },
   { NULL }
};
