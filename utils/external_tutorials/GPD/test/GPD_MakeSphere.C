/*
 * Copyright (c) 2008
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of the GPD library in source and binary forms, with
 * or without modification, are permitted provided that the following
 * conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 */
/*
 * GPD_MakeSphere.C
 *
 *	This simple program creates a NURBS sphere and saves it as a .geo.
 *
 */

#include "GPD_Detail.h"
#include "GPD_PrimNURBSurf.h"
#include "GPD_PrimType.h"
#include "GPD_Vertex.h"
#include "GPD_Point.h"
#include <iostream>

using namespace std;
#include <stdlib.h>
#include <math.h>

void
setSphereCVs(GPD_PrimNURBSurf *surf)
{
    static float	cornerw = cos(float(3.1415*0.25));
    float		w;
    int			c, r, i, j;
    int			degree = 2;
    int			mr = 2;
    int			rows = 5;

    // set cvs for the first span manually (using order 3)
    // building it clockwise, as for a left-handed system:
    // adjust rationality at this point

    w = cornerw;
    surf->getVertex(mr,0)->getPt()->pos[0] = 1.0f;
    surf->getVertex(mr,0)->getPt()->pos[1] = 0.0f;
    surf->getVertex(mr,0)->getPt()->pos[2] = 0.0f;
    surf->getVertex(mr,0)->getPt()->pos[3] = 1.0f;

    surf->getVertex(mr,1)->getPt()->pos[0] = w;
    surf->getVertex(mr,1)->getPt()->pos[1] = w;
    surf->getVertex(mr,1)->getPt()->pos[2] = 0.0f;
    surf->getVertex(mr,1)->getPt()->pos[3] = w;

    surf->getVertex(mr,2)->getPt()->pos[0] = 0.0f;
    surf->getVertex(mr,2)->getPt()->pos[1] = 1.0f;
    surf->getVertex(mr,2)->getPt()->pos[2] = 0.0f;
    surf->getVertex(mr,2)->getPt()->pos[3] = 1.0f;

    // copy this span to the one section of the rows (assuming vorder = 3)
    for (i = 0; i<degree; i++)
    {
	float *p = surf->getVertex(mr, i)->getPt()->pos;
	float *d;

	d = surf->getVertex(mr-1, i)->getPt()->pos;
	
	d[0] = p[0] * w;
	d[1] = p[1] * w;
	d[2] = p[3] * w;
	d[3] = p[3] * w;
	
	d = surf->getVertex(mr-2, i)->getPt()->pos;

	d[0] = 0.0f;
	d[1] = 0.0f;
	d[2] = p[3];
	d[3] = p[3];
    }

    // copy and reflect octant points to other octants
    for (r=mr; r>=0; r--)
    {
	for (i=0; i<degree; i++)
	{
	    float 	*p1 = surf->getVertex(r, i+0*degree)->getPt()->pos;
	    float 	*p2 = surf->getVertex(r, i+1*degree)->getPt()->pos;
	    float 	*p3 = surf->getVertex(r, i+2*degree)->getPt()->pos;
	    float 	*p4 = surf->getVertex(r, i+3*degree)->getPt()->pos;

	    // readjust weight before copying
	    float       ws  = 1.0F / p1[3];

	    p1[0] *= ws;	p1[1] *= ws;	p1[2] *= ws;

	    p2[0] = -p1[1];	p2[1] = p1[0];	p2[2] = p1[2];	p2[3] = p1[3];
	    p3[0] = -p2[1];	p3[1] = p2[0];	p3[2] = p2[2];	p3[3] = p2[3];
	    p4[0] = -p3[1];	p4[1] = p3[0];	p4[2] = p3[2];	p4[3] = p3[3];

	    // reflect it to the other hemisphere as well
	    if (r<mr)
	    {
		int	nr = rows - r - 1;

		float	*p5 = surf->getVertex(nr, i+0*degree)->getPt()->pos;
		float	*p6 = surf->getVertex(nr, i+1*degree)->getPt()->pos;
		float	*p7 = surf->getVertex(nr, i+2*degree)->getPt()->pos;
		float	*p8 = surf->getVertex(nr, i+3*degree)->getPt()->pos;

		p5[0] = p1[0];	p5[1] = p1[1];	p5[2] = -p1[2];	p5[3] = p1[3];
		p6[0] = p2[0];	p6[1] = p2[1];	p6[2] = -p2[2];	p6[3] = p2[3];
		p7[0] = p3[0];	p7[1] = p3[1];	p7[2] = -p3[2];	p7[3] = p3[3];
		p8[0] = p4[0];	p8[1] = p4[1];	p8[2] = -p4[2];	p8[3] = p4[3];
	    }
	}
    }
}

int
main(int argc, char *argv[])
{
    GPD_Detail	 	*gdp;
    GPD_PrimNURBSurf	*surf;
    float		 ubasis[8] = { 0, 0.25, 0.25, 0.5, 0.5, 
				       0.75, 0.75, 1 };
    float		 vbasis[4] = { 0, 0.5, 0.5, 1 };

    if (argc != 2)
    {
	cerr << "Usage: " << argv[0] << " <outfile>" << endl;
	exit(1);
    }

    gdp = new GPD_Detail;
    surf = (GPD_PrimNURBSurf *)gdp->buildPrimitive(GPDPRIMNURBSURF);
    surf->setSize(5, 8);
    surf->setUBasis(3, 8, ubasis);
    surf->setVBasis(3, 4, vbasis);
    surf->wrapU();
    surf->setSurfaceType(GPD_PATCH_QUADS);
    surf->setEndInterp(1, 1);

    setSphereCVs(surf);

    if (gdp->save(argv[1]) < 0)
    {
	cerr << "Unable to save " << argv[1] << endl;
	exit(1);
    }
    return 0;
}

