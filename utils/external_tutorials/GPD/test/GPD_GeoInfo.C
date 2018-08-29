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
 * GPD_GeoInfo.C
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

int
main(int argc, char *argv[])
{
    GPD_Detail	 	*gdp;
    GPD_PrimNURBSurf	*surf;
    GPD_Primitive	*prim;
    GPD_Vertex		*vtx;
    int			 i, j, k;
    float		 bbox[3][2], *pos;

    if (argc != 2)
    {
	cerr << "Usage: " << argv[0] << " <infile>" << endl;
	exit(1);
    }

    gdp = new GPD_Detail;

    if (gdp->load(argv[1]) < 0)
    {
	cerr << "Unable to load " << argv[1] << endl;
	exit(1);
    }

    cout << "Geo Dump on " << argv[1] << endl;
    cout << "Points: " << gdp->numpoint() << endl;
    cout << "Primitives: " << gdp->numprim() << endl;
    
    // Dump all the point data...
    // We just calc a bbox to output.
    for (i = 0; i < gdp->numpoint(); i++)
    {
	pos = gdp->point(i)->pos;
	if (!i)
	{
	    for (j = 0; j < 3; j++)
	    {
		bbox[j][0] = bbox[j][1] = pos[j];
	    }
	}
	else
	{
	    for (j = 0; j < 3; j++)
	    {
		if (bbox[j][0] > pos[j])
		    bbox[j][0] = pos[j];
		else if (bbox[j][1] < pos[j])
		    bbox[j][1] = pos[j];
	    }
	}
    }

    cout << "Bounding box:" << endl;
    cout << bbox[0][0] << "\t" << bbox[1][0] << "\t" << bbox[2][0] << endl;
    cout << bbox[0][1] << "\t" << bbox[1][1] << "\t" << bbox[2][1] << endl;

    // Now, dump the info all primitives found..
    for (i = 0; i < gdp->numprim(); i++)
    {
	prim = gdp->prim(i);

	cout << "Primitive number " << i << " type is " 
	     << GPD_Primitive::getPrimitiveName(prim->getPrimitiveId()) << endl;

	// If it is a NURBs, we do extra output...
	if (prim->getPrimitiveId() == GPDPRIMNURBSURF)
	{
	    surf = (GPD_PrimNURBSurf *) prim; 
	    cout << "This NURBs surface is " << surf->numRows() << " by " 
		 << surf->numCols() << endl;
	    if (surf->isWrappedU())
		cout << "It is wrapped in the U direction" << endl;
	    if (surf->isWrappedV())
		cout << "It is wrapped in the V direction" << endl;
	    cout << "It's surface type is " << surf->getSurfaceType() << endl;

	    int		uend, vend;
	    surf->getEndInterp(uend, vend);
	    cout << "End interpolation conditions " << uend << " and " << vend << endl;

	    // Dump the knot vectors:
	    int		 numknot, order;
	    float	*knots;

	    knots = surf->getUBasis(order, numknot);
	    cout << "U Basis order is " << order << endl;
	    for (j = 0; j < numknot; j++)
	    {
		if (j) cout << ", ";
		cout << knots[j];
	    }
	    cout << endl;
	    knots = surf->getVBasis(order, numknot);
	    cout << "V Basis order is " << order << endl;
	    for (j = 0; j < numknot; j++)
	    {
		if (j) cout << ", ";
		cout << knots[j];
	    }
	    cout << endl;
	}
    }
    
    return 0;
}

