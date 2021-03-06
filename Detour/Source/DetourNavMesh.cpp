//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <float.h>
#include <string.h>
#include <stdio.h>
#include "DetourNavMesh.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourNavMeshBuilder.h"
#include <new>


inline bool overlapSlabs(const float* amin, const float* amax,
						 const float* bmin, const float* bmax,
						 const float px, const float py)
{
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	const float minx = dtMax(amin[0]+px,bmin[0]+px);
	const float maxx = dtMin(amax[0]-px,bmax[0]-px);
	if (minx > maxx)
		return false;
	
	// Check vertical overlap.
	const float ad = (amax[1]-amin[1]) / (amax[0]-amin[0]);
	const float ak = amin[1] - ad*amin[0];
	const float bd = (bmax[1]-bmin[1]) / (bmax[0]-bmin[0]);
	const float bk = bmin[1] - bd*bmin[0];
	const float aminy = ad*minx + ak;
	const float amaxy = ad*maxx + ak;
	const float bminy = bd*minx + bk;
	const float bmaxy = bd*maxx + bk;
	const float dmin = bminy - aminy;
	const float dmax = bmaxy - amaxy;
		
	// Crossing segments always overlap.
	if (dmin*dmax < 0)
		return true;
		
	// Check for overlap at endpoints.
	const float thr = dtSqr(py*2);
	if (dmin*dmin <= thr || dmax*dmax <= thr)
		return true;
		
	return false;
}

static float getSlabCoord(const float* va, const int side)
{
	if (side == 0 || side == 4)
		return va[0];
	else if (side == 2 || side == 6)
		return va[2];
	return 0;
}

static void calcSlabEndPoints(const float* va, const float* vb, float* bmin, float* bmax, const int side)
{
	if (side == 0 || side == 4)
	{
		if (va[2] < vb[2])
		{
			bmin[0] = va[2];
			bmin[1] = va[1];
			bmax[0] = vb[2];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[2];
			bmin[1] = vb[1];
			bmax[0] = va[2];
			bmax[1] = va[1];
		}
	}
	else if (side == 2 || side == 6)
	{
		if (va[0] < vb[0])
		{
			bmin[0] = va[0];
			bmin[1] = va[1];
			bmax[0] = vb[0];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[0];
			bmin[1] = vb[1];
			bmax[0] = va[0];
			bmax[1] = va[1];
		}
	}
}

inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}

inline unsigned int allocLink(dtMeshTile* tile)
{
	if (tile->linksFreeList == DT_NULL_LINK)
		return DT_NULL_LINK;
	unsigned int link = tile->linksFreeList;
	tile->linksFreeList = tile->links[link].next;
	return link;
}

inline void freeLink(dtMeshTile* tile, unsigned int link)
{
	tile->links[link].next = tile->linksFreeList;
	tile->linksFreeList = link;
}


dtNavMesh* dtAllocNavMesh()
{
	void* mem = dtAlloc(sizeof(dtNavMesh), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtNavMesh;
}

/// @par
///
/// This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
/// flag set.
void dtFreeNavMesh(dtNavMesh* navmesh)
{
	if (!navmesh) return;
	navmesh->~dtNavMesh();
	dtFree(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////

/**
@class dtNavMesh

The navigation mesh consists of one or more tiles defining three primary types of structural data:

A polygon mesh which defines most of the navigation graph. (See rcPolyMesh for its structure.)
A detail mesh used for determining surface height on the polygon mesh. (See rcPolyMeshDetail for its structure.)
Off-mesh connections, which define custom point-to-point edges within the navigation graph.

The general build process is as follows:

-# Create rcPolyMesh and rcPolyMeshDetail data using the Recast build pipeline.
-# Optionally, create off-mesh connection data.
-# Combine the source data into a dtNavMeshCreateParams structure.
-# Create a tile data array using dtCreateNavMeshData().
-# Allocate at dtNavMesh object and initialize it. (For single tile navigation meshes,
   the tile data is loaded during this step.)
-# For multi-tile navigation meshes, load the tile data using dtNavMesh::addTile().

Notes:

- This class is usually used in conjunction with the dtNavMeshQuery class for pathfinding.
- Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized 
  to have only a single tile.
- This class does not implement any asynchronous methods. So the ::dtStatus result of all methods will 
  always contain either a success or failure flag.

@see dtNavMeshQuery, dtCreateNavMeshData, dtNavMeshCreateParams, #dtAllocNavMesh, #dtFreeNavMesh
*/

dtNavMesh::dtNavMesh() :
	m_tileWidth(0),
	m_tileHeight(0),
	m_maxTiles(0),
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFree(0),
	m_tiles(0)
{
#ifndef DT_POLYREF64
	m_saltBits = 0;
	m_tileBits = 0;
	m_polyBits = 0;
#endif
	memset(&m_params, 0, sizeof(dtNavMeshParams));
	m_orig[0] = 0;
	m_orig[1] = 0;
	m_orig[2] = 0;
}

dtNavMesh::~dtNavMesh()
{
	for (int i = 0; i < m_maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_TILE_FREE_DATA)
		{
			dtFree(m_tiles[i].data);
			m_tiles[i].data = 0;
			m_tiles[i].dataSize = 0;
		}
	}
	dtFree(m_posLookup);
	dtFree(m_tiles);
}
		
dtStatus dtNavMesh::init(const dtNavMeshParams* params)
{
	memcpy(&m_params, params, sizeof(dtNavMeshParams));
	dtVcopy(m_orig, params->orig);
	m_tileWidth = params->tileWidth;
	m_tileHeight = params->tileHeight;
	
	// Init tiles
	m_maxTiles = params->maxTiles;
	m_tileLutSize = dtNextPow2(params->maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	m_tileLutMask = m_tileLutSize-1;
	
	m_tiles = (dtMeshTile*)dtAlloc(sizeof(dtMeshTile)*m_maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	m_posLookup = (dtMeshTile**)dtAlloc(sizeof(dtMeshTile*)*m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(dtMeshTile)*m_maxTiles);
	memset(m_posLookup, 0, sizeof(dtMeshTile*)*m_tileLutSize);
	m_nextFree = 0;
	for (int i = m_maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFree;
		m_nextFree = &m_tiles[i];
	}
	
	// Init ID generator values.
#ifndef DT_POLYREF64
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)params->maxTiles));
	m_polyBits = dtIlog2(dtNextPow2((unsigned int)params->maxPolys));
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits - m_polyBits);

	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
#endif
	
	return DT_SUCCESS;
}

dtStatus dtNavMesh::init(unsigned char* data, const int dataSize, const int flags)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

	dtNavMeshParams params;
	dtVcopy(params.orig, header->bmin);
	params.tileWidth = header->bmax[0] - header->bmin[0];
	params.tileHeight = header->bmax[2] - header->bmin[2];
	params.maxTiles = 1;
	params.maxPolys = header->polyCount;
	
	dtStatus status = init(&params);
	if (dtStatusFailed(status))
		return status;

	return addTile(data, dataSize, flags, 0, 0);
}

/// @par
///
/// @note The parameters are created automatically when the single tile
/// initialization is performed.
const dtNavMeshParams* dtNavMesh::getParams() const
{
	return &m_params;
}

//////////////////////////////////////////////////////////////////////////////////////////
int dtNavMesh::findConnectingPolys(const float* va, const float* vb,
								   const dtMeshTile* tile, int side,
								   dtPolyRef* con, float* conarea, int maxcon) const
{
	if (!tile) return 0;
	
	float amin[2], amax[2];
	calcSlabEndPoints(va, vb, amin, amax, side);
	const float apos = getSlabCoord(va, side);

	// Remove links pointing to 'side' and compact the links array. 
	float bmin[2], bmax[2];
	unsigned short m = DT_EXT_LINK | (unsigned short)side;
	int n = 0;
	
	dtPolyRef base = getPolyRefBase(tile);
	
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			
			const float* vc = &tile->verts[poly->verts[j]*3];
			const float* vd = &tile->verts[poly->verts[(j+1) % nv]*3];
			const float bpos = getSlabCoord(vc, side);
			
			// Segments are not close enough.
			if (dtAbs(apos-bpos) > 0.01f)
				continue;
			
			// Check if the segments touch.
			calcSlabEndPoints(vc,vd, bmin,bmax, side);
			
			if (!overlapSlabs(amin,amax, bmin,bmax, 0.01f, tile->header->walkableClimb)) continue;
			
			// Add return value.
			if (n < maxcon)
			{
				conarea[n*2+0] = dtMax(amin[0], bmin[0]);
				conarea[n*2+1] = dtMin(amax[0], bmax[0]);
				con[n] = base | (dtPolyRef)i;
				n++;
			}
			break;
		}
	}
	return n;
}

void dtNavMesh::unconnectLinks(dtMeshTile* tile, dtMeshTile* target)
{
	if (!tile || !target) return;

	const unsigned int targetNum = decodePolyIdTile(getTileRef(target));

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		unsigned int j = poly->firstLink;
		unsigned int pj = DT_NULL_LINK;
		while (j != DT_NULL_LINK)
		{
			if (decodePolyIdTile(tile->links[j].ref) == targetNum)
			{
				// Remove link.
				unsigned int nj = tile->links[j].next;
				if (pj == DT_NULL_LINK)
					poly->firstLink = nj;
				else
					tile->links[pj].next = nj;
				freeLink(tile, j);
				j = nj;
			}
			else
			{
				// Advance
				pj = j;
				j = tile->links[j].next;
			}
		}
	}
}

void dtNavMesh::connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;
	
	// Connect border links.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];

		// Create new links.
//		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip non-portal edges.
			if ((poly->neis[j] & DT_EXT_LINK) == 0)
				continue;
			
			const int dir = (int)(poly->neis[j] & 0xff);
			if (side != -1 && dir != side)
				continue;
			
			// Create new links
			const float* va = &tile->verts[poly->verts[j]*3];
			const float* vb = &tile->verts[poly->verts[(j+1) % nv]*3];
			dtPolyRef nei[4];
			float neia[4*2];
			int nnei = findConnectingPolys(va,vb, target, dtOppositeTile(dir), nei,neia,4);
			for (int k = 0; k < nnei; ++k)
			{
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &tile->links[idx];
					link->ref = nei[k];
					link->edge = (unsigned char)j;
					link->side = (unsigned char)dir;
					
					link->next = poly->firstLink;
					poly->firstLink = idx;

					// Compress portal limits to a byte value.
					if (dir == 0 || dir == 4)
					{
						float tmin = (neia[k*2+0]-va[2]) / (vb[2]-va[2]);
						float tmax = (neia[k*2+1]-va[2]) / (vb[2]-va[2]);
						if (tmin > tmax)
							dtSwap(tmin,tmax);
						link->bmin = (unsigned char)(dtClamp(tmin, 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(dtClamp(tmax, 0.0f, 1.0f)*255.0f);
					}
					else if (dir == 2 || dir == 6)
					{
						float tmin = (neia[k*2+0]-va[0]) / (vb[0]-va[0]);
						float tmax = (neia[k*2+1]-va[0]) / (vb[0]-va[0]);
						if (tmin > tmax)
							dtSwap(tmin,tmax);
						link->bmin = (unsigned char)(dtClamp(tmin, 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(dtClamp(tmax, 0.0f, 1.0f)*255.0f);
					}
				}
			}
		}
	}
}

void dtNavMesh::connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side, int beginIndex)
{
	if (!tile) return;
	
	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	const unsigned char oppositeSide = (side == -1) ? 0xff : (unsigned char)dtOppositeTile(side);
	
	for (int i = beginIndex; i < target->header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* targetCon = &target->offMeshCons[i];
		if (targetCon->side != oppositeSide)
			continue;

		dtPoly* targetPoly = &target->polys[targetCon->poly];
		// Skip off-mesh connections which start location could not be connected at all.
		if (targetPoly->firstLink == DT_NULL_LINK)
			continue;
		
		const float halfExtents[3] = { targetCon->rad, target->header->walkableClimb, targetCon->rad };
		
		// Find polygon to connect to.
		const float* p = &targetCon->pos[3];
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
		if (!ref)
			continue;
		// findNearestPoly may return too optimistic results, further check to make sure. 
		if (dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(targetCon->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &target->verts[targetPoly->verts[1]*3];
		dtVcopy(v, nearestPt);
				
		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(target);
		if (idx != DT_NULL_LINK)
		{
			dtLink* link = &target->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)1;
			link->side = oppositeSide;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = targetPoly->firstLink;
			targetPoly->firstLink = idx;
		}
		
		// Link target poly to off-mesh connection.
		if (targetCon->flags & DT_OFFMESH_CON_BIDIR)
		{
			unsigned int tidx = allocLink(tile);
			if (tidx != DT_NULL_LINK)
			{
				const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
				dtPoly* landPoly = &tile->polys[landPolyIdx];
				dtLink* link = &tile->links[tidx];
				link->ref = getPolyRefBase(target) | (dtPolyRef)(targetCon->poly);
				link->edge = 0xff;
				link->side = (unsigned char)(side == -1 ? 0xff : side);
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = landPoly->firstLink;
				landPoly->firstLink = tidx;
			}
		}
	}

}

void dtNavMesh::connectIntLinks(dtMeshTile* tile)
{
	if (!tile) return;

	dtPolyRef base = getPolyRefBase(tile);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		poly->firstLink = DT_NULL_LINK;

		if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;
			
		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for (int j = poly->vertCount-1; j >= 0; --j)
		{
			// Skip hard and non-internal edges.
			if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK)) continue;

			unsigned int idx = allocLink(tile);
			if (idx != DT_NULL_LINK)
			{
				dtLink* link = &tile->links[idx];
				link->ref = base | (dtPolyRef)(poly->neis[j]-1);
				link->edge = (unsigned char)j;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = poly->firstLink;
				poly->firstLink = idx;
			}
		}			
	}
}

void dtNavMesh::baseOffMeshLinks(dtMeshTile* tile, int beginIdex)
{
	if (!tile) return;
	
	dtPolyRef base = getPolyRefBase(tile);
	
	// Base off-mesh connection start points.
	for (int i = beginIdex; i < tile->header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &tile->offMeshCons[i];
		dtPoly* poly = &tile->polys[con->poly];
	
		const float halfExtents[3] = { con->rad, tile->header->walkableClimb, con->rad };
		
		// Find polygon to connect to.
		const float* p = &con->pos[0]; // First vertex
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
		if (!ref) continue;
		// findNearestPoly may return too optimistic results, further check to make sure. 
		if (dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(con->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &tile->verts[poly->verts[0]*3];
		dtVcopy(v, nearestPt);

		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(tile);
		if (idx != DT_NULL_LINK)
		{
			dtLink* link = &tile->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)0;
			link->side = 0xff;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = poly->firstLink;
			poly->firstLink = idx;
		}

		// Start end-point is always connect back to off-mesh connection. 
		unsigned int tidx = allocLink(tile);
		if (tidx != DT_NULL_LINK)
		{
			const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
			dtPoly* landPoly = &tile->polys[landPolyIdx];
			dtLink* link = &tile->links[tidx];
			link->ref = base | (dtPolyRef)(con->poly);
			link->edge = 0xff;
			link->side = 0xff;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = landPoly->firstLink;
			landPoly->firstLink = tidx;
		}
	}
}

void dtNavMesh::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const
{
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	getTileAndPolyByRefUnsafe(ref, &tile, &poly);
	
	// Off-mesh connections don't have detail polygons.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0]*3];
		const float* v1 = &tile->verts[poly->verts[1]*3];
		const float d0 = dtVdist(pos, v0);
		const float d1 = dtVdist(pos, v1);
		const float u = d0 / (d0+d1);
		dtVlerp(closest, v0, v1, u);
		if (posOverPoly)
			*posOverPoly = false;
		return;
	}
	
	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];
	
	// Clamp point to be inside the polygon.
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	const int nv = poly->vertCount;
	for (int i = 0; i < nv; ++i)
		dtVcopy(&verts[i*3], &tile->verts[poly->verts[i]*3]);
	
	dtVcopy(closest, pos);
	if (!dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget))
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		float dmin = edged[0];
		int imin = 0;
		for (int i = 1; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin*3];
		const float* vb = &verts[((imin+1)%nv)*3];
		dtVlerp(closest, va, vb, edget[imin]);
		
		if (posOverPoly)
			*posOverPoly = false;
	}
	else
	{
		if (posOverPoly)
			*posOverPoly = true;
	}
	
	// Find height at the location.
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &tile->verts[poly->verts[t[k]]*3];
			else
				v[k] = &tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
		}
		float h;
		if (dtClosestHeightPointTriangle(closest, v[0], v[1], v[2], h))
		{
			closest[1] = h;
			break;
		}
	}
}

dtPolyRef dtNavMesh::findNearestPolyInTile(const dtMeshTile* tile,
										   const float* center, const float* halfExtents,
										   float* nearestPt) const
{
	float bmin[3], bmax[3];
	dtVsub(bmin, center, halfExtents);
	dtVadd(bmax, center, halfExtents);
	
	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = queryPolygonsInTile(tile, bmin, bmax, polys, 128);
	
	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polys[i];
		float closestPtPoly[3];
		float diff[3];
		bool posOverPoly = false;
		float d;
		closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff, center, closestPtPoly);
		if (posOverPoly)
		{
			d = dtAbs(diff[1]) - tile->header->walkableClimb;
			d = d > 0 ? d*d : 0;			
		}
		else
		{
			d = dtVlenSqr(diff);
		}
		
		if (d < nearestDistanceSqr)
		{
			dtVcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

int dtNavMesh::queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
								   dtPolyRef* polys, const int maxPolys) const
{
	if (tile->bvTree)
	{
		const dtBVNode* node = &tile->bvTree[0];
		const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const float* tbmin = tile->header->bmin;
		const float* tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;
		
		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// dtClamp query box to world box.
		float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;
		
		// Traverse tree
		dtPolyRef base = getPolyRefBase(tile);
		int n = 0;
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;
			
			if (isLeafNode && overlap)
			{
				if (n < maxPolys)
					polys[n++] = base | (dtPolyRef)node->i;
			}
			
			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
		
		return n;
	}
	else
	{
		float bmin[3], bmax[3];
		int n = 0;
		dtPolyRef base = getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			dtPoly* p = &tile->polys[i];
			// Do not return off-mesh connection polygons.
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			// Calc polygon bounds.
			const float* v = &tile->verts[p->verts[0]*3];
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j]*3];
				dtVmin(bmin, v);
				dtVmax(bmax, v);
			}
			if (dtOverlapBounds(qmin,qmax, bmin,bmax))
			{
				if (n < maxPolys)
					polys[n++] = base | (dtPolyRef)i;
			}
		}
		return n;
	}
}

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #dtPolyRef's for the
/// tile will be restored to the same values they were before the tile was 
/// removed.
///
/// The nav mesh assumes exclusive access to the data passed and will make
/// changes to the dynamic portion of the data. For that reason the data
/// should not be reused in other nav meshes until the tile has been successfully
/// removed from this nav mesh.
///
/// @see dtCreateNavMeshData, #removeTile
dtStatus dtNavMesh::addTile(unsigned char* data, int dataSize, int flags,
							dtTileRef lastRef, dtTileRef* result)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
		
	// Make sure the location is free.
	if (getTileAt(header->x, header->y, header->layer))
		return DT_FAILURE | DT_ALREADY_OCCUPIED;
		
	// Allocate a tile.
	dtMeshTile* tile = 0;
	if (!lastRef)
	{
		if (m_nextFree)
		{
			tile = m_nextFree;
			m_nextFree = tile->next;
			tile->next = 0;
		}
	}
	else
	{
		// Try to relocate the tile to specific index with same salt.
		int tileIndex = (int)decodePolyIdTile((dtPolyRef)lastRef);
		if (tileIndex >= m_maxTiles)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Try to find the specific tile id from the free list.
		dtMeshTile* target = &m_tiles[tileIndex];
		dtMeshTile* prev = 0;
		tile = m_nextFree;
		while (tile && tile != target)
		{
			prev = tile;
			tile = tile->next;
		}
		// Could not find the correct location.
		if (tile != target)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Remove from freelist
		if (!prev)
			m_nextFree = tile->next;
		else
			prev->next = tile->next;

		// Restore salt.
		tile->salt = decodePolyIdSalt((dtPolyRef)lastRef);
	}

	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	int h = computeTileHash(header->x, header->y, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*header->polyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*(header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode)*header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection)*header->offMeshConCount);
	
	unsigned char* d = data + headerSize;
	tile->verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	tile->links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile->detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
	tile->offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);

	// If there are no items in the bvtree, reset the tree pointer.
	if (!bvtreeSize)
		tile->bvTree = 0;

	// Build links freelist
	tile->linksFreeList = 0;
	tile->links[header->maxLinkCount-1].next = DT_NULL_LINK;
	for (int i = 0; i < header->maxLinkCount-1; ++i)
		tile->links[i].next = i+1;

	// Init tile.
	tile->header = header;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->flags = flags;

	connectIntLinks(tile);

	// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	baseOffMeshLinks(tile);
	connectExtOffMeshLinks(tile, tile, -1);

	// Create connections with neighbour tiles.
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];
	int nneis;
	
	// Connect with layers in current tile.
	nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile)
			continue;
	
		connectExtLinks(tile, neis[j], -1);
		connectExtLinks(neis[j], tile, -1);
		connectExtOffMeshLinks(tile, neis[j], -1);
		connectExtOffMeshLinks(neis[j], tile, -1);
	}
	
	// Connect with neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
		{
			connectExtLinks(tile, neis[j], i);
			connectExtLinks(neis[j], tile, dtOppositeTile(i));
			connectExtOffMeshLinks(tile, neis[j], i);
			connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
		}
	}
	
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

const dtMeshTile* dtNavMesh::getTileAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

int dtNavMesh::getNeighbourTilesAt(const int x, const int y, const int side, dtMeshTile** tiles, const int maxTiles) const
{
	int nx = x, ny = y;
	switch (side)
	{
		case 0: nx++; break;
		case 1: nx++; ny++; break;
		case 2: ny++; break;
		case 3: nx--; ny++; break;
		case 4: nx--; break;
		case 5: nx--; ny--; break;
		case 6: ny--; break;
		case 7: nx++; ny--; break;
	};

	return getTilesAt(nx, ny, tiles, maxTiles);
}

int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile** tiles, const int maxTiles) const
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}
	
	return n;
}

/// @par
///
/// This function will not fail if the tiles array is too small to hold the
/// entire result set.  It will simply fill the array to capacity.
int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile const** tiles, const int maxTiles) const
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}
	
	return n;
}


dtTileRef dtNavMesh::getTileRefAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return getTileRef(tile);
		}
		tile = tile->next;
	}
	return 0;
}

const dtMeshTile* dtNavMesh::getTileByRef(dtTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return 0;
	const dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

dtMeshTile* dtNavMesh::getTileByRef(dtTileRef ref)
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return 0;
	dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

int dtNavMesh::getMaxTiles() const
{
	return m_maxTiles;
}

dtMeshTile* dtNavMesh::getTile(int i)
{
	return &m_tiles[i];
}

const dtMeshTile* dtNavMesh::getTile(int i) const
{
	return &m_tiles[i];
}

void dtNavMesh::calcTileLoc(const float* pos, int* tx, int* ty) const
{
	*tx = (int)floorf((pos[0]-m_orig[0]) / m_tileWidth);
	*ty = (int)floorf((pos[2]-m_orig[2]) / m_tileHeight);
}

dtStatus dtNavMesh::getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	*tile = &m_tiles[it];
	*poly = &m_tiles[it].polys[ip];
	return DT_SUCCESS;
}

/// @par
///
/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
void dtNavMesh::getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	*tile = &m_tiles[it];
	*poly = &m_tiles[it].polys[ip];
}

bool dtNavMesh::isValidPolyRef(dtPolyRef ref) const
{
	if (!ref) return false;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return false;
	return true;
}

/// @par
///
/// This function returns the data for the tile so that, if desired,
/// it can be added back to the navigation mesh at a later point.
///
/// @see #addTile
dtStatus dtNavMesh::removeTile(dtTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	int h = computeTileHash(tile->header->x,tile->header->y,m_tileLutMask);
	dtMeshTile* prev = 0;
	dtMeshTile* cur = m_posLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				m_posLookup[h] = cur->next;
			break;
		}
		prev = cur;
		cur = cur->next;
	}
	
	// Remove connections to neighbour tiles.
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];
	int nneis;
	
	// Disconnect from other layers in current tile.
	nneis = getTilesAt(tile->header->x, tile->header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile) continue;
		unconnectLinks(neis[j], tile);
	}
	
	// Disconnect from neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(tile->header->x, tile->header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
			unconnectLinks(neis[j], tile);
	}
		
	// Reset tile.
	if (tile->flags & DT_TILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}

	tile->header = 0;
	tile->flags = 0;
	tile->linksFreeList = 0;
	tile->polys = 0;
	tile->verts = 0;
	tile->links = 0;
	tile->detailMeshes = 0;
	tile->detailVerts = 0;
	tile->detailTris = 0;
	tile->bvTree = 0;
	tile->offMeshCons = 0;

	// Update salt, salt should never be zero.
#ifdef DT_POLYREF64
	tile->salt = (tile->salt+1) & ((1<<DT_SALT_BITS)-1);
#else
	tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
#endif
	if (tile->salt == 0)
		tile->salt++;

	// Add to free list.
	tile->next = m_nextFree;
	m_nextFree = tile;

	return DT_SUCCESS;
}

dtTileRef dtNavMesh::getTileRef(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (dtTileRef)encodePolyId(tile->salt, it, 0);
}

/// @par
///
/// Example use case:
/// @code
///
/// const dtPolyRef base = navmesh->getPolyRefBase(tile);
/// for (int i = 0; i < tile->header->polyCount; ++i)
/// {
///     const dtPoly* p = &tile->polys[i];
///     const dtPolyRef ref = base | (dtPolyRef)i;
///     
///     // Use the reference to access the polygon data.
/// }
/// @endcode
dtPolyRef dtNavMesh::getPolyRefBase(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return encodePolyId(tile->salt, it, 0);
}

struct dtTileState
{
	int magic;								// Magic number, used to identify the data.
	int version;							// Data version number.
	dtTileRef ref;							// Tile ref at the time of storing the data.
};

struct dtPolyState
{
	unsigned short flags;						// Flags (see dtPolyFlags).
	unsigned char area;							// Area ID of the polygon.
};

///  @see #storeTileState
int dtNavMesh::getTileStateSize(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const int headerSize = dtAlign4(sizeof(dtTileState));
	const int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	return headerSize + polyStateSize;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note The state data is only valid until the tile reference changes.
/// @see #getTileStateSize, #restoreTileState
dtStatus dtNavMesh::storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
		
	dtTileState* tileState = dtGetThenAdvanceBufferPointer<dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));
	
	// Store tile state.
	tileState->magic = DT_NAVMESH_STATE_MAGIC;
	tileState->version = DT_NAVMESH_STATE_VERSION;
	tileState->ref = getTileRef(tile);
	
	// Store per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		dtPolyState* s = &polyStates[i];
		s->flags = p->flags;
		s->area = p->getArea();
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note This function does not impact the tile's #dtTileRef and #dtPolyRef's.
/// @see #storeTileState
dtStatus dtNavMesh::restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize)
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	const dtTileState* tileState = dtGetThenAdvanceBufferPointer<const dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	const dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<const dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));
	
	// Check that the restore is possible.
	if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (tileState->version != DT_NAVMESH_STATE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	if (tileState->ref != getTileRef(tile))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Restore per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* p = &tile->polys[i];
		const dtPolyState* s = &polyStates[i];
		p->flags = s->flags;
		p->setArea(s->area);
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Off-mesh connections are stored in the navigation mesh as special 2-vertex 
/// polygons with a single edge. At least one of the vertices is expected to be 
/// inside a normal polygon. So an off-mesh connection is "entered" from a 
/// normal polygon at one of its endpoints. This is the polygon identified by 
/// the prevRef parameter.
dtStatus dtNavMesh::getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const
{
	unsigned int salt, it, ip;

	if (!polyRef)
		return DT_FAILURE;
	
	// Get current polygon
	decodePolyId(polyRef, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const dtPoly* poly = &tile->polys[ip];

	// Make sure that the current poly is indeed off-mesh link.
	if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE;

	// Figure out which way to hand out the vertices.
	int idx0 = 0, idx1 = 1;
	
	// Find link that points to first vertex.
	for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
	{
		if (tile->links[i].edge == 0)
		{
			if (tile->links[i].ref != prevRef)
			{
				idx0 = 1;
				idx1 = 0;
			}
			break;
		}
	}
	
	dtVcopy(startPos, &tile->verts[poly->verts[idx0]*3]);
	dtVcopy(endPos, &tile->verts[poly->verts[idx1]*3]);

	return DT_SUCCESS;
}


const dtOffMeshConnection* dtNavMesh::getOffMeshConnectionByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	
	if (!ref)
		return 0;
	
	// Get current polygon
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return 0;
	const dtPoly* poly = &tile->polys[ip];
	
	// Make sure that the current poly is indeed off-mesh link.
	if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
		return 0;

	const unsigned int idx =  ip - tile->header->offMeshBase;
	dtAssert(idx < (unsigned int)tile->header->offMeshConCount);
	return &tile->offMeshCons[idx];
}


dtStatus dtNavMesh::setPolyFlags(dtPolyRef ref, unsigned short flags)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	dtPoly* poly = &tile->polys[ip];
	
	// Change flags.
	poly->flags = flags;
	
	return DT_SUCCESS;
}

dtStatus dtNavMesh::getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const dtPoly* poly = &tile->polys[ip];

	*resultFlags = poly->flags;
	
	return DT_SUCCESS;
}

dtStatus dtNavMesh::setPolyArea(dtPolyRef ref, unsigned char area)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	dtPoly* poly = &tile->polys[ip];
	
	poly->setArea(area);
	
	return DT_SUCCESS;
}

dtStatus dtNavMesh::getPolyArea(dtPolyRef ref, unsigned char* resultArea) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const dtPoly* poly = &tile->polys[ip];
	
	*resultArea = poly->getArea();
	
	return DT_SUCCESS;
}
dtStatus dtNavMesh::AddOffMeshLink(dtTileRef ref, dtGridOffmesh& gridOffmesh)
{
	dtStatus flag = DT_SUCCESS;
	do
	{
		dtMeshTile* _currentTile = getTileByRef(ref);
		if (nullptr == _currentTile)
		{
			flag = DT_FAILURE;
			break;
		}

		unsigned char* data = _currentTile->data;
		int dataSize = _currentTile->dataSize;


		dtMeshHeader* header = _currentTile->header;
		if (header->magic != DT_NAVMESH_MAGIC)
		{
			flag = DT_FAILURE | DT_WRONG_MAGIC;
			break;
		}

		if (header->version != DT_NAVMESH_VERSION)
		{
			flag = DT_FAILURE | DT_WRONG_VERSION;
			break;
		}

		int storedOffMeshConCount = 0;
		unsigned char* offMeshConClass = 0;
		// begin to cook off mesh data
		{
			if (gridOffmesh.offMeshConCount > 0)
			{
				offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char)*gridOffmesh.offMeshConCount * 2, DT_ALLOC_TEMP);
				if (nullptr == offMeshConClass)
				{
					flag = DT_FAILURE;
					break;
				}

				// Find tight heigh bounds, used for culling out off-mesh start locations.
				float hmin = FLT_MAX;
				float hmax = -FLT_MAX;

				float xmin = FLT_MAX;
				float xmax = -FLT_MAX;

				float zmin = FLT_MAX;
				float zmax = -FLT_MAX;

				for (int i = 0; i < header->vertCount; ++i)
				{
					const float h = _currentTile->verts[i * 3 + 1];
					hmin = dtMin(hmin, h);
					hmax = dtMax(hmax, h);

					const float x = _currentTile->verts[i * 3 + 0];
					xmin = dtMin(xmin, x);
					xmax = dtMax(xmax, x);

					const float z = _currentTile->verts[i * 3 + 2];
					zmin = dtMin(zmin, z);
					zmax = dtMax(zmax, z);
				}

				hmin -= header->walkableClimb;
				hmax += header->walkableClimb;

				xmin -= header->walkableClimb;
				xmax += header->walkableClimb;

				zmin -= header->walkableClimb;
				zmax += header->walkableClimb;

				float bmin[3], bmax[3];
				//dtVcopy(bmin, header->bmin);
				//dtVcopy(bmax, header->bmax);
				bmin[0] = dtMin(xmin, header->bmin[0]);
				bmax[0] = dtMax(xmax, header->bmax[0]);
				bmin[1] = hmin;
				bmax[1] = hmax;
				bmin[2] = dtMin(zmin, header->bmin[2]);
				bmax[2] = dtMax(zmax, header->bmax[2]);

				for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
				{
					const float* p0 = &gridOffmesh.offMeshConVerts[(i * 2 + 0) * 3];
					const float* p1 = &gridOffmesh.offMeshConVerts[(i * 2 + 1) * 3];
					offMeshConClass[i * 2 + 0] = classifyOffMeshPoint(p0, bmin, bmax);
					offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, bmin, bmax);

					// Zero out off-mesh start positions which are not even potentially touching the mesh.
					if (offMeshConClass[i * 2 + 0] == 0xff)
					{
						if (p0[1] < bmin[1] || p0[1] > bmax[1])
							offMeshConClass[i * 2 + 0] = 0;
					}

					if (offMeshConClass[i * 2 + 0] == 0xff)
						storedOffMeshConCount++;
				}
			}
		}

		int _validOffmeshIndex = header->offMeshBase;
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			dtPoly* _OffMeshPoly = &_currentTile->polys[header->offMeshBase + i];
			if (0 == _OffMeshPoly->vertCount)
			{
				if (header->offMeshConCount - i < storedOffMeshConCount)//no enough space
				{
					storedOffMeshConCount = header->offMeshConCount - i;
				}

				_validOffmeshIndex = header->offMeshBase + i;

				//////////////////////////////////////////////////////////////////////////
				// verts new Off-mesh link vertices.
				int n = 0;
				for (int j = 0; j < gridOffmesh.offMeshConCount; ++j)
				{
					// Only store connections which start from this tile.
					if (offMeshConClass[j * 2 + 0] == 0xff)
					{
						if (n >= storedOffMeshConCount)
						{
							break;
						}
						const float* linkv = &gridOffmesh.offMeshConVerts[j * 2 * 3];
						float* v = &_currentTile->verts[(_OffMeshPoly->verts[0] + n * 2) * 3];
						dtVcopy(&v[0], &linkv[0]);
						dtVcopy(&v[3], &linkv[3]);
						n++;
					}
				}
				// poly
				n = 0;
				for (int k = 0; k < gridOffmesh.offMeshConCount; ++k)
				{
					// Only store connections which start from this tile.
					if (offMeshConClass[k * 2 + 0] == 0xff)
					{
						if (n >= storedOffMeshConCount)
						{
							break;
						}
						dtPoly* p = &_currentTile->polys[_validOffmeshIndex + n];
						p->vertCount = 2;
						//p->verts[0] = (unsigned short)(header->vertCount + gridInfo.vertsCount + n * 2 + 0);
						//p->verts[1] = (unsigned short)(header->vertCount + gridInfo.vertsCount + n * 2 + 1);
						p->flags = gridOffmesh.offMeshConFlags[i];
						p->setArea(gridOffmesh.offMeshConAreas[i]);
						p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
						n++;
					}
				}

				// Store Off-Mesh connections.
				n = 0;
				for (int q = 0; q < gridOffmesh.offMeshConCount; ++q)
				{
					// Only store connections which start from this tile.
					if (offMeshConClass[q * 2 + 0] == 0xff)
					{
						if (n >= storedOffMeshConCount)
						{
							break;
						}
						dtOffMeshConnection* con = &_currentTile->offMeshCons[n + i];
						//con->poly = (unsigned short)(header->polyCount + gridInfo.polyCount + n);
						// Copy connection end-points.
						const float* endPts = &gridOffmesh.offMeshConVerts[q * 2 * 3];
						dtVcopy(&con->pos[0], &endPts[0]);
						dtVcopy(&con->pos[3], &endPts[3]);
						con->rad = gridOffmesh.offMeshConRad[q];
						con->flags = gridOffmesh.offMeshConDir[q] ? DT_OFFMESH_CON_BIDIR : 0;
						con->side = offMeshConClass[q * 2 + 1];
						if (gridOffmesh.offMeshConUserID)
							con->userId = gridOffmesh.offMeshConUserID[q];
						n++;
					}
				}

				//////////////////////////////////////////////////////////////////////////
				// make link
				baseOffMeshLinks(_currentTile, i);
				connectExtOffMeshLinks(_currentTile, _currentTile, -1, i);
				
				// Create connections with neighbour tiles.
				static const int MAX_NEIS = 32;
				dtMeshTile* neis[MAX_NEIS];
				int nneis;

				// Connect with layers in current tile.
				nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
				for (int j = 0; j < nneis; ++j)
				{
					if (neis[j] == _currentTile)
						continue;

					connectExtLinks(_currentTile, neis[j], -1);
					connectExtLinks(neis[j], _currentTile, -1);
					connectExtOffMeshLinks(_currentTile, neis[j], -1);
					connectExtOffMeshLinks(neis[j], _currentTile, -1);
				}

				// Connect with neighbour tiles.
				for (int ii = 0; ii < 8; ++ii)
				{
					nneis = getNeighbourTilesAt(header->x, header->y, ii, neis, MAX_NEIS);
					for (int j = 0; j < nneis; ++j)
					{
						connectExtLinks(_currentTile, neis[j], ii);
						connectExtLinks(neis[j], _currentTile, dtOppositeTile(ii));
						connectExtOffMeshLinks(_currentTile, neis[j], i);
						connectExtOffMeshLinks(neis[j], _currentTile, dtOppositeTile(ii));
					}
				}
				

				break;
			}
		}

		dtFree(offMeshConClass);
	} while (false);
	return flag;
}

dtStatus dtNavMesh::ReAddTitle(dtTileRef ref, dtGrid& gridInfo, dtGridOffmesh& gridOffmesh)
{
	dtStatus flag = DT_SUCCESS;
	do 
	{

		const dtMeshTile* _oldTile = getTileByRef(ref);
		if (nullptr == _oldTile)
		{
			flag = DT_FAILURE;
			break;
		}

		unsigned char* data = _oldTile->data;
		int dataSize = _oldTile->dataSize;
		

		dtMeshHeader* header = _oldTile->header;
		if (header->magic != DT_NAVMESH_MAGIC)
		{
			flag = DT_FAILURE | DT_WRONG_MAGIC;
			break;
		}
		
		if (header->version != DT_NAVMESH_VERSION)
		{
			flag = DT_FAILURE | DT_WRONG_VERSION;
			break;
		}

		int storedOffMeshConCount = 0;
		int offMeshConLinkCount = 0;
		unsigned char* offMeshConClass = 0;
		// begin to cook off mesh data
		{
			if (gridOffmesh.offMeshConCount > 0)
			{
				offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char)*gridOffmesh.offMeshConCount * 2, DT_ALLOC_TEMP);
				if (nullptr == offMeshConClass)
				{
					flag = DT_FAILURE;
					break;
				}

				// Find tight heigh bounds, used for culling out off-mesh start locations.
				float hmin = FLT_MAX;
				float hmax = -FLT_MAX;

				for (int i = 0; i < header->vertCount; ++i)
				{
					const float h = _oldTile->verts[i * 3 + 1];
					hmin = dtMin(hmin, h);
					hmax = dtMax(hmax, h);
				}

				hmin = dtMin(hmin, gridInfo.baseZ);
				hmax = dtMax(hmax, gridInfo.baseZ);

				hmin -= header->walkableClimb;
				hmax += header->walkableClimb;
				float bmin[3], bmax[3];
				dtVcopy(bmin, header->bmin);
				dtVcopy(bmax, header->bmax);
				bmin[1] = hmin;
				bmax[1] = hmax;

				for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
				{
					const float* p0 = &gridOffmesh.offMeshConVerts[(i * 2 + 0) * 3];
					const float* p1 = &gridOffmesh.offMeshConVerts[(i * 2 + 1) * 3];
					offMeshConClass[i * 2 + 0] = classifyOffMeshPoint(p0, bmin, bmax);
					offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, bmin, bmax);

					// Zero out off-mesh start positions which are not even potentially touching the mesh.
					if (offMeshConClass[i * 2 + 0] == 0xff)
					{
						if (p0[1] < bmin[1] || p0[1] > bmax[1])
							offMeshConClass[i * 2 + 0] = 0;
					}

					// Cound how many links should be allocated for off-mesh connections.
					if (offMeshConClass[i * 2 + 0] == 0xff)
						offMeshConLinkCount++;
					if (offMeshConClass[i * 2 + 1] == 0xff)
						offMeshConLinkCount++;

					if (offMeshConClass[i * 2 + 0] == 0xff)
						storedOffMeshConCount++;
				}
			}
		}

		const int headerSize = dtAlign4(sizeof(dtMeshHeader));

		const int _srcGroundVertsCount = header->vertCount - header->offMeshConCount * 2;
		const int _srcGroundPolyCount = header->offMeshBase;// header->polyCount - header->offMeshConCount;

		const int _gridvertsSize = dtAlign4(sizeof(float) * 3 * gridInfo.vertsCount );
		const int _gridpolysSize = dtAlign4(sizeof(dtPoly) * gridInfo.polyCount);
		const int _gridlinksSize = dtAlign4(sizeof(dtLink) * ( (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 4));
		const int _griddetailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * gridInfo.polyCount);
		const int _griddetailVertsSize = dtAlign4(sizeof(float) * 3 * 0);
		const int _griddetailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 2);
		const int _gridbvTreeSize = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode) * gridInfo.polyCount * 2) : 0;
		const int _gridoffMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * gridOffmesh.offMeshConCount);

		const int _srcvertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
		const int _srcpolysSize = dtAlign4(sizeof(dtPoly) * header->polyCount);
		const int _srclinksSize = dtAlign4(sizeof(dtLink) * header->maxLinkCount);
		const int _srcdetailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * _srcGroundPolyCount);
		const int _srcdetailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
		const int _srcdetailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
		const int _srcbvTreeSize = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode)*header->bvNodeCount) : 0;
		const int _srcoffMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);

		const int _newOffMeshvertsSize = dtAlign4(sizeof(float) * 3 *  storedOffMeshConCount * 2);
		const int _newOffMeshpolysSize = dtAlign4(sizeof(dtPoly) * storedOffMeshConCount);
		const int _newOffMeshlinksSize = dtAlign4(sizeof(dtLink) * (offMeshConLinkCount * 2));
		const int _newOffMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * storedOffMeshConCount);

		const int _reserveOffMeshvertsSize = dtAlign4(sizeof(float) * 3 * DT_RESERVE_OFFMESH * 2);
		const int _reserveOffMeshpolysSize = dtAlign4(sizeof(dtPoly) * DT_RESERVE_OFFMESH);
		const int _reserveOffMeshlinksSize = dtAlign4(sizeof(dtLink) * (DT_RESERVE_OFFMESH * 2));
		const int _reserveOffMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * DT_RESERVE_OFFMESH);
		
		const int _totalOffMeshCount = (storedOffMeshConCount + header->offMeshConCount) + DT_RESERVE_OFFMESH;
		

			

		const int _totalVertsSize = _gridvertsSize + _srcvertsSize + _newOffMeshvertsSize + _reserveOffMeshvertsSize;
		const int _totalPolySize = _gridpolysSize + _srcpolysSize + _newOffMeshpolysSize + _reserveOffMeshpolysSize;
		const int _totalLinkSize = _gridlinksSize + _srclinksSize + _newOffMeshlinksSize + _reserveOffMeshlinksSize;
		const int _totalDetailMeshesSize = _griddetailMeshesSize + _srcdetailMeshesSize;
		const int _totalDetailVertsSize = _griddetailVertsSize + _srcdetailVertsSize;
		const int _totalDetailTrisSize = _griddetailTrisSize + _srcdetailTrisSize;
		const int _totalbvTreeSize = _gridbvTreeSize + _srcbvTreeSize;
		const int _totalOffMeshConSize = _gridoffMeshConsSize + _srcoffMeshConsSize + _newOffMeshConsSize + _reserveOffMeshConsSize;

		int _totaldataSize = headerSize + _totalVertsSize + _totalPolySize + _totalLinkSize + _totalDetailMeshesSize + _totalDetailVertsSize + _totalDetailTrisSize + _totalbvTreeSize + _totalOffMeshConSize;

		unsigned char* _griddata = (unsigned char*)dtAlloc(sizeof(unsigned char)*_totaldataSize, DT_ALLOC_PERM);
		if (nullptr == _griddata)
		{
			flag = DT_FAILURE;
			break;
		}

		memset(_griddata, 0, _totaldataSize);

		unsigned char* dGrid = _griddata;
		dtMeshHeader* _gridheader = dtGetThenAdvanceBufferPointer<dtMeshHeader>(dGrid, headerSize);
		float* _gridnavVerts = dtGetThenAdvanceBufferPointer<float>(dGrid, _totalVertsSize);
		dtPoly* _gridnavPolys = dtGetThenAdvanceBufferPointer<dtPoly>(dGrid, _totalPolySize);
		dtLink* _gridlink = dtGetThenAdvanceBufferPointer<dtLink>(dGrid, _totalLinkSize);;
		dtPolyDetail* _gridnavDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(dGrid, _totalDetailMeshesSize);
		float* _gridnavDVerts = dtGetThenAdvanceBufferPointer<float>(dGrid, _totalDetailVertsSize);
		unsigned char* _gridnavDTris = dtGetThenAdvanceBufferPointer<unsigned char>(dGrid, _totalDetailTrisSize);
		dtBVNode* _gridnavBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(dGrid, _totalbvTreeSize);
		dtOffMeshConnection* _gridoffMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(dGrid, _totalOffMeshConSize);

		unsigned char* dSrc = data;
		dtMeshHeader* _srcheader = dtGetThenAdvanceBufferPointer<dtMeshHeader>(dSrc, headerSize);
		float* _srcnavVerts = dtGetThenAdvanceBufferPointer<float>(dSrc, _srcvertsSize);
		dtPoly* _srcnavPolys = dtGetThenAdvanceBufferPointer<dtPoly>(dSrc, _srcpolysSize);
		dtLink* _srclink = dtGetThenAdvanceBufferPointer<dtLink>(dSrc, _srclinksSize);;
		dtPolyDetail* _srcnavDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(dSrc, _srcdetailMeshesSize);
		float* _srcnavDVerts = dtGetThenAdvanceBufferPointer<float>(dSrc, _srcdetailVertsSize);
		unsigned char* _srcnavDTris = dtGetThenAdvanceBufferPointer<unsigned char>(dSrc, _srcdetailTrisSize);
		dtBVNode* _srcnavBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(dSrc, _srcbvTreeSize);
		dtOffMeshConnection* _srcoffMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(dSrc, _srcoffMeshConsSize);

		// header
		memcpy(_griddata, data, headerSize);
		
		// verts
		memcpy(_gridnavVerts, _srcnavVerts, _srcvertsSize);

		// verts new ground poly's verts
		for (int i = 0; i < gridInfo.vertsCount; i++)
		{
			const float* iv = &gridInfo.verts[i * 3];
			float* v = &_gridnavVerts[(i + _srcGroundVertsCount) * 3];// make sure ground poly is before off mesh poly
			v[0] = iv[0];
			v[1] = iv[1];
			v[2] = iv[2];
		}

		// verts recopy old off mesh info
		int n = 0;
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			const float* _oldv = &_srcnavVerts[(_srcGroundVertsCount + n * 2) * 3];
			float* v = &_gridnavVerts[(_srcGroundVertsCount + gridInfo.vertsCount + n * 2) * 3];
			dtVcopy(&v[0], &_oldv[0]);
			dtVcopy(&v[3], &_oldv[3]);
			n++;
		}

		// verts new Off-mesh link vertices.
		n = 0;
		for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
		{
			// Only store connections which start from this tile.
			if (offMeshConClass[i * 2 + 0] == 0xff)
			{
				const float* linkv = &gridOffmesh.offMeshConVerts[i * 2 * 3];
				float* v = &_gridnavVerts[(header->vertCount + gridInfo.vertsCount + n * 2) * 3];
				dtVcopy(&v[0], &linkv[0]);
				dtVcopy(&v[3], &linkv[3]);
				n++;
			}
		}

		// verts reserved Off-mesh link vertices.
		// default to 0
		
		// poly
		memcpy(_gridnavPolys, _srcnavPolys, _srcpolysSize);
		
		// poly new ground poly
		int _girdVertsXIndex = _srcGroundVertsCount;
		for (int i = 0; i < gridInfo.polyCount; ++i)
		{
			dtPoly* p = &_gridnavPolys[_srcGroundPolyCount + i];
			p->vertCount = 4;
			p->flags = 1;
			p->setArea(0);
			p->setType(DT_POLYTYPE_GROUND);

			int _gridZindex = i / (DT_grid_count_plusone - 1);
			int _currentPolyVertsXIndex = _girdVertsXIndex + _gridZindex * DT_grid_count_plusone + i % (DT_grid_count_plusone - 1) ;

			p->verts[0] = _currentPolyVertsXIndex + 0;
			p->verts[1] = _currentPolyVertsXIndex + 1;
			p->verts[2] = _currentPolyVertsXIndex + 1 + DT_grid_count_plusone;
			p->verts[3] = _currentPolyVertsXIndex + 0 + DT_grid_count_plusone;
		}

		for (int i = 0; i < gridInfo.polyCount; ++i)
		{
			dtPoly* p = &_gridnavPolys[_srcGroundPolyCount + i];

			for (int j = 0; j < 4; ++j)
			{
				p->neis[j] = DT_EXT_LINK | 0 | 2 | 4 | 6;

				if (0 == i % ( DT_grid_count_plusone - 1) && 3 == j)
				{
					p->neis[j] = 0;
				}

				if (DT_grid_count_plusone - 2 == i % (DT_grid_count_plusone - 1) && 1 == j)
				{
					p->neis[j] = 0;
				}

				if (i < DT_grid_count_plusone - 1  && 0 == j)
				{
					p->neis[j] = 0;
				}

				if (i >= (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 2) && 2 == j)
				{
					p->neis[j] = 0;
				}
			}
		}

		// poly recopy old off mesh info
		n = 0;
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			dtPoly* _oldpoly = &_srcnavPolys[_srcGroundPolyCount + i];
			dtPoly* _newpoly = &_gridnavPolys[_srcGroundPolyCount + gridInfo.polyCount + i];
			_newpoly->vertCount = _oldpoly->vertCount;
			_newpoly->verts[0] = (unsigned short)(_srcGroundVertsCount + gridInfo.vertsCount + n * 2 + 0);
			_newpoly->verts[1] = (unsigned short)(_srcGroundVertsCount + gridInfo.vertsCount + n * 2 + 1);
			_newpoly->flags = _oldpoly->flags;
			_newpoly->areaAndtype = _oldpoly->areaAndtype;
			n++;
		}

		// poly new Off-mesh link poly.
		n = 0;
		for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
		{
			// Only store connections which start from this tile.
			if (offMeshConClass[i * 2 + 0] == 0xff)
			{
				dtPoly* p = &_gridnavPolys[header->polyCount + gridInfo.polyCount + n];
				p->vertCount = 2;
				p->verts[0] = (unsigned short)(header->vertCount + gridInfo.vertsCount + n * 2 + 0);
				p->verts[1] = (unsigned short)(header->vertCount + gridInfo.vertsCount + n * 2 + 1);
				p->flags = gridOffmesh.offMeshConFlags[i];
				p->setArea(gridOffmesh.offMeshConAreas[i]);
				p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
				n++;
			}
		}

		// poly reserve Off-mesh link poly.
		for (int i = 0; i < DT_RESERVE_OFFMESH; ++i)
		{
			dtPoly* p = &_gridnavPolys[header->polyCount + gridInfo.polyCount + n + i];
			p->vertCount = 0;
			p->verts[0] = (unsigned short)(header->vertCount + gridInfo.vertsCount + n * 2 + i * 2 + 0);
			p->verts[1] = (unsigned short)(header->vertCount + gridInfo.vertsCount + n * 2 + i * 2 + 1);
			p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
			p->firstLink = DT_NULL_LINK;
		}

		//link 
		//memcpy(_gridlink, _srclink, _srclinksSize);

		// detail meshes
		{
			memcpy(_gridnavDMeshes, _srcnavDMeshes, _srcdetailMeshesSize);
			memcpy(_gridnavDVerts, _srcnavDVerts, _srcdetailVertsSize);
			memcpy(_gridnavDTris, _srcnavDTris, _srcdetailTrisSize);
			// Create dummy detail mesh by triangulating polys.
			int tbase = _srcnavDMeshes[_srcGroundPolyCount - 1].triBase + _srcnavDMeshes[_srcGroundPolyCount - 1].triCount;
			for (int i = 0; i < gridInfo.polyCount; ++i)
			{
				dtPolyDetail& dtl = _gridnavDMeshes[_srcGroundPolyCount + i];
				const int nv = _gridnavPolys[_srcGroundPolyCount + i].vertCount;
				dtl.vertBase = 0;
				dtl.vertCount = 0;
				dtl.triBase = (unsigned int)tbase;
				dtl.triCount = (unsigned char)(nv - 2);
				// Triangulate polygon (local indices).
				for (int j = 2; j < nv; ++j)
				{
					unsigned char* t = &_gridnavDTris[tbase * 4];
					t[0] = 0;
					t[1] = (unsigned char)(j - 1);
					t[2] = (unsigned char)j;
					// Bit for each edge that belongs to poly boundary.
					t[3] = (1 << 2);
					if (j == 2) t[3] |= (1 << 0);
					if (j == nv - 1) t[3] |= (1 << 4);
					tbase++;
				}
			}
		}

		//bvtree 
		memcpy(_gridnavBvtree, _srcnavBvtree, _srcbvTreeSize);
		CreateGridBVTree(_gridheader->bmin, _gridheader->bvQuantFactor, _gridnavVerts, (void*)&_gridnavPolys[_srcGroundPolyCount], gridInfo.polyCount, (void*)&_gridnavBvtree[header->bvNodeCount], _srcGroundPolyCount, header->bvNodeCount);

		//offmesh
		//memcpy(_gridoffMeshCons, _srcoffMeshCons, _srcoffMeshConsSize);
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			dtOffMeshConnection* _newcon = &_gridoffMeshCons[i];
			dtOffMeshConnection* _oldcon = &_srcoffMeshCons[i];

			_newcon->poly = (unsigned short)(_oldcon->poly + gridInfo.polyCount);
			// Copy connection end-points.
			dtVcopy(&_newcon->pos[0], &_oldcon->pos[0]);
			dtVcopy(&_newcon->pos[3], &_oldcon->pos[3]);
			_newcon->rad = _oldcon->rad;
			_newcon->flags = _oldcon->flags;
			_newcon->side = _oldcon->side;
			_newcon->userId = _oldcon->userId;
		}

		// Store Off-Mesh connections.
		n = 0;
		for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
		{
			// Only store connections which start from this tile.
			if (offMeshConClass[i * 2 + 0] == 0xff)
			{
				dtOffMeshConnection* con = &_gridoffMeshCons[n + header->offMeshConCount];
				con->poly = (unsigned short)(header->polyCount + gridInfo.polyCount + n);
				// Copy connection end-points.
				const float* endPts = &gridOffmesh.offMeshConVerts[i * 2 * 3];
				dtVcopy(&con->pos[0], &endPts[0]);
				dtVcopy(&con->pos[3], &endPts[3]);
				con->rad = gridOffmesh.offMeshConRad[i];
				con->flags = gridOffmesh.offMeshConDir[i] ? DT_OFFMESH_CON_BIDIR : 0;
				con->side = offMeshConClass[i * 2 + 1];
				if (gridOffmesh.offMeshConUserID)
					con->userId = gridOffmesh.offMeshConUserID[i];
				n++;
			}
		}

		// reserve off-mesh connections
		for (int i = 0; i < DT_RESERVE_OFFMESH; ++i)
		{
			dtOffMeshConnection* con = &_gridoffMeshCons[i + n + header->offMeshConCount];
			con->poly = (unsigned short)(header->polyCount + gridInfo.polyCount + n + i);
			con->pos[0] = con->pos[1] = con->pos[2] = con->pos[3] = con->pos[4] = con->pos[5] = 0.f;
			con->rad = 0.f;
			con->flags = DT_OFFMESH_CON_BIDIR;
			con->side = 0xff;
		}

		dtFree(offMeshConClass);

		_gridheader->vertCount += (gridInfo.vertsCount + storedOffMeshConCount * 2 + DT_RESERVE_OFFMESH * 2);
		_gridheader->polyCount += (gridInfo.polyCount + storedOffMeshConCount + DT_RESERVE_OFFMESH);
		_gridheader->maxLinkCount += ((DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 4 + offMeshConLinkCount * 2 + DT_RESERVE_OFFMESH * 2);
		_gridheader->detailMeshCount += gridInfo.polyCount;
		_gridheader->detailTriCount += (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 2;
		_gridheader->bvNodeCount += header->bvNodeCount > 0 ? gridInfo.polyCount * 2 : 0;
		_gridheader->offMeshBase = _srcGroundPolyCount + gridInfo.polyCount;
		_gridheader->offMeshConCount = header->offMeshConCount + storedOffMeshConCount + DT_RESERVE_OFFMESH;

		removeTile(ref, &data, &dataSize);
		addTile(_griddata, _totaldataSize, 0, 0, 0);
	} while (false);

	return flag;
}

dtStatus dtNavMesh::AddStraightLadder(dtTileRef ref, dtStraightLadder& ladder)
{
	dtStatus flag = DT_SUCCESS;

	do 
	{
		dtMeshTile* _currentTile = getTileByRef(ref);
		if (nullptr == _currentTile)
		{
			flag = DT_FAILURE;
			break;
		}

		unsigned char* data = _currentTile->data;
		int dataSize = _currentTile->dataSize;


		dtMeshHeader* header = _currentTile->header;
		if (header->magic != DT_NAVMESH_MAGIC)
		{
			flag = DT_FAILURE | DT_WRONG_MAGIC;
			break;
		}

		const int headerSize = dtAlign4(sizeof(dtMeshHeader));

		const int _srcGroundVertsCount = header->vertCount - header->offMeshConCount * 2;
		const int _srcGroundPolyCount = header->offMeshBase;// header->polyCount - header->offMeshConCount;

		const int _gridvertsSize = dtAlign4(sizeof(float) * 3 * ladder.vertsCount);
		const int _gridpolysSize = dtAlign4(sizeof(dtPoly) * ladder.polyCount);
		const int _gridlinksSize = dtAlign4(sizeof(dtLink) *  4);
		const int _griddetailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * ladder.polyCount);
		const int _griddetailVertsSize = dtAlign4(sizeof(float) * 3 * 0);
		const int _griddetailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * 2);
		const int _gridbvTreeSize = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode) * ladder.polyCount * 2) : 0;
		const int _gridoffMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * 0);

		const int _srcvertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
		const int _srcpolysSize = dtAlign4(sizeof(dtPoly) * header->polyCount);
		const int _srclinksSize = dtAlign4(sizeof(dtLink) * header->maxLinkCount);
		const int _srcdetailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * _srcGroundPolyCount);
		const int _srcdetailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
		const int _srcdetailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
		const int _srcbvTreeSize = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode)*header->bvNodeCount) : 0;
		const int _srcoffMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);


		const int _totalVertsSize = _gridvertsSize + _srcvertsSize;
		const int _totalPolySize = _gridpolysSize + _srcpolysSize;
		const int _totalLinkSize = _gridlinksSize + _srclinksSize;
		const int _totalDetailMeshesSize = _griddetailMeshesSize + _srcdetailMeshesSize;
		const int _totalDetailVertsSize = _griddetailVertsSize + _srcdetailVertsSize;
		const int _totalDetailTrisSize = _griddetailTrisSize + _srcdetailTrisSize;
		const int _totalbvTreeSize = _gridbvTreeSize + _srcbvTreeSize;
		const int _totalOffMeshConSize = _gridoffMeshConsSize + _srcoffMeshConsSize;

		int _totaldataSize = headerSize + _totalVertsSize + _totalPolySize + _totalLinkSize + _totalDetailMeshesSize + _totalDetailVertsSize + _totalDetailTrisSize + _totalbvTreeSize + _totalOffMeshConSize;

		unsigned char* _griddata = (unsigned char*)dtAlloc(sizeof(unsigned char)*_totaldataSize, DT_ALLOC_PERM);
		if (nullptr == _griddata)
		{
			flag = DT_FAILURE;
			break;
		}

		memset(_griddata, 0, _totaldataSize);

		unsigned char* dGrid = _griddata;
		dtMeshHeader* _gridheader = dtGetThenAdvanceBufferPointer<dtMeshHeader>(dGrid, headerSize);
		float* _gridnavVerts = dtGetThenAdvanceBufferPointer<float>(dGrid, _totalVertsSize);
		dtPoly* _gridnavPolys = dtGetThenAdvanceBufferPointer<dtPoly>(dGrid, _totalPolySize);
		dtLink* _gridlink = dtGetThenAdvanceBufferPointer<dtLink>(dGrid, _totalLinkSize);;
		dtPolyDetail* _gridnavDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(dGrid, _totalDetailMeshesSize);
		float* _gridnavDVerts = dtGetThenAdvanceBufferPointer<float>(dGrid, _totalDetailVertsSize);
		unsigned char* _gridnavDTris = dtGetThenAdvanceBufferPointer<unsigned char>(dGrid, _totalDetailTrisSize);
		dtBVNode* _gridnavBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(dGrid, _totalbvTreeSize);
		dtOffMeshConnection* _gridoffMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(dGrid, _totalOffMeshConSize);

		unsigned char* dSrc = data;
		dtMeshHeader* _srcheader = dtGetThenAdvanceBufferPointer<dtMeshHeader>(dSrc, headerSize);
		float* _srcnavVerts = dtGetThenAdvanceBufferPointer<float>(dSrc, _srcvertsSize);
		dtPoly* _srcnavPolys = dtGetThenAdvanceBufferPointer<dtPoly>(dSrc, _srcpolysSize);
		dtLink* _srclink = dtGetThenAdvanceBufferPointer<dtLink>(dSrc, _srclinksSize);;
		dtPolyDetail* _srcnavDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(dSrc, _srcdetailMeshesSize);
		float* _srcnavDVerts = dtGetThenAdvanceBufferPointer<float>(dSrc, _srcdetailVertsSize);
		unsigned char* _srcnavDTris = dtGetThenAdvanceBufferPointer<unsigned char>(dSrc, _srcdetailTrisSize);
		dtBVNode* _srcnavBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(dSrc, _srcbvTreeSize);
		dtOffMeshConnection* _srcoffMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(dSrc, _srcoffMeshConsSize);

		// header
		memcpy(_griddata, data, headerSize);

		// verts
		memcpy(_gridnavVerts, _srcnavVerts, _srcvertsSize);

		// verts new ground poly's verts
		for (int i = 0; i < ladder.vertsCount; i++)
		{
			const float* iv = &ladder.verts[i * 3];
			float* v = &_gridnavVerts[(i + _srcGroundVertsCount) * 3];// make sure ground poly is before off mesh poly
			v[0] = iv[0];
			v[1] = iv[1];
			v[2] = iv[2];
		}

		// verts recopy old off mesh info
		int n = 0;
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			const float* _oldv = &_srcnavVerts[(_srcGroundVertsCount + n * 2) * 3];
			float* v = &_gridnavVerts[(_srcGroundVertsCount + ladder.vertsCount + n * 2) * 3];
			dtVcopy(&v[0], &_oldv[0]);
			dtVcopy(&v[3], &_oldv[3]);
			n++;
		}


		// poly
		memcpy(_gridnavPolys, _srcnavPolys, _srcpolysSize);

		// poly new ground poly
		int _girdVertsXIndex = _srcGroundVertsCount;
		for (int i = 0; i < ladder.polyCount; ++i)
		{
			dtPoly* p = &_gridnavPolys[_srcGroundPolyCount + i];
			p->vertCount = 4;
			p->flags = 1;
			p->setArea(0);
			p->setType(DT_POLYTYPE_GROUND);

			int _gridZindex = i * 4;
			int _currentPolyVertsXIndex = _girdVertsXIndex + _gridZindex ;

			p->verts[0] = _currentPolyVertsXIndex + 0;
			p->verts[1] = _currentPolyVertsXIndex + 1;
			p->verts[2] = _currentPolyVertsXIndex + 3;
			p->verts[3] = _currentPolyVertsXIndex + 2;
		}

		for (int i = 0; i < ladder.polyCount; ++i)
		{
			dtPoly* p = &_gridnavPolys[_srcGroundPolyCount + i];

			for (int j = 0; j < 4; ++j)
			{
				p->neis[j] = DT_EXT_LINK;
			}
		}

		// poly recopy old off mesh info
		n = 0;
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			dtPoly* _oldpoly = &_srcnavPolys[_srcGroundPolyCount + i];
			dtPoly* _newpoly = &_gridnavPolys[_srcGroundPolyCount + ladder.polyCount + i];
			_newpoly->vertCount = _oldpoly->vertCount;
			_newpoly->verts[0] = (unsigned short)(_srcGroundVertsCount + ladder.vertsCount + n * 2 + 0);
			_newpoly->verts[1] = (unsigned short)(_srcGroundVertsCount + ladder.vertsCount + n * 2 + 1);
			_newpoly->flags = _oldpoly->flags;
			_newpoly->areaAndtype = _oldpoly->areaAndtype;
			n++;
		}

		// detail meshes
		{
			memcpy(_gridnavDMeshes, _srcnavDMeshes, _srcdetailMeshesSize);
			memcpy(_gridnavDVerts, _srcnavDVerts, _srcdetailVertsSize);
			memcpy(_gridnavDTris, _srcnavDTris, _srcdetailTrisSize);
			// Create dummy detail mesh by triangulating polys.
			int tbase = _srcnavDMeshes[_srcGroundPolyCount - 1].triBase + _srcnavDMeshes[_srcGroundPolyCount - 1].triCount;
			for (int i = 0; i < ladder.polyCount; ++i)
			{
				dtPolyDetail& dtl = _gridnavDMeshes[_srcGroundPolyCount + i];
				const int nv = _gridnavPolys[_srcGroundPolyCount + i].vertCount;
				dtl.vertBase = 0;
				dtl.vertCount = 0;
				dtl.triBase = (unsigned int)tbase;
				dtl.triCount = (unsigned char)(nv - 2);
				// Triangulate polygon (local indices).
				for (int j = 2; j < nv; ++j)
				{
					unsigned char* t = &_gridnavDTris[tbase * 4];
					t[0] = 0;
					t[1] = (unsigned char)(j - 1);
					t[2] = (unsigned char)j;
					// Bit for each edge that belongs to poly boundary.
					t[3] = (1 << 2);
					if (j == 2) t[3] |= (1 << 0);
					if (j == nv - 1) t[3] |= (1 << 4);
					tbase++;
				}
			}
		}

		//bvtree 
		memcpy(_gridnavBvtree, _srcnavBvtree, _srcbvTreeSize);
		CreateGridBVTree(_gridheader->bmin, _gridheader->bvQuantFactor, _gridnavVerts, (void*)&_gridnavPolys[_srcGroundPolyCount], ladder.polyCount, (void*)&_gridnavBvtree[header->bvNodeCount], _srcGroundPolyCount, header->bvNodeCount);

		//offmesh
		//memcpy(_gridoffMeshCons, _srcoffMeshCons, _srcoffMeshConsSize);
		for (int i = 0; i < header->offMeshConCount; ++i)
		{
			dtOffMeshConnection* _newcon = &_gridoffMeshCons[i];
			dtOffMeshConnection* _oldcon = &_srcoffMeshCons[i];

			_newcon->poly = (unsigned short)(_oldcon->poly + ladder.polyCount);
			// Copy connection end-points.
			dtVcopy(&_newcon->pos[0], &_oldcon->pos[0]);
			dtVcopy(&_newcon->pos[3], &_oldcon->pos[3]);
			_newcon->rad = _oldcon->rad;
			_newcon->flags = _oldcon->flags;
			_newcon->side = _oldcon->side;
			_newcon->userId = _oldcon->userId;
		}

		_gridheader->vertCount += ladder.vertsCount;
		_gridheader->polyCount += ladder.polyCount;
		_gridheader->maxLinkCount += 4;
		_gridheader->detailMeshCount += ladder.polyCount;
		_gridheader->detailTriCount += 2;
		_gridheader->bvNodeCount += header->bvNodeCount > 0 ? ladder.polyCount * 2 : 0;
		_gridheader->offMeshBase = _srcGroundPolyCount + ladder.polyCount;
		_gridheader->offMeshConCount = header->offMeshConCount;

		removeTile(ref, &data, &dataSize);

		dtTileRef _newTileref;
		addTile(_griddata, _totaldataSize, 0, 0, &_newTileref);

		//////////////////////////////////////////////////////////////////////////
		const int _Offmeshcount = 1;
		float m_offMeshConVerts[_Offmeshcount * 3 * 2];
		float m_offMeshConRads[_Offmeshcount];
		unsigned char m_offMeshConDirs[_Offmeshcount];
		unsigned char m_offMeshConAreas[_Offmeshcount];
		unsigned short m_offMeshConFlags[_Offmeshcount];
		unsigned int m_offMeshConId[_Offmeshcount];

		m_offMeshConVerts[0] = ladder.verts[0 * 3 + 0];
		m_offMeshConVerts[1] = ladder.verts[0 * 3 + 1];
		m_offMeshConVerts[2] = ladder.verts[0 * 3 + 2];
		m_offMeshConVerts[3] = ladder.baseX;
		m_offMeshConVerts[4] = ladder.baseY;
		m_offMeshConVerts[5] = ladder.baseZ;

		m_offMeshConRads[0] = 0.6f;
		m_offMeshConDirs[0] = DT_OFFMESH_CON_BIDIR;
		m_offMeshConAreas[0] = 5;
		m_offMeshConFlags[0] = 8;
		m_offMeshConId[0] = 10000;

		dtGridOffmesh _gridOffmesh;
		_gridOffmesh.offMeshConVerts = m_offMeshConVerts;
		_gridOffmesh.offMeshConRad = m_offMeshConRads;
		_gridOffmesh.offMeshConDir = m_offMeshConDirs;
		_gridOffmesh.offMeshConAreas = m_offMeshConAreas;
		_gridOffmesh.offMeshConFlags = m_offMeshConFlags;
		_gridOffmesh.offMeshConUserID = m_offMeshConId;
		_gridOffmesh.offMeshConCount = _Offmeshcount;

		AddOffMeshLink(_newTileref, _gridOffmesh);

	} while (false);

	return flag;
}

dtStatus dtNavMesh::CreateGridTile(dtTileRef ref, dtGrid* gridFloorInfo, int floorCount,
	dtStraightLadder* ladder, int ladderCount,
	dtGridOffmesh& gridOffmesh)
{
	dtStatus flag = DT_SUCCESS;
	do
	{
		flag = IsValidTile(ref);
		if (DT_SUCCESS != flag) break;

		dtMeshTile* _oldTile = getTileByRef(ref);

		UpdateBounder(_oldTile->header, gridFloorInfo, floorCount);

		int _metaDataSize[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE] = { 0 };
		int _metaDataCount[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT] = { 0 };;

		int _gridFloorSize = GetGridFloorSize(_oldTile->header, gridFloorInfo, floorCount, 
			_metaDataSize[dtGridMetaDataCategory::GRIDFLOOR], _metaDataCount[dtGridMetaDataCategory::GRIDFLOOR]);

		int _InputNavMeshSize = GetInputNavMeshSize(_oldTile->header, 
			_metaDataSize[dtGridMetaDataCategory::INPUTNAVMESH], _metaDataCount[dtGridMetaDataCategory::INPUTNAVMESH]);

		int _storedOffMeshConCount = 0, _offMeshConLinkCount = 0;
		CookGeometryOffMeshLinkData(_oldTile->header, gridOffmesh, _storedOffMeshConCount, _offMeshConLinkCount);
		int _OffMeshLinkSize = GetGeometryOffMeshLinkSize(_oldTile->header, _storedOffMeshConCount, _offMeshConLinkCount, 
			_metaDataSize[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK], _metaDataCount[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK]);

		int _ladderSize = GetLadderSize(_oldTile->header, ladder, ladderCount,
			_metaDataSize[dtGridMetaDataCategory::LADDER], _metaDataCount[dtGridMetaDataCategory::LADDER]);

		int _reservedLadderSize = GetReservedLadderGroundPolySize(_oldTile->header,
			_metaDataSize[dtGridMetaDataCategory::RESERVEDLADDER], _metaDataCount[dtGridMetaDataCategory::RESERVEDLADDER]);

		int _reservedSize = GetReservedOffMeshSize(_oldTile->header, 
			_metaDataSize[dtGridMetaDataCategory::RESERVEDOFFMESH], _metaDataCount[dtGridMetaDataCategory::RESERVEDOFFMESH]);

		int _totaldataSize = GetTotalSize(_metaDataSize, _metaDataCount) + dtAlign4(sizeof(dtMeshHeader));

		unsigned char* _totaldata = (unsigned char*)dtAlloc(sizeof(unsigned char)*_totaldataSize, DT_ALLOC_PERM);
		if (nullptr == _totaldata)
		{
			flag = DT_FAILURE;
			break;
		}
		memset(_totaldata, 0, _totaldataSize);

		dtDataPointerHelper _dstTile, _srcTile;
		UpdateInputNavMeshDataPointerInfo(_srcTile, _oldTile->data, _metaDataSize[dtGridMetaDataCategory::INPUTNAVMESH]);
		UpdateDataPointerInfo(_dstTile, _totaldata, _metaDataSize);

		memcpy(_totaldata, _oldTile->data, dtAlign4(sizeof(dtMeshHeader)));
		UpdateHeader(*_dstTile.pheader, _metaDataCount);

		SetInputNavMeshData(_dstTile, _srcTile, _metaDataSize, _metaDataCount);
		SetGridFloorData(_dstTile, _srcTile, gridFloorInfo, floorCount, _metaDataSize, _metaDataCount);
		SetGeometryOffMeshLinkData(_dstTile, _srcTile, gridOffmesh, _metaDataSize, _metaDataCount);
		SetLadderData(_dstTile, _srcTile, ladder, ladderCount, _metaDataSize, _metaDataCount);
		SetReservedLadderData(_dstTile, _srcTile, _metaDataSize, _metaDataCount);
		SetReservedOffMeshData(_dstTile, _srcTile, _metaDataSize, _metaDataCount);

		removeTile(ref, &_oldTile->data, &_oldTile->dataSize);
		addTile(_totaldata, _totaldataSize, 0, 0, 0);
		
	} while (false);

	return flag;
}

dtStatus dtNavMesh::IsValidTile(dtTileRef ref)
{
	dtStatus flag = DT_SUCCESS;
	do
	{
		const dtMeshTile* _oldTile = getTileByRef(ref);
		if (nullptr == _oldTile)
		{
			flag = DT_FAILURE;
			break;
		}

		dtMeshHeader* header = _oldTile->header;
		if (nullptr == header)
		{
			flag = DT_FAILURE;
			break;
		}

		if (header->magic != DT_NAVMESH_MAGIC)
		{
			flag = DT_FAILURE | DT_WRONG_MAGIC;
			break;
		}

		if (header->version != DT_NAVMESH_VERSION)
		{
			flag = DT_FAILURE | DT_WRONG_VERSION;
			break;
		}
	} while (false);

	return flag;
}

void dtNavMesh::UpdateBounder(dtMeshHeader* header, dtGrid* gridFloorInfo, int floorCount)
{
	for (int i = 0; i < floorCount; i++)
	{
		float _xmin = gridFloorInfo[i].baseX - (DT_grid_count_plusone - 1) * DT_grid_UnitSize * 0.5f;
		float _xmax = gridFloorInfo[i].baseX + (DT_grid_count_plusone - 1) * DT_grid_UnitSize * 0.5f;
		
		float _zmin = gridFloorInfo[i].baseZ - (DT_grid_count_plusone - 1) * DT_grid_UnitSize * 0.5f;
		float _zmax = gridFloorInfo[i].baseZ + (DT_grid_count_plusone - 1) * DT_grid_UnitSize * 0.5f;

		float _gridVert1[3] = { _xmin, gridFloorInfo[i].baseY, _zmin };
		float _gridVert2[3] = { _xmin, gridFloorInfo[i].baseY, _zmax };
		float _gridVert3[3] = { _xmax, gridFloorInfo[i].baseY, _zmin };
		float _gridVert4[3] = { _xmax, gridFloorInfo[i].baseY, _zmax };

		dtVmin(header->bmin, _gridVert1);
		dtVmin(header->bmin, _gridVert2);
		dtVmin(header->bmin, _gridVert3);
		dtVmin(header->bmin, _gridVert4);

		dtVmax(header->bmax, _gridVert1);
		dtVmax(header->bmax, _gridVert2);
		dtVmax(header->bmax, _gridVert3);
		dtVmax(header->bmax, _gridVert4);
	}
}

int dtNavMesh::GetGridFloorSize(dtMeshHeader* header, dtGrid* gridFloorInfo, int floorCount, 
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE], 
	int CountInfo[dtGridMetaDataCount::COUNT])
{
	int _size = 0;

	for (int i = 0; i < floorCount; i++)
	{
		SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE] += dtAlign4(sizeof(float) * 3 * gridFloorInfo[i].vertsCount);
		SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE] += 0;
		SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE] += dtAlign4(sizeof(dtPoly) * gridFloorInfo[i].polyCount);
		SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE] += 0;
		SizeInfo[dtGridMetaDataSize::LINK_SIZE] += dtAlign4(sizeof(dtLink) * ((DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 4));
		SizeInfo[dtGridMetaDataSize::DETAILMESHES_SIZE] += dtAlign4(sizeof(dtPolyDetail) * gridFloorInfo[i].polyCount);
		SizeInfo[dtGridMetaDataSize::DETAILVERTS_SIZE] += dtAlign4(sizeof(float) * 3 * 0);
		SizeInfo[dtGridMetaDataSize::DETAILTRIS_SIZE] += dtAlign4(sizeof(unsigned char) * 4 * (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 2);
		SizeInfo[dtGridMetaDataSize::BVTREE_SIZE] += header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode) * gridFloorInfo[i].polyCount * 2) : 0;
		SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE] += 0;

		CountInfo[dtGridMetaDataCount::GROUND_VERTS] += gridFloorInfo[i].vertsCount;
		CountInfo[dtGridMetaDataCount::OFFMESH_VERTS] += 0;
		CountInfo[dtGridMetaDataCount::GROUND_POLY] += gridFloorInfo[i].polyCount;
		CountInfo[dtGridMetaDataCount::OFFMESH_POLY] += 0;
		CountInfo[dtGridMetaDataCount::MAXLINK] += (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 4;
		CountInfo[dtGridMetaDataCount::DETAILMESH] += gridFloorInfo[i].polyCount;
		CountInfo[dtGridMetaDataCount::DETAILTRI] += (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 1) * 2;
		CountInfo[dtGridMetaDataCount::BVNODE] += header->bvNodeCount > 0 ? gridFloorInfo[i].polyCount * 2 : 0;
	}

	for (int i = 0; i < dtGridMetaDataSize::COUNT_SIZE; i++)
	{
		_size += SizeInfo[i];
	}
	return _size;
}

int dtNavMesh::GetInputNavMeshSize(dtMeshHeader* header, 
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE], 
	int CountInfo[dtGridMetaDataCount::COUNT])
{
	int _size = 0;
	SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE] = dtAlign4(sizeof(float) * 3 * ( header->vertCount - header->offMeshConCount * 2));
	SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE] = dtAlign4(sizeof(float) * 3 * header->offMeshConCount * 2);
	SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE] = dtAlign4(sizeof(dtPoly) * header->offMeshBase);
	SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE] = dtAlign4(sizeof(dtPoly) * header->offMeshConCount);
	SizeInfo[dtGridMetaDataSize::LINK_SIZE] = dtAlign4(sizeof(dtLink) * header->maxLinkCount);
	SizeInfo[dtGridMetaDataSize::DETAILMESHES_SIZE] = dtAlign4(sizeof(dtPolyDetail) * header->offMeshBase);
	SizeInfo[dtGridMetaDataSize::DETAILVERTS_SIZE] = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	SizeInfo[dtGridMetaDataSize::DETAILTRIS_SIZE] = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
	SizeInfo[dtGridMetaDataSize::BVTREE_SIZE] = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode)*header->bvNodeCount) : 0;
	SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE] = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);

	CountInfo[dtGridMetaDataCount::GROUND_VERTS] = header->vertCount - header->offMeshConCount * 2;
	CountInfo[dtGridMetaDataCount::OFFMESH_VERTS] = header->offMeshConCount * 2;
	CountInfo[dtGridMetaDataCount::GROUND_POLY] = header->offMeshBase;
	CountInfo[dtGridMetaDataCount::OFFMESH_POLY] = header->offMeshConCount;
	CountInfo[dtGridMetaDataCount::MAXLINK] = header->maxLinkCount;
	CountInfo[dtGridMetaDataCount::DETAILMESH] = header->detailMeshCount;
	CountInfo[dtGridMetaDataCount::DETAILTRI] = header->detailTriCount;
	CountInfo[dtGridMetaDataCount::BVNODE] = header->bvNodeCount;

	for (int i = 0; i < dtGridMetaDataSize::COUNT_SIZE; i++)
	{
		_size += SizeInfo[i];
	}
	return _size;
}
int dtNavMesh::GetGeometryOffMeshLinkSize(dtMeshHeader* header, int storedOffMeshConCount, int offMeshConLinkCount, 
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE], 
	int CountInfo[dtGridMetaDataCount::COUNT])
{
	int _size = 0;
	SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE] = 0;
	SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE] = dtAlign4(sizeof(float) * 3 * storedOffMeshConCount * 2);
	SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE] = 0;
	SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE] = dtAlign4(sizeof(dtPoly) * storedOffMeshConCount);
	SizeInfo[dtGridMetaDataSize::LINK_SIZE] = dtAlign4(sizeof(dtLink) * (offMeshConLinkCount * 2));
	//output[dtGridMetaDataSize::DETAILMESHES_SIZE] = dtAlign4(sizeof(dtPolyDetail) * header->offMeshBase);
	//output[dtGridMetaDataSize::DETAILVERTS_SIZE] = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	//output[dtGridMetaDataSize::DETAILTRIS_SIZE] = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
	//output[dtGridMetaDataSize::BVTREE_SIZE] = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode)*header->bvNodeCount) : 0;
	SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE] = dtAlign4(sizeof(dtOffMeshConnection) * storedOffMeshConCount);

	CountInfo[dtGridMetaDataCount::GROUND_VERTS] = 0;
	CountInfo[dtGridMetaDataCount::OFFMESH_VERTS] = storedOffMeshConCount * 2;
	CountInfo[dtGridMetaDataCount::GROUND_POLY] = 0;
	CountInfo[dtGridMetaDataCount::OFFMESH_POLY] = storedOffMeshConCount;

	CountInfo[dtGridMetaDataCount::MAXLINK] = offMeshConLinkCount * 2;
	CountInfo[dtGridMetaDataCount::DETAILMESH] = 0;
	CountInfo[dtGridMetaDataCount::DETAILTRI] = 0;
	CountInfo[dtGridMetaDataCount::BVNODE] = 0;

	for (int i = 0; i < dtGridMetaDataSize::COUNT_SIZE; i++)
	{
		_size += SizeInfo[i];
	}
	return _size;
}

void dtNavMesh::CookGeometryOffMeshLinkData(dtMeshHeader* header, dtGridOffmesh& gridOffmesh, int& storedOffMeshConCount, int& offMeshConLinkCount)
{
	storedOffMeshConCount = 0;
	offMeshConLinkCount = 0;
	unsigned char* offMeshConClass = gridOffmesh.offMeshConClass;

	for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
	{
		const float* p0 = &gridOffmesh.offMeshConVerts[(i * 2 + 0) * 3];
		const float* p1 = &gridOffmesh.offMeshConVerts[(i * 2 + 1) * 3];
		offMeshConClass[i * 2 + 0] = classifyOffMeshPoint(p0, header->bmin, header->bmax);
		offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, header->bmin, header->bmax);

		// Zero out off-mesh start positions which are not even potentially touching the mesh.
		if (offMeshConClass[i * 2 + 0] == 0xff)
		{
			if (p0[1] < header->bmin[1] || p0[1] > header->bmax[1]) offMeshConClass[i * 2 + 0] = 0;
		}

		// Cound how many links should be allocated for off-mesh connections.
		if (offMeshConClass[i * 2 + 0] == 0xff) offMeshConLinkCount++;
		if (offMeshConClass[i * 2 + 1] == 0xff) offMeshConLinkCount++;
		if (offMeshConClass[i * 2 + 0] == 0xff) storedOffMeshConCount++;
	}
}

int dtNavMesh::GetLadderSize(dtMeshHeader* header, dtStraightLadder* ladder, int ladderCount,
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCount::COUNT])
{
	int _size = 0;
	for (int i = 0; i < ladderCount; i++)
	{
		SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE] += dtAlign4(sizeof(float) * 3 * ladder[i].vertsCount);
		SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE] += dtAlign4(sizeof(float) * 3 * 2 * ladder[i].OffMeshCount);
		SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE] += dtAlign4(sizeof(dtPoly) * ladder[i].polyCount);
		SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE] += dtAlign4(sizeof(dtPoly) * ladder[i].OffMeshCount);;
		SizeInfo[dtGridMetaDataSize::LINK_SIZE] += dtAlign4(sizeof(dtLink) * (4 * ladder[i].polyCount + 2 * ladder[i].OffMeshCount));
		SizeInfo[dtGridMetaDataSize::DETAILMESHES_SIZE] += dtAlign4(sizeof(dtPolyDetail) * ladder[i].polyCount);
		SizeInfo[dtGridMetaDataSize::DETAILVERTS_SIZE] += dtAlign4(sizeof(float) * 3 * 0);
		SizeInfo[dtGridMetaDataSize::DETAILTRIS_SIZE] += dtAlign4(sizeof(unsigned char) * 4 * 2 * ladder[i].polyCount);
		SizeInfo[dtGridMetaDataSize::BVTREE_SIZE] += header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode) * ladder[i].polyCount * 2) : 0;
		SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE] = dtAlign4(sizeof(dtOffMeshConnection) * ladder[i].OffMeshCount);

		CountInfo[dtGridMetaDataCount::GROUND_VERTS] += ladder[i].vertsCount;
		CountInfo[dtGridMetaDataCount::OFFMESH_VERTS] += 2 * ladder[i].OffMeshCount;
		CountInfo[dtGridMetaDataCount::GROUND_POLY] += ladder[i].polyCount;
		CountInfo[dtGridMetaDataCount::OFFMESH_POLY] += ladder[i].OffMeshCount;
		CountInfo[dtGridMetaDataCount::MAXLINK] += 4 * ladder[i].polyCount + 2 * ladder[i].OffMeshCount;
		CountInfo[dtGridMetaDataCount::DETAILMESH] += ladder[i].polyCount;
		CountInfo[dtGridMetaDataCount::DETAILTRI] += ladder[i].polyCount * 2;
		CountInfo[dtGridMetaDataCount::BVNODE] += header->bvNodeCount > 0 ? ladder[i].polyCount * 2 : 0;
	}

	for (int i = 0; i < dtGridMetaDataSize::COUNT_SIZE; i++)
	{
		_size += SizeInfo[i];
	}
	return _size;
}
int dtNavMesh::GetReservedLadderGroundPolySize(dtMeshHeader* header,
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCount::COUNT])
{
	int _size = 0;
	SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE] = dtAlign4(sizeof(float) * 3 * DT_RESERVE_LADDER * 4);
	SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE] = 0;
	SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE] = dtAlign4(sizeof(dtPoly) * DT_RESERVE_LADDER);
	SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE] = 0;
	SizeInfo[dtGridMetaDataSize::LINK_SIZE] = dtAlign4(sizeof(dtLink) * (DT_RESERVE_LADDER * 4));
	SizeInfo[dtGridMetaDataSize::DETAILMESHES_SIZE] = dtAlign4(sizeof(dtPolyDetail) * DT_RESERVE_LADDER);
	SizeInfo[dtGridMetaDataSize::DETAILVERTS_SIZE] = dtAlign4(sizeof(float) * 3 * 0);
	SizeInfo[dtGridMetaDataSize::DETAILTRIS_SIZE] = dtAlign4(sizeof(unsigned char) * 4 * 2 * DT_RESERVE_LADDER);
	SizeInfo[dtGridMetaDataSize::BVTREE_SIZE] = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode) * DT_RESERVE_LADDER * 2) : 0;
	SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE] = 0;

	CountInfo[dtGridMetaDataCount::GROUND_VERTS] = DT_RESERVE_LADDER * 4;
	CountInfo[dtGridMetaDataCount::OFFMESH_VERTS] = 0;
	CountInfo[dtGridMetaDataCount::GROUND_POLY] = DT_RESERVE_LADDER;
	CountInfo[dtGridMetaDataCount::OFFMESH_POLY] = 0;
	CountInfo[dtGridMetaDataCount::MAXLINK] = DT_RESERVE_LADDER * 4;
	CountInfo[dtGridMetaDataCount::DETAILMESH] = DT_RESERVE_LADDER;
	CountInfo[dtGridMetaDataCount::DETAILTRI] = DT_RESERVE_LADDER * 2;
	CountInfo[dtGridMetaDataCount::BVNODE] = header->bvNodeCount > 0 ? DT_RESERVE_LADDER * 2 : 0;

	for (int i = 0; i < dtGridMetaDataSize::COUNT_SIZE; i++)
	{
		_size += SizeInfo[i];
	}
	return _size;
}

int dtNavMesh::GetReservedOffMeshSize(dtMeshHeader* header, 
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE], 
	int CountInfo[dtGridMetaDataCount::COUNT])
{
	int _size = 0;
	SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE] = 0;
	SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE] = dtAlign4(sizeof(float) * 3 * DT_RESERVE_OFFMESH * 2);
	SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE] = 0;
	SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE] = dtAlign4(sizeof(dtPoly) * DT_RESERVE_OFFMESH);
	SizeInfo[dtGridMetaDataSize::LINK_SIZE] = dtAlign4(sizeof(dtLink) * (DT_RESERVE_OFFMESH * 2));
	//output[dtGridMetaDataSize::DETAILMESHES_SIZE] = dtAlign4(sizeof(dtPolyDetail) * header->offMeshBase);
	//output[dtGridMetaDataSize::DETAILVERTS_SIZE] = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	//output[dtGridMetaDataSize::DETAILTRIS_SIZE] = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
	//output[dtGridMetaDataSize::BVTREE_SIZE] = header->bvNodeCount > 0 ? dtAlign4(sizeof(dtBVNode)*header->bvNodeCount) : 0;
	SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE] = dtAlign4(sizeof(dtOffMeshConnection) * DT_RESERVE_OFFMESH);

	CountInfo[dtGridMetaDataCount::GROUND_VERTS] = 0;
	CountInfo[dtGridMetaDataCount::OFFMESH_VERTS] = DT_RESERVE_OFFMESH * 2;
	CountInfo[dtGridMetaDataCount::GROUND_POLY] = 0;
	CountInfo[dtGridMetaDataCount::OFFMESH_POLY] = DT_RESERVE_OFFMESH;
	CountInfo[dtGridMetaDataCount::MAXLINK] = DT_RESERVE_OFFMESH * 2;
	CountInfo[dtGridMetaDataCount::DETAILMESH] = 0;
	CountInfo[dtGridMetaDataCount::DETAILTRI] = 0;
	CountInfo[dtGridMetaDataCount::BVNODE] = 0;

	for (int i = 0; i < dtGridMetaDataSize::COUNT_SIZE; i++)
	{
		_size += SizeInfo[i];
	}
	return _size;
}

int dtNavMesh::GetTotalSize(int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE], 
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	int _totalSize = 0;
	for (int i = 0; i < dtGridMetaDataCategory::CATEGORY_SIZE - 1; i++)
	{
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::VERTS_GROUND_SIZE] += SizeInfo[i][dtGridMetaDataSize::VERTS_GROUND_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::VERTS_OFFMESH_SIZE] += SizeInfo[i][dtGridMetaDataSize::VERTS_OFFMESH_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::POLY_GROUND_SIZE] += SizeInfo[i][dtGridMetaDataSize::POLY_GROUND_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::POLY_OFFMESH_SIZE] += SizeInfo[i][dtGridMetaDataSize::POLY_OFFMESH_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::LINK_SIZE] += SizeInfo[i][dtGridMetaDataSize::LINK_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::DETAILMESHES_SIZE] += SizeInfo[i][dtGridMetaDataSize::DETAILMESHES_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::DETAILVERTS_SIZE] += SizeInfo[i][dtGridMetaDataSize::DETAILVERTS_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::DETAILTRIS_SIZE] += SizeInfo[i][dtGridMetaDataSize::DETAILTRIS_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::BVTREE_SIZE] += SizeInfo[i][dtGridMetaDataSize::BVTREE_SIZE];
		SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::OFFMESHCON_SIZE] += SizeInfo[i][dtGridMetaDataSize::OFFMESHCON_SIZE];

		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS] += CountInfo[i][dtGridMetaDataCount::GROUND_VERTS];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::OFFMESH_VERTS] += CountInfo[i][dtGridMetaDataCount::OFFMESH_VERTS];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY] += CountInfo[i][dtGridMetaDataCount::GROUND_POLY];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::OFFMESH_POLY] += CountInfo[i][dtGridMetaDataCount::OFFMESH_POLY];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::MAXLINK] += CountInfo[i][dtGridMetaDataCount::MAXLINK];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::DETAILMESH] += CountInfo[i][dtGridMetaDataCount::DETAILMESH];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::DETAILTRI] += CountInfo[i][dtGridMetaDataCount::DETAILTRI];
		CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::BVNODE] += CountInfo[i][dtGridMetaDataCount::BVNODE];

	}
	for (int j = 0; j < dtGridMetaDataSize::COUNT_SIZE; j++)
	{
		_totalSize += SizeInfo[dtGridMetaDataCategory::TOTAL][j];
	}
	return _totalSize;
}

void dtNavMesh::UpdateHeader(dtMeshHeader& header,
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	header.vertCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::OFFMESH_VERTS];
	header.polyCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY]
		+ CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::OFFMESH_POLY];

	header.maxLinkCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::MAXLINK];
	header.detailMeshCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::DETAILMESH];
	header.detailTriCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::DETAILTRI];
	header.bvNodeCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::BVNODE];

	header.offMeshBase = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY];
	header.offMeshConCount = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::OFFMESH_POLY];
}

void dtNavMesh::UpdateInputNavMeshDataPointerInfo(dtDataPointerHelper& dataPointerhelper, unsigned char* data,
	int SizeInfo[dtGridMetaDataSize::COUNT_SIZE])
{
	dataPointerhelper.pdata = data;
	dataPointerhelper.pheader = dtGetThenAdvanceBufferPointer<dtMeshHeader>(data, dtAlign4(sizeof(dtMeshHeader)));
	dataPointerhelper.pnavVerts = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	dataPointerhelper.pnavOffMeshVerts = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);
	dataPointerhelper.pnavPolys = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataSize::POLY_GROUND_SIZE]);
	dataPointerhelper.pnavOffMeshPolys = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataSize::POLY_OFFMESH_SIZE]);
	dataPointerhelper.plink = dtGetThenAdvanceBufferPointer<dtLink>(data, SizeInfo[dtGridMetaDataSize::LINK_SIZE]);;
	dataPointerhelper.pnavDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(data, SizeInfo[dtGridMetaDataSize::DETAILMESHES_SIZE]);
	dataPointerhelper.pnavDVerts = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataSize::DETAILVERTS_SIZE]);
	dataPointerhelper.pnavDTris = dtGetThenAdvanceBufferPointer<unsigned char>(data, SizeInfo[dtGridMetaDataSize::DETAILTRIS_SIZE]);
	dataPointerhelper.pnavBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(data, SizeInfo[dtGridMetaDataSize::BVTREE_SIZE]);
	dataPointerhelper.poffMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataSize::OFFMESHCON_SIZE]);
}

void dtNavMesh::UpdateDataPointerInfo(dtDataPointerHelper& dataPointerhelper, unsigned char* data, 
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE])
{
	dataPointerhelper.pdata = data;
	dataPointerhelper.pheader = dtGetThenAdvanceBufferPointer<dtMeshHeader>(data, dtAlign4(sizeof(dtMeshHeader)));

	dataPointerhelper.pnavVerts = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	dataPointerhelper.pnavVertsGridFloor = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	dataPointerhelper.pnavVertsGeometry = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	dataPointerhelper.pnavVertsLadder = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	dataPointerhelper.pnavVertsReservedLadder = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDLADDER][dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	dataPointerhelper.pnavVertsReservedOffMesh = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDOFFMESH][dtGridMetaDataSize::VERTS_GROUND_SIZE]);

	dataPointerhelper.pnavOffMeshVerts = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshVertsGridFloor = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshVertsGeometry = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshVertsLadder = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshVertsReservedLadder = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDLADDER][dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshVertsReservedOffMesh = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDOFFMESH][dtGridMetaDataSize::VERTS_OFFMESH_SIZE]);

	dataPointerhelper.pnavPolys = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::POLY_GROUND_SIZE]);
	dataPointerhelper.pnavPolysGridFloor = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataSize::POLY_GROUND_SIZE]);
	dataPointerhelper.pnavPolysGeometry = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataSize::POLY_GROUND_SIZE]);
	dataPointerhelper.pnavPolysLadder = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataSize::POLY_GROUND_SIZE]);
	dataPointerhelper.pnavPolysReservedLadder = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDLADDER][dtGridMetaDataSize::POLY_GROUND_SIZE]);
	dataPointerhelper.pnavPolysReservedOffMesh = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDOFFMESH][dtGridMetaDataSize::POLY_GROUND_SIZE]);

	dataPointerhelper.pnavOffMeshPolys = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::POLY_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshPolysGridFloor = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataSize::POLY_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshPolysGeometry = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataSize::POLY_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshPolysLadder = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataSize::POLY_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshPolysReservedLadder = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDLADDER][dtGridMetaDataSize::POLY_OFFMESH_SIZE]);
	dataPointerhelper.pnavOffMeshPolysReservedOffMesh = dtGetThenAdvanceBufferPointer<dtPoly>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDOFFMESH][dtGridMetaDataSize::POLY_OFFMESH_SIZE]);

	dataPointerhelper.plink = dtGetThenAdvanceBufferPointer<dtLink>(data, SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::LINK_SIZE]);;
	dataPointerhelper.pnavDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(data, SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::DETAILMESHES_SIZE]);
	dataPointerhelper.pnavDVerts = dtGetThenAdvanceBufferPointer<float>(data, SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::DETAILVERTS_SIZE]);
	dataPointerhelper.pnavDTris = dtGetThenAdvanceBufferPointer<unsigned char>(data, SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::DETAILTRIS_SIZE]);
	dataPointerhelper.pnavBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(data, SizeInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataSize::BVTREE_SIZE]);

	dataPointerhelper.poffMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::OFFMESHCON_SIZE]);
	dataPointerhelper.poffMeshConsGridFloor = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataSize::OFFMESHCON_SIZE]);
	dataPointerhelper.poffMeshConsGeometry = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataSize::OFFMESHCON_SIZE]);
	dataPointerhelper.poffMeshConsLadder = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataSize::OFFMESHCON_SIZE]);
	dataPointerhelper.poffMeshConsReservedLadder = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDLADDER][dtGridMetaDataSize::OFFMESHCON_SIZE]);
	dataPointerhelper.poffMeshConsReservedOffMesh = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(data, SizeInfo[dtGridMetaDataCategory::RESERVEDOFFMESH][dtGridMetaDataSize::OFFMESHCON_SIZE]);
}

void dtNavMesh::SetInputNavMeshData(dtDataPointerHelper& dst, dtDataPointerHelper& src, 
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	// verts
	memcpy(dst.pnavVerts, src.pnavVerts, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::VERTS_GROUND_SIZE]);
	// verts recopy old off mesh info
	for (int i = 0; i < src.pheader->offMeshConCount; ++i)
	{
		const float* _oldv = &src.pnavOffMeshVerts[( i * 2) * 3];
		float* v = &dst.pnavOffMeshVerts[(i * 2) * 3];
		dtVcopy(&v[0], &_oldv[0]);
		dtVcopy(&v[3], &_oldv[3]);
	}

	// poly
	memcpy(dst.pnavPolys, src.pnavPolys, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::POLY_GROUND_SIZE]);
	// poly recopy old off mesh info
	for (int i = 0; i < src.pheader->offMeshConCount; ++i)
	{
		dtPoly* _oldpoly = &src.pnavOffMeshPolys[i];
		dtPoly* _newpoly = &dst.pnavOffMeshPolys[i];
		_newpoly->vertCount = _oldpoly->vertCount;
		_newpoly->verts[0] = (unsigned short)(CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS] + i * 2 + 0);
		_newpoly->verts[1] = (unsigned short)(CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS] + i * 2 + 1);
		_newpoly->flags = _oldpoly->flags;
		_newpoly->areaAndtype = _oldpoly->areaAndtype;
	}

	// detail meshes
	memcpy(dst.pnavDMeshes, src.pnavDMeshes, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::DETAILMESHES_SIZE]);
	memcpy(dst.pnavDVerts, src.pnavDVerts, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::DETAILVERTS_SIZE]);
	memcpy(dst.pnavDTris, src.pnavDTris, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::DETAILTRIS_SIZE]);

	//bvtree 
	memcpy(dst.pnavBvtree, src.pnavBvtree, SizeInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataSize::BVTREE_SIZE]);

	//offmesh
	for (int i = 0; i < src.pheader->offMeshConCount; ++i)
	{
		dtOffMeshConnection* _newcon = &dst.poffMeshCons[i];
		dtOffMeshConnection* _oldcon = &src.poffMeshCons[i];

		_newcon->poly = (unsigned short)(CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY] + i);
		// Copy connection end-points.
		dtVcopy(&_newcon->pos[0], &_oldcon->pos[0]);
		dtVcopy(&_newcon->pos[3], &_oldcon->pos[3]);
		_newcon->rad = _oldcon->rad;
		_newcon->flags = _oldcon->flags;
		_newcon->side = _oldcon->side;
		_newcon->userId = _oldcon->userId;
	}
}
void dtNavMesh::SetGridFloorData(dtDataPointerHelper& dst, dtDataPointerHelper& src, dtGrid* gridFloorInfo, int floorCount, 
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	const int _srcGroundPolyCount = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::GROUND_POLY];
	
	// verts new ground poly's verts
	int n = 0;
	for (int i = 0; i < floorCount; i++)
	{
		for (int j = 0; j < gridFloorInfo[i].vertsCount; j++)
		{
			const float* iv = &gridFloorInfo[i].verts[j * 3];
			float* v = &dst.pnavVertsGridFloor[n * 3];// make sure ground poly is before off mesh poly
			v[0] = iv[0];
			v[1] = iv[1];
			v[2] = iv[2];
			n++;
		}
	}

	// poly new ground poly
	n = 0;
	int _beginPolyVertsXIndex = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::GROUND_VERTS];
	
	for (int i = 0; i < floorCount; i++)
	{
		int _currentPolyVertsXIndex = _beginPolyVertsXIndex;

		for (int j = 0; j < gridFloorInfo[i].polyCount; j++)
		{
			dtPoly* p = &dst.pnavPolysGridFloor[n];
			p->vertCount = 4;
			p->flags = 1;
			p->setArea(0);
			p->setType(DT_POLYTYPE_GROUND);

			p->verts[0] = _currentPolyVertsXIndex + 0;
			p->verts[1] = _currentPolyVertsXIndex + 1;
			p->verts[2] = _currentPolyVertsXIndex + 1 + DT_grid_count_plusone;
			p->verts[3] = _currentPolyVertsXIndex + 0 + DT_grid_count_plusone;
			n++;
			_currentPolyVertsXIndex++;

			if (0 == n % (DT_grid_count_plusone - 1))
			{
				_currentPolyVertsXIndex++;
			}
		}
		_beginPolyVertsXIndex += gridFloorInfo[i].vertsCount;
	}

	for (int i = 0; i < n; ++i)
	{
		dtPoly* p = &dst.pnavPolys[_srcGroundPolyCount + i];

		for (int j = 0; j < 4; ++j)
		{
			if (3 == j)
			{
				if (0 == i % (DT_grid_count_plusone - 1))
				{
					p->neis[j] = 0;
				}
				else
				{
					p->neis[j] = _srcGroundPolyCount + i - 1 + 1;
				}
			}


			if (1 == j)
			{
				if (DT_grid_count_plusone - 2 == i % (DT_grid_count_plusone - 1))
				{
					p->neis[j] = 0;
				}
				else
				{
					p->neis[j] = _srcGroundPolyCount + i + 1 + 1;
				}
			}

			if (0 == j)
			{
				if (i < DT_grid_count_plusone - 1)
				{
					p->neis[j] = 0;
				}
				else
				{
					p->neis[j] = _srcGroundPolyCount + i - (DT_grid_count_plusone - 1) + 1;
				}
			}

			if (2 == j)
			{
				if (i >= (DT_grid_count_plusone - 1) * (DT_grid_count_plusone - 2))
				{
					p->neis[j] = 0;
				}
				else
				{
					p->neis[j] = _srcGroundPolyCount + i + (DT_grid_count_plusone - 1) + 1;
				}
			}
		}
	}

	// detail meshes, Create dummy detail mesh by triangulating polys.
	int tbase = src.pnavDMeshes[_srcGroundPolyCount - 1].triBase + src.pnavDMeshes[_srcGroundPolyCount - 1].triCount;
	for (int i = 0; i < CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::GROUND_POLY]; ++i)
	{
		dtPolyDetail& dtl = dst.pnavDMeshes[_srcGroundPolyCount + i];
		const int nv = dst.pnavPolysGridFloor[i].vertCount;
		dtl.vertBase = 0;
		dtl.vertCount = 0;
		dtl.triBase = (unsigned int)tbase;
		dtl.triCount = (unsigned char)(nv - 2);
		// Triangulate polygon (local indices).
		for (int j = 2; j < nv; ++j)
		{
			unsigned char* t = &dst.pnavDTris[tbase * 4];
			t[0] = 0;
			t[1] = (unsigned char)(j - 1);
			t[2] = (unsigned char)j;
			// Bit for each edge that belongs to poly boundary.
			t[3] = (1 << 2);
			if (j == 2) t[3] |= (1 << 0);
			if (j == nv - 1) t[3] |= (1 << 4);
			tbase++;
		}
	}
	int _bvCount = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::BVNODE];
	//bvtree 
	CreateGridBVTree(dst.pheader->bmin, dst.pheader->bvQuantFactor, 
		dst.pnavVerts, (void*)&dst.pnavPolysGridFloor[0], n,
		(void*)&dst.pnavBvtree[_bvCount], _srcGroundPolyCount, 0);
}

void dtNavMesh::SetLadderData(dtDataPointerHelper& dst, dtDataPointerHelper& src, dtStraightLadder* ladder, int ladderCount,
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	// verts new ground poly's verts
	int n = 0, m = 0;
	for (int i = 0; i < ladderCount; i++)
	{
		for (int j = 0; j < ladder[i].vertsCount; j++)
		{
			const float* iv = &ladder[i].verts[j * 3];
			float* v = &dst.pnavVertsLadder[n * 3];// make sure ground poly is before off mesh poly
			v[0] = iv[0];
			v[1] = iv[1];
			v[2] = iv[2];
			n++;
		}

		for (int k = 0; k < ladder[i].OffMeshCount; ++k)
		{
			float* v = &dst.pnavOffMeshVertsLadder[(m * 2) * 3];
			v[0] = ladder->baseX;
			v[1] = ladder->baseY;
			v[2] = ladder->baseZ;

			v[3] = ladder->baseX;
			v[4] = ladder->baseY + ladder->height;
			v[5] = ladder->baseZ;
			
			m++;
		}
	}

	// poly new ground poly
	int _girdVertsXIndex = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::GROUND_VERTS];

	int _OffMeshVertsXIndex = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::OFFMESH_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::OFFMESH_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::OFFMESH_VERTS];

	const int _polyIndex = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::GROUND_POLY]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::GROUND_POLY]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::GROUND_POLY];

	const int _OffMeshpolyIndex = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY]
		+ CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::OFFMESH_POLY]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::OFFMESH_POLY]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::OFFMESH_POLY];
	n = 0;
	for (int i = 0; i < ladderCount; i++)
	{
		for (int j = 0; j < ladder[i].polyCount; j++)
		{
			dtPoly* p = &dst.pnavPolysLadder[n];
			p->vertCount = 4;
			p->flags = 1;
			p->setArea(0);
			p->setType(DT_POLYTYPE_GROUND);

			int _gridZindex = n * 4;
			int _currentPolyVertsXIndex = _gridZindex;

			p->verts[0] = _currentPolyVertsXIndex + 0;
			p->verts[1] = _currentPolyVertsXIndex + 1;
			p->verts[2] = _currentPolyVertsXIndex + 3;
			p->verts[3] = _currentPolyVertsXIndex + 2;
			n++;
		}
	}

	for (int i = 0; i < n; ++i)
	{
		dtPoly* p = &dst.pnavPolysLadder[i];
		for (int j = 0; j < 4; ++j)
		{
			p->neis[j] = DT_EXT_LINK;
		}
	}

	m = 0;
	for (int i = 0; i < ladderCount; i++)
	{
		for (int k = 0; k < ladder[i].OffMeshCount; ++k)
		{
			dtPoly* _newpoly = &dst.pnavOffMeshPolysLadder[m];
			_newpoly->vertCount = 2;
			_newpoly->verts[0] = (unsigned short)(_OffMeshVertsXIndex + m * 2 + 0);
			_newpoly->verts[1] = (unsigned short)(_OffMeshVertsXIndex + m * 2 + 1);
			_newpoly->flags = 8;
			_newpoly->areaAndtype = 5;
		}

		for (int q = 0; q < ladder[i].OffMeshCount; ++q)
		{
			dtOffMeshConnection* con = &dst.poffMeshConsLadder[m];
			con->poly = (unsigned short)(_OffMeshpolyIndex + m);
			// Copy connection end-points.
			const float* endPts = &dst.pnavOffMeshVertsLadder[(m * 2) * 3];
			dtVcopy(&con->pos[0], &endPts[0]);
			dtVcopy(&con->pos[3], &endPts[3]);
			con->rad = 0.6f;
			con->flags = DT_OFFMESH_CON_BIDIR;
			con->side = classifyOffMeshPoint(&endPts[3], dst.pheader->bmin, dst.pheader->bmax);;
			con->userId = 10000;
		}
		m++;
	}

	int _bvCount = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::BVNODE]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::BVNODE];
	CreateGridBVTree(dst.pheader->bmin, dst.pheader->bvQuantFactor, 
		dst.pnavVerts, (void*)&dst.pnavPolysLadder[0], n, 
		(void*)&dst.pnavBvtree[_bvCount], _polyIndex, 0);

}

void dtNavMesh::SetGeometryOffMeshLinkData(dtDataPointerHelper& dst, dtDataPointerHelper& src, dtGridOffmesh& gridOffmesh, 
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	int _geometryVertsIndex = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::OFFMESH_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::OFFMESH_VERTS];

	int _geometryPolyIndex = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY]
		+ CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::OFFMESH_POLY]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::OFFMESH_POLY];

	// verts new Off-mesh link vertices.
	int n = 0;
	for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (gridOffmesh.offMeshConClass[i * 2 + 0] == 0xff)
		{
			const float* linkv = &gridOffmesh.offMeshConVerts[i * 2 * 3];
			float* v = &dst.pnavOffMeshVertsGeometry[n * 2 * 3];
			dtVcopy(&v[0], &linkv[0]);
			dtVcopy(&v[3], &linkv[3]);
			n++;
		}
	}

	// poly new Off-mesh link poly.
	n = 0;
	for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (gridOffmesh.offMeshConClass[i * 2 + 0] == 0xff)
		{
			dtPoly* p = &dst.pnavOffMeshPolysGeometry[n];
			p->vertCount = 2;
			p->verts[0] = (unsigned short)(_geometryVertsIndex + n * 2 + 0);
			p->verts[1] = (unsigned short)(_geometryVertsIndex + n * 2 + 1);
			p->flags = gridOffmesh.offMeshConFlags[i];
			p->setArea(gridOffmesh.offMeshConAreas[i]);
			p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
			n++;
		}
	}

	// Store Off-Mesh connections.
	n = 0;
	for (int i = 0; i < gridOffmesh.offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (gridOffmesh.offMeshConClass[i * 2 + 0] == 0xff)
		{
			dtOffMeshConnection* con = &dst.poffMeshConsGeometry[n];
			con->poly = (unsigned short)(_geometryPolyIndex + n);
			// Copy connection end-points.
			const float* endPts = &gridOffmesh.offMeshConVerts[i * 2 * 3];
			dtVcopy(&con->pos[0], &endPts[0]);
			dtVcopy(&con->pos[3], &endPts[3]);
			con->rad = gridOffmesh.offMeshConRad[i];
			con->flags = gridOffmesh.offMeshConDir[i] ? DT_OFFMESH_CON_BIDIR : 0;
			con->side = gridOffmesh.offMeshConClass[i * 2 + 1];
			if (gridOffmesh.offMeshConUserID)
				con->userId = gridOffmesh.offMeshConUserID[i];
			n++;
		}
	}
}

void dtNavMesh::SetReservedLadderData(dtDataPointerHelper& dst, dtDataPointerHelper& src,
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	int _reservedVertsIndex = CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::GROUND_VERTS];

	// verts reserved Off-mesh link vertices.
	// default to 0

	// poly 
	for (int i = 0; i < DT_RESERVE_LADDER; ++i)
	{
		dtPoly* p = &dst.pnavPolysReservedLadder[i];
		p->vertCount = 0;
		p->verts[0] = (unsigned short)(_reservedVertsIndex + i * 4 + 0);
		p->verts[1] = (unsigned short)(_reservedVertsIndex + i * 4 + 1);
		p->verts[2] = (unsigned short)(_reservedVertsIndex + i * 4 + 3);
		p->verts[3] = (unsigned short)(_reservedVertsIndex + i * 4 + 2);
		p->flags = 1;
		p->setArea(0);
		p->setType(DT_POLYTYPE_GROUND);
		p->firstLink = DT_NULL_LINK;
	}
	
}
void dtNavMesh::SetReservedOffMeshData(dtDataPointerHelper& dst, dtDataPointerHelper& src, 
	int SizeInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataSize::COUNT_SIZE],
	int CountInfo[dtGridMetaDataCategory::CATEGORY_SIZE][dtGridMetaDataCount::COUNT])
{
	int _reservedVertsIndex = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_VERTS]
		+ CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::OFFMESH_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::OFFMESH_VERTS]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::OFFMESH_VERTS]
		+ CountInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataCount::OFFMESH_VERTS];

	int _reservedPolyIndex = CountInfo[dtGridMetaDataCategory::TOTAL][dtGridMetaDataCount::GROUND_POLY]
		+ CountInfo[dtGridMetaDataCategory::INPUTNAVMESH][dtGridMetaDataCount::OFFMESH_POLY]
		+ CountInfo[dtGridMetaDataCategory::GRIDFLOOR][dtGridMetaDataCount::OFFMESH_POLY]
		+ CountInfo[dtGridMetaDataCategory::GEOMETRYOFFMESHLINK][dtGridMetaDataCount::OFFMESH_POLY]
		+ CountInfo[dtGridMetaDataCategory::LADDER][dtGridMetaDataCount::OFFMESH_VERTS];

	// verts reserved Off-mesh link vertices.
	// default to 0

	// poly reserve Off-mesh link poly.
	for (int i = 0; i < DT_RESERVE_OFFMESH; ++i)
	{
		dtPoly* p = &dst.pnavOffMeshPolysReservedOffMesh[i];
		p->vertCount = 0;
		p->verts[0] = (unsigned short)(_reservedVertsIndex + i * 2 + 0);
		p->verts[1] = (unsigned short)(_reservedVertsIndex + i * 2 + 1);
		p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
		p->firstLink = DT_NULL_LINK;
	}

	// reserve off-mesh connections
	for (int i = 0; i < DT_RESERVE_OFFMESH; ++i)
	{
		dtOffMeshConnection* con = &dst.poffMeshConsReservedOffMesh[i];
		con->poly = (unsigned short)(_reservedPolyIndex + i);
		con->pos[0] = con->pos[1] = con->pos[2] = con->pos[3] = con->pos[4] = con->pos[5] = 0.f;
		con->rad = 0.f;
		con->flags = DT_OFFMESH_CON_BIDIR;
		con->side = 0xff;
	}
}