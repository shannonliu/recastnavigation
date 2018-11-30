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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

inline bool overlapBounds(const float* amin, const float* amax, const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

inline bool overlapInterval(unsigned short amin, unsigned short amax,
							unsigned short bmin, unsigned short bmax)
{
	if (amax < bmin) return false;
	if (amin > bmax) return false;
	return true;
}


static rcSpan* allocSpan(rcHeightfield& hf)
{
	// If running out of memory, allocate new page and update the freelist.
	if (!hf.freelist || !hf.freelist->next)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* pool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (!pool) return 0;

		// Add the pool into the list of pools.
		pool->next = hf.pools;
		hf.pools = pool;
		// Add new items to the free list.
		rcSpan* freelist = hf.freelist;
		rcSpan* head = &pool->items[0];
		rcSpan* it = &pool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freelist;
			freelist = it;
		}
		while (it != head);
		hf.freelist = it;
	}
	
	// Pop item from in front of the free list.
	rcSpan* it = hf.freelist;
	hf.freelist = hf.freelist->next;
	return it;
}

static void freeSpan(rcHeightfield& hf, rcSpan* ptr)
{
	if (!ptr) return;
	// Add the node in front of the free list.
	ptr->next = hf.freelist;
	hf.freelist = ptr;
}

///  @param hf			高度域
///  @param x			span所在的格子x轴方向的索引
///  @param z			span所在的格子z轴方向的索引
///  @param smin		span在y轴方向格子高度索引的最小值
///  @param smax		span在y轴方向格子高度索引的最大值
///  @param area		形成span的原始三角形是否可以行走的标识
static bool addSpan(rcHeightfield& hf, const int x, const int z,
					const unsigned short smin, const unsigned short smax,
					const unsigned char area, const int flagMergeThr)
{
	
	int idx = x + z*hf.width;
	
	rcSpan* s = allocSpan(hf);//新建一个span
	if (!s)
		return false;
	s->smin = smin;
	s->smax = smax;
	s->area = area;
	s->next = 0;
	
	// Empty cell, add the first span.
	if (!hf.spans[idx])
	{
		hf.spans[idx] = s;
		return true;
	}
	rcSpan* prev = 0;
	rcSpan* cur = hf.spans[idx];
	
	// Insert and merge spans.
	while (cur)
	{
		if (cur->smin > s->smax)//链表中存放的span，按y轴高度，从下往上连起来，因此新的数据如果在当前span下面则直接跳过合并，插入到当前元素之前
		{
			// Current span is further than the new span, break.
			break;
		}
		else if (cur->smax < s->smin)//链表中存放的span，按y轴高度，从下往上连起来，因此新的数据如果在当前span上面则跳过当前span考察下一个span
		{
			// Current span is before the new span advance.
			prev = cur;
			cur = cur->next;
		}
		else
		{
			// Merge spans.
			if (cur->smin < s->smin)
				s->smin = cur->smin;
			if (cur->smax > s->smax)
				s->smax = cur->smax;
			
			// Merge flags.
			if (rcAbs((int)s->smax - (int)cur->smax) <= flagMergeThr)
				s->area = rcMax(s->area, cur->area);
			
			// Remove current span.
			rcSpan* next = cur->next;
			freeSpan(hf, cur);
			if (prev)
				prev->next = next;
			else
				hf.spans[idx] = next;
			cur = next;
		}
	}
	
	// Insert new span.
	if (prev)
	{
		s->next = prev->next;
		prev->next = s;
	}
	else
	{
		s->next = hf.spans[idx];
		hf.spans[idx] = s;
	}

	return true;
}

/// @par
///
/// The span addition can be set to favor flags. If the span is merged to
/// another span and the new @p smax is within @p flagMergeThr units
/// from the existing span, the span flags are merged.
///
/// @see rcHeightfield, rcSpan.
bool rcAddSpan(rcContext* ctx, rcHeightfield& hf, const int x, const int y,
			   const unsigned short smin, const unsigned short smax,
			   const unsigned char area, const int flagMergeThr)
{
	rcAssert(ctx);

	if (!addSpan(hf, x, y, smin, smax, area, flagMergeThr))
	{
		ctx->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

///  @param in			三角形的顶点
///  @param nin			顶点数量3
///  @param out1		输出数据1,面近侧存入out1
///  @param nout1		输出数据1的数量
///  @param out2		输出数据2,面远侧存入out2
///  @param nout2		输出数据2的数量
///  @param plane		axis标识对应的面的坐标
///  @param axis        0、2分别表示对x、z轴进行处理
// divides a convex polygons into two convex polygons on both sides of a line
static void dividePoly(const float* in, int nin,
					  float* out1, int* nout1,
					  float* out2, int* nout2,
					  float plane, int axis)
{
	float d[12];//用于保存每个顶点到axis标识的对应平面的距离，最多支持12个顶点
	for (int i = 0; i < nin; ++i)
		d[i] = plane - in[i*3+axis];

	//循环内只考察当前点，面近侧存入out1,面远侧存入out2,面上同时存入out1out2
	int m = 0, n = 0;
	for (int i = 0, j = nin-1; i < nin; j=i, ++i)// i表示当前顶点的索引，j表示上一个顶点的索引
	{
		bool ina = d[j] >= 0;
		bool inb = d[i] >= 0;
		if (ina != inb)//两个顶点不在同一侧，插值计算交点，并将交点同时存到输出out1和out2
		{
			float s = d[j] / (d[j] - d[i]);
			out1[m*3+0] = in[j*3+0] + (in[i*3+0] - in[j*3+0])*s;
			out1[m*3+1] = in[j*3+1] + (in[i*3+1] - in[j*3+1])*s;
			out1[m*3+2] = in[j*3+2] + (in[i*3+2] - in[j*3+2])*s;
			rcVcopy(out2 + n*3, out1 + m*3);
			m++;//m表示输出out1的数量
			n++;//n表示输出out2的数量
			// add the i'th point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (d[i] > 0)//如果在面近侧，则存入out1
			{
				rcVcopy(out1 + m*3, in + i*3);
				m++;
			}
			else if (d[i] < 0)//如果在面远侧，则存入out2
			{
				rcVcopy(out2 + n*3, in + i*3);
				n++;
			}
		}
		else // same side
		{
			// add the i'th point to the right polygon. Addition is done even for points on the dividing line
			if (d[i] >= 0)//如果在面近侧，则存入out1
			{
				rcVcopy(out1 + m*3, in + i*3);
				m++;
				if (d[i] != 0)
					continue;
			}
			rcVcopy(out2 + n*3, in + i*3);//如果就在面上，则同时也存入out2
			n++;
		}
	}

	*nout1 = m;
	*nout2 = n;
}


/// 体素化一个三角形
///  @param[in]	    v0,v1,v2		三角形三个顶点的坐标
///  @param[in]		areas			该三角形是否可以行走
///  @param[in]		hf				高度场
///  @param[in]		bmin,bmax		高度场的AABB包围盒最小最大坐标
///  @param[in]		cs				高度场在xz平面的格子大小
///  @param[in]		ics				ics倒数
///  @param[in]	    ich				高度场在y轴格子大小的倒数
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& hf,
						 const float* bmin, const float* bmax,
						 const float cs, const float ics, const float ich,
						 const int flagMergeThr)
{
	const int w = hf.width;//x方向格子数
	const int h = hf.height;//z方向格子数
	float tmin[3], tmax[3];
	const float by = bmax[1] - bmin[1];//高度场y轴的高度
	
	// Calculate the bounding box of the triangle.计算三角形的包围盒
	rcVcopy(tmin, v0);
	rcVcopy(tmax, v0);
	rcVmin(tmin, v1);
	rcVmin(tmin, v2);
	rcVmax(tmax, v1);
	rcVmax(tmax, v2);
	
	// If the triangle does not touch the bbox of the heightfield, skip the triagle.
	if (!overlapBounds(bmin, bmax, tmin, tmax))
		return true;
	
	// Calculate the footprint of the triangle on the grid's y-axis
	// 计算三角形包围盒在Z轴方向上所占据格子的范围
	int z0 = (int)((tmin[2] - bmin[2])*ics);
	int z1 = (int)((tmax[2] - bmin[2])*ics);
	z0 = rcClamp(z0, 0, h-1);
	z1 = rcClamp(z1, 0, h-1);
	
	// Clip the triangle into all grid cells it touches.
	float buf[7*3*4];//按21个元素一组分成4份
	float *in = buf, *inrow = buf+7*3, *p1 = inrow+7*3, *p2 = p1+7*3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1*3], v1);
	rcVcopy(&in[2*3], v2);
	int nvrow, nvIn = 3;
	
	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cz = bmin[2] + z*cs;//将三角形包围盒最小值从格子索引转回世界坐标系

		//考虑Z轴方向的一个面cz+cs，对三角形进行切割,面近侧点存入inrow,面远侧点存入p1
		dividePoly(in, nvIn, inrow, &nvrow, p1, &nvIn, cz+cs, 2);
		rcSwap(in, p1);//最终面近侧点存入inrow,面远侧点存入in
		if (nvrow < 3) continue;
		
		// find the horizontal bounds in the row
		// 计算z轴某个平面切割后的顶点在x轴方向上的包围盒
		float minX = inrow[0], maxX = inrow[0];
		for (int i=1; i<nvrow; ++i)
		{
			if (minX > inrow[i*3])	minX = inrow[i*3];
			if (maxX < inrow[i*3])	maxX = inrow[i*3];
		}

		// 计算切割后顶点包围盒在x轴方向上所占据格子的范围
		int x0 = (int)((minX - bmin[0])*ics);
		int x1 = (int)((maxX - bmin[0])*ics);
		x0 = rcClamp(x0, 0, w-1);
		x1 = rcClamp(x1, 0, w-1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = bmin[0] + x*cs;
			//考虑x轴方向的一个面cx+cs，对z轴平面切割后的面近侧点进行再次切割,面近侧点存入p1,面远侧点存入p2
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx+cs, 0);
			rcSwap(inrow, p2);//最终面近侧点存入p1,面远侧点存入inrow

			//至此，z轴面远侧点存入in，x轴面远侧点存入inrow, 两个面共有的近侧点存入p1

			if (nv < 3) continue;
			
			//对x、z轴两个面共有的近侧点p1中的数据进行y轴高度方向的处理，找出y轴的最大最小值，即构成span的原始高度数据
			// Calculate min and max of the span.
			float smin = p1[1], smax = p1[1];
			for (int i = 1; i < nv; ++i)
			{
				smin = rcMin(smin, p1[i*3+1]);
				smax = rcMax(smax, p1[i*3+1]);
			}
			smin -= bmin[1];
			smax -= bmin[1];
			// Skip the span if it is outside the heightfield bbox
			if (smax < 0.0f) continue;
			if (smin > by) continue;
			// Clamp the span to the heightfield bbox.
			if (smin < 0.0f) smin = 0;
			if (smax > by) smax = by;
			
			// Snap the span to the heightfield height grid.
			// 将span原始高度数据转换成高度的格子索引
			unsigned short ismin = (unsigned short)rcClamp((int)floorf(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short ismax = (unsigned short)rcClamp((int)ceilf(smax * ich), (int)ismin+1, RC_SPAN_MAX_HEIGHT);
			
			if (!addSpan(hf, x, z, ismin, ismax, area, flagMergeThr))
				return false;
		}
	}

	return true;
}

/// @par
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
bool rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& solid,
						 const int flagMergeThr)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);

	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	if (!rasterizeTri(v0, v1, v2, area, solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
	{
		ctx->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
///  @param[in]		verts			顶点buf起始地址
///  @param[in]		nv				顶点数量
///  @param[in]		tris			三角形的索引
///  @param[in]		areas			每个三角形是否可行走的标志buf
///  @param[in]		nt				三角形的数量
///  @param[in,out]	solid			高度场
/// @see rcHeightfield
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
						  const int* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		// 对每一个三角形进行体素化处理
		if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
		{
			ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
						  const unsigned short* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
		{
			ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[(i*3+0)*3];
		const float* v1 = &verts[(i*3+1)*3];
		const float* v2 = &verts[(i*3+2)*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
		{
			ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
