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

///  @param hf			�߶���
///  @param x			span���ڵĸ���x�᷽�������
///  @param z			span���ڵĸ���z�᷽�������
///  @param smin		span��y�᷽����Ӹ߶���������Сֵ
///  @param smax		span��y�᷽����Ӹ߶����������ֵ
///  @param area		�γ�span��ԭʼ�������Ƿ�������ߵı�ʶ
static bool addSpan(rcHeightfield& hf, const int x, const int z,
					const unsigned short smin, const unsigned short smax,
					const unsigned char area, const int flagMergeThr)
{
	
	int idx = x + z*hf.width;
	
	rcSpan* s = allocSpan(hf);//�½�һ��span
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
		if (cur->smin > s->smax)//�����д�ŵ�span����y��߶ȣ���������������������µ���������ڵ�ǰspan������ֱ�������ϲ������뵽��ǰԪ��֮ǰ
		{
			// Current span is further than the new span, break.
			break;
		}
		else if (cur->smax < s->smin)//�����д�ŵ�span����y��߶ȣ���������������������µ���������ڵ�ǰspan������������ǰspan������һ��span
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

///  @param in			�����εĶ���
///  @param nin			��������3
///  @param out1		�������1,��������out1
///  @param nout1		�������1������
///  @param out2		�������2,��Զ�����out2
///  @param nout2		�������2������
///  @param plane		axis��ʶ��Ӧ���������
///  @param axis        0��2�ֱ��ʾ��x��z����д���
// divides a convex polygons into two convex polygons on both sides of a line
static void dividePoly(const float* in, int nin,
					  float* out1, int* nout1,
					  float* out2, int* nout2,
					  float plane, int axis)
{
	float d[12];//���ڱ���ÿ�����㵽axis��ʶ�Ķ�Ӧƽ��ľ��룬���֧��12������
	for (int i = 0; i < nin; ++i)
		d[i] = plane - in[i*3+axis];

	//ѭ����ֻ���쵱ǰ�㣬��������out1,��Զ�����out2,����ͬʱ����out1out2
	int m = 0, n = 0;
	for (int i = 0, j = nin-1; i < nin; j=i, ++i)// i��ʾ��ǰ�����������j��ʾ��һ�����������
	{
		bool ina = d[j] >= 0;
		bool inb = d[i] >= 0;
		if (ina != inb)//�������㲻��ͬһ�࣬��ֵ���㽻�㣬��������ͬʱ�浽���out1��out2
		{
			float s = d[j] / (d[j] - d[i]);
			out1[m*3+0] = in[j*3+0] + (in[i*3+0] - in[j*3+0])*s;
			out1[m*3+1] = in[j*3+1] + (in[i*3+1] - in[j*3+1])*s;
			out1[m*3+2] = in[j*3+2] + (in[i*3+2] - in[j*3+2])*s;
			rcVcopy(out2 + n*3, out1 + m*3);
			m++;//m��ʾ���out1������
			n++;//n��ʾ���out2������
			// add the i'th point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (d[i] > 0)//���������࣬�����out1
			{
				rcVcopy(out1 + m*3, in + i*3);
				m++;
			}
			else if (d[i] < 0)//�������Զ�࣬�����out2
			{
				rcVcopy(out2 + n*3, in + i*3);
				n++;
			}
		}
		else // same side
		{
			// add the i'th point to the right polygon. Addition is done even for points on the dividing line
			if (d[i] >= 0)//���������࣬�����out1
			{
				rcVcopy(out1 + m*3, in + i*3);
				m++;
				if (d[i] != 0)
					continue;
			}
			rcVcopy(out2 + n*3, in + i*3);//����������ϣ���ͬʱҲ����out2
			n++;
		}
	}

	*nout1 = m;
	*nout2 = n;
}


/// ���ػ�һ��������
///  @param[in]	    v0,v1,v2		�������������������
///  @param[in]		areas			���������Ƿ��������
///  @param[in]		hf				�߶ȳ�
///  @param[in]		bmin,bmax		�߶ȳ���AABB��Χ����С�������
///  @param[in]		cs				�߶ȳ���xzƽ��ĸ��Ӵ�С
///  @param[in]		ics				ics����
///  @param[in]	    ich				�߶ȳ���y����Ӵ�С�ĵ���
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& hf,
						 const float* bmin, const float* bmax,
						 const float cs, const float ics, const float ich,
						 const int flagMergeThr)
{
	const int w = hf.width;//x���������
	const int h = hf.height;//z���������
	float tmin[3], tmax[3];
	const float by = bmax[1] - bmin[1];//�߶ȳ�y��ĸ߶�
	
	// Calculate the bounding box of the triangle.���������εİ�Χ��
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
	// ���������ΰ�Χ����Z�᷽������ռ�ݸ��ӵķ�Χ
	int z0 = (int)((tmin[2] - bmin[2])*ics);
	int z1 = (int)((tmax[2] - bmin[2])*ics);
	z0 = rcClamp(z0, 0, h-1);
	z1 = rcClamp(z1, 0, h-1);
	
	// Clip the triangle into all grid cells it touches.
	float buf[7*3*4];//��21��Ԫ��һ��ֳ�4��
	float *in = buf, *inrow = buf+7*3, *p1 = inrow+7*3, *p2 = p1+7*3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1*3], v1);
	rcVcopy(&in[2*3], v2);
	int nvrow, nvIn = 3;
	
	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cz = bmin[2] + z*cs;//�������ΰ�Χ����Сֵ�Ӹ�������ת����������ϵ

		//����Z�᷽���һ����cz+cs���������ν����и�,���������inrow,��Զ������p1
		dividePoly(in, nvIn, inrow, &nvrow, p1, &nvIn, cz+cs, 2);
		rcSwap(in, p1);//�������������inrow,��Զ������in
		if (nvrow < 3) continue;
		
		// find the horizontal bounds in the row
		// ����z��ĳ��ƽ���и��Ķ�����x�᷽���ϵİ�Χ��
		float minX = inrow[0], maxX = inrow[0];
		for (int i=1; i<nvrow; ++i)
		{
			if (minX > inrow[i*3])	minX = inrow[i*3];
			if (maxX < inrow[i*3])	maxX = inrow[i*3];
		}

		// �����и�󶥵��Χ����x�᷽������ռ�ݸ��ӵķ�Χ
		int x0 = (int)((minX - bmin[0])*ics);
		int x1 = (int)((maxX - bmin[0])*ics);
		x0 = rcClamp(x0, 0, w-1);
		x1 = rcClamp(x1, 0, w-1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = bmin[0] + x*cs;
			//����x�᷽���һ����cx+cs����z��ƽ���и�������������ٴ��и�,���������p1,��Զ������p2
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx+cs, 0);
			rcSwap(inrow, p2);//�������������p1,��Զ������inrow

			//���ˣ�z����Զ������in��x����Զ������inrow, �����湲�еĽ�������p1

			if (nv < 3) continue;
			
			//��x��z�������湲�еĽ����p1�е����ݽ���y��߶ȷ���Ĵ����ҳ�y��������Сֵ��������span��ԭʼ�߶�����
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
			// ��spanԭʼ�߶�����ת���ɸ߶ȵĸ�������
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
///  @param[in]		verts			����buf��ʼ��ַ
///  @param[in]		nv				��������
///  @param[in]		tris			�����ε�����
///  @param[in]		areas			ÿ���������Ƿ�����ߵı�־buf
///  @param[in]		nt				�����ε�����
///  @param[in,out]	solid			�߶ȳ�
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
		// ��ÿһ�������ν������ػ�����
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
