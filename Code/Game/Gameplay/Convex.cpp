//----------------------------------------------------------------------------------------------------
// Convex.cpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#include "Game/Gameplay/Convex.hpp"
#include "Engine/Math/RaycastUtils.hpp"
#include "Engine/Math/MathUtils.hpp"
#include <float.h>

//----------------------------------------------------------------------------------------------------
// Default Constructor
//----------------------------------------------------------------------------------------------------
Convex2::Convex2()
	: m_convexPoly(ConvexPoly2({Vec2(), Vec2()}))
{
}

//----------------------------------------------------------------------------------------------------
// Constructor from ConvexPoly2
//----------------------------------------------------------------------------------------------------
Convex2::Convex2(ConvexPoly2 const& convexPoly2)
	: m_convexPoly(convexPoly2)
	, m_convexHull(convexPoly2)
{
}

//----------------------------------------------------------------------------------------------------
// Constructor from ConvexHull2
//----------------------------------------------------------------------------------------------------
Convex2::Convex2(ConvexHull2 const& convexHull2)
	: m_convexHull(convexHull2)
	, m_convexPoly(convexHull2)
{
}

//----------------------------------------------------------------------------------------------------
// Constructor from vertex list
//----------------------------------------------------------------------------------------------------
Convex2::Convex2(std::vector<Vec2> const& vertices)
	: m_convexPoly(ConvexPoly2(vertices))
	, m_convexHull(ConvexHull2(m_convexPoly))
{
}

//----------------------------------------------------------------------------------------------------
// Translate - Move convex shape by offset
//----------------------------------------------------------------------------------------------------
void Convex2::Translate(Vec2 const& offset)
{
	m_convexHull.Translate(offset);
	m_convexPoly.Translate(offset);
	m_boundingAABB.Translate(offset);
	m_boundingDiscCenter += offset;
}

//----------------------------------------------------------------------------------------------------
// Rotate - Rotate convex shape around reference point
//----------------------------------------------------------------------------------------------------
void Convex2::Rotate(float degrees, Vec2 const& refPoint)
{
	// Rotate bounding disc center
	m_boundingDiscCenter -= refPoint;
	m_boundingDiscCenter.RotateDegrees(degrees);
	m_boundingDiscCenter += refPoint;

	// Rotate both representations
	m_convexHull.Rotate(degrees, refPoint);
	m_convexPoly.Rotate(degrees, refPoint);

	// Rebuild AABB since rotation changes axis-aligned bounds
	RebuildBoundingBox();
}

//----------------------------------------------------------------------------------------------------
// Scale - Scale convex shape around reference point
//----------------------------------------------------------------------------------------------------
void Convex2::Scale(float scaleFactor, Vec2 const& refPoint)
{
	// Calculate actual scale factor relative to current scale
	float actualFactor = (m_scale + scaleFactor) / m_scale;
	m_scale += scaleFactor;

	// Scale bounding disc
	m_boundingRadius *= actualFactor;
	m_boundingDiscCenter -= refPoint;
	m_boundingDiscCenter *= actualFactor;
	m_boundingDiscCenter += refPoint;

	// Scale both representations
	m_convexHull.Scale(actualFactor, refPoint);
	m_convexPoly.Scale(actualFactor, refPoint);

	// Rebuild AABB since scaling changes bounds
	RebuildBoundingBox();
}

//----------------------------------------------------------------------------------------------------
// RebuildBoundingBox - Recalculate AABB from current vertices
//----------------------------------------------------------------------------------------------------
void Convex2::RebuildBoundingBox()
{
	float minX = FLT_MAX;
	float minY = FLT_MAX;
	float maxX = -FLT_MAX;
	float maxY = -FLT_MAX;

	// Find min/max coordinates from all vertices
	for (auto const& vert : m_convexPoly.GetVertexArray())
	{
		if (vert.x < minX)
		{
			minX = vert.x;
		}
		if (vert.x > maxX)
		{
			maxX = vert.x;
		}
		if (vert.y < minY)
		{
			minY = vert.y;
		}
		if (vert.y > maxY)
		{
			maxY = vert.y;
		}
	}

	m_boundingAABB = AABB2(Vec2(minX, minY), Vec2(maxX, maxY));
}

//----------------------------------------------------------------------------------------------------
// IsPointInside - Test if a point is inside the convex polygon
//----------------------------------------------------------------------------------------------------
bool Convex2::IsPointInside(Vec2 const& point) const
{
	return IsPointInsideConvexHull2D(point, m_convexHull);
}

//----------------------------------------------------------------------------------------------------
// RayCastVsConvex2D - Raycast against convex polygon with optional optimizations
//----------------------------------------------------------------------------------------------------
bool Convex2::RayCastVsConvex2D(RaycastResult2D& out_rayCastRes, Vec2 const& startPos, Vec2 const& forwardNormal, float maxDist, bool discRejection, bool boxRejection)
{
	// Optional: Broad-phase disc rejection
	if (discRejection)
	{
		RaycastResult2D discResult = RaycastVsDisc2D(startPos, forwardNormal, maxDist, m_boundingDiscCenter, m_boundingRadius);
		if (discResult.m_didImpact)
		{
			// Disc hit, proceed to narrow-phase test
			out_rayCastRes = RaycastVsConvexHull2D(startPos, forwardNormal, maxDist, m_convexHull);
			return out_rayCastRes.m_didImpact;
		}
		// Disc miss, early exit
		out_rayCastRes.m_didImpact = false;
		return false;
	}
	// Optional: Broad-phase AABB rejection
	else if (boxRejection)
	{
		Vec2 aabb2Mins = m_boundingAABB.m_mins;
		Vec2 aabb2Maxs = m_boundingAABB.m_maxs;
		RaycastResult2D aabbResult = RaycastVsAABB2D(startPos, forwardNormal, maxDist, aabb2Mins, aabb2Maxs);
		if (aabbResult.m_didImpact)
		{
			// AABB hit, proceed to narrow-phase test
			out_rayCastRes = RaycastVsConvexHull2D(startPos, forwardNormal, maxDist, m_convexHull);
			return out_rayCastRes.m_didImpact;
		}
		// AABB miss, early exit
		out_rayCastRes.m_didImpact = false;
		return false;
	}

	// No optimization, direct narrow-phase test
	out_rayCastRes = RaycastVsConvexHull2D(startPos, forwardNormal, maxDist, m_convexHull);
	return out_rayCastRes.m_didImpact;
}
