//----------------------------------------------------------------------------------------------------
// Convex.hpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#pragma once
#include "Engine/Math/ConvexHull2.hpp"
// #include "Engine/Math/ConvexPoly2.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/Vec2.hpp"

//----------------------------------------------------------------------------------------------------
// Forward Declarations
//----------------------------------------------------------------------------------------------------
struct RaycastResult2D;

//----------------------------------------------------------------------------------------------------
// Convex2 - 2D Convex Polygon with dual representation
//
// Maintains both plane-based (ConvexHull2) and vertex-based (ConvexPoly2) representations
// for efficient raycasting and rendering. Includes bounding volumes for optimization.
//----------------------------------------------------------------------------------------------------
struct Convex2
{
public:
	//------------------------------------------------------------------------------------------------
	// Constructors
	//------------------------------------------------------------------------------------------------
	Convex2();
	Convex2(ConvexPoly2 const& convexPoly2);
	Convex2(ConvexHull2 const& convexHull2);
	explicit Convex2(std::vector<Vec2> const& vertices);

	//------------------------------------------------------------------------------------------------
	// Query Methods
	//------------------------------------------------------------------------------------------------
	bool IsPointInside(Vec2 const& point) const;
	bool RayCastVsConvex2D(RaycastResult2D& out_rayCastRes, Vec2 const& startPos, Vec2 const& forwardNormal, float maxDist, bool discRejection = true, bool boxRejection = false);

	//------------------------------------------------------------------------------------------------
	// Transform Methods
	//------------------------------------------------------------------------------------------------
	void Translate(Vec2 const& offset);
	void Rotate(float degrees, Vec2 const& refPoint = Vec2(0.f, 0.f));
	void Scale(float scaleFactor, Vec2 const& refPoint = Vec2(0.f, 0.f));
	void RebuildBoundingBox();

	//------------------------------------------------------------------------------------------------
	// Data Members
	//------------------------------------------------------------------------------------------------
	ConvexHull2 m_convexHull;              // Plane-based representation (for raycasting)
	ConvexPoly2 m_convexPoly;              // Vertex-based representation (for rendering)
	AABB2       m_boundingAABB;            // Axis-aligned bounding box
	Vec2        m_boundingDiscCenter;      // Bounding disc center
	float       m_boundingRadius = 0.f;    // Bounding disc radius
	float       m_scale = 1.f;             // Current scale factor
	bool        m_symmetricQuadTreeFlag = false; // For QuadTree deduplication
};
