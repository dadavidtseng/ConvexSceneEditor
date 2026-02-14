//----------------------------------------------------------------------------------------------------
// BVH.hpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#pragma once
//----------------------------------------------------------------------------------------------------
#include "Engine/Math/AABB2.hpp"
//----------------------------------------------------------------------------------------------------
#include <vector>

//----------------------------------------------------------------------------------------------------
struct Convex2;
struct Vec2;

//----------------------------------------------------------------------------------------------------
struct AABB2TreeNode
{
	AABB2                  m_bounds;
	std::vector<Convex2*>  m_containingConvex;
};

//----------------------------------------------------------------------------------------------------
class AABB2Tree
{
public:
	void BuildTree(std::vector<Convex2*> const& convexArray, int numOfRecursive, AABB2 const& totalBounds);
	void SolveRayResult(Vec2 const& startPos, Vec2 const& forwardVec, float maxDist, std::vector<Convex2*>& out_latentRes);

	std::vector<AABB2TreeNode> m_nodes;

	int  GetStartOfLastLevel() const { return m_startOfLastLevel; }
	void SetStartOfLastLevel(int value) { m_startOfLastLevel = value; }

protected:
	int GetParentIndex(int index);
	int m_startOfLastLevel = 0;
};
