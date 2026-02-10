//----------------------------------------------------------------------------------------------------
// BVH.cpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#include "Game/Gameplay/Convex.hpp"
#include "Game/Gameplay/BVH.hpp"

#include "Engine/Math/RaycastUtils.hpp"

#include <cfloat>

//----------------------------------------------------------------------------------------------------
static int IntPow(int x, unsigned int p)
{
	if (p == 0) return 1;
	if (p == 1) return x;

	int tmp = IntPow(x, p / 2);
	if (p % 2 == 0) return tmp * tmp;
	return x * tmp * tmp;
}

//----------------------------------------------------------------------------------------------------
void AABB2Tree::BuildTree(std::vector<Convex2*> const& convexArray, int numOfRecursive, AABB2 const& totalBounds)
{
	m_nodes.clear();

	int numOfNodes = 0;
	for (int i = 0; i < numOfRecursive; ++i)
	{
		numOfNodes += IntPow(2, i);
	}
	m_nodes.resize(numOfNodes);
	if (numOfNodes == 0)
	{
		return;
	}
	m_nodes[0].m_containingConvex = convexArray;

	int sumK = 0;
	for (int i = 0; i < numOfRecursive; ++i)
	{
		if (i == 0)
		{
			m_nodes[sumK].m_bounds = totalBounds;
			++sumK;
		}
		else
		{
			int numOfJInLevel = IntPow(2, i);
			if (i == numOfRecursive - 1)
			{
				m_startOfLastLevel = sumK;
			}
			for (int j = 0; j < numOfJInLevel; ++j)
			{
				int parentIndex = GetParentIndex(sumK);
				AABB2 const& parentBounds = m_nodes[parentIndex].m_bounds;
				bool isLeftChild = (sumK == parentIndex * 2 + 1);
				bool isVerticalSplit = (i % 2 == 1);

				if (isVerticalSplit)
				{
					float xPivot = (parentBounds.m_maxs.x + parentBounds.m_mins.x) * 0.5f;
					for (auto convex : m_nodes[parentIndex].m_containingConvex)
					{
						bool goesLeft = convex->m_boundingDiscCenter.x < xPivot;
						if (isLeftChild == goesLeft)
						{
							m_nodes[sumK].m_containingConvex.push_back(convex);
						}
					}
				}
				else
				{
					float yPivot = (parentBounds.m_maxs.y + parentBounds.m_mins.y) * 0.5f;
					for (auto convex : m_nodes[parentIndex].m_containingConvex)
					{
						bool goesTop = convex->m_boundingDiscCenter.y >= yPivot;
						if (isLeftChild == goesTop)
						{
							m_nodes[sumK].m_containingConvex.push_back(convex);
						}
					}
				}

				// Compute tight AABB from contained convex vertices
				if (!m_nodes[sumK].m_containingConvex.empty())
				{
					float minX = FLT_MAX, maxX = -FLT_MAX;
					float minY = FLT_MAX, maxY = -FLT_MAX;
					for (auto convex : m_nodes[sumK].m_containingConvex)
					{
						for (auto const& vert : convex->m_convexPoly.GetVertexArray())
						{
							if (vert.x > maxX) maxX = vert.x;
							if (vert.x < minX) minX = vert.x;
							if (vert.y < minY) minY = vert.y;
							if (vert.y > maxY) maxY = vert.y;
						}
					}
					m_nodes[sumK].m_bounds = AABB2(Vec2(minX, minY), Vec2(maxX, maxY));
				}
				else
				{
					m_nodes[sumK].m_bounds = AABB2(Vec2(-1.f, -1.f), Vec2(0.f, 0.f));
				}
				++sumK;
			}
		}
	}
}

//----------------------------------------------------------------------------------------------------
static bool RayHitsAABB2D(Vec2 const& startPos, Vec2 const& forwardVec, float maxDist, AABB2 const& bounds)
{
	RaycastResult2D result = RaycastVsAABB2D(startPos, forwardVec, maxDist, bounds.m_mins, bounds.m_maxs);
	return result.m_didImpact;
}

//----------------------------------------------------------------------------------------------------
void AABB2Tree::SolveRayResult(Vec2 const& startPos, Vec2 const& forwardVec, float maxDist, std::vector<Convex2*>& out_latentRes)
{
	int ptr = 0;
	while (ptr < static_cast<int>(m_nodes.size()))
	{
		if (RayHitsAABB2D(startPos, forwardVec, maxDist, m_nodes[ptr].m_bounds))
		{
			if (ptr >= m_startOfLastLevel)
			{
				// Leaf node: collect convexes
				for (auto convex : m_nodes[ptr].m_containingConvex)
				{
					out_latentRes.push_back(convex);
				}
				// Backtrack to next unvisited sibling
				while (ptr % 2 == 0 && ptr != 0)
				{
					ptr = GetParentIndex(ptr);
				}
				if (ptr == 0) break;
				++ptr;
			}
			else
			{
				// Internal node: descend to left child
				int child = ptr * 2 + 1;
				if (child >= static_cast<int>(m_nodes.size()))
				{
					while (ptr % 2 == 0 && ptr != 0)
					{
						ptr = GetParentIndex(ptr);
					}
					if (ptr == 0) break;
					++ptr;
				}
				else
				{
					ptr = child;
				}
			}
		}
		else
		{
			// Ray misses this node: backtrack
			while (ptr % 2 == 0 && ptr != 0)
			{
				ptr = GetParentIndex(ptr);
			}
			if (ptr == 0) break;
			++ptr;
		}
	}
}

//----------------------------------------------------------------------------------------------------
int AABB2Tree::GetParentIndex(int index)
{
	if (index % 2 == 0)
	{
		return (index >> 1) - 1;
	}
	return index >> 1;
}
