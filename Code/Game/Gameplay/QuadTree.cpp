//----------------------------------------------------------------------------------------------------
// QuadTree.cpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#include "Game/Gameplay/Convex.hpp"
#include "Game/Gameplay/QuadTree.hpp"

#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/RaycastUtils.hpp"

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
static AABB2 ComputeChildBounds(AABB2 const& parentBounds, int childIndex, int parentIndex)
{
	Vec2 halfDim = parentBounds.GetDimensions() * 0.5f;
	int which = childIndex - (parentIndex * 4 + 1); // 0=LB, 1=RB, 2=LT, 3=RT

	switch (which)
	{
	case 0: // Left-Bottom
		return AABB2(parentBounds.m_mins, parentBounds.m_mins + halfDim);
	case 1: // Right-Bottom
	{
		Vec2 minsPos = parentBounds.m_mins + Vec2(parentBounds.m_maxs.x - parentBounds.m_mins.x, 0.f) * 0.5f;
		return AABB2(minsPos, minsPos + halfDim);
	}
	case 2: // Left-Top
	{
		Vec2 minsPos = parentBounds.m_mins + Vec2(0.f, parentBounds.m_maxs.y - parentBounds.m_mins.y) * 0.5f;
		return AABB2(minsPos, minsPos + halfDim);
	}
	case 3: // Right-Top
	default:
		return AABB2(parentBounds.m_mins + halfDim, parentBounds.m_maxs);
	}
}

//----------------------------------------------------------------------------------------------------
void SymmetricQuadTree::BuildTree(std::vector<Convex2*> const& convexArray, int numOfRecursive, AABB2 const& totalBounds)
{
	m_nodes.clear();

	int numOfNodes = 0;
	for (int i = 0; i < numOfRecursive; ++i)
	{
		numOfNodes += IntPow(4, i);
	}
	m_nodes.resize(numOfNodes);

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
			int numOfJInLevel = IntPow(4, i);
			bool isLastLevel = (i == numOfRecursive - 1);

			for (int j = 0; j < numOfJInLevel; ++j)
			{
				int parentIndex = GetParentIndex(sumK);
				m_nodes[sumK].m_bounds = ComputeChildBounds(m_nodes[parentIndex].m_bounds, sumK, parentIndex);

				// Only assign convexes at the last level (leaf nodes)
				if (isLastLevel)
				{
					for (auto convex : convexArray)
					{
						if (DoAABB2sOverlap2D(convex->m_boundingAABB, m_nodes[sumK].m_bounds))
						{
							m_nodes[sumK].m_containingConvex.push_back(convex);
						}
					}
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
void SymmetricQuadTree::SolveRayResult(Vec2 const& startPos, Vec2 const& forwardVec, float maxDist, std::vector<Convex2*> const& convexArray, std::vector<Convex2*>& out_latentRes)
{
	for (auto convex : convexArray)
	{
		convex->m_symmetricQuadTreeFlag = false;
	}

	int ptr = 0;
	while (ptr < static_cast<int>(m_nodes.size()))
	{
		if (RayHitsAABB2D(startPos, forwardVec, maxDist, m_nodes[ptr].m_bounds))
		{
			if (!m_nodes[ptr].m_containingConvex.empty())
			{
				for (auto convex : m_nodes[ptr].m_containingConvex)
				{
					if (!convex->m_symmetricQuadTreeFlag)
					{
						convex->m_symmetricQuadTreeFlag = true;
						out_latentRes.push_back(convex);
					}
				}
				while (ptr % 4 == 0 && ptr != 0)
				{
					ptr = GetParentIndex(ptr);
				}
				if (ptr == 0) break;
				++ptr;
			}
			else
			{
				int child = GetFirstLBChild(ptr);
				if (child >= static_cast<int>(m_nodes.size()))
				{
					while (ptr % 4 == 0 && ptr != 0)
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
			while (ptr % 4 == 0 && ptr != 0)
			{
				ptr = GetParentIndex(ptr);
			}
			if (ptr == 0) break;
			++ptr;
		}
	}
}

//----------------------------------------------------------------------------------------------------
int SymmetricQuadTree::GetFirstLBChild(int index)
{
	return index * 4 + 1;
}

//----------------------------------------------------------------------------------------------------
int SymmetricQuadTree::GetSecondRBChild(int index)
{
	return index * 4 + 2;
}

//----------------------------------------------------------------------------------------------------
int SymmetricQuadTree::GetThirdLTChild(int index)
{
	return index * 4 + 3;
}

//----------------------------------------------------------------------------------------------------
int SymmetricQuadTree::GetForthRTChild(int index)
{
	return index * 4 + 4;
}

//----------------------------------------------------------------------------------------------------
int SymmetricQuadTree::GetParentIndex(int index)
{
	return (index - 1) / 4;
}
