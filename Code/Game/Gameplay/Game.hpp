//----------------------------------------------------------------------------------------------------
// Game.hpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#pragma once
//----------------------------------------------------------------------------------------------------
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Game/Gameplay/BVH.hpp"
#include "Game/Gameplay/QuadTree.hpp"

struct Rgba8;
struct ConvexPoly2;
struct Vertex_PCU;
struct Vec2;
//-Forward-Declaration--------------------------------------------------------------------------------
class Camera;
class Clock;
struct Convex2;

//----------------------------------------------------------------------------------------------------
enum class eGameState : int8_t
{
    ATTRACT,
    GAME
};

//----------------------------------------------------------------------------------------------------
class Game
{
public:
    //------------------------------------------------------------------------------------------------
    // Construct / Destruct
    //------------------------------------------------------------------------------------------------
    Game();
    ~Game();

    //------------------------------------------------------------------------------------------------
    // Life cycle
    //------------------------------------------------------------------------------------------------
    void Update();
    void Render() const;

    //------------------------------------------------------------------------------------------------
    // Game state
    //------------------------------------------------------------------------------------------------
    eGameState GetGameState() const;
    void       SetGameState(eGameState newState);
    bool       IsAttractState() const;
    bool       IsGameState() const;

private:
    //------------------------------------------------------------------------------------------------
    // Game state
    //------------------------------------------------------------------------------------------------
    static bool OnGameStateChanged(EventArgs& args);

    //------------------------------------------------------------------------------------------------
    // Update
    //------------------------------------------------------------------------------------------------
    void UpdateGame();
    void UpdateTime() const;
    void UpdateWindow() const;

    //------------------------------------------------------------------------------------------------
    // Render
    //------------------------------------------------------------------------------------------------
    void RenderAttract() const;
    void RenderGame() const;

    //------------------------------------------------------------------------------------------------
    // Convex generation
    //------------------------------------------------------------------------------------------------
    static Convex2* CreateRandomConvex(Vec2 const& center, float minRadius, float maxRadius);

    //------------------------------------------------------------------------------------------------
    // Scene management
    //------------------------------------------------------------------------------------------------
    void RebuildAllTrees();
    void ClearScene();

    //------------------------------------------------------------------------------------------------
    // Interaction
    //------------------------------------------------------------------------------------------------
    void UpdateHoverDetection();

    //------------------------------------------------------------------------------------------------
    // Rendering helpers
    //------------------------------------------------------------------------------------------------
    void AddVertsForConvexPolyEdges(std::vector<Vertex_PCU>& verts, ConvexPoly2 const& convexPoly2, float thickness, Rgba8 const& color) const;
    void RenderRaycast(std::vector<Vertex_PCU>& verts) const;
    void TestRays();

    //------------------------------------------------------------------------------------------------
    // Member variables
    //------------------------------------------------------------------------------------------------
    eGameState m_gameState    = eGameState::ATTRACT;
    Camera*    m_screenCamera = nullptr;
    Camera*    m_worldCamera  = nullptr;
    Clock*     m_gameClock    = nullptr;

    // Convex objects
    std::vector<Convex2*> m_convexes;

    // Interaction state
    Convex2* m_hoveringConvex = nullptr;
    Vec2     m_cursorPrevPos;
    bool     m_isDragging    = false;
    bool     m_drawEdgesMode = false;
    bool     m_showBoundingDiscs = false;
    bool     m_showSpatialStructure = false;
    bool     m_debugDrawBVHMode     = false;
    int      m_rayOptimizationMode = 0; // 0=None, 1=Disc, 2=AABB

    // Random generation
    unsigned int m_seed = 1;

    // Raycast testing
    Vec2 m_rayStart;
    Vec2 m_rayEnd;
    int  m_numOfRandomRays = 1024;

    // Performance metrics
    float m_avgDist                      = 0.f;
    float m_lastRayTestNormalTime        = 0.f;
    float m_lastRayTestDiscRejectionTime = 0.f;
    float m_lastRayTestAABBRejectionTime = 0.f;
    float m_lastRayTestSymmetricTreeTime = 0.f;
    float m_lastRayTestAABBTreeTime      = 0.f;

    // Spatial structures
    SymmetricQuadTree m_symQuadTree;
    AABB2Tree         m_AABB2Tree;
};
