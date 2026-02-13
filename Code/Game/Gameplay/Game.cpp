//----------------------------------------------------------------------------------------------------
// Game.cpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#include "Game/Gameplay/Game.hpp"
//----------------------------------------------------------------------------------------------------
#include "Game/Framework/App.hpp"
#include "Game/Framework/GameCommon.hpp"
#include "Game/Gameplay/Convex.hpp"
#include "Game/Gameplay/BVH.hpp"
#include "Game/Gameplay/QuadTree.hpp"
//----------------------------------------------------------------------------------------------------
#include "Engine/Audio/AudioSystem.hpp"
#include "Engine/Core/Clock.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/LogSubsystem.hpp"
#include "Engine/Input/InputSystem.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/RandomNumberGenerator.hpp"
#include "Engine/Platform/Window.hpp"
#include "Engine/Renderer/DebugRenderSystem.hpp"
#include "Engine/Renderer/VertexUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/RaycastUtils.hpp"
//----------------------------------------------------------------------------------------------------
#include <algorithm>
#include <cfloat>

//----------------------------------------------------------------------------------------------------
// Constants
//----------------------------------------------------------------------------------------------------
constexpr float WORLD_SIZE_X = 200.f;
constexpr float WORLD_SIZE_Y = 100.f;
constexpr float MIN_CONVEX_RADIUS = 2.f;
constexpr float MAX_CONVEX_RADIUS = 8.f;
constexpr int   INITIAL_CONVEX_COUNT = 8;

//----------------------------------------------------------------------------------------------------
Game::Game()
{
    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(Game)(start)");

    g_eventSystem->SubscribeEventCallbackFunction("OnGameStateChanged", OnGameStateChanged);

    m_screenCamera = new Camera();
    m_worldCamera = new Camera();

    Vec2 const bottomLeft     = Vec2::ZERO;
    Vec2 const screenTopRight = Window::s_mainWindow->GetClientDimensions();

    m_screenCamera->SetOrthoGraphicView(bottomLeft, screenTopRight);
    m_screenCamera->SetNormalizedViewport(AABB2::ZERO_TO_ONE);
    m_worldCamera->SetOrthoGraphicView(Vec2::ZERO, Vec2(WORLD_SIZE_X, WORLD_SIZE_Y));
    m_worldCamera->SetNormalizedViewport(AABB2::ZERO_TO_ONE);

    m_gameClock = new Clock(Clock::GetSystemClock());

    // Spawn initial random convexes
    for (int i = 0; i < INITIAL_CONVEX_COUNT; ++i)
    {
        Vec2 randomPos = Vec2(
            g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_X),
            g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_Y)
        );
        Convex2* convex = CreateRandomConvex(randomPos, MIN_CONVEX_RADIUS, MAX_CONVEX_RADIUS);
        m_convexes.push_back(convex);
    }

    // Initialize ray with random start/end points
    m_rayStart = Vec2(
        g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_X),
        g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_Y)
    );
    m_rayEnd = Vec2(
        g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_X),
        g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_Y)
    );

    // Build spatial acceleration structures for initial convexes
    RebuildAllTrees();

    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(Game)(end)");
}

//----------------------------------------------------------------------------------------------------
Game::~Game()
{
    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(~Game)(start)");

    // Clean up convexes
    for (Convex2* convex : m_convexes)
    {
        delete convex;
    }
    m_convexes.clear();

    GAME_SAFE_RELEASE(m_screenCamera);

    g_eventSystem->UnsubscribeEventCallbackFunction("OnGameStateChanged", OnGameStateChanged);

    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(~Game)(end)");
}

//----------------------------------------------------------------------------------------------------
void Game::Update()
{
    Vec2 const      screenTopLeft = m_screenCamera->GetOrthographicTopLeft();
    float constexpr textHeight    = 15.f;
    int             lineIndex     = 1;

    DebugAddScreenText(Stringf("Time: %.2f FPS: %.2f Scale: %.1f", m_gameClock->GetTotalSeconds(), 1.f / m_gameClock->GetDeltaSeconds(), m_gameClock->GetTimeScale()), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f);
    ++lineIndex;

    DebugAddScreenText(Stringf("LMB/RMB=RayStart/End, W/R=Rotate, L/K=Scale, F1=Discs, F3=BVH, F4=AABB, F2=DrawMode, F8=Randomize"), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f);
    ++lineIndex;

    DebugAddScreenText(Stringf("%d convex shapes (Y/U to double/halve); T=Test with %d random rays (M/N to double/halve)", static_cast<int>(m_convexes.size()), m_numOfRandomRays), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f);
    ++lineIndex;

    if (m_avgDist != 0.f)
    {
        DebugAddScreenText(Stringf("%d Rays Vs. %d objects: avg dist %.3f", m_numOfRandomRays, static_cast<int>(m_convexes.size()), m_avgDist), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f, Rgba8::YELLOW, Rgba8::YELLOW);
        ++lineIndex;

        DebugAddScreenText(Stringf("No Opt: %.2fms  Disc: %.2fms  AABB: %.2fms", m_lastRayTestNormalTime, m_lastRayTestDiscRejectionTime, m_lastRayTestAABBRejectionTime), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f, Rgba8::YELLOW, Rgba8::YELLOW);
        ++lineIndex;

        DebugAddScreenText(Stringf("QuadTree: %.2fms  BVH: %.2fms", m_lastRayTestSymmetricTreeTime, m_lastRayTestAABBTreeTime), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f, Rgba8::YELLOW, Rgba8::YELLOW);
        ++lineIndex;
    }

    UpdateGame();
    UpdateTime();
    UpdateWindow();
}

//----------------------------------------------------------------------------------------------------
void Game::Render() const
{
    //-Start-of-Screen-Camera-------------------------------------------------------------------------
    g_renderer->BeginCamera(*m_screenCamera);

    if (IsAttractState())
    {
        RenderAttract();
    }
    else if (IsGameState())
    {
        DebugRenderScreen(*m_screenCamera);
    }

    g_renderer->EndCamera(*m_screenCamera);
    //-End-of-Screen-Camera---------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------
    //-Start-of-World-Camera--------------------------------------------------------------------------
    g_renderer->BeginCamera(*m_worldCamera);

    if (IsGameState())
    {
        RenderGame();
    }

    g_renderer->EndCamera(*m_worldCamera);
    //-End-of-World-Camera----------------------------------------------------------------------------
}

//----------------------------------------------------------------------------------------------------
/// @brief Get current game state
/// @return current game state
///
eGameState Game::GetGameState() const
{
    return m_gameState;
}

//----------------------------------------------------------------------------------------------------
///
/// 1. Set current game state to new game state.
/// 2. Fire the OnGameStateChanged event to all subscribers.
///
/// @param newState new game state for current game state to change to.
///
void Game::SetGameState(eGameState const newState)
{
    if (newState == m_gameState) return;

    EventArgs args;

    if (newState == eGameState::ATTRACT) args.SetValue("OnGameStateChanged", "ATTRACT");
    else if (newState == eGameState::GAME) args.SetValue("OnGameStateChanged", "GAME");

    m_gameState = newState;

    g_eventSystem->FireEvent("OnGameStateChanged", args);
}

//----------------------------------------------------------------------------------------------------
///
/// @return true if current game state is ATTRACT.
//----------------------------------------------------------------------------------------------------
bool Game::IsAttractState() const
{
    return m_gameState == eGameState::ATTRACT;
}

//----------------------------------------------------------------------------------------------------
///
/// @return true if current game state is GAME.
//----------------------------------------------------------------------------------------------------
bool Game::IsGameState() const
{
    return m_gameState == eGameState::GAME;
}

//----------------------------------------------------------------------------------------------------
/// @brief Event call back handler when changing game state.
/// @param args Event arguments.
/// 1. ATTRACT
/// 2. GAME
/// @return true to allow event propagation to other subscribers, false to stop propagation.
STATIC bool Game::OnGameStateChanged(EventArgs& args)
{
    String const newState = args.GetValue("OnGameStateChanged", "DEFAULT");

    if (newState == "ATTRACT")
    {
        SoundID const clickSound = g_audio->CreateOrGetSound("Data/Audio/TestSound.mp3", eAudioSystemSoundDimension::Sound2D);
        g_audio->StartSound(clickSound);
    }
    else if (newState == "GAME")
    {
        SoundID const clickSound = g_audio->CreateOrGetSound("Data/Audio/TestSound.mp3", eAudioSystemSoundDimension::Sound2D);
        g_audio->StartSound(clickSound, false, 1.f, 0.f, 0.5f);
    }

    return true;
}

//----------------------------------------------------------------------------------------------------
void Game::UpdateGame()
{
    if (IsAttractState())
    {
        if (g_input->WasKeyJustPressed(KEYCODE_ESC))
        {
            App::RequestQuit();
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_SPACE))
        {
            SetGameState(eGameState::GAME);
        }
    }
    else if (IsGameState())
    {
        // Get cursor position for rotation/scaling
        Vec2 cursorUV = g_window->GetNormalizedMouseUV();
        Vec2 cursorPos = m_worldCamera->GetCursorWorldPosition(cursorUV);
        float deltaSeconds = (float)m_gameClock->GetDeltaSeconds();

        // Handle object scaling
        if (m_hoveringConvex && g_input->IsKeyDown('L'))
        {
            m_hoveringConvex->Scale(1.f * deltaSeconds, cursorPos);
            RebuildAllTrees();
        }
        if (m_hoveringConvex && g_input->IsKeyDown('K'))
        {
            m_hoveringConvex->Scale(-1.f * deltaSeconds, cursorPos);
            RebuildAllTrees();
        }

        // Handle object rotation
        if (m_hoveringConvex && g_input->IsKeyDown('W'))
        {
            m_hoveringConvex->Rotate(90.f * deltaSeconds, cursorPos);
            RebuildAllTrees();
        }
        if (m_hoveringConvex && g_input->IsKeyDown('R'))
        {
            m_hoveringConvex->Rotate(-90.f * deltaSeconds, cursorPos);
            RebuildAllTrees();
        }

        // Handle object dragging
        if (m_hoveringConvex && g_input->WasKeyJustPressed(KEYCODE_LEFT_MOUSE))
        {
            m_isDragging = true;
        }

        if (m_isDragging && m_hoveringConvex && g_input->IsKeyDown(KEYCODE_LEFT_MOUSE))
        {
            Vec2 delta = cursorPos - m_cursorPrevPos;
            m_hoveringConvex->Translate(delta);
            m_cursorPrevPos = cursorPos;
            RebuildAllTrees();
        }

        if (g_input->WasKeyJustReleased(KEYCODE_LEFT_MOUSE))
        {
            m_isDragging = false;
        }

        // Update hover detection (skipped during drag for sticky focus)
        UpdateHoverDetection();

        // Update ray endpoints via mouse buttons (only when not hovering a convex)
        if (!m_hoveringConvex)
        {
            if (g_input->IsKeyDown(KEYCODE_LEFT_MOUSE))
            {
                m_rayStart = cursorPos;
            }
            if (g_input->IsKeyDown(KEYCODE_RIGHT_MOUSE))
            {
                m_rayEnd = cursorPos;
            }
        }

        if (g_input->WasKeyJustPressed(KEYCODE_ESC))
        {
            SetGameState(eGameState::ATTRACT);
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_F8))
        {
            // Re-randomize all shapes, keeping current count
            int numShapes = static_cast<int>(m_convexes.size());
            ClearScene();
            m_seed += 1;
            for (int i = 0; i < numShapes; ++i)
            {
                Vec2 randomPos = Vec2(
                    g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_X),
                    g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_Y)
                );
                Convex2* convex = CreateRandomConvex(randomPos, MIN_CONVEX_RADIUS, MAX_CONVEX_RADIUS);
                m_convexes.push_back(convex);
            }
            RebuildAllTrees();
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_F1))
        {
            m_showBoundingDiscs = !m_showBoundingDiscs;
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_F2))
        {
            m_drawEdgesMode = !m_drawEdgesMode;
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_F3))
        {
            m_debugDrawBVHMode = !m_debugDrawBVHMode;
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_F4))
        {
            m_showSpatialStructure = !m_showSpatialStructure;
        }
        else if (g_input->WasKeyJustPressed('C'))
        {
            // Spawn convex at mouse position
            Vec2 mouseUV = g_window->GetNormalizedMouseUV();
            Vec2 worldPos = m_worldCamera->GetCursorWorldPosition(mouseUV);
            Convex2* convex = CreateRandomConvex(worldPos, MIN_CONVEX_RADIUS, MAX_CONVEX_RADIUS);
            m_convexes.push_back(convex);
        }
        else if (g_input->WasKeyJustPressed('Y'))
        {
            // Double object count (max 2048)
            int numOfShapesToAdd = static_cast<int>(m_convexes.size());
            if (numOfShapesToAdd == 0)
            {
                numOfShapesToAdd = 1;
            }
            if (static_cast<int>(m_convexes.size()) < 2048)
            {
                for (int i = 0; i < numOfShapesToAdd && static_cast<int>(m_convexes.size()) < 2048; ++i)
                {
                    Vec2 randomPos = Vec2(
                        g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_X),
                        g_rng->RollRandomFloatInRange(0.f, WORLD_SIZE_Y)
                    );
                    Convex2* convex = CreateRandomConvex(randomPos, MIN_CONVEX_RADIUS, MAX_CONVEX_RADIUS);
                    m_convexes.push_back(convex);
                }
                RebuildAllTrees();
            }
        }
        else if (g_input->WasKeyJustPressed('U'))
        {
            // Halve object count (min 1)
            int numOfShapesToRemove = static_cast<int>(m_convexes.size()) / 2;
            if (static_cast<int>(m_convexes.size()) == 1)
            {
                numOfShapesToRemove = 1;
            }
            for (int i = 0; i < numOfShapesToRemove; ++i)
            {
                if (m_convexes.back() == m_hoveringConvex)
                {
                    m_hoveringConvex = nullptr;
                }
                delete m_convexes.back();
                m_convexes.pop_back();
            }
            RebuildAllTrees();
        }
        else if (g_input->WasKeyJustPressed('M'))
        {
            // Double raycast count (max 134217728)
            if (m_numOfRandomRays < 134217728)
            {
                m_numOfRandomRays *= 2;
                if (m_numOfRandomRays > 134217728)
                {
                    m_numOfRandomRays = 134217728;
                }
            }
        }
        else if (g_input->WasKeyJustPressed('N'))
        {
            // Halve raycast count (min 1)
            m_numOfRandomRays /= 2;
            if (m_numOfRandomRays < 1)
            {
                m_numOfRandomRays = 1;
            }
        }
        else if (g_input->WasKeyJustPressed('T'))
        {
            TestRays();
        }
    }
}

//----------------------------------------------------------------------------------------------------
void Game::UpdateTime() const
{
    if (g_input->WasKeyJustPressed(KEYCODE_P))
    {
        m_gameClock->TogglePause();
    }
    else if (g_input->WasKeyJustPressed(KEYCODE_O))
    {
        m_gameClock->StepSingleFrame();
    }
    else if (g_input->IsKeyDown(KEYCODE_T))
    {
        m_gameClock->SetTimeScale(0.1f);
    }
    else if (g_input->WasKeyJustReleased(KEYCODE_T))
    {
        m_gameClock->SetTimeScale(1.f);
    }
}

//----------------------------------------------------------------------------------------------------
void Game::UpdateWindow() const
{
    if (IsAttractState())
    {
        if (g_input->WasKeyJustPressed(KEYCODE_R))
        {
            Window::s_mainWindow->SetWindowType(eWindowType::FULLSCREEN_STRETCH);
            Window::s_mainWindow->ReconfigureWindow();
        }
    }
}

//----------------------------------------------------------------------------------------------------
///
/// @brief Render a simple outlined disc 2D in ATTRACT state.
//
void Game::RenderAttract() const
{
    Vec2 const clientDimensions = Window::s_mainWindow->GetClientDimensions();

    VertexList_PCU verts;

    AddVertsForDisc2D(verts, Vec2(clientDimensions.x * 0.5f, clientDimensions.y * 0.5f), 300.f, 10.f, Rgba8::YELLOW);

    g_renderer->SetModelConstants();
    g_renderer->SetBlendMode(eBlendMode::OPAQUE);
    g_renderer->SetRasterizerMode(eRasterizerMode::SOLID_CULL_BACK);
    g_renderer->SetSamplerMode(eSamplerMode::BILINEAR_CLAMP);
    g_renderer->SetDepthMode(eDepthMode::DISABLED);
    g_renderer->BindTexture(nullptr);
    g_renderer->BindShader(nullptr);
    g_renderer->DrawVertexArray(verts);
}

//----------------------------------------------------------------------------------------------------
///
/// @brief Render convex polygons in GAME state.
//
void Game::RenderGame() const
{
    VertexList_PCU verts;

    if (m_drawEdgesMode)
    {
        // Mode B (F2 on): Thick edges first, then opaque fill (composite concave appearance)
        // Pass 1: All non-hovered edges
        for (Convex2 const* convex : m_convexes)
        {
            if (convex == m_hoveringConvex) continue;
            AddVertsForConvexPolyEdges(verts, convex->m_convexPoly, 0.8f, Rgba8(0, 0, 153));
        }
        // Pass 2: All non-hovered fills (drawn on top of edges)
        for (Convex2 const* convex : m_convexes)
        {
            if (convex == m_hoveringConvex) continue;
            AddVertsForConvexPoly2D(verts, convex->m_convexPoly, Rgba8(153, 204, 255));
        }
        // Pass 3: Hovered convex on top
        if (m_hoveringConvex)
        {
            AddVertsForConvexPoly2D(verts, m_hoveringConvex->m_convexPoly, Rgba8(255, 255, 153));
            AddVertsForConvexPolyEdges(verts, m_hoveringConvex->m_convexPoly, 0.8f, Rgba8(255, 153, 0));
        }
    }
    else
    {
        // Mode A (F2 off): Translucent fill first, then opaque edges
        // Pass 1: All non-hovered fills
        for (Convex2 const* convex : m_convexes)
        {
            if (convex == m_hoveringConvex) continue;
            AddVertsForConvexPoly2D(verts, convex->m_convexPoly, Rgba8(204, 229, 255, 128));
        }
        // Pass 2: All non-hovered edges
        for (Convex2 const* convex : m_convexes)
        {
            if (convex == m_hoveringConvex) continue;
            AddVertsForConvexPolyEdges(verts, convex->m_convexPoly, 0.5f, Rgba8(0, 0, 153));
        }
        // Pass 3: Hovered convex on top
        if (m_hoveringConvex)
        {
            AddVertsForConvexPoly2D(verts, m_hoveringConvex->m_convexPoly, Rgba8(255, 255, 153, 128));
            AddVertsForConvexPolyEdges(verts, m_hoveringConvex->m_convexPoly, 0.5f, Rgba8(255, 153, 0));
        }
    }

    // Debug visualization: bounding discs (F1)
    if (m_showBoundingDiscs)
    {
        for (Convex2 const* convex : m_convexes)
        {
            AddVertsForDisc2D(verts, convex->m_boundingDiscCenter, convex->m_boundingRadius, 0.3f, Rgba8(0, 255, 0, 128));
        }
    }

    // Debug visualization: per-object bounding volumes (F4)
    if (m_showSpatialStructure)
    {
        for (Convex2 const* convex : m_convexes)
        {
            DebugDrawRing(convex->m_boundingDiscCenter, convex->m_boundingRadius, 0.3f, Rgba8(100, 100, 100, 160));
            AABB2 const& box = convex->m_boundingAABB;
            DebugDrawLine(box.m_mins, Vec2(box.m_mins.x, box.m_maxs.y), 0.3f, Rgba8(100, 100, 100, 160));
            DebugDrawLine(Vec2(box.m_mins.x, box.m_maxs.y), box.m_maxs, 0.3f, Rgba8(100, 100, 100, 160));
            DebugDrawLine(Vec2(box.m_maxs.x, box.m_mins.y), box.m_maxs, 0.3f, Rgba8(100, 100, 100, 160));
            DebugDrawLine(Vec2(box.m_maxs.x, box.m_mins.y), box.m_mins, 0.3f, Rgba8(100, 100, 100, 160));
        }
    }

    // Debug visualization: BVH tree node bounds (F3)
    if (m_debugDrawBVHMode)
    {
        for (auto const& node : m_AABB2Tree.m_nodes)
        {
            AABB2 const& box = node.m_bounds;
            DebugDrawLine(box.m_mins, Vec2(box.m_mins.x, box.m_maxs.y), 0.3f, Rgba8(100, 100, 100, 160));
            DebugDrawLine(Vec2(box.m_mins.x, box.m_maxs.y), box.m_maxs, 0.3f, Rgba8(100, 100, 100, 160));
            DebugDrawLine(Vec2(box.m_maxs.x, box.m_mins.y), box.m_maxs, 0.3f, Rgba8(100, 100, 100, 160));
            DebugDrawLine(Vec2(box.m_maxs.x, box.m_mins.y), box.m_mins, 0.3f, Rgba8(100, 100, 100, 160));
        }
    }

    // Single object mode: infinite plane lines are drawn inside RenderRaycast
    
    // Raycast visualization (always visible)
    RenderRaycast(verts);
    
    g_renderer->SetModelConstants();
    g_renderer->SetBlendMode(eBlendMode::OPAQUE);
    g_renderer->SetRasterizerMode(eRasterizerMode::SOLID_CULL_BACK);
    g_renderer->SetSamplerMode(eSamplerMode::BILINEAR_CLAMP);
    g_renderer->SetDepthMode(eDepthMode::DISABLED);
    g_renderer->BindTexture(nullptr);
    g_renderer->BindShader(nullptr);
    g_renderer->DrawVertexArray(verts);
}

//----------------------------------------------------------------------------------------------------
void Game::AddVertsForConvexPolyEdges(std::vector<Vertex_PCU>& verts, ConvexPoly2 const& convexPoly2, float thickness, Rgba8 const& color) const
{
    std::vector<Vec2> const& points = convexPoly2.GetVertexArray();
    int numPoints = static_cast<int>(points.size());

    for (int i = 0; i < numPoints; ++i)
    {
        Vec2 const& start = points[i];
        Vec2 const& end = points[(i + 1) % numPoints];
        AddVertsForLineSegment2D(verts, start, end, thickness, false, color);
    }
}

//----------------------------------------------------------------------------------------------------
void Game::RenderRaycast(std::vector<Vertex_PCU>& verts) const
{
    float constexpr rayThickness    = 0.3f;
    float constexpr normalLength    = 3.f;
    float constexpr normalThickness = 0.3f;
    float constexpr normalArrowSize = 1.f;

    // Compute ray direction and length dynamically from start/end points
    float rayMaxLength = (m_rayEnd - m_rayStart).GetLength();
    if (rayMaxLength < 0.001f)
    {
        return; // Degenerate ray
    }
    Vec2 rayNormal = (m_rayEnd - m_rayStart) / rayMaxLength;

    // Find closest raycast hit across all convexes
    RaycastResult2D closestResult;
    closestResult.m_didImpact = false;

    for (Convex2* convex : m_convexes)
    {
        RaycastResult2D result;
        bool didHit = convex->RayCastVsConvex2D(result, m_rayStart, rayNormal, rayMaxLength);
        if (didHit && (!closestResult.m_didImpact || result.m_impactLength < closestResult.m_impactLength))
        {
            closestResult = result;
        }
    }

    // Always draw the full ray arrow (black, behind everything)
    AddVertsForArrow2D(verts, m_rayStart, m_rayEnd, normalArrowSize, rayThickness, Rgba8(0, 0, 0));

    if (closestResult.m_didImpact)
    {
        Vec2 const& impactPos = closestResult.m_impactPosition;
        Vec2 const& impactNormal = closestResult.m_impactNormal;

        // Green segment from start to impact (drawn on top of black arrow)
        AddVertsForLineSegment2D(verts, m_rayStart, impactPos, rayThickness, false, Rgba8(0, 255, 0));

        // Red impact normal arrow
        Vec2 normalEnd = impactPos + impactNormal * normalLength;
        AddVertsForArrow2D(verts, impactPos, normalEnd, normalArrowSize, normalThickness, Rgba8(255, 0, 0));
    }

    // Single object mode: draw infinite lines for each bounding plane, color-coded by status/rejection
    if (m_convexes.size() == 1)
    {
        for (auto const& plane : m_convexes[0]->m_convexHull.m_boundingPlanes)
        {
            float altitude = plane.GetAltitudeOfPoint(m_rayStart);
            float NdotF    = DotProduct2D(rayNormal, plane.m_normal);

            Vec2 vert1 = plane.GetOriginPoint() + 1000.f * plane.m_normal.GetRotated90Degrees();
            Vec2 vert2 = plane.GetOriginPoint() - 1000.f * plane.m_normal.GetRotated90Degrees();

            if (altitude > 0.f && NdotF < 0.f)
            {
                // Entry candidate: ray outside, facing toward plane (magenta)
                AddVertsForLineSegment2D(verts, vert1, vert2, 0.2f, false, Rgba8(255, 0, 255));

                // Draw filled disc at ray-plane intersection
                float SdotN = DotProduct2D(m_rayStart, plane.m_normal);
                float dist  = (plane.m_distanceFromOrigin - SdotN) / NdotF;
                AddVertsForDisc2D(verts, m_rayStart + dist * rayNormal, 0.5f, Rgba8(255, 0, 255));
            }
            else if (altitude > 0.f && NdotF >= 0.f)
            {
                // Rejected: ray outside, facing away/parallel — can never enter (red)
                AddVertsForLineSegment2D(verts, vert1, vert2, 0.2f, false, Rgba8(255, 0, 0));
            }
            else if (altitude <= 0.f && NdotF < 0.f)
            {
                // Already inside, moving deeper — past this boundary (green)
                AddVertsForLineSegment2D(verts, vert1, vert2, 0.2f, false, Rgba8(0, 255, 0));
            }
            else
            {
                // Exit candidate: ray inside, facing away from plane (cyan)
                AddVertsForLineSegment2D(verts, vert1, vert2, 0.2f, false, Rgba8(0, 255, 255));

                // Draw filled disc at ray-plane exit intersection
                float SdotN = DotProduct2D(m_rayStart, plane.m_normal);
                float dist  = (plane.m_distanceFromOrigin - SdotN) / NdotF;
                AddVertsForDisc2D(verts, m_rayStart + dist * rayNormal, 0.5f, Rgba8(0, 255, 255));
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------
Convex2* Game::CreateRandomConvex(Vec2 const& center, float minRadius, float maxRadius)
{
    // Generate random number of sides (3-8)
    int numSides = g_rng->RollRandomIntInRange(3, 8);

    // Generate random radius
    float radius = g_rng->RollRandomFloatInRange(minRadius, maxRadius);

    // Generate random angles with variation, then sort to guarantee CCW winding
    float angleStep = 360.f / static_cast<float>(numSides);
    std::vector<float> angles;
    for (int i = 0; i < numSides; ++i)
    {
        float baseAngle = angleStep * static_cast<float>(i);
        float angleVariation = g_rng->RollRandomFloatInRange(-angleStep * 0.3f, angleStep * 0.3f);
        angles.push_back(baseAngle + angleVariation);
    }
    std::sort(angles.begin(), angles.end());

    // Create vertices from sorted angles with uniform radius
    std::vector<Vec2> vertices;
    for (int i = 0; i < numSides; ++i)
    {
        Vec2 vertex = center + Vec2::MakeFromPolarDegrees(angles[i], radius);
        vertices.push_back(vertex);
    }

    // Create ConvexPoly2 from sorted vertices, then construct Convex2
    ConvexPoly2 poly(vertices);
    return new Convex2(poly);
}

//----------------------------------------------------------------------------------------------------
void Game::TestRays()
{
    RebuildAllTrees();

    int numRays = m_numOfRandomRays;

    // Generate random rays
    std::vector<Vec2> rayStartPos(numRays);
    std::vector<Vec2> rayForwardNormal(numRays);
    std::vector<float> rayMaxDist(numRays);

    AABB2 worldBounds(Vec2(0.f, 0.f), Vec2(WORLD_SIZE_X, WORLD_SIZE_Y));
    for (int j = 0; j < numRays; ++j)
    {
        Vec2 p1(g_rng->RollRandomFloatInRange(worldBounds.m_mins.x, worldBounds.m_maxs.x),
                g_rng->RollRandomFloatInRange(worldBounds.m_mins.y, worldBounds.m_maxs.y));
        Vec2 p2(g_rng->RollRandomFloatInRange(worldBounds.m_mins.x, worldBounds.m_maxs.x),
                g_rng->RollRandomFloatInRange(worldBounds.m_mins.y, worldBounds.m_maxs.y));
        rayStartPos[j] = p1;
        Vec2 disp = p2 - p1;
        rayMaxDist[j] = disp.GetLength();
        rayForwardNormal[j] = disp.GetNormalized();
    }

    RaycastResult2D rayRes;
    double startTime = 0.0;
    double endTime = 0.0;
    float sumDist = 0.f;
    int numOfRayHit = 0;
    int correctNumOfRayHit = 0;
    float thisAvgDist = 0.f;

    // Mode 1: No optimization (baseline)
    sumDist = 0.f;
    numOfRayHit = 0;
    startTime = GetCurrentTimeSeconds();
    for (int j = 0; j < numRays; ++j)
    {
        float minDist = FLT_MAX;
        for (int i = 0; i < static_cast<int>(m_convexes.size()); ++i)
        {
            if (m_convexes[i]->RayCastVsConvex2D(rayRes, rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], false, false))
            {
                if (rayRes.m_impactLength < minDist)
                {
                    minDist = rayRes.m_impactLength;
                }
            }
        }
        if (minDist != FLT_MAX)
        {
            sumDist += minDist;
            ++numOfRayHit;
        }
    }
    endTime = GetCurrentTimeSeconds();
    m_avgDist = sumDist / static_cast<float>(numOfRayHit);
    correctNumOfRayHit = numOfRayHit;
    m_lastRayTestNormalTime = static_cast<float>((endTime - startTime) * 1000.0);

    // Mode 2: Disc rejection
    sumDist = 0.f;
    numOfRayHit = 0;
    startTime = GetCurrentTimeSeconds();
    for (int j = 0; j < numRays; ++j)
    {
        float minDist = FLT_MAX;
        for (int i = 0; i < static_cast<int>(m_convexes.size()); ++i)
        {
            if (m_convexes[i]->RayCastVsConvex2D(rayRes, rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], true, false))
            {
                if (rayRes.m_impactLength < minDist)
                {
                    minDist = rayRes.m_impactLength;
                }
            }
        }
        if (minDist != FLT_MAX)
        {
            sumDist += minDist;
            ++numOfRayHit;
        }
    }
    endTime = GetCurrentTimeSeconds();
    thisAvgDist = sumDist / static_cast<float>(numOfRayHit);
    GUARANTEE_OR_DIE(numOfRayHit == correctNumOfRayHit, "Disc rejection mismatch");
    m_lastRayTestDiscRejectionTime = static_cast<float>((endTime - startTime) * 1000.0);

    // Mode 3: AABB rejection
    sumDist = 0.f;
    numOfRayHit = 0;
    startTime = GetCurrentTimeSeconds();
    for (int j = 0; j < numRays; ++j)
    {
        float minDist = FLT_MAX;
        for (int i = 0; i < static_cast<int>(m_convexes.size()); ++i)
        {
            if (m_convexes[i]->RayCastVsConvex2D(rayRes, rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], true, true))
            {
                if (rayRes.m_impactLength < minDist)
                {
                    minDist = rayRes.m_impactLength;
                }
            }
        }
        if (minDist != FLT_MAX)
        {
            sumDist += minDist;
            ++numOfRayHit;
        }
    }
    endTime = GetCurrentTimeSeconds();
    thisAvgDist = sumDist / static_cast<float>(numOfRayHit);
    GUARANTEE_OR_DIE(numOfRayHit == correctNumOfRayHit, "AABB rejection mismatch");
    m_lastRayTestAABBRejectionTime = static_cast<float>((endTime - startTime) * 1000.0);

    // Mode 4: QuadTree
    sumDist = 0.f;
    numOfRayHit = 0;
    startTime = GetCurrentTimeSeconds();
    for (int j = 0; j < numRays; ++j)
    {
        float minDist = FLT_MAX;
        std::vector<Convex2*> candidates;
        m_symQuadTree.SolveRayResult(rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], m_convexes, candidates);
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i)
        {
            if (candidates[i]->RayCastVsConvex2D(rayRes, rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], true, true))
            {
                if (rayRes.m_impactLength < minDist)
                {
                    minDist = rayRes.m_impactLength;
                }
            }
        }
        if (minDist != FLT_MAX)
        {
            sumDist += minDist;
            ++numOfRayHit;
        }
    }
    endTime = GetCurrentTimeSeconds();
    thisAvgDist = sumDist / static_cast<float>(numOfRayHit);
    GUARANTEE_OR_DIE(numOfRayHit == correctNumOfRayHit, "QuadTree mismatch");
    m_lastRayTestSymmetricTreeTime = static_cast<float>((endTime - startTime) * 1000.0);

    // Mode 5: BVH (AABB2Tree)
    sumDist = 0.f;
    numOfRayHit = 0;
    startTime = GetCurrentTimeSeconds();
    for (int j = 0; j < numRays; ++j)
    {
        float minDist = FLT_MAX;
        std::vector<Convex2*> candidates;
        m_AABB2Tree.SolveRayResult(rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], candidates);
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i)
        {
            if (candidates[i]->RayCastVsConvex2D(rayRes, rayStartPos[j], rayForwardNormal[j], rayMaxDist[j], true, true))
            {
                if (rayRes.m_impactLength < minDist)
                {
                    minDist = rayRes.m_impactLength;
                }
            }
        }
        if (minDist != FLT_MAX)
        {
            sumDist += minDist;
            ++numOfRayHit;
        }
    }
    endTime = GetCurrentTimeSeconds();
    thisAvgDist = sumDist / static_cast<float>(numOfRayHit);
    GUARANTEE_OR_DIE(numOfRayHit == correctNumOfRayHit, "BVH mismatch");
    m_lastRayTestAABBTreeTime = static_cast<float>((endTime - startTime) * 1000.0);
}

//----------------------------------------------------------------------------------------------------
void Game::RebuildAllTrees()
{
    AABB2 totalBounds = AABB2(Vec2(0.f, 0.f), Vec2(WORLD_SIZE_X, WORLD_SIZE_Y));

    int numConvexes = static_cast<int>(m_convexes.size());
    int bvhDepth = 0;
    if (numConvexes > 0)
    {
        bvhDepth = static_cast<int>(log2(static_cast<double>(numConvexes))) - 3;
        if (bvhDepth < 3) bvhDepth = 3;
    }

    m_AABB2Tree.BuildTree(m_convexes, bvhDepth, totalBounds);
    m_symQuadTree.BuildTree(m_convexes, 4, totalBounds);
}

//----------------------------------------------------------------------------------------------------
void Game::ClearScene()
{
    // Delete all convexes
    for (Convex2* convex : m_convexes)
    {
        delete convex;
    }
    m_convexes.clear();

    // Reset interaction state
    m_hoveringConvex = nullptr;
    m_isDragging = false;
}

//----------------------------------------------------------------------------------------------------
void Game::UpdateHoverDetection()
{
    // Skip hover detection during drag (sticky focus)
    if (m_isDragging)
    {
        return;
    }

    // Get cursor position in world coordinates
    Vec2 cursorUV = g_window->GetNormalizedMouseUV();
    Vec2 cursorPos = m_worldCamera->GetCursorWorldPosition(cursorUV);

    // Check for hover from back to front (prioritize recently added objects)
    m_hoveringConvex = nullptr;
    for (int i = static_cast<int>(m_convexes.size()) - 1; i >= 0; --i)
    {
        if (m_convexes[i]->IsPointInside(cursorPos))
        {
            m_hoveringConvex = m_convexes[i];
            break;
        }
    }

    // Update previous cursor position for next frame
    m_cursorPrevPos = cursorPos;
}
