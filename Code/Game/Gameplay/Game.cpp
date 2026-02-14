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
#include "Engine/Core/BufferParser.hpp"
#include "Engine/Core/BufferWriter.hpp"
#include "Engine/Core/DevConsole.hpp"
#include "Engine/Core/FileUtils.hpp"
//----------------------------------------------------------------------------------------------------
#include <algorithm>
#include <cfloat>
#include <filesystem>
#include <unordered_map>

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
    g_eventSystem->SubscribeEventCallbackFunction("SaveConvexScene", SaveConvexSceneCommand);
    g_eventSystem->SubscribeEventCallbackFunction("LoadConvexScene", LoadConvexSceneCommand);

    m_screenCamera = new Camera();
    m_worldCamera = new Camera();

    Vec2 const bottomLeft     = Vec2::ZERO;
    Vec2 const screenTopRight = Window::s_mainWindow->GetClientDimensions();

    m_screenCamera->SetOrthoGraphicView(bottomLeft, screenTopRight);
    m_screenCamera->SetNormalizedViewport(AABB2::ZERO_TO_ONE);
    m_worldCamera->SetOrthoGraphicView(Vec2::ZERO, Vec2(WORLD_SIZE_X, WORLD_SIZE_Y));
    m_worldCamera->SetNormalizedViewport(AABB2::ZERO_TO_ONE);

    m_gameClock = new Clock(Clock::GetSystemClock());

    // Validate BufferWriter/BufferParser against Test.binary
    ValidateTestBinary();

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
    g_eventSystem->UnsubscribeEventCallbackFunction("SaveConvexScene", SaveConvexSceneCommand);
    g_eventSystem->UnsubscribeEventCallbackFunction("LoadConvexScene", LoadConvexSceneCommand);

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

    char const* optModeNames[] = {"None", "Disc", "AABB"};
    DebugAddScreenText(Stringf("LMB/RMB=RayStart/End, W/R=Rotate, L/K=Scale, F1=Discs, F3=BVH, F4=AABB, F2=DrawMode, F8=Randomize, F9=Opt(%s)", optModeNames[m_rayOptimizationMode]), screenTopLeft - Vec2(0.f, textHeight * static_cast<float>(lineIndex)), textHeight, Vec2(1, 1), 0.f);
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
STATIC bool Game::SaveConvexSceneCommand(EventArgs& args)
{
    String name = args.GetValue("name", "default");
    g_devConsole->AddLine(DevConsole::INFO_MINOR, Stringf("> SaveConvexScene name=%s", name.c_str()));
    if (g_game->SaveSceneToFile("Data/Scenes/" + name + ".ghcs"))
    {
        g_devConsole->AddLine(DevConsole::INFO_MAJOR, Stringf("Saved scene to Data/Scenes/%s.ghcs", name.c_str()));
    }
    return true;
}

//----------------------------------------------------------------------------------------------------
STATIC bool Game::LoadConvexSceneCommand(EventArgs& args)
{
    String name = args.GetValue("name", "default");
    g_devConsole->AddLine(DevConsole::INFO_MINOR, Stringf("> LoadConvexScene name=%s", name.c_str()));
    if (g_game->LoadSceneFromFile("Data/Scenes/" + name + ".ghcs"))
    {
        g_devConsole->AddLine(DevConsole::INFO_MAJOR, Stringf("Loaded scene from Data/Scenes/%s.ghcs", name.c_str()));
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
            m_sceneModified = true;
            RebuildAllTrees();
        }
        if (m_hoveringConvex && g_input->IsKeyDown('K'))
        {
            m_hoveringConvex->Scale(-1.f * deltaSeconds, cursorPos);
            m_sceneModified = true;
            RebuildAllTrees();
        }

        // Handle object rotation
        if (m_hoveringConvex && g_input->IsKeyDown('W'))
        {
            m_hoveringConvex->Rotate(90.f * deltaSeconds, cursorPos);
            m_sceneModified = true;
            RebuildAllTrees();
        }
        if (m_hoveringConvex && g_input->IsKeyDown('R'))
        {
            m_hoveringConvex->Rotate(-90.f * deltaSeconds, cursorPos);
            m_sceneModified = true;
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
            m_sceneModified = true;
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
            // Reset to default view if a scene was loaded
            if (m_hasLoadedScene)
            {
                m_hasLoadedScene = false;
                m_worldCamera->SetOrthoGraphicView(Vec2::ZERO, Vec2(WORLD_SIZE_X, WORLD_SIZE_Y));
            }
            // Re-randomize all shapes, keeping current count
            int numShapes = static_cast<int>(m_convexes.size());
            ClearScene();
            m_sceneModified = true;
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
        else if (g_input->WasKeyJustPressed(KEYCODE_F9))
        {
            m_rayOptimizationMode = (m_rayOptimizationMode + 1) % 3;
        }
        else if (g_input->WasKeyJustPressed('C'))
        {
            // Spawn convex at mouse position
            Vec2 mouseUV = g_window->GetNormalizedMouseUV();
            Vec2 worldPos = m_worldCamera->GetCursorWorldPosition(mouseUV);
            Convex2* convex = CreateRandomConvex(worldPos, MIN_CONVEX_RADIUS, MAX_CONVEX_RADIUS);
            m_convexes.push_back(convex);
            m_sceneModified = true;
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
                m_sceneModified = true;
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
            m_sceneModified = true;
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

    // Loaded scene bounds outline (white rectangle to delineate scene area from letterbox/pillarbox)
    if (m_hasLoadedScene)
    {
        AABB2 const& sb = m_loadedSceneBounds;
        float constexpr borderThickness = 0.3f;
        Rgba8 const borderColor = Rgba8::WHITE;
        AddVertsForLineSegment2D(verts, sb.m_mins, Vec2(sb.m_maxs.x, sb.m_mins.y), borderThickness, false, borderColor);
        AddVertsForLineSegment2D(verts, Vec2(sb.m_maxs.x, sb.m_mins.y), sb.m_maxs, borderThickness, false, borderColor);
        AddVertsForLineSegment2D(verts, sb.m_maxs, Vec2(sb.m_mins.x, sb.m_maxs.y), borderThickness, false, borderColor);
        AddVertsForLineSegment2D(verts, Vec2(sb.m_mins.x, sb.m_maxs.y), sb.m_mins, borderThickness, false, borderColor);
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
        bool discRejection = (m_rayOptimizationMode == 1);
        bool boxRejection  = (m_rayOptimizationMode == 2);
        bool didHit = convex->RayCastVsConvex2D(result, m_rayStart, rayNormal, rayMaxLength, discRejection, boxRejection);
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

    // Clear preserved chunks from loaded file
    m_preservedChunks.clear();

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

//----------------------------------------------------------------------------------------------------
void Game::ValidateTestBinary()
{
    std::vector<uint8_t> buffer;
    if (!FileReadToBuffer(buffer, "Data/Test.binary"))
    {
        ERROR_RECOVERABLE("Failed to read Data/Test.binary");
        return;
    }

    GUARANTEE_OR_DIE(buffer.size() == 208, "Test.binary should be 208 bytes");

    BufferParser bufParse(buffer);

    // --- First half: Little Endian (bytes 0-103) ---
    {
        bufParse.SetEndianMode(eEndianMode::LITTLE);
        char fourCC0 = bufParse.ParseChar();
        char fourCC1 = bufParse.ParseChar();
        char fourCC2 = bufParse.ParseChar();
        char fourCC3 = bufParse.ParseChar();
        uint8_t version = bufParse.ParseByte();
        uint8_t endianByte = bufParse.ParseByte();
        uint8_t shouldBeFalse = bufParse.ParseByte(); // bool as byte
        uint8_t shouldBeTrue = bufParse.ParseByte();  // bool as byte
        unsigned int largeUint = bufParse.ParseUint32();
        int negativeSeven = bufParse.ParseInt32();
        float oneF = bufParse.ParseFloat();
        double pi = bufParse.ParseDouble();

        std::string helloString, isThisThingOnString;
        bufParse.ParseZeroTerminatedString(helloString);
        bufParse.ParseLengthPrecededString(isThisThingOnString);

        Rgba8 rustColor = bufParse.ParseRgba8();
        uint8_t eight = bufParse.ParseByte();
        // ParseRgb8: 3 bytes (RGB), alpha assumed 255
        uint8_t seashellR = bufParse.ParseByte();
        uint8_t seashellG = bufParse.ParseByte();
        uint8_t seashellB = bufParse.ParseByte();
        Rgba8 seashellColor(seashellR, seashellG, seashellB, 255);
        uint8_t nine = bufParse.ParseByte();
        IntVec2 highDefRes = bufParse.ParseIntVec2();
        Vec2 normal2D = bufParse.ParseVec2();
        Vertex_PCU vertex = bufParse.ParseVertexPCU();

        GUARANTEE_OR_DIE(fourCC0 == 'T', "LE: fourCC[0] != 'T'");
        GUARANTEE_OR_DIE(fourCC1 == 'E', "LE: fourCC[1] != 'E'");
        GUARANTEE_OR_DIE(fourCC2 == 'S', "LE: fourCC[2] != 'S'");
        GUARANTEE_OR_DIE(fourCC3 == 'T', "LE: fourCC[3] != 'T'");
        GUARANTEE_OR_DIE(version == 2, "LE: version != 2");
        GUARANTEE_OR_DIE(endianByte == 1, "LE: endianness != 1");
        GUARANTEE_OR_DIE(shouldBeFalse == 0, "LE: shouldBeFalse != 0");
        GUARANTEE_OR_DIE(shouldBeTrue == 1, "LE: shouldBeTrue != 1");
        GUARANTEE_OR_DIE(largeUint == 0x12345678, "LE: uint32 mismatch");
        GUARANTEE_OR_DIE(negativeSeven == -7, "LE: int32 mismatch");
        GUARANTEE_OR_DIE(oneF == 1.f, "LE: float mismatch");
        GUARANTEE_OR_DIE(pi == 3.1415926535897932384626433832795, "LE: double mismatch");
        GUARANTEE_OR_DIE(helloString == "Hello", "LE: zero-term string mismatch");
        GUARANTEE_OR_DIE(isThisThingOnString == "Is this thing on?", "LE: length-prec string mismatch");
        GUARANTEE_OR_DIE(rustColor == Rgba8(200, 100, 50, 255), "LE: Rgba8 mismatch");
        GUARANTEE_OR_DIE(eight == 8, "LE: byte 8 mismatch");
        GUARANTEE_OR_DIE(seashellColor == Rgba8(238, 221, 204, 255), "LE: Rgb8 mismatch");
        GUARANTEE_OR_DIE(nine == 9, "LE: byte 9 mismatch");
        GUARANTEE_OR_DIE(highDefRes == IntVec2(1920, 1080), "LE: IntVec2 mismatch");
        GUARANTEE_OR_DIE(normal2D == Vec2(-0.6f, 0.8f), "LE: Vec2 mismatch");
        GUARANTEE_OR_DIE(vertex.m_position == Vec3(3.f, 4.f, 5.f), "LE: VertexPCU position mismatch");
        GUARANTEE_OR_DIE(vertex.m_color == Rgba8(100, 101, 102, 103), "LE: VertexPCU color mismatch");
        GUARANTEE_OR_DIE(vertex.m_uvTexCoords == Vec2(0.125f, 0.625f), "LE: VertexPCU UV mismatch");
    }

    // --- Second half: Big Endian (bytes 104-207) ---
    {
        bufParse.SetEndianMode(eEndianMode::BIG);
        char fourCC0 = bufParse.ParseChar();
        char fourCC1 = bufParse.ParseChar();
        char fourCC2 = bufParse.ParseChar();
        char fourCC3 = bufParse.ParseChar();
        uint8_t version = bufParse.ParseByte();
        uint8_t endianByte = bufParse.ParseByte();
        uint8_t shouldBeFalse = bufParse.ParseByte();
        uint8_t shouldBeTrue = bufParse.ParseByte();
        unsigned int largeUint = bufParse.ParseUint32();
        int negativeSeven = bufParse.ParseInt32();
        float oneF = bufParse.ParseFloat();
        double pi = bufParse.ParseDouble();

        std::string helloString, isThisThingOnString;
        bufParse.ParseZeroTerminatedString(helloString);
        bufParse.ParseLengthPrecededString(isThisThingOnString);

        Rgba8 rustColor = bufParse.ParseRgba8();
        uint8_t eight = bufParse.ParseByte();
        uint8_t seashellR = bufParse.ParseByte();
        uint8_t seashellG = bufParse.ParseByte();
        uint8_t seashellB = bufParse.ParseByte();
        Rgba8 seashellColor(seashellR, seashellG, seashellB, 255);
        uint8_t nine = bufParse.ParseByte();
        IntVec2 highDefRes = bufParse.ParseIntVec2();
        Vec2 normal2D = bufParse.ParseVec2();
        Vertex_PCU vertex = bufParse.ParseVertexPCU();

        GUARANTEE_OR_DIE(fourCC0 == 'T', "BE: fourCC[0] != 'T'");
        GUARANTEE_OR_DIE(fourCC1 == 'E', "BE: fourCC[1] != 'E'");
        GUARANTEE_OR_DIE(fourCC2 == 'S', "BE: fourCC[2] != 'S'");
        GUARANTEE_OR_DIE(fourCC3 == 'T', "BE: fourCC[3] != 'T'");
        GUARANTEE_OR_DIE(version == 2, "BE: version != 2");
        GUARANTEE_OR_DIE(endianByte == 2, "BE: endianness != 2");
        GUARANTEE_OR_DIE(shouldBeFalse == 0, "BE: shouldBeFalse != 0");
        GUARANTEE_OR_DIE(shouldBeTrue == 1, "BE: shouldBeTrue != 1");
        GUARANTEE_OR_DIE(largeUint == 0x12345678, "BE: uint32 mismatch");
        GUARANTEE_OR_DIE(negativeSeven == -7, "BE: int32 mismatch");
        GUARANTEE_OR_DIE(oneF == 1.f, "BE: float mismatch");
        GUARANTEE_OR_DIE(pi == 3.1415926535897932384626433832795, "BE: double mismatch");
        GUARANTEE_OR_DIE(helloString == "Hello", "BE: zero-term string mismatch");
        GUARANTEE_OR_DIE(isThisThingOnString == "Is this thing on?", "BE: length-prec string mismatch");
        GUARANTEE_OR_DIE(rustColor == Rgba8(200, 100, 50, 255), "BE: Rgba8 mismatch");
        GUARANTEE_OR_DIE(eight == 8, "BE: byte 8 mismatch");
        GUARANTEE_OR_DIE(seashellColor == Rgba8(238, 221, 204, 255), "BE: Rgb8 mismatch");
        GUARANTEE_OR_DIE(nine == 9, "BE: byte 9 mismatch");
        GUARANTEE_OR_DIE(highDefRes == IntVec2(1920, 1080), "BE: IntVec2 mismatch");
        GUARANTEE_OR_DIE(normal2D == Vec2(-0.6f, 0.8f), "BE: Vec2 mismatch");
        GUARANTEE_OR_DIE(vertex.m_position == Vec3(3.f, 4.f, 5.f), "BE: VertexPCU position mismatch");
        GUARANTEE_OR_DIE(vertex.m_color == Rgba8(100, 101, 102, 103), "BE: VertexPCU color mismatch");
        GUARANTEE_OR_DIE(vertex.m_uvTexCoords == Vec2(0.125f, 0.625f), "BE: VertexPCU UV mismatch");
    }

    GUARANTEE_OR_DIE(bufParse.GetCurrentPosition() == 208, "Did not consume all 208 bytes");
}

//----------------------------------------------------------------------------------------------------
bool Game::SaveSceneToFile(std::string const& filePath)
{
    // Chunk header is 10 bytes: GHCK(4) + type(1) + endian(1) + dataSize(4)
    // Chunk footer is 4 bytes: ENDC
    constexpr unsigned int CHUNK_HEADER_SIZE = 10;
    constexpr unsigned int CHUNK_FOOTER_SIZE = 4;
    constexpr unsigned int CHUNK_OVERHEAD    = CHUNK_HEADER_SIZE + CHUNK_FOOTER_SIZE;

    struct ChunkInfo
    {
        uint8_t type;
        size_t  startPos;   // absolute position of GHCK
        size_t  dataStart;  // absolute position of private data
        size_t  dataEnd;    // absolute position after private data
    };

    std::vector<ChunkInfo> chunkInfos;
    std::vector<uint8_t> buffer;
    buffer.reserve(4096);
    BufferWriter bufWrite(buffer);
    bufWrite.SetEndianMode(eEndianMode::LITTLE);

    // --- File Header (24 bytes) ---
    bufWrite.AppendChar('G');
    bufWrite.AppendChar('H');
    bufWrite.AppendChar('C');
    bufWrite.AppendChar('S');
    bufWrite.AppendByte(34);  // cohort
    bufWrite.AppendByte(1);   // major version
    bufWrite.AppendByte(1);   // minor version
    bufWrite.AppendByte(1);   // endianness: 1=LE
    bufWrite.AppendUint32(0); // placeholder for total file size (backpatched later, byte offset 8)
    bufWrite.AppendUint32(0); // placeholder for data hash (backpatched later, byte offset 12)
    bufWrite.AppendUint32(0); // placeholder for ToC offset (backpatched later, byte offset 16)

    // File header footer
    bufWrite.AppendChar('E');
    bufWrite.AppendChar('N');
    bufWrite.AppendChar('D');
    bufWrite.AppendChar('H');

    // Helper lambda to write chunk header and record positions
    auto BeginChunk = [&](uint8_t chunkType) -> size_t
    {
        ChunkInfo info;
        info.type = chunkType;
        info.startPos = bufWrite.GetTotalSize();

        bufWrite.AppendChar('G');
        bufWrite.AppendChar('H');
        bufWrite.AppendChar('C');
        bufWrite.AppendChar('K');
        bufWrite.AppendByte(chunkType);
        bufWrite.AppendByte(1); // endianness: 1=LE
        bufWrite.AppendUint32(0); // placeholder for data size

        info.dataStart = bufWrite.GetTotalSize();
        info.dataEnd = 0; // filled in EndChunk
        chunkInfos.push_back(info);
        return chunkInfos.size() - 1;
    };

    auto EndChunk = [&](size_t chunkIndex)
    {
        ChunkInfo& info = chunkInfos[chunkIndex];
        info.dataEnd = bufWrite.GetTotalSize();

        // Backpatch data size (at dataStart - 4)
        unsigned int dataSize = static_cast<unsigned int>(info.dataEnd - info.dataStart);
        bufWrite.OverwriteUint32(info.dataStart - sizeof(unsigned int), dataSize);

        bufWrite.AppendChar('E');
        bufWrite.AppendChar('N');
        bufWrite.AppendChar('D');
        bufWrite.AppendChar('C');
    };

    // --- Chunk 0x01: SceneInfo ---
    {
        size_t idx = BeginChunk(0x01);
        AABB2 cameraBounds(m_worldCamera->GetOrthographicBottomLeft(), m_worldCamera->GetOrthographicTopRight());
        bufWrite.AppendAABB2(cameraBounds);
        bufWrite.AppendUshort(static_cast<unsigned short>(m_convexes.size()));
        EndChunk(idx);
    }

    // --- Chunk 0x02: ConvexPolys ---
    {
        size_t idx = BeginChunk(0x02);
        bufWrite.AppendUshort(static_cast<unsigned short>(m_convexes.size()));
        for (Convex2 const* convex : m_convexes)
        {
            std::vector<Vec2> const& verts = convex->m_convexPoly.GetVertexArray();
            bufWrite.AppendByte(static_cast<uint8_t>(verts.size()));
            for (Vec2 const& v : verts)
            {
                bufWrite.AppendVec2(v);
            }
        }
        EndChunk(idx);
    }

    // --- Chunk 0x81: BoundingDiscs ---
    {
        size_t idx = BeginChunk(0x81);
        bufWrite.AppendUshort(static_cast<unsigned short>(m_convexes.size()));
        for (Convex2 const* convex : m_convexes)
        {
            bufWrite.AppendVec2(convex->m_boundingDiscCenter);
            bufWrite.AppendFloat(convex->m_boundingRadius);
        }
        EndChunk(idx);
    }

    // --- Chunk 0x80: ConvexHulls ---
    {
        size_t idx = BeginChunk(0x80);
        bufWrite.AppendUshort(static_cast<unsigned short>(m_convexes.size()));
        for (Convex2 const* convex : m_convexes)
        {
            std::vector<Plane2> const& planes = convex->m_convexHull.m_boundingPlanes;
            bufWrite.AppendByte(static_cast<uint8_t>(planes.size()));
            for (Plane2 const& p : planes)
            {
                bufWrite.AppendPlane2(p);
            }
        }
        EndChunk(idx);
    }

    // --- Chunk 0x82: BoundingAABBs (custom non-canonical) ---
    {
        size_t idx = BeginChunk(0x82);
        bufWrite.AppendUshort(static_cast<unsigned short>(m_convexes.size()));
        for (Convex2 const* convex : m_convexes)
        {
            bufWrite.AppendAABB2(convex->m_boundingAABB);
        }
        EndChunk(idx);
    }

    // --- Build pointer-to-index map for tree serialization ---
    std::unordered_map<Convex2*, uint16_t> convexIndexMap;
    for (uint16_t i = 0; i < static_cast<uint16_t>(m_convexes.size()); ++i)
    {
        convexIndexMap[m_convexes[i]] = i;
    }

    // --- Chunk 0x83: AABB2 Tree (BVH) ---
    if (!m_AABB2Tree.m_nodes.empty())
    {
        size_t idx = BeginChunk(0x83);
        bufWrite.AppendByte(static_cast<uint8_t>(m_AABB2Tree.m_nodes.size() > 0 ? 1 : 0)); // depth flag (non-zero = valid)
        bufWrite.AppendUint32(static_cast<unsigned int>(m_AABB2Tree.m_nodes.size()));
        bufWrite.AppendUint32(static_cast<unsigned int>(m_AABB2Tree.GetStartOfLastLevel()));
        for (AABB2TreeNode const& node : m_AABB2Tree.m_nodes)
        {
            bufWrite.AppendAABB2(node.m_bounds);
            bufWrite.AppendUshort(static_cast<unsigned short>(node.m_containingConvex.size()));
            for (Convex2* convex : node.m_containingConvex)
            {
                auto it = convexIndexMap.find(convex);
                bufWrite.AppendUshort(it != convexIndexMap.end() ? it->second : static_cast<unsigned short>(0xFFFF));
            }
        }
        EndChunk(idx);
    }

    // --- Chunk 0x87: Symmetric Quadtree ---
    if (!m_symQuadTree.m_nodes.empty())
    {
        size_t idx = BeginChunk(0x87);
        bufWrite.AppendUint32(static_cast<unsigned int>(m_symQuadTree.m_nodes.size()));
        for (SymmetricQuadTreeNode const& node : m_symQuadTree.m_nodes)
        {
            bufWrite.AppendAABB2(node.m_bounds);
            bufWrite.AppendUshort(static_cast<unsigned short>(node.m_containingConvex.size()));
            for (Convex2* convex : node.m_containingConvex)
            {
                auto it = convexIndexMap.find(convex);
                bufWrite.AppendUshort(it != convexIndexMap.end() ? it->second : static_cast<unsigned short>(0xFFFF));
            }
        }
        EndChunk(idx);
    }

    // --- Write preserved unrecognized chunks (if scene unmodified) ---
    if (!m_sceneModified)
    {
        for (UnrecognizedChunk const& preserved : m_preservedChunks)
        {
            size_t preservedStart = bufWrite.GetTotalSize();
            for (uint8_t byte : preserved.rawData)
            {
                bufWrite.AppendByte(byte);
            }
            // Record in chunkInfos for ToC
            ChunkInfo info;
            info.type      = preserved.chunkType;
            info.startPos  = preservedStart;
            info.dataStart = preservedStart + CHUNK_HEADER_SIZE;
            info.dataEnd   = bufWrite.GetTotalSize() - CHUNK_FOOTER_SIZE;
            chunkInfos.push_back(info);
        }
    }

    // --- Backpatch ToC offset (at byte 16) ---
    bufWrite.OverwriteUint32(16, static_cast<unsigned int>(bufWrite.GetTotalSize()));

    // --- Table of Contents ---
    bufWrite.AppendChar('G');
    bufWrite.AppendChar('H');
    bufWrite.AppendChar('T');
    bufWrite.AppendChar('C');
    bufWrite.AppendByte(static_cast<uint8_t>(chunkInfos.size()));

    for (ChunkInfo const& info : chunkInfos)
    {
        unsigned int chunkTotalSize = static_cast<unsigned int>(info.dataEnd - info.dataStart) + CHUNK_OVERHEAD;
        bufWrite.AppendByte(info.type);
        bufWrite.AppendUint32(static_cast<unsigned int>(info.startPos));
        bufWrite.AppendUint32(chunkTotalSize);
    }

    bufWrite.AppendChar('E');
    bufWrite.AppendChar('N');
    bufWrite.AppendChar('D');
    bufWrite.AppendChar('T');

    // --- Backpatch total file size (at byte 8) ---
    bufWrite.OverwriteUint32(8, static_cast<unsigned int>(buffer.size()));

    // --- Backpatch data hash (at byte 12) ---
    // Hash everything after the 24-byte header
    {
        constexpr size_t HEADER_SIZE = 24;
        unsigned int hash = 0;
        for (size_t i = HEADER_SIZE; i < buffer.size(); ++i)
        {
            hash *= 31;
            hash += buffer[i];
        }
        bufWrite.OverwriteUint32(12, hash);
    }

    // --- Write to file ---
    // Extract directory path and ensure it exists
    size_t lastSlash = filePath.find_last_of("/\\");
    if (lastSlash != std::string::npos)
    {
        EnsureDirectoryExists(filePath.substr(0, lastSlash));
    }
    FileWriteFromBuffer(buffer, filePath);
    return true;
}

//----------------------------------------------------------------------------------------------------
bool Game::LoadSceneFromFile(std::string const& filePath)
{
    // Check file existence and size before FileReadToBuffer (which triggers ERROR_RECOVERABLE popups on failure)
    if (!std::filesystem::exists(filePath))
    {
        g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: File not found: %s", filePath.c_str()));
        return false;
    }
    if (std::filesystem::file_size(filePath) == 0)
    {
        g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: File is empty: %s", filePath.c_str()));
        return false;
    }

    std::vector<uint8_t> buffer;
    if (!FileReadToBuffer(buffer, filePath))
    {
        g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Could not read file %s", filePath.c_str()));
        return false;
    }

    BufferParser bufParse(buffer);

    // Minimum valid GHCS file: 24-byte header + 9-byte ToC = 33 bytes
    if (buffer.size() < 33)
    {
        g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: File too small (%zu bytes), not a valid GHCS file", buffer.size()));
        return false;
    }

    // --- Parse and validate file header (24 bytes: magic(4)+cohort(1)+major(1)+minor(1)+endian(1)+fileSize(4)+hash(4)+tocOffset(4)+ENDH(4)) ---
    char magic0 = bufParse.ParseChar();
    char magic1 = bufParse.ParseChar();
    char magic2 = bufParse.ParseChar();
    char magic3 = bufParse.ParseChar();
    if (magic0 != 'G' || magic1 != 'H' || magic2 != 'C' || magic3 != 'S')
    {
        g_devConsole->AddLine(DevConsole::ERROR, "Error: Invalid GHCS file header");
        return false;
    }

    uint8_t cohort       = bufParse.ParseByte();
    uint8_t majorVersion = bufParse.ParseByte();
    uint8_t minorVersion = bufParse.ParseByte();
    uint8_t endianByte   = bufParse.ParseByte();
    UNUSED(cohort);
    UNUSED(majorVersion);
    UNUSED(minorVersion);

    // Set parser endian mode from file's endianness byte (1=LE, 2=BE)
    if (endianByte == 1)
    {
        bufParse.SetEndianMode(eEndianMode::LITTLE);
    }
    else if (endianByte == 2)
    {
        bufParse.SetEndianMode(eEndianMode::BIG);
    }
    else
    {
        g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Invalid endianness byte %d", endianByte));
        return false;
    }

    unsigned int totalFileSize = bufParse.ParseUint32();
    unsigned int storedHash    = bufParse.ParseUint32();
    unsigned int tocOffset     = bufParse.ParseUint32();

    // Validate total file size
    if (totalFileSize != static_cast<unsigned int>(buffer.size()))
    {
        g_devConsole->AddLine(DevConsole::WARNING, Stringf("Warning: totalFileSize (%u) != actual buffer size (%zu)", totalFileSize, buffer.size()));
    }

    // Validate data hash (hash everything after the 24-byte header)
    {
        constexpr size_t HEADER_SIZE = 24;
        unsigned int computedHash = 0;
        for (size_t i = HEADER_SIZE; i < buffer.size(); ++i)
        {
            computedHash *= 31;
            computedHash += buffer[i];
        }
        if (storedHash != computedHash)
        {
            g_devConsole->AddLine(DevConsole::WARNING, Stringf("Warning: data hash mismatch (stored=0x%08X, computed=0x%08X)", storedHash, computedHash));
        }
    }

    // Verify ENDH footer
    if (bufParse.ParseChar() != 'E' || bufParse.ParseChar() != 'N' ||
        bufParse.ParseChar() != 'D' || bufParse.ParseChar() != 'H')
    {
        g_devConsole->AddLine(DevConsole::ERROR, "Error: Missing ENDH footer in file header");
        return false;
    }

    // --- Jump to Table of Contents ---
    if (static_cast<size_t>(tocOffset) + 9 > buffer.size()) // GHTC(4) + numChunks(1) + ENDT(4) minimum
    {
        g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: ToC offset %u exceeds buffer size %zu", tocOffset, buffer.size()));
        return false;
    }
    bufParse.SetCurrentPosition(static_cast<size_t>(tocOffset));

    if (bufParse.ParseChar() != 'G' || bufParse.ParseChar() != 'H' ||
        bufParse.ParseChar() != 'T' || bufParse.ParseChar() != 'C')
    {
        g_devConsole->AddLine(DevConsole::ERROR, "Error: Invalid ToC magic (expected GHTC)");
        return false;
    }

    struct ToCEntry
    {
        uint8_t      type;
        unsigned int startPos;
        unsigned int totalSize;
    };

    uint8_t numChunks = bufParse.ParseByte();
    std::vector<ToCEntry> tocEntries;
    for (int i = 0; i < static_cast<int>(numChunks); ++i)
    {
        ToCEntry entry;
        entry.type      = bufParse.ParseByte();
        entry.startPos  = bufParse.ParseUint32();
        entry.totalSize = bufParse.ParseUint32();
        tocEntries.push_back(entry);
    }

    // Verify ENDT footer
    if (bufParse.ParseChar() != 'E' || bufParse.ParseChar() != 'N' ||
        bufParse.ParseChar() != 'D' || bufParse.ParseChar() != 'T')
    {
        g_devConsole->AddLine(DevConsole::ERROR, "Error: Missing ENDT footer in ToC");
        return false;
    }

    // --- Process each chunk via ToC entries ---
    std::vector<Convex2*> tempConvexes;
    std::vector<UnrecognizedChunk> tempPreservedChunks;
    AABB2    sceneBounds;
    uint16_t recordedNumObjects = static_cast<uint16_t>(-1);
    bool     hasSceneInfo       = false;
    bool     hasConvexPolys     = false;
    bool     hasConvexHulls     = false;
    bool     hasBoundingDiscs   = false;
    bool     hasBoundingAABBs   = false;
    bool     hasAABB2Tree       = false;
    bool     hasSymQuadTree     = false;
    AABB2Tree              tempAABB2Tree;
    SymmetricQuadTree      tempSymQuadTree;

    for (ToCEntry const& entry : tocEntries)
    {
        // Validate chunk start position fits within buffer (at least 14 bytes: GHCK + type + endian + dataSize + ENDC)
        if (static_cast<size_t>(entry.startPos) + 14 > buffer.size())
        {
            g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Chunk startPos %u exceeds buffer size %zu", entry.startPos, buffer.size()));
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }
        bufParse.SetCurrentPosition(static_cast<size_t>(entry.startPos));
        size_t chunkStartPos = bufParse.GetCurrentPosition();

        // Verify GHCK chunk header
        if (bufParse.ParseChar() != 'G' || bufParse.ParseChar() != 'H' ||
            bufParse.ParseChar() != 'C' || bufParse.ParseChar() != 'K')
        {
            g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Invalid chunk header at offset %zu", chunkStartPos));
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }

        uint8_t chunkType = bufParse.ParseByte();
        if (chunkType != entry.type)
        {
            g_devConsole->AddLine(DevConsole::ERROR, "Error: Chunk type mismatch between header and ToC");
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }

        // Per-chunk endianness
        uint8_t chunkEndian = bufParse.ParseByte();
        if (chunkEndian == 1)
            bufParse.SetEndianMode(eEndianMode::LITTLE);
        else if (chunkEndian == 2)
            bufParse.SetEndianMode(eEndianMode::BIG);

        unsigned int dataSize    = bufParse.ParseUint32();
        size_t       dataStartPos = bufParse.GetCurrentPosition();

        // Validate chunk data fits within buffer (prevents GuardRead system popup on malformed files)
        if (dataStartPos + static_cast<size_t>(dataSize) + 4 > buffer.size()) // +4 for ENDC footer
        {
            g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Chunk at offset %zu claims %u data bytes but exceeds buffer size %zu",
                chunkStartPos, dataSize, buffer.size()));
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }

        // --- Process known chunk types ---
        if (chunkType == 0x01) // SceneInfo
        {
            hasSceneInfo = true;
            sceneBounds = bufParse.ParseAABB2();
            recordedNumObjects = bufParse.ParseUshort();
        }
        else if (chunkType == 0x02) // ConvexPolys
        {
            hasConvexPolys = true;
            uint16_t numObjects = bufParse.ParseUshort();
            if (recordedNumObjects != static_cast<uint16_t>(-1) && recordedNumObjects != numObjects)
            {
                g_devConsole->AddLine(DevConsole::ERROR, "Error: Object count mismatch between SceneInfo and ConvexPolys");
                for (Convex2* c : tempConvexes) delete c;
                return false;
            }
            for (int i = 0; i < static_cast<int>(numObjects); ++i)
            {
                uint8_t numVerts = bufParse.ParseByte();
                std::vector<Vec2> verts;
                for (int j = 0; j < static_cast<int>(numVerts); ++j)
                {
                    verts.push_back(bufParse.ParseVec2());
                }
                Convex2* newConvex = new Convex2();
                newConvex->m_convexPoly = ConvexPoly2(verts);
                tempConvexes.push_back(newConvex);
            }
        }
        else if (chunkType == 0x80) // ConvexHulls
        {
            hasConvexHulls = true;
            uint16_t numObjects = bufParse.ParseUshort();
            for (int i = 0; i < static_cast<int>(numObjects) && i < static_cast<int>(tempConvexes.size()); ++i)
            {
                uint8_t numPlanes = bufParse.ParseByte();
                std::vector<Plane2> planes;
                for (int j = 0; j < static_cast<int>(numPlanes); ++j)
                {
                    planes.push_back(bufParse.ParsePlane2());
                }
                tempConvexes[i]->m_convexHull = ConvexHull2(planes);
            }
        }
        else if (chunkType == 0x81) // BoundingDiscs
        {
            hasBoundingDiscs = true;
            uint16_t numObjects = bufParse.ParseUshort();
            for (int i = 0; i < static_cast<int>(numObjects) && i < static_cast<int>(tempConvexes.size()); ++i)
            {
                tempConvexes[i]->m_boundingDiscCenter = bufParse.ParseVec2();
                tempConvexes[i]->m_boundingRadius     = bufParse.ParseFloat();
            }
        }
        else if (chunkType == 0x82) // BoundingAABBs (custom)
        {
            hasBoundingAABBs = true;
            uint16_t numObjects = bufParse.ParseUshort();
            for (int i = 0; i < static_cast<int>(numObjects) && i < static_cast<int>(tempConvexes.size()); ++i)
            {
                tempConvexes[i]->m_boundingAABB = bufParse.ParseAABB2();
            }
        }
        else if (chunkType == 0x83) // AABB2 Tree (BVH)
        {
            hasAABB2Tree = true;
            uint8_t depthFlag = bufParse.ParseByte();
            UNUSED(depthFlag);
            unsigned int numNodes = bufParse.ParseUint32();
            unsigned int startOfLastLevel = bufParse.ParseUint32();
            tempAABB2Tree.m_nodes.resize(numNodes);
            tempAABB2Tree.SetStartOfLastLevel(static_cast<int>(startOfLastLevel));
            for (unsigned int n = 0; n < numNodes; ++n)
            {
                tempAABB2Tree.m_nodes[n].m_bounds = bufParse.ParseAABB2();
                uint16_t numConvex = bufParse.ParseUshort();
                for (int c = 0; c < static_cast<int>(numConvex); ++c)
                {
                    uint16_t objIdx = bufParse.ParseUshort();
                    if (objIdx < static_cast<uint16_t>(tempConvexes.size()))
                    {
                        tempAABB2Tree.m_nodes[n].m_containingConvex.push_back(tempConvexes[objIdx]);
                    }
                }
            }
        }
        else if (chunkType == 0x87) // Symmetric Quadtree
        {
            hasSymQuadTree = true;
            unsigned int numNodes = bufParse.ParseUint32();
            tempSymQuadTree.m_nodes.resize(numNodes);
            for (unsigned int n = 0; n < numNodes; ++n)
            {
                tempSymQuadTree.m_nodes[n].m_bounds = bufParse.ParseAABB2();
                uint16_t numConvex = bufParse.ParseUshort();
                for (int c = 0; c < static_cast<int>(numConvex); ++c)
                {
                    uint16_t objIdx = bufParse.ParseUshort();
                    if (objIdx < static_cast<uint16_t>(tempConvexes.size()))
                    {
                        tempSymQuadTree.m_nodes[n].m_containingConvex.push_back(tempConvexes[objIdx]);
                    }
                }
            }
        }
        else
        {
            // Unknown chunk — skip past private data for now; raw bytes captured after ENDC verification
            bufParse.SetCurrentPosition(dataStartPos + static_cast<size_t>(dataSize));
        }

        // Validate private data size
        size_t dataEndPos = bufParse.GetCurrentPosition();
        if (dataEndPos - dataStartPos != static_cast<size_t>(dataSize))
        {
            g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Chunk data size mismatch at offset %zu (expected %u, got %zu)",
                chunkStartPos, dataSize, dataEndPos - dataStartPos));
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }

        // Verify ENDC footer
        if (bufParse.ParseChar() != 'E' || bufParse.ParseChar() != 'N' ||
            bufParse.ParseChar() != 'D' || bufParse.ParseChar() != 'C')
        {
            g_devConsole->AddLine(DevConsole::ERROR, Stringf("Error: Missing ENDC footer at offset %zu", bufParse.GetCurrentPosition() - 4));
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }

        // Validate total chunk size vs ToC
        size_t chunkEndPos = bufParse.GetCurrentPosition();
        if (chunkEndPos - chunkStartPos != static_cast<size_t>(entry.totalSize))
        {
            g_devConsole->AddLine(DevConsole::ERROR, "Error: Chunk total size mismatch with ToC entry");
            for (Convex2* c : tempConvexes) delete c;
            return false;
        }

        // Preserve unknown chunks as raw bytes (complete: header + data + footer)
        if (chunkType != 0x01 && chunkType != 0x02 && chunkType != 0x80 &&
            chunkType != 0x81 && chunkType != 0x82)
        {
            UnrecognizedChunk preserved;
            preserved.chunkType  = chunkType;
            preserved.endianness = chunkEndian;
            preserved.rawData.assign(buffer.begin() + chunkStartPos, buffer.begin() + chunkEndPos);
            tempPreservedChunks.push_back(preserved);
        }
    }

    // --- Validate required chunks ---
    if (!hasSceneInfo)
    {
        g_devConsole->AddLine(DevConsole::ERROR, "Error: Missing required SceneInfo chunk");
        for (Convex2* c : tempConvexes) delete c;
        return false;
    }
    if (!hasConvexPolys)
    {
        g_devConsole->AddLine(DevConsole::ERROR, "Error: Missing required ConvexPolys chunk");
        for (Convex2* c : tempConvexes) delete c;
        return false;
    }

    // --- Regenerate missing optional data ---
    for (Convex2* convex : tempConvexes)
    {
        // Rebuild hull from poly if not loaded
        if (!hasConvexHulls || convex->m_convexHull.m_boundingPlanes.empty())
        {
            convex->m_convexHull = ConvexHull2(convex->m_convexPoly);
        }

        // Rebuild bounding volumes if not loaded
        if (!hasBoundingDiscs || !hasBoundingAABBs)
        {
            convex->RebuildBoundingVolumes();
        }
    }

    // --- Replace current scene ---
    ClearScene();
    m_convexes         = tempConvexes;
    m_preservedChunks  = tempPreservedChunks;
    m_sceneModified    = false;

    // --- Letterbox/pillarbox camera adjustment ---
    m_loadedSceneBounds = sceneBounds;
    m_hasLoadedScene    = true;

    float windowAspect = WORLD_SIZE_X / WORLD_SIZE_Y; // default 2:1
    float sceneWidth   = sceneBounds.m_maxs.x - sceneBounds.m_mins.x;
    float sceneHeight  = sceneBounds.m_maxs.y - sceneBounds.m_mins.y;
    float sceneAspect  = sceneWidth / sceneHeight;

    if (sceneAspect > windowAspect)
    {
        // Scene is wider → letterbox (black bars top/bottom)
        float viewHeight = sceneWidth / windowAspect;
        float offsetY    = (viewHeight - sceneHeight) * 0.5f;
        m_worldCamera->SetOrthoGraphicView(
            Vec2(sceneBounds.m_mins.x, sceneBounds.m_mins.y - offsetY),
            Vec2(sceneBounds.m_maxs.x, sceneBounds.m_maxs.y + offsetY));
    }
    else
    {
        // Scene is taller or equal → pillarbox (black bars left/right)
        float viewWidth = sceneHeight * windowAspect;
        float offsetX   = (viewWidth - sceneWidth) * 0.5f;
        m_worldCamera->SetOrthoGraphicView(
            Vec2(sceneBounds.m_mins.x - offsetX, sceneBounds.m_mins.y),
            Vec2(sceneBounds.m_maxs.x + offsetX, sceneBounds.m_maxs.y));
    }

    // --- Restore or rebuild spatial acceleration structures ---
    if (hasAABB2Tree)
    {
        m_AABB2Tree = std::move(tempAABB2Tree);
    }
    if (hasSymQuadTree)
    {
        m_symQuadTree = std::move(tempSymQuadTree);
    }
    if (!hasAABB2Tree || !hasSymQuadTree)
    {
        // Rebuild any trees not loaded from file
        AABB2 totalBounds(Vec2(0.f, 0.f), Vec2(WORLD_SIZE_X, WORLD_SIZE_Y));
        int numConvexes = static_cast<int>(m_convexes.size());
        int bvhDepth = 0;
        if (numConvexes > 0)
        {
            bvhDepth = static_cast<int>(log2(static_cast<double>(numConvexes))) - 3;
            if (bvhDepth < 3) bvhDepth = 3;
        }
        if (!hasAABB2Tree)
        {
            m_AABB2Tree.BuildTree(m_convexes, bvhDepth, totalBounds);
        }
        if (!hasSymQuadTree)
        {
            m_symQuadTree.BuildTree(m_convexes, 4, totalBounds);
        }
    }
    return true;
}
