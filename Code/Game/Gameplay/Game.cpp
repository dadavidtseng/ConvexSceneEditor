//----------------------------------------------------------------------------------------------------
// Game.cpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#include "Game/Gameplay/Game.hpp"
//----------------------------------------------------------------------------------------------------
#include "Game/Framework/App.hpp"
#include "Game/Framework/GameCommon.hpp"
//----------------------------------------------------------------------------------------------------
#include "Engine/Audio/AudioSystem.hpp"
#include "Engine/Core/Clock.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/LogSubsystem.hpp"
#include "Engine/Input/InputSystem.hpp"
#include "Engine/Platform/Window.hpp"
#include "Engine/Renderer/DebugRenderSystem.hpp"

//----------------------------------------------------------------------------------------------------
Game::Game()
{
    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(Game)(start)");

    g_eventSystem->SubscribeEventCallbackFunction("OnGameStateChanged", OnGameStateChanged);

    m_screenCamera = new Camera();

    Vec2 const bottomLeft     = Vec2::ZERO;
    Vec2 const screenTopRight = Window::s_mainWindow->GetClientDimensions();

    m_screenCamera->SetOrthoGraphicView(bottomLeft, screenTopRight);
    m_screenCamera->SetNormalizedViewport(AABB2::ZERO_TO_ONE);

    m_gameClock = new Clock(Clock::GetSystemClock());

    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(Game)(end)");
}

//----------------------------------------------------------------------------------------------------
Game::~Game()
{
    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(~Game)(start)");

    GAME_SAFE_RELEASE(m_screenCamera);

    g_eventSystem->UnsubscribeEventCallbackFunction("OnGameStateChanged", OnGameStateChanged);

    DAEMON_LOG(LogGame, eLogVerbosity::Display, "(~Game)(end)");
}

//----------------------------------------------------------------------------------------------------
void Game::Update()
{
    Vec2 const      screenTopLeft = m_screenCamera->GetOrthographicTopLeft();
    float constexpr textHeight    = 20.f;

    DebugAddScreenText(Stringf("Time: %.2f FPS: %.2f Scale: %.1f", m_gameClock->GetTotalSeconds(), 1.f / m_gameClock->GetDeltaSeconds(), m_gameClock->GetTimeScale()), screenTopLeft - Vec2(0.f, textHeight), textHeight, Vec2(1, 1), 0.f);
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
        RenderGame();
        DebugRenderScreen(*m_screenCamera);
    }

    g_renderer->EndCamera(*m_screenCamera);
    //-End-of-Screen-Camera---------------------------------------------------------------------------
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
        if (g_input->WasKeyJustPressed(KEYCODE_ESC))
        {
            SetGameState(eGameState::ATTRACT);
        }
        else if (g_input->WasKeyJustPressed(KEYCODE_F8))
        {
            g_app->DeleteAndCreateNewGame();
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
/// @brief Render two line segments 2D in GAME state.
//
void Game::RenderGame() const
{
    Vec2 const clientDimensions = Window::s_mainWindow->GetClientDimensions();
    Vec2 const offsetFromCorner = Vec2(200.f, 100.f);
    Vec2 const topRight         = clientDimensions - offsetFromCorner;
    Vec2 const bottomLeft       = offsetFromCorner;
    Vec2 const topLeft          = Vec2(offsetFromCorner.x, clientDimensions.y - offsetFromCorner.y);
    Vec2 const bottomRight      = Vec2(clientDimensions.x - offsetFromCorner.x, offsetFromCorner.y);

    VertexList_PCU verts;

    AddVertsForLineSegment2D(verts, topRight, bottomLeft, 10.f, false, Rgba8::GREEN);
    AddVertsForLineSegment2D(verts, topLeft, bottomRight, 10.f, false, Rgba8::GREEN);

    g_renderer->SetModelConstants();
    g_renderer->SetBlendMode(eBlendMode::OPAQUE);
    g_renderer->SetRasterizerMode(eRasterizerMode::SOLID_CULL_BACK);
    g_renderer->SetSamplerMode(eSamplerMode::BILINEAR_CLAMP);
    g_renderer->SetDepthMode(eDepthMode::DISABLED);
    g_renderer->BindTexture(nullptr);
    g_renderer->BindShader(nullptr);
    g_renderer->DrawVertexArray(verts);
}
