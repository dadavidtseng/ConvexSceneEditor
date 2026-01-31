//----------------------------------------------------------------------------------------------------
// GameCommon.hpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#pragma once

class ResourceSubsystem;
//-Forward-Declaration--------------------------------------------------------------------------------
struct Rgba8;
struct Vec2;
class App;
class BitmapFont;
class Game;

// one-time declaration
extern App*                   g_app;
extern BitmapFont*            g_bitmapFont;
extern Game*                  g_game;

//----------------------------------------------------------------------------------------------------
// DebugRender-related
//
void DebugDrawRing(Vec2 const& center, float radius, float thickness, Rgba8 const& color);
void DebugDrawLine(Vec2 const& start, Vec2 const& end, float thickness, Rgba8 const& color);

//----------------------------------------------------------------------------------------------------
template <typename T>
void GAME_SAFE_RELEASE(T*& pointer)
{
    if (pointer != nullptr)
    {
        delete pointer;
        pointer = nullptr;
    }
}
