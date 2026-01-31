//----------------------------------------------------------------------------------------------------
// App.hpp
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
#pragma once
//----------------------------------------------------------------------------------------------------
#include "Engine/Core/EventSystem.hpp"

//-Forward-Declaration--------------------------------------------------------------------------------
class Camera;

//----------------------------------------------------------------------------------------------------
class App
{
public:
    App();
    ~App();

    void Startup();
    void Shutdown();

    void RunMainLoop();
    void RunFrame();

    static bool OnCloseButtonClicked(EventArgs& args);
    static void RequestQuit();
    static bool m_isQuitting;
    void        DeleteAndCreateNewGame();

private:
    void BeginFrame() const;
    void Update();
    void Render() const;
    void EndFrame() const;

    void UpdateCursorMode();

    Camera* m_devConsoleCamera = nullptr;
};
