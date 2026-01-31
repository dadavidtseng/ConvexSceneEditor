//-----------------------------------------------------------------------------------------------
// EngineBuildPreferences.hpp
//
// Defines build preferences that the Engine should use when building for this particular game.
//
// Note that this file is an exception to the rule "engine code shall not know about game code".
//	Purpose: Each game can now direct the engine via #defines to build differently for that game.
//	Downside: ALL games must now have this Code/Game/EngineBuildPreferences.hpp file.
//

#pragma once
// #define ENGINE_DISABLE_AUDIO	// Disables AudioSystem code and removes FMOD linkage
#define ENGINE_DEBUG_RENDER
#define ENGINE_DISABLE_SCRIPT	// Disables ScriptSubsystem code and removes V8 linkage