# Requirements Document: Convex Scene Editor

## Introduction

The Convex Scene Editor is a 2D interactive application for visualizing and testing raycast algorithms against convex polygons. This educational tool demonstrates spatial optimization techniques (BVH, QuadTree) and provides real-time performance metrics for comparing different raycast optimization strategies. The application allows users to manipulate convex shapes, perform batch raycasting tests, and analyze performance characteristics across different object counts and optimization modes.

## Alignment with Product Vision

This project serves as a practical implementation of computational geometry concepts, specifically:
- Convex hull representation and manipulation
- Ray-convex intersection algorithms
- Spatial partitioning optimization techniques (BVH, QuadTree)
- Performance analysis and algorithmic complexity observation

The tool provides hands-on experience with game engine architecture patterns and real-time rendering systems.

## Requirements

### REQ-1: Project Setup (3 points)

**User Story:** As a developer, I want to create a new project cloned from Protogame2D called ConvexSceneEditor, so that I have a working foundation for the assignment.

#### Acceptance Criteria

1. WHEN the project is created THEN it SHALL be named "ConvexSceneEditor"
2. WHEN the project is created THEN it SHALL be cloned from the Protogame2D template
3. WHEN the project builds THEN it SHALL compile without errors in Debug and Release configurations
4. WHEN the project runs THEN it SHALL display a window with the game engine initialized

### REQ-2: Random Convex Scene Generation (10 points)

**User Story:** As a user, I want to see a scene with N=8 randomly generated static convex polygons, so that I can test raycast algorithms against varied geometric configurations.

#### Acceptance Criteria

1. WHEN the application starts THEN the system SHALL display N=8 random static (non-moving) convex objects
2. WHEN a convex polygon is generated THEN it SHALL have between 3-8 vertices
3. WHEN vertices are generated THEN they SHALL form a valid convex polygon (CCW winding order)
4. WHEN polygons are displayed THEN they SHALL fit within the world bounds
5. WHEN polygons are generated THEN each SHALL have a unique random position, size, and rotation
6. WHEN the scene is rendered THEN all convex objects SHALL be visible and non-overlapping (or overlapping is acceptable)

### REQ-3: Dual Rendering Modes (5 points)

**User Story:** As a user, I want to toggle between two different rendering modes with F2, so that I can visualize how overlapping convex shapes create composite concave appearances.

#### Acceptance Criteria

1. WHEN the user presses F2 (or similar key) THEN the system SHALL toggle between Mode A and Mode B
2. WHEN Mode A is active THEN objects SHALL be drawn with translucent fill and thick opaque edges
3. WHEN Mode B is active (default) THEN all object edges SHALL be drawn first (thick, opaque, dark)
4. WHEN Mode B is active THEN all object interiors SHALL be drawn second (opaque, light)
5. WHEN Mode B is active AND objects overlap THEN the system SHALL create the appearance of a composite concave shape(s)

### REQ-4: Visible Raycast Visualization (12 points)

**User Story:** As a user, I want to see one visible raycast with its impact point and surface normal, so that I can understand how ray-convex intersection works visually.

#### Acceptance Criteria

1. WHEN the scene is rendered THEN the system SHALL display one visible raycast
2. WHEN the raycast is drawn THEN it SHALL show the ray line
3. WHEN the raycast hits an object THEN it SHALL show the impact position
4. WHEN the raycast hits an object THEN it SHALL show the surface normal drawn nicely using 2D arrows
5. WHEN the raycast misses all objects THEN it SHALL still be visible as a ray line

### REQ-5: Batch Raycast Performance Testing (15 points)

**User Story:** As a user, I want to fire M=1024 invisible random raycasts when I press 'T' and see the elapsed time, so that I can measure raycast performance.

#### Acceptance Criteria

1. WHEN the user hits the 'T' key THEN M=1024 invisible random raycasts SHALL fire
2. WHEN raycasts fire THEN each SHALL be from a random position in the scene to another random position within the scene
3. WHEN batch raycasting starts THEN the current time SHALL be measured immediately before the loop
4. WHEN batch raycasting completes THEN the current time SHALL be measured immediately after the loop
5. WHEN timing is complete THEN the elapsed time SHALL be displayed on the screen afterwards
6. WHEN raycasts are generated THEN they SHALL all be invisible (not drawn)

### REQ-6: Debug Visualization for Single Object (6 points)

**User Story:** As a developer, I want to see convex hull planes and ray test points when only 1 object exists, so that I can debug the ray-convex intersection algorithm.

#### Acceptance Criteria

1. WHEN only 1 object is present THEN the system SHALL draw convex hull planes
2. WHEN only 1 object is present THEN the system SHALL draw ray intercept points
3. WHEN only 1 object is present THEN the system SHALL draw ray test points
4. WHEN debug visualization is shown THEN elements SHALL be color-coded by status/rejection
5. WHEN multiple objects exist THEN this debug visualization SHALL NOT be shown

### REQ-7: Mouse Hover Detection (5 points)

**User Story:** As a user, I want to see which convex object my mouse is hovering over highlighted as the "current" object, so that I can identify shapes for interaction.

#### Acceptance Criteria

1. WHEN the mouse position is inside a convex object THEN that object SHALL be highlighted
2. WHEN the mouse position is inside multiple overlapping objects THEN at most one object SHALL be highlighted as the "current" object
3. WHEN an object is highlighted THEN it SHALL be visually distinct from non-highlighted objects
4. WHEN the mouse moves outside all objects THEN no object SHALL be highlighted

### REQ-8: Sticky Hover Behavior (4 points)

**User Story:** As a user, I want the highlighted object to remain locked while hovering, so that I can reliably interact with the same object.

#### Acceptance Criteria

1. WHEN an object is highlighted THEN the highlight SHALL never change to another object
2. WHEN an object is highlighted THEN the system SHALL always keep the current object highlighted
3. WHEN the mouse moves outside the current object THEN the highlight SHALL be cleared
4. WHEN the mouse re-enters the scene THEN a new object MAY be highlighted

### REQ-9: Spatial Partitioning Implementation (12 points)

**User Story:** As a developer, I want to implement at least one spatial hashing/partitioning scheme, so that I can optimize raycast performance for scenes with many objects.

#### Acceptance Criteria

1. WHEN the system is implemented THEN it SHALL include at least one spatial hashing and/or partitioning scheme
2. WHEN choosing a scheme THEN it MAY be BSP, BVH, bit-buckets, or another spatial optimization technique
3. WHEN the spatial structure is built THEN it SHALL correctly partition the scene objects
4. WHEN raycasts are performed THEN the spatial structure SHALL be used to accelerate queries
5. WHEN objects are added/removed/transformed THEN the spatial structure SHALL be rebuilt as needed

### REQ-10: Keyboard and Mouse Controls (12 points)

**User Story:** As a user, I want comprehensive keyboard and mouse controls for all interactions, so that I can manipulate the scene and test different configurations.

#### Acceptance Criteria - All controls SHALL be displayed in-app/onscreen:

1. **Raycast Control:**
   - WHEN the user holds 'S' or LMB THEN the visible raycast Start SHALL snap (drag) to the mouse position
   - WHEN the user holds 'E' or RMB THEN the visible raycast End SHALL snap (drag) to the mouse position

2. **Object Count Control:**
   - WHEN the user presses 'Y' THEN the number of static scene objects SHALL be halved (minimum 1)
   - WHEN the user presses 'U' THEN the number of static scene objects SHALL be doubled

3. **Raycast Count Control:**
   - WHEN the user presses 'M' THEN the number of invisible raycasts SHALL be halved (minimum 1)
   - WHEN the user presses 'N' THEN the number of invisible raycasts SHALL be doubled
   - WHEN 'T' is next pressed THEN the new raycast count SHALL be used

4. **Object Rotation:**
   - WHEN the user holds 'W' or 'Q' THEN the current object pointed-at SHALL rotate clockwise around the mouse cursor position
   - WHEN the user holds 'R' or 'E' THEN the current object pointed-at SHALL rotate counter-clockwise around the mouse cursor position

5. **Object Scaling:**
   - WHEN the user holds 'K' THEN the current object pointed-at SHALL uniformly scale (deflate) around the mouse cursor position
   - WHEN the user holds 'L' THEN the current object pointed-at SHALL uniformly scale (inflate) around the mouse cursor position

6. **Object Dragging:**
   - WHEN the user performs left mouse click-and-drag THEN the current object SHALL be dragged around
   - WHEN dragging THEN the object SHALL NOT "snap to" cursor; it SHALL preserve the object's relative offset from grabbed position
   - WHEN dragging THEN the object SHALL NOT "lose focus" when dragging past/over others; the "current" object SHALL be locked
   - WHEN LMB is pressed THEN dragging SHALL only work if there was a "current" object at start of drag (LMB-down); otherwise, it SHALL do nothing

7. **Scene Regeneration:**
   - WHEN the user presses F8 THEN all shapes SHALL be re-randomized
   - WHEN F8 is pressed THEN the entire scene SHALL be rebuilt
   - WHEN F8 is pressed THEN the current shape count SHALL be kept

8. **Bounding Disc Debug:**
   - WHEN the user presses F1 THEN the debug draw of each object's own bounding disc SHALL be toggled

9. **Spatial Partitioning Debug:**
   - WHEN the user presses F4 THEN the debug draw of the spatial hashing/partitioning scheme(s) SHALL be toggled

10. **Optimization Mode Cycling:**
    - WHEN the user presses F9 THEN the system SHALL cycle between different combinations of enabling/disabling:
      - Narrow-phase ray-disc optimization
      - Spatial/partitioning scheme optimization(s)

11. **Control Documentation:**
    - WHEN the application runs THEN key/mouse controls MAY vary from the defaults
    - WHEN controls vary THEN they SHALL be documented clearly onscreen
    - WHEN controls vary THEN they SHALL be documented clearly in the ReadMe

### REQ-11: On-Screen Metrics Display (6 points)

**User Story:** As a user, I want to see real-time performance metrics on screen, so that I can analyze raycast performance characteristics.

#### Acceptance Criteria

1. WHEN the application is running THEN the following metrics/variables SHALL be displayed onscreen:
   - Number of objects
   - Number of invisible raycasts to use when 'T' is next pressed
   - Partitioning scheme name (or "None")
   - FPS (frames per second)
   - The total amount of time (in milliseconds) to perform all of the invisible raycasts from the last time 'T' was pressed

2. WHEN metrics are displayed THEN they SHALL be clearly readable
3. WHEN metrics update THEN the display SHALL refresh to show current values

### REQ-12: Performance Analysis Documentation (10 points)

**User Story:** As a developer, I want to document performance findings in a README.txt, so that I can analyze and communicate the effectiveness of different optimization strategies.

#### Acceptance Criteria

1. WHEN the project is submitted THEN a ReadMe.txt file SHALL exist
2. WHEN the ReadMe is read THEN it SHALL contain a clear and succinct (but not academic-formal) summary of findings
3. WHEN the summary is written THEN it SHALL include:
   - a. How many raycasts/ms can you do vs. 16 objects? 256? 1024?
   - b. How does this improve (or worsen!) when you enable the early ray-disc rejection test?
   - c. How does this improve (or worsen!) when you enable your chosen hashing/partitioning scheme?
   - d. How do speeds compare in each build config (Debug/Release, and ideally also DebugInline, FastBreak)?
   - e. Any general trends you can observe, i.e. the speed seems to be O(N) or O(N²) with #objects, etc.
   - f. Any data specific to your hashing/partitioning scheme you can observe (e.g. AABB Tree depth)
   - g. Anything else interesting you observe about your results
4. WHEN measurements are taken THEN all speed measurements (other than cross-build comparisons) SHALL be taken in Release builds

## Non-Functional Requirements

### Code Architecture and Modularity

- **Single Responsibility Principle**: Each class should have a single, well-defined purpose
  - `Convex2` class handles convex shape representation and operations
  - `AABB2Tree` class handles BVH spatial partitioning
  - `SymmetricQuadTree` class handles quad tree spatial partitioning
  - `Game` class orchestrates scene management and user interaction
- **Modular Design**: Components should be isolated and reusable
  - Convex shape generation should be a separate function
  - Raycast algorithms should be encapsulated in the `Convex2` class
  - Spatial structures should implement a common interface
- **Dependency Management**: Minimize interdependencies between modules
  - Spatial structures should only depend on `Convex2` pointers
  - Rendering should be separated from game logic
- **Clear Interfaces**: Define clean contracts between components
  - `Convex2::RayCastVsConvex2D()` provides a consistent raycast interface
  - Spatial structures provide `BuildTree()` and `SolveRayResult()` methods

### Performance

- **Raycast Performance**: The system SHALL support at least 1024 raycasts against 1024 objects in reasonable time (Release build)
- **Frame Rate**: The system SHALL maintain at least 60 FPS during normal operation
- **Spatial Structure Build Time**: Tree rebuilding SHALL complete quickly for up to 1024 objects
- **Memory Efficiency**: Spatial structures SHALL use O(N) memory where N is the number of objects

### Reliability

- **Deterministic Random Generation**: Using the same seed SHALL produce identical scenes
- **Numerical Stability**: Raycast algorithms SHALL handle edge cases (parallel rays, grazing hits, etc.)
- **Bounds Checking**: All objects SHALL remain within world bounds during generation and manipulation

### Usability

- **Visual Clarity**: All UI text SHALL be clearly readable with sufficient contrast
- **Control Documentation**: All keyboard/mouse controls SHALL be displayed on-screen AND in ReadMe
- **Immediate Feedback**: All user interactions SHALL provide immediate visual feedback
- **Intuitive Controls**: Object manipulation SHALL feel natural and responsive

### Testability

- **Validation**: Batch raycast results SHALL be validated for correctness across optimization modes
- **Reproducibility**: Performance measurements SHALL be reproducible with the same scene configuration
- **Debug Visualization**: Debug modes SHALL provide sufficient information for algorithm verification

## Technical Constraints

- **Language**: C++ (C++17 or later)
- **Engine**: Custom "SD" game engine (Guildhall student engine)
- **Platform**: Windows (Visual Studio)
- **Build Configurations**: Debug, Release, DebugInline, FastBreak
- **Submission**: Perforce changelist with comment "SD4-A1: COMPLETE"

## Success Criteria

The Convex Scene Editor is considered complete when:

1. ✅ All 12 functional requirements (100 points total) are implemented and tested
2. ✅ The application runs at 60+ FPS with reasonable object counts
3. ✅ Batch raycasting completes in reasonable time
4. ✅ All keyboard/mouse controls work as specified and are documented onscreen AND in ReadMe
5. ✅ Performance metrics are accurately displayed on-screen
6. ✅ ReadMe.txt contains comprehensive performance analysis with all required sections
7. ✅ Debug visualizations correctly display spatial structures and ray test points
8. ✅ The application is stable and does not crash during normal operation
9. ✅ A Release-built .EXE is included in the submission
10. ✅ A voice-narrated video demonstrates full functionality

## Submission Requirements

### Perforce Submission
- **Changelist Comment**: "SD4-A1: COMPLETE"
- **Required Files**:
  - Updated ReadMe.txt
  - Current Release-built .EXE
  - All required code & data files

### Canvas Submission
- **Filename**: C34_SD4_A1_yourP4username.zip
- **Contents**:
  - A very short (informal) voice-narrated video demonstrating the full functionality
  - A copy of the assignment document with:
    - Completed items highlighted cyan
    - Omitted items highlighted red
    - Partially completed items highlighted yellow (with inserted bullets-text underneath explaining)

## Out of Scope

The following features are explicitly NOT required for this assignment:

- ❌ Concave polygon support
- ❌ Moving/animated objects
- ❌ Physics simulation (collision response, gravity, etc.)
- ❌ Save/load scene functionality
- ❌ Multiple visible raycasts (only one visible ray required)
- ❌ Audio feedback
- ❌ Network multiplayer
- ❌ Advanced rendering effects (shaders, lighting, shadows)
