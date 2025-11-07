# CSCI 522 Game Engine Development — Navigation & Obstacle Avoidance System

# Demo Video
https://youtu.be/ni2wydib5-k

# What's Implemented

# 1) Navigation Manager & Navigation Mesh
- Where: NavigationManager.h/cpp, NavigationMesh.h/cpp
- What:
  - NavigationManager is a singleton that manages the navigation mesh lifecycle
  - NavigationMesh stores walkable triangles with adjacency information and performs pathfinding
  - Both follow PrimeEngine's Handle-based memory management pattern

# 2) Triangle-based Navigation Mesh Construction
- Where: NavigationMesh::AddMeshData(), NavigationMesh::BuildAdjacency(), ClientCharacterControlGame::initGame()
- What:
  - Load ground plane mesh (cobbleplane) and generate a grid of instances covering the game area
  - Transform vertices by position matrix (2.5m spacing) to create ~200 walkable triangles
  - Build adjacency graph by detecting shared edges between triangles
  - Filter out steep slopes (>45°) and tiny triangles (<0.1 area)

# 3) A* Pathfinding Algorithm
- Where: NavigationMesh::FindPath()
- What:
  - Standard A* using Euclidean distance heuristic between triangle centers
  - Use heap-based Array<T,0> instead of stack-based to prevent stack overflow
  - Modified for large triangles: when start/goal in same triangle, sample path every 0.5m to detect obstacles within 3m
  - If direct path blocked, force A* to search through neighbor triangles by marking start triangle as closed
  - Special goal-reached condition: accept when reaching neighbor of goal triangle (handles blocked goal triangle case)

# 4) Static Obstacle Marking
- Where: NavigationMesh::MarkStaticObstacle(), ClientCharacterControlGame::initGame()
- What:
  - Mark triangles within radius of obstacle position as non-traversable
  - Applied to three car positions at game startup
  - A* pathfinding skips marked triangles during search

# 5) Coordinate Scaling Discovery & Correction
- Where: ClientCharacterControlGame::initGame()
- What:
  - Discovered level file coordinates were 100x larger than actual game world
  - Divided all car obstacle positions by 100 (e.g., 453.35 → 4.53) to match soldier positions
  - Critical for obstacle detection to work correctly

# 6) NPC Behavior Integration
- Where: SoldierNPCBehaviorSM::do_UPDATE(), SoldierNPCBehaviorSM.h
- What:
  - Added m_pNavMesh pointer and m_currentPath array to soldier behavior state machine
  - Path recalculation every 0.5 seconds (via timer) to balance performance and responsiveness
  - When path found, move to next waypoint (m_currentPath[1]) instead of final destination
  - Fallback to direct movement if pathfinding fails
  - Maintains 3-meter follow distance from player

# 7) Engine Wiring & Initialization
- Where: ClientCharacterControlGame::initGame(), SoldierNPC::addDefaultComponents()
- What:
  - Create NavigationManager singleton before level load
  - Generate navigation mesh grid, finalize adjacency, mark obstacles
  - Pass navigation mesh pointer to each soldier NPC during creation
  - Soldiers use navigation for obstacle avoidance in FOLLOWING_PLAYER state
