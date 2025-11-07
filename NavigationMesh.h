#ifndef _CHARACTER_CONTROL_NAVIGATION_MESH_H_
#define _CHARACTER_CONTROL_NAVIGATION_MESH_H_

#include "PrimeEngine/Math/Vector3.h"
#include "PrimeEngine/Math/Matrix4x4.h"
#include "PrimeEngine/Utils/Array/Array.h"
#include "PrimitiveTypes/PrimitiveTypes.h"
#include "PrimeEngine/MemoryManagement/Handle.h"
#include "PrimeEngine/Geometry/MeshCPU/MeshCPU.h"

namespace CharacterControl {

    // Navigation mesh triangle - pure data structure
    struct NavTriangle
    {
        Vector3 vertices[3];        // Three vertices
        Vector3 center;             // Center point
        Vector3 normal;             // Normal vector
        PrimitiveTypes::Int32 id;   // Triangle ID
        PrimitiveTypes::Bool isTraversable;         // Is walkable
        PrimitiveTypes::Float32 cost;               // Movement cost

        NavTriangle()
            : id(-1)
            , isTraversable(true)
            , cost(1.0f)
        {
        }

        // Calculate center point and normal
        void Calculate();

        // Check if point is inside triangle (2D projection)
        PrimitiveTypes::Bool ContainsPoint(const Vector3& point) const;

        // Get shared edge with neighbor triangle
        PrimitiveTypes::Bool GetSharedEdge(const NavTriangle& other, Vector3& edgeStart, Vector3& edgeEnd) const;
    };

    // A* pathfinding node
    struct PathNode
    {
        PrimitiveTypes::Int32 triangleId;
        PrimitiveTypes::Float32 gCost;  // Cost from start to current
        PrimitiveTypes::Float32 hCost;  // Estimated cost from current to goal
        PrimitiveTypes::Int32 parent;   // Parent node

        PathNode() : triangleId(-1), gCost(0), hCost(0), parent(-1) {}
        PathNode(PrimitiveTypes::Int32 id, PrimitiveTypes::Float32 g,
            PrimitiveTypes::Float32 h, PrimitiveTypes::Int32 p)
            : triangleId(id), gCost(g), hCost(h), parent(p) {
        }

        PrimitiveTypes::Float32 fCost() const { return gCost + hCost; }
    };

    // Dynamic obstacle (for soldier avoidance)
    struct NavObstacle
    {
        void* operator new(size_t size, PE::MemoryArena arena)
        {
            return pemalloc(arena, size);
        }

        Vector3 position;
        PrimitiveTypes::Float32 radius;
        PE::Handle hGameObject;  // Corresponding game object handle
        PE::MemoryArena m_arena;
        PE::GameContext* m_pContext;

        NavObstacle(PE::GameContext& context, PE::MemoryArena arena)
            : radius(0.5f)
            , m_arena(arena)
            , m_pContext(&context)
        {
        }
    };

    // Main navigation mesh class
    class NavigationMesh
    {
    public:
        void* operator new(size_t size, PE::Handle& h)
        {
            return h.getObject();
        }

        NavigationMesh(PE::GameContext& context, PE::MemoryArena arena);
        ~NavigationMesh();

        // Initialize
        void Initialize();

        // Build navigation mesh from MeshCPU
        void BuildFromMeshCPU(PE::MeshCPU* pMeshCPU, const Matrix4x4& transform);

        // Build from vertex and index buffers
        void BuildFromMeshBuffers(const PrimitiveTypes::Float32* vertices,
            PrimitiveTypes::Int32 vertexCount,
            const PrimitiveTypes::UInt16* indices,
            PrimitiveTypes::Int32 indexCount,
            const Matrix4x4& transform);

        // Add mesh data
        void AddMeshData(const PrimitiveTypes::Float32* vertices,
            PrimitiveTypes::Int32 vertexCount,
            const PrimitiveTypes::UInt16* indices,
            PrimitiveTypes::Int32 indexCount,
            const Matrix4x4& transform);

        // Finalize building
        void Finalize();

        // Pathfinding
        PrimitiveTypes::Bool FindPath(const Vector3& start, const Vector3& goal,
            Array<Vector3, 0>& outPath);

        // Find nearest walkable point
        PrimitiveTypes::Bool FindNearestPoint(const Vector3& position,
            Vector3& outPoint,
            PrimitiveTypes::Int32& outTriangleId);

        // Check if point is on navigation mesh
        PrimitiveTypes::Bool IsOnNavMesh(const Vector3& position,
            PrimitiveTypes::Float32 tolerance = 0.1f);

        

        // Parameter settings
        void SetMaxSlope(PrimitiveTypes::Float32 degrees) { m_maxSlope = degrees; }
        void SetMinTriangleArea(PrimitiveTypes::Float32 area) { m_minTriangleArea = area; }
        void MarkStaticObstacle(const Vector3& position, PrimitiveTypes::Float32 radius);

        // Get triangle count (for debugging)
        PrimitiveTypes::Int32 GetTriangleCount() { return m_triangles.m_size; }

    private:
        Array<NavTriangle, 1> m_triangles;  // Triangle array (value type)
        Array<Array<PrimitiveTypes::Int32, 1>, 1> m_triangleNeighbors;  // Neighbor ID list for each triangle

        PrimitiveTypes::Bool m_isInFindPath;  // Prevent recursive FindPath calls
        

        PrimitiveTypes::Float32 m_maxSlope;         // Maximum walkable slope (degrees)
        PrimitiveTypes::Float32 m_minTriangleArea;  // Minimum triangle area

        // Build adjacency information
        void BuildAdjacency();

        // Check if triangle is walkable
        PrimitiveTypes::Bool IsTriangleWalkable(const NavTriangle& triangle);

        // Find triangle containing point
        PrimitiveTypes::Int32 FindTriangleContainingPoint(const Vector3& point);

        // A* heuristic function
        PrimitiveTypes::Float32 Heuristic(const Vector3& a, const Vector3& b);

        // Check if edge is blocked by obstacle
        PrimitiveTypes::Bool IsEdgeBlocked(const Vector3& start, const Vector3& end);

        // Smooth path
        void SmoothPath(Array<Vector3, 0>& path);

        PE::MemoryArena m_arena;
        PE::GameContext* m_pContext;
    };

} // namespace CharacterControl

#endif
