#include "NavigationMesh.h"
#include "PrimeEngine/Geometry/MeshCPU/MeshCPU.h"
#include "PrimeEngine/Geometry/PositionBufferCPU/PositionBufferCPU.h"
#include "PrimeEngine/Geometry/IndexBufferCPU/IndexBufferCPU.h"
#include <float.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

namespace CharacterControl {

    void NavTriangle::Calculate()
    {
        center = (vertices[0] + vertices[1] + vertices[2]) / 3.0f;

        Vector3 edge1 = vertices[1] - vertices[0];
        Vector3 edge2 = vertices[2] - vertices[0];
        normal = edge1.crossProduct(edge2);
        normal.normalize();
    }

    PrimitiveTypes::Bool NavTriangle::ContainsPoint(const Vector3& point) const
    {
        // Use barycentric coordinates for detection
        Vector3 v0 = vertices[2] - vertices[0];
        Vector3 v1 = vertices[1] - vertices[0];
        Vector3 v2 = point - vertices[0];

        PrimitiveTypes::Float32 dot00 = v0.dotProduct(v0);
        PrimitiveTypes::Float32 dot01 = v0.dotProduct(v1);
        PrimitiveTypes::Float32 dot02 = v0.dotProduct(v2);
        PrimitiveTypes::Float32 dot11 = v1.dotProduct(v1);
        PrimitiveTypes::Float32 dot12 = v1.dotProduct(v2);

        PrimitiveTypes::Float32 invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        PrimitiveTypes::Float32 u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        PrimitiveTypes::Float32 v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    PrimitiveTypes::Bool NavTriangle::GetSharedEdge(const NavTriangle& other,
        Vector3& edgeStart, Vector3& edgeEnd) const
    {
        PrimitiveTypes::Int32 sharedCount = 0;
        Vector3 sharedVerts[2];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Vector3 diff = vertices[i] - other.vertices[j];
                if (diff.lengthSqr() < 0.0001f) {
                    if (sharedCount < 2) {
                        sharedVerts[sharedCount] = vertices[i];
                    }
                    sharedCount++;
                }
            }
        }

        if (sharedCount == 2) {
            edgeStart = sharedVerts[0];
            edgeEnd = sharedVerts[1];
            return true;
        }

        return false;
    }

    NavigationMesh::NavigationMesh(PE::GameContext& context, PE::MemoryArena arena)
        : m_triangles(context, arena, 100)
        , m_triangleNeighbors(context, arena, 100)
        , m_maxSlope(45.0f)
        , m_minTriangleArea(0.01f)
        , m_isInFindPath(false)
    {
        m_arena = arena;
        m_pContext = &context;
    }

    NavigationMesh::~NavigationMesh()
    {
    }

    void NavigationMesh::Initialize()
    {
    }

    void NavigationMesh::BuildFromMeshCPU(PE::MeshCPU* pMeshCPU, const Matrix4x4& transform)
    {
        if (!pMeshCPU || !pMeshCPU->m_hPositionBufferCPU.isValid() ||
            !pMeshCPU->m_hIndexBufferCPU.isValid())
        {
            return;
        }

        PositionBufferCPU* pPosBuf =
            pMeshCPU->m_hPositionBufferCPU.getObject<PositionBufferCPU>();
        IndexBufferCPU* pIdxBuf =
            pMeshCPU->m_hIndexBufferCPU.getObject<IndexBufferCPU>();

        BuildFromMeshBuffers(pPosBuf->m_values.getFirstPtr(),
            pPosBuf->m_values.m_size / 3,
            pIdxBuf->m_values.getFirstPtr(),
            pIdxBuf->m_values.m_size,
            transform);
    }

    void NavigationMesh::BuildFromMeshBuffers(const PrimitiveTypes::Float32* vertices,
        PrimitiveTypes::Int32 vertexCount,
        const PrimitiveTypes::UInt16* indices,
        PrimitiveTypes::Int32 indexCount,
        const Matrix4x4& transform)
    {
        m_triangles.clear();
        m_triangleNeighbors.clear();
        AddMeshData(vertices, vertexCount, indices, indexCount, transform);
        Finalize();
    }

    void NavigationMesh::AddMeshData(const PrimitiveTypes::Float32* vertices,
        PrimitiveTypes::Int32 vertexCount,
        const PrimitiveTypes::UInt16* indices,
        PrimitiveTypes::Int32 indexCount,
        const Matrix4x4& transform)
    {
        PrimitiveTypes::Int32 baseTriangleId = m_triangles.m_size;

        for (PrimitiveTypes::Int32 i = 0; i < indexCount; i += 3) {
            NavTriangle triangle;
            triangle.id = baseTriangleId + (i / 3);

            for (int j = 0; j < 3; j++) {
                PrimitiveTypes::Int32 idx = indices[i + j] * 3;
                Vector3 v(vertices[idx], vertices[idx + 1], vertices[idx + 2]);
                triangle.vertices[j] = transform * v;
            }

            triangle.Calculate();

            Vector3 edge1 = triangle.vertices[1] - triangle.vertices[0];
            Vector3 edge2 = triangle.vertices[2] - triangle.vertices[0];
            PrimitiveTypes::Float32 area = edge1.crossProduct(edge2).length() * 0.5f;

            if (area >= m_minTriangleArea && IsTriangleWalkable(triangle)) {
                m_triangles.add(triangle);
                Array<PrimitiveTypes::Int32, 1> neighbors(*m_pContext, m_arena, 3);
                m_triangleNeighbors.add(neighbors);
            }
        }
    }

    void NavigationMesh::Finalize()
    {
        BuildAdjacency();
    }

    void NavigationMesh::BuildAdjacency()
    {
        for (PrimitiveTypes::UInt32 i = 0; i < m_triangles.m_size; i++) {
            m_triangleNeighbors[i].clear();

            for (PrimitiveTypes::UInt32 j = 0; j < m_triangles.m_size; j++) {
                if (i == j) continue;

                Vector3 edgeStart, edgeEnd;
                if (m_triangles[i].GetSharedEdge(m_triangles[j], edgeStart, edgeEnd)) {
                    m_triangleNeighbors[i].add(m_triangles[j].id);
                }
            }
        }
    }

    PrimitiveTypes::Bool NavigationMesh::IsTriangleWalkable(const NavTriangle& triangle)
    {
        Vector3 up(0, 1, 0);
        Vector3 normal = triangle.normal;
        PrimitiveTypes::Float32 angle = acos(normal.dotProduct(up)) * 180.0f / PI;
        return angle <= m_maxSlope;
    }

    PrimitiveTypes::Int32 NavigationMesh::FindTriangleContainingPoint(const Vector3& point)
    {
        PrimitiveTypes::Float32 minDist = FLT_MAX;
        PrimitiveTypes::Int32 bestTriangle = -1;

        for (PrimitiveTypes::UInt32 i = 0; i < m_triangles.m_size; i++) {
            if (!m_triangles[i].isTraversable) continue;

            if (m_triangles[i].ContainsPoint(point)) {
                Vector3 diff = point - m_triangles[i].vertices[0];
                Vector3 normal = m_triangles[i].normal;
                PrimitiveTypes::Float32 dist = fabs(diff.dotProduct(normal));
                if (dist < minDist) {
                    minDist = dist;
                    bestTriangle = i;
                }
            }
        }

        if (bestTriangle == -1) {
            for (PrimitiveTypes::UInt32 i = 0; i < m_triangles.m_size; i++) {
                if (!m_triangles[i].isTraversable) continue;

                PrimitiveTypes::Float32 dist = (point - m_triangles[i].center).lengthSqr();
                if (dist < minDist) {
                    minDist = dist;
                    bestTriangle = i;
                }
            }
        }

        return bestTriangle;
    }

    PrimitiveTypes::Bool NavigationMesh::FindNearestPoint(const Vector3& position,
        Vector3& outPoint,
        PrimitiveTypes::Int32& outTriangleId)
    {
        outTriangleId = FindTriangleContainingPoint(position);
        if (outTriangleId < 0) return false;

        NavTriangle& tri = m_triangles[outTriangleId];
        Vector3 normal = tri.normal;
        Vector3 diff = position - tri.vertices[0];
        PrimitiveTypes::Float32 dist = diff.dotProduct(normal);
        Vector3 offset = normal * dist;
        outPoint = position - offset;

        return true;
    }

    PrimitiveTypes::Bool NavigationMesh::IsOnNavMesh(const Vector3& position,
        PrimitiveTypes::Float32 tolerance)
    {
        PrimitiveTypes::Int32 triangleId = FindTriangleContainingPoint(position);
        if (triangleId < 0) return false;

        NavTriangle& tri = m_triangles[triangleId];
        Vector3 normal = tri.normal;
        Vector3 diff = position - tri.vertices[0];
        PrimitiveTypes::Float32 dist = fabs(diff.dotProduct(normal));
        return dist <= tolerance;
    }

    PrimitiveTypes::Float32 NavigationMesh::Heuristic(const Vector3& a, const Vector3& b)
    {
        return (b - a).length();
    }

    PrimitiveTypes::Bool NavigationMesh::IsEdgeBlocked(const Vector3& start, const Vector3& end)
    {
        return false;  
    }

    PrimitiveTypes::Bool NavigationMesh::FindPath(const Vector3& start, const Vector3& goal,
        Array<Vector3, 0>& outPath)
    {
        if (m_isInFindPath) {
            return false;
        }
        m_isInFindPath = true;

        if (m_triangles.m_size == 0) {
            m_isInFindPath = false;
            return false;
        }

        outPath.clear();
        outPath.reset(m_triangles.m_size + 10);

        PrimitiveTypes::Int32 startTriId = FindTriangleContainingPoint(start);
        PrimitiveTypes::Int32 goalTriId = FindTriangleContainingPoint(goal);

        if (startTriId < 0 || goalTriId < 0) {
            m_isInFindPath = false;
            return false;
        }

        if (startTriId == goalTriId) {
            Vector3 direction = goal - start;
            PrimitiveTypes::Float32 totalDist = direction.length();

            if (totalDist > 0.01f) {
                direction = direction / totalDist;
                bool pathBlocked = false;

                // Sample along path every 0.5 meters
                for (PrimitiveTypes::Float32 dist = 0; dist <= totalDist; dist += 0.5f) {
                    Vector3 samplePoint = start + direction * dist;

                    // Check if sample point is near any non-traversable triangles
                    for (PrimitiveTypes::UInt32 i = 0; i < m_triangles.m_size; i++) {
                        if (!m_triangles[i].isTraversable) {
                            Vector3 diff = samplePoint - m_triangles[i].center;
                            PrimitiveTypes::Float32 distToObstacle = diff.length();

                            if (distToObstacle < 3.0f) {
                                pathBlocked = true;
                                break;
                            }
                        }
                    }
                    if (pathBlocked) break;
                }

                if (!pathBlocked) {
                    outPath.add(start);
                    outPath.add(goal);
                    m_isInFindPath = false;
                    return true;
                }
            }
            else {
                outPath.add(start);
                m_isInFindPath = false;
                return true;
            }
        }

        Array<PathNode, 0> openSet(*m_pContext, m_arena);
        Array<PrimitiveTypes::Bool, 0> closedSet(*m_pContext, m_arena);
        Array<PathNode, 0> nodeMap(*m_pContext, m_arena);

        openSet.reset(m_triangles.m_size);
        closedSet.reset(m_triangles.m_size);
        nodeMap.reset(m_triangles.m_size);

        for (PrimitiveTypes::UInt32 i = 0; i < m_triangles.m_size; i++) {
            closedSet.add(false);
            nodeMap.add(PathNode());
        }

        // If start and goal in same triangle and path blocked, start from neighbors
        if (startTriId == goalTriId) {
            closedSet[startTriId] = true;

            Array<PrimitiveTypes::Int32, 1>& neighbors = m_triangleNeighbors[startTriId];
            for (PrimitiveTypes::UInt32 i = 0; i < neighbors.m_size; i++) {
                PrimitiveTypes::Int32 neighborId = neighbors[i];

                if (m_triangles[neighborId].isTraversable) {
                    Vector3 diff = m_triangles[neighborId].center - start;
                    PrimitiveTypes::Float32 gCost = diff.length();
                    PrimitiveTypes::Float32 hCost = Heuristic(m_triangles[neighborId].center, goal);

                    PathNode neighborNode(neighborId, gCost, hCost, startTriId);
                    openSet.add(neighborNode);
                    nodeMap[neighborId] = neighborNode;
                }
            }

            if (openSet.m_size == 0) {
                m_isInFindPath = false;
                return false;
            }
        }
        else {
            PathNode startNode(startTriId, 0,
                Heuristic(m_triangles[startTriId].center, m_triangles[goalTriId].center), -1);
            openSet.add(startNode);
            nodeMap[startTriId] = startNode;
        }

        while (openSet.m_size > 0) {
            PrimitiveTypes::Int32 currentIdx = 0;
            for (PrimitiveTypes::UInt32 i = 1; i < openSet.m_size; i++) {
                if (openSet[i].fCost() < openSet[currentIdx].fCost()) {
                    currentIdx = i;
                }
            }

            PathNode current = openSet[currentIdx];
            openSet.remove(currentIdx);

            if (closedSet[current.triangleId]) {
                continue;
            }

            closedSet[current.triangleId] = true;

            bool reachedGoal = false;
            if (current.triangleId == goalTriId) {
                reachedGoal = true;
            }
            else {
                Array<PrimitiveTypes::Int32, 1>& neighbors = m_triangleNeighbors[current.triangleId];
                for (PrimitiveTypes::UInt32 i = 0; i < neighbors.m_size; i++) {
                    if (neighbors[i] == goalTriId) {
                        reachedGoal = true;
                        break;
                    }
                }
            }

            if (reachedGoal) {
                Array<PrimitiveTypes::Int32, 0> trianglePath(*m_pContext, m_arena);
                trianglePath.reset(m_triangles.m_size);

                PrimitiveTypes::Int32 nodeId = current.triangleId;
                PrimitiveTypes::UInt32 pathSteps = 0;
                PrimitiveTypes::UInt32 maxPathSteps = m_triangles.m_size + 10;

                while (nodeId != -1 && pathSteps < maxPathSteps) {
                    trianglePath.add(nodeId);

                    PrimitiveTypes::Int32 nextNode = nodeMap[nodeId].parent;

                    if (nextNode == nodeId) {
                        break;
                    }

                    nodeId = nextNode;
                    pathSteps++;
                }

                if (pathSteps >= maxPathSteps) {
                    m_isInFindPath = false;
                    return false;
                }

                outPath.add(start);
                for (PrimitiveTypes::Int32 i = trianglePath.m_size - 1; i >= 0; i--) {
                    outPath.add(m_triangles[trianglePath[i]].center);
                }
                outPath.add(goal);

                m_isInFindPath = false;
                return true;
            }

            Array<PrimitiveTypes::Int32, 1>& neighbors = m_triangleNeighbors[current.triangleId];
            for (PrimitiveTypes::UInt32 i = 0; i < neighbors.m_size; i++) {
                PrimitiveTypes::Int32 neighborId = neighbors[i];

                if (closedSet[neighborId]) {
                    continue;
                }

                if (!m_triangles[neighborId].isTraversable) {
                    continue;
                }

                Vector3 centerDiff = m_triangles[neighborId].center - m_triangles[current.triangleId].center;
                PrimitiveTypes::Float32 newGCost = current.gCost +
                    centerDiff.length() * m_triangles[neighborId].cost;

                if (nodeMap[neighborId].triangleId == -1 || newGCost < nodeMap[neighborId].gCost) {
                    PrimitiveTypes::Float32 hCost = Heuristic(m_triangles[neighborId].center,
                        m_triangles[goalTriId].center);
                    PathNode neighborNode(neighborId, newGCost, hCost, current.triangleId);
                    openSet.add(neighborNode);
                    nodeMap[neighborId] = neighborNode;
                }
            }
        }

        m_isInFindPath = false;
        return false;
    }

    void NavigationMesh::SmoothPath(Array<Vector3, 0>& path)
    {
        if (path.m_size <= 2) return;

        Array<Vector3, 1> smoothed(*m_pContext, m_arena);
        smoothed.add(path[0]);

        PrimitiveTypes::UInt32 current = 0;
        while (current < path.m_size - 1) {
            PrimitiveTypes::UInt32 farthest = current + 1;

            for (PrimitiveTypes::UInt32 i = current + 2; i < path.m_size; i++) {
                if (!IsEdgeBlocked(path[current], path[i])) {
                    farthest = i;
                }
                else {
                    break;
                }
            }

            smoothed.add(path[farthest]);
            current = farthest;
        }

        path.clear();
        for (PrimitiveTypes::UInt32 i = 0; i < smoothed.m_size; i++) {
            path.add(smoothed[i]);
        }
    }

    void NavigationMesh::MarkStaticObstacle(const Vector3& position, PrimitiveTypes::Float32 radius)
    {
        for (PrimitiveTypes::UInt32 i = 0; i < m_triangles.m_size; i++)
        {
            Vector3 diff = m_triangles[i].center - position;
            PrimitiveTypes::Float32 distance = diff.length();

            if (distance <= radius)
            {
                m_triangles[i].isTraversable = false;
            }
        }
    }

} // namespace CharacterControl
