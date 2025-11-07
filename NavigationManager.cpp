#include "NavigationManager.h"
#include "PrimeEngine/Geometry/PositionBufferCPU/PositionBufferCPU.h"
#include "PrimeEngine/Geometry/IndexBufferCPU/IndexBufferCPU.h"

namespace CharacterControl {

    NavigationManager* NavigationManager::s_pInstance = NULL;
    PE::Handle NavigationManager::s_hInstance;

    NavigationManager* NavigationManager::GetInstance()
    {
        return s_pInstance;
    }

    void NavigationManager::CreateInstance(PE::GameContext& context, PE::MemoryArena arena)
    {
        if (!s_pInstance)
        {
            // Create following SoldierNPC pattern
            s_hInstance = PE::Handle("NavigationManager", sizeof(NavigationManager));
            s_pInstance = new(s_hInstance) NavigationManager(context, arena);
        }
    }

    void NavigationManager::DestroyInstance()
    {
        if (s_pInstance)
        {
            s_pInstance->~NavigationManager();
            s_hInstance.release();
            s_pInstance = NULL;
        }
    }

    NavigationManager::NavigationManager(PE::GameContext& context, PE::MemoryArena arena)
        : m_pNavMesh(NULL)
        , m_arena(arena)
        , m_pContext(&context)
    {
    }

    NavigationManager::~NavigationManager()
    {
        if (m_hNavMesh.isValid())
        {
            m_hNavMesh.release();
        }
        m_pNavMesh = NULL;
    }

    void NavigationManager::Initialize()
    {
        if (!m_pNavMesh)
        {
            // Create NavigationMesh following SoldierNPC pattern
            m_hNavMesh = PE::Handle("NavigationMesh", sizeof(NavigationMesh));
            m_pNavMesh = new(m_hNavMesh) NavigationMesh(*m_pContext, m_arena);

            // Set default parameters
            m_pNavMesh->SetMaxSlope(45.0f);
            m_pNavMesh->SetMinTriangleArea(0.1f);
        }
    }

    void NavigationManager::BuildFromMeshCPU(PE::MeshCPU* pMeshCPU, const Matrix4x4& transform)
    {
        if (!pMeshCPU || !m_pNavMesh) return;

        // Get position buffer from MeshCPU
        PE::Handle hPositionBuffer = pMeshCPU->m_hPositionBufferCPU;
        PE::Handle hIndexBuffer = pMeshCPU->m_hIndexBufferCPU;

        if (!hPositionBuffer.isValid() || !hIndexBuffer.isValid()) return;

        PositionBufferCPU* pPositionBuffer = hPositionBuffer.getObject<PositionBufferCPU>();
        IndexBufferCPU* pIndexBuffer = hIndexBuffer.getObject<IndexBufferCPU>();

        if (!pPositionBuffer || !pIndexBuffer) return;

        // Get vertex data
        const PrimitiveTypes::Float32* vertices = &pPositionBuffer->m_values[0];
        PrimitiveTypes::Int32 vertexCount = pPositionBuffer->m_values.m_size / 3;

        // Get index data
        const PrimitiveTypes::UInt16* indices = &pIndexBuffer->m_values[0];
        PrimitiveTypes::Int32 indexCount = pIndexBuffer->m_values.m_size;

        // Add to navigation mesh
        m_pNavMesh->AddMeshData(vertices, vertexCount, indices, indexCount, transform);

        PEINFO("NavigationManager: Added mesh to navmesh - vertices: %d, indices: %d\n",
            vertexCount, indexCount);
    }

    void NavigationManager::Finalize()
    {
        if (m_pNavMesh)
        {
            m_pNavMesh->Finalize();
            PEINFO("NavigationManager: Navmesh finalized with %d triangles\n",
                m_pNavMesh->GetTriangleCount());
        }
    }

    void NavigationManager::Update(PrimitiveTypes::Float32 deltaTime)
    {
        // Can add update logic here
    }


    void NavigationManager::MarkStaticObstacle(const Vector3& position, PrimitiveTypes::Float32 radius)
    {
        if (m_pNavMesh) {
            m_pNavMesh->MarkStaticObstacle(position, radius);
            PEINFO("NavigationManager: Marked static obstacle at (%.2f, %.2f, %.2f) with radius %.2f\n",
                position.m_x, position.m_y, position.m_z, radius);
        }
    }
  


} // namespace CharacterControl
