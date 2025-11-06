#ifndef _CHARACTER_CONTROL_NAVIGATION_MANAGER_H_
#define _CHARACTER_CONTROL_NAVIGATION_MANAGER_H_

#include "PrimeEngine/Events/Component.h"
#include "NavigationMesh.h"
#include "PrimeEngine/Scene/Mesh.h"
#include "PrimeEngine/Geometry/MeshCPU/MeshCPU.h"

namespace CharacterControl {

    // Navigation mesh manager - Singleton pattern
    class NavigationManager
    {
    public:
        // Custom operator new for Handle
        void* operator new(size_t size, PE::Handle& h)
        {
            return h.getObject();
        }

        static NavigationManager* GetInstance();
        static void CreateInstance(PE::GameContext& context, PE::MemoryArena arena);
        static void DestroyInstance();

        // Initialize navigation mesh
        void Initialize();

        // Build from mesh CPU data
        void BuildFromMeshCPU(PE::MeshCPU* pMeshCPU, const Matrix4x4& transform);

        // Finalize building
        void Finalize();

        // Get navigation mesh
        NavigationMesh* GetNavMesh() { return m_pNavMesh; }

        // Update (called per frame)
        void Update(PrimitiveTypes::Float32 deltaTime);

        void MarkStaticObstacle(const Vector3& position, PrimitiveTypes::Float32 radius);

    private:
        NavigationManager(PE::GameContext& context, PE::MemoryArena arena);
        ~NavigationManager();

        static NavigationManager* s_pInstance;
        static PE::Handle s_hInstance;

        NavigationMesh* m_pNavMesh;
        PE::Handle m_hNavMesh;
        PE::MemoryArena m_arena;
        PE::GameContext* m_pContext;
    };

} // namespace CharacterControl

#endif
