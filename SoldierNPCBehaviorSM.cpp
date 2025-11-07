#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Scene/DebugRenderer.h"
#include "../ClientGameObjectManagerAddon.h"
#include "../CharacterControlContext.h"
#include "SoldierNPCMovementSM.h"
#include "SoldierNPCAnimationSM.h"
#include "SoldierNPCBehaviorSM.h"
#include "SoldierNPC.h"
#include "PrimeEngine/Scene/SceneNode.h"
#include "PrimeEngine/Render/IRenderer.h"

using namespace PE::Components;
using namespace PE::Events;
using namespace CharacterControl::Events;

namespace CharacterControl {
    namespace Components {

        // External global variable (defined in SoldierNPC.cpp)
        extern PE::Handle g_hPlayerSoldier;

        PE_IMPLEMENT_CLASS1(SoldierNPCBehaviorSM, Component);

        SoldierNPCBehaviorSM::SoldierNPCBehaviorSM(PE::GameContext& context, PE::MemoryArena arena,
            PE::Handle hMyself, PE::Handle hMovementSM)
            : Component(context, arena, hMyself)
            , m_hMovementSM(hMovementSM)
            , m_followPlayer(true)
            , m_followDistance(3.0f)
            , m_pathUpdateTimer(0.0f)
            , m_pathUpdateInterval(0.5f)
            , m_pNavMesh(NULL)
            , m_currentPath(context, arena)
        {
        }

        void SoldierNPCBehaviorSM::start()
        {
            if (m_followPlayer)
            {
                m_state = FOLLOWING_PLAYER;
            }
            else if (m_havePatrolWayPoint)
            {
                m_state = WAITING_FOR_WAYPOINT;
            }
            else
            {
                m_state = IDLE;

                PE::Handle h("SoldierNPCMovementSM_Event_STOP", sizeof(SoldierNPCMovementSM_Event_STOP));
                SoldierNPCMovementSM_Event_STOP* pEvt = new(h) SoldierNPCMovementSM_Event_STOP();
                m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
                h.release();
            }
        }

        void SoldierNPCBehaviorSM::addDefaultComponents()
        {
            Component::addDefaultComponents();

            PE_REGISTER_EVENT_HANDLER(SoldierNPCMovementSM_Event_TARGET_REACHED,
                SoldierNPCBehaviorSM::do_SoldierNPCMovementSM_Event_TARGET_REACHED);
            PE_REGISTER_EVENT_HANDLER(Event_UPDATE, SoldierNPCBehaviorSM::do_UPDATE);
            PE_REGISTER_EVENT_HANDLER(Event_PRE_RENDER_needsRC, SoldierNPCBehaviorSM::do_PRE_RENDER_needsRC);
        }

        void SoldierNPCBehaviorSM::do_SoldierNPCMovementSM_Event_TARGET_REACHED(PE::Events::Event* pEvt)
        {
            if (m_state == FOLLOWING_PLAYER)
            {
                // When following player, do nothing on target reached
                // Path will be continuously updated in UPDATE
            }
            else if (m_state == PATROLLING_WAYPOINTS)
            {
                ClientGameObjectManagerAddon* pGameObjectManagerAddon =
                    (ClientGameObjectManagerAddon*)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon());
                if (pGameObjectManagerAddon)
                {
                    WayPoint* pWP = pGameObjectManagerAddon->getWayPoint(m_curPatrolWayPoint);
                    if (pWP && StringOps::length(pWP->m_nextWayPointName) > 0)
                    {
                        pWP = pGameObjectManagerAddon->getWayPoint(pWP->m_nextWayPointName);
                        if (pWP)
                        {
                            StringOps::writeToString(pWP->m_name, m_curPatrolWayPoint, 32);
                            m_state = PATROLLING_WAYPOINTS;

                            PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
                            Events::SoldierNPCMovementSM_Event_MOVE_TO* pEvt =
                                new(h) SoldierNPCMovementSM_Event_MOVE_TO(pWP->m_base.getPos());
                            m_hMovementSM.getObject<Component>()->handleEvent(pEvt);
                            h.release();
                        }
                    }
                    else
                    {
                        m_state = IDLE;
                    }
                }
            }
        }

        void SoldierNPCBehaviorSM::do_PRE_RENDER_needsRC(PE::Events::Event* pEvt)
        {
            Event_PRE_RENDER_needsRC* pRealEvent = (Event_PRE_RENDER_needsRC*)(pEvt);

            SoldierNPC* pSol = getFirstParentByTypePtr<SoldierNPC>();
            PE::Handle hSoldierSceneNode = pSol->getFirstComponentHandle<PE::Components::SceneNode>();
            Matrix4x4 base = hSoldierSceneNode.getObject<PE::Components::SceneNode>()->m_worldTransform;

            if (m_state == FOLLOWING_PLAYER)
            {
                DebugRenderer::Instance()->createTextMesh(
                    "Following Player", false, false, true, false, 0,
                    base.getPos(), 0.01f, pRealEvent->m_threadOwnershipMask);

                if (g_hPlayerSoldier.isValid())
                {
                    SoldierNPC* pPlayerSoldier = g_hPlayerSoldier.getObject<SoldierNPC>();
                    if (pPlayerSoldier)
                    {
                        PE::Handle hPlayerSN = pPlayerSoldier->getFirstComponentHandle<PE::Components::SceneNode>();
                        if (hPlayerSN.isValid())
                        {
                            SceneNode* pPlayerSN = hPlayerSN.getObject<SceneNode>();
                            Vector3 target = pPlayerSN->m_base.getPos();
                            Vector3 pos = base.getPos();
                            Vector3 color(0.0f, 1.0f, 0.0f);
                            Vector3 linepts[] = { pos, color, target, color };

                            DebugRenderer::Instance()->createLineMesh(true, base, &linepts[0].m_x, 2, 0);
                        }
                        else
                        {
                            DebugRenderer::Instance()->createLineMesh(true, base, NULL, 0, 0);
                        }
                    }
                }
                else
                {
                    DebugRenderer::Instance()->createLineMesh(true, base, NULL, 0, 0);
                }
            }
            else if (m_havePatrolWayPoint)
            {
                char buf[80];
                sprintf(buf, "Patrol Waypoint: %s", m_curPatrolWayPoint);

                DebugRenderer::Instance()->createTextMesh(
                    buf, false, false, true, false, 0,
                    base.getPos(), 0.01f, pRealEvent->m_threadOwnershipMask);

                {
                    bool sent = false;
                    ClientGameObjectManagerAddon* pGameObjectManagerAddon =
                        (ClientGameObjectManagerAddon*)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon());
                    if (pGameObjectManagerAddon)
                    {
                        WayPoint* pWP = pGameObjectManagerAddon->getWayPoint(m_curPatrolWayPoint);
                        if (pWP)
                        {
                            Vector3 target = pWP->m_base.getPos();
                            Vector3 pos = base.getPos();
                            Vector3 color(1.0f, 1.0f, 0);
                            Vector3 linepts[] = { pos, color, target, color };

                            DebugRenderer::Instance()->createLineMesh(true, base, &linepts[0].m_x, 2, 0);
                            sent = true;
                        }
                    }
                    if (!sent) {
                        DebugRenderer::Instance()->createLineMesh(true, base, NULL, 0, 0);
                    }
                }
            }
        }

        void SoldierNPCBehaviorSM::do_UPDATE(PE::Events::Event* pEvt)
        {
            if (m_state == FOLLOWING_PLAYER)
            {
                // Follow player logic - using global handle instead of PlayerNPC
                if (!g_hPlayerSoldier.isValid())
                {
                    // No player, switch to IDLE
                    m_state = IDLE;

                    PE::Handle h("SoldierNPCMovementSM_Event_STOP", sizeof(SoldierNPCMovementSM_Event_STOP));
                    SoldierNPCMovementSM_Event_STOP* pStopEvt = new(h) SoldierNPCMovementSM_Event_STOP();
                    m_hMovementSM.getObject<Component>()->handleEvent(pStopEvt);
                    h.release();
                    return;
                }

                SoldierNPC* pPlayerSoldier = g_hPlayerSoldier.getObject<SoldierNPC>();
                if (!pPlayerSoldier)
                {
                    m_state = IDLE;
                    return;
                }

                PE::Handle hPlayerSN = pPlayerSoldier->getFirstComponentHandle<PE::Components::SceneNode>();
                if (!hPlayerSN.isValid())
                {
                    return;
                }
                SceneNode* pPlayerSN = hPlayerSN.getObject<SceneNode>();
                Vector3 playerPos = pPlayerSN->m_base.getPos();

                SoldierNPC* pSol = getFirstParentByTypePtr<SoldierNPC>();
                PE::Handle hSoldierSN = pSol->getFirstComponentHandle<PE::Components::SceneNode>();
                SceneNode* pSoldierSN = hSoldierSN.getObject<SceneNode>();
                Vector3 soldierPos = pSoldierSN->m_base.getPos();

                Vector3 toPlayer = playerPos - soldierPos;
                toPlayer.m_y = 0;
                PrimitiveTypes::Float32 distance = toPlayer.length();

                Event_UPDATE* pUpdateEvt = (Event_UPDATE*)(pEvt);
                m_pathUpdateTimer += pUpdateEvt->m_frameTime;

                if (distance > m_followDistance * 1.5f || m_pathUpdateTimer >= m_pathUpdateInterval)
                {
                    m_pathUpdateTimer = 0.0f;

                    if (m_pNavMesh)
                    {
                        Vector3 targetPos = playerPos;

                        // Calculate target position (maintain distance around player)
                        if (distance > m_followDistance)
                        {
                            Vector3 direction = toPlayer / distance;
                            targetPos = playerPos - direction * m_followDistance;
                        }

                        m_currentPath.reset(0);

                        if (m_pNavMesh->FindPath(soldierPos, targetPos, m_currentPath))
                        {
                            // If path found, move to next waypoint
                            if (m_currentPath.m_size > 1)
                            {
                                PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
                                Events::SoldierNPCMovementSM_Event_MOVE_TO* pMoveEvt =
                                    new(h) SoldierNPCMovementSM_Event_MOVE_TO(m_currentPath[1]);
                                m_hMovementSM.getObject<Component>()->handleEvent(pMoveEvt);
                                h.release();
                            }
                        }
                        else
                        {
                            // If no path found, try direct movement
                            if (distance > m_followDistance)
                            {
                                PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
                                Events::SoldierNPCMovementSM_Event_MOVE_TO* pMoveEvt =
                                    new(h) SoldierNPCMovementSM_Event_MOVE_TO(targetPos);
                                m_hMovementSM.getObject<Component>()->handleEvent(pMoveEvt);
                                h.release();
                            }
                        }
                    }
                    else
                    {
                        // No navigation mesh, move directly
                        if (distance > m_followDistance)
                        {
                            Vector3 direction = toPlayer / distance;
                            Vector3 targetPos = playerPos - direction * m_followDistance;

                            PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
                            Events::SoldierNPCMovementSM_Event_MOVE_TO* pMoveEvt =
                                new(h) SoldierNPCMovementSM_Event_MOVE_TO(targetPos);
                            m_hMovementSM.getObject<Component>()->handleEvent(pMoveEvt);
                            h.release();
                        }
                    }
                }
            }
            else if (m_state == WAITING_FOR_WAYPOINT)
            {
                if (m_havePatrolWayPoint)
                {
                    ClientGameObjectManagerAddon* pGameObjectManagerAddon =
                        (ClientGameObjectManagerAddon*)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon());
                    if (pGameObjectManagerAddon)
                    {
                        WayPoint* pWP = pGameObjectManagerAddon->getWayPoint(m_curPatrolWayPoint);
                        if (pWP)
                        {
                            m_state = PATROLLING_WAYPOINTS;
                            PE::Handle h("SoldierNPCMovementSM_Event_MOVE_TO", sizeof(SoldierNPCMovementSM_Event_MOVE_TO));
                            Events::SoldierNPCMovementSM_Event_MOVE_TO* pMoveEvt =
                                new(h) SoldierNPCMovementSM_Event_MOVE_TO(pWP->m_base.getPos());
                            m_hMovementSM.getObject<Component>()->handleEvent(pMoveEvt);
                            h.release();
                        }
                    }
                }
                else
                {
                    m_state = IDLE;

                    PE::Handle h("SoldierNPCMovementSM_Event_STOP", sizeof(SoldierNPCMovementSM_Event_STOP));
                    SoldierNPCMovementSM_Event_STOP* pStopEvt = new(h) SoldierNPCMovementSM_Event_STOP();
                    m_hMovementSM.getObject<Component>()->handleEvent(pStopEvt);
                    h.release();
                }
            }
        }

    }
}
