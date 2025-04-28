#pragma once

#include <random>
#include "Engine.h"
#include "box3d/box3d.h"
#include "box3d/id.h"

template <typename ValueType>
class PhysicsEngine<Box3dEngine, ValueType, b3BodyId>
{
    typedef TVector3<ValueType> Vector3;

public:
    PhysicsEngine() = default;
    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;
    PhysicsEngine(PhysicsEngine&&) = delete;
    PhysicsEngine& operator=(PhysicsEngine&&) = delete;
    ~PhysicsEngine() = default;

    void Init(unsigned int seed);

    b3BodyId AddBodyAt(BodyType Type, const Vector3& Pos, ValueType ScaleFactor, bool bIsDynamic);
    void SetBodyPosition(b3BodyId Handle, const Vector3& Pos);
    bool Raycast(const Vector3& Start, const Vector3& Direction);
    void OptimizeBroadPhase();

protected:
    b3WorldId m_worldId;

    std::mt19937 rng;

    inline ValueType randomFloat(ValueType min, ValueType max)
    {
        std::uniform_real_distribution<ValueType> dist(min, max);
        return dist(rng);
    }

    inline int randomInt(int min, int max)
    {
        std::uniform_int_distribution<int> dist(min, max);
        return dist(rng);
    }
};

template <typename ValueType>
void PhysicsEngine<Box3dEngine, ValueType, b3BodyId>::Init(unsigned int seed)
{
    b3WorldDef worldDef = b3DefaultWorldDef();
    /*
    worldDef.workerCount = m_settings->workerCount;
    worldDef.enqueueTask = EnqueueTask;
    worldDef.finishTask = FinishTask;
    */
    worldDef.userTaskContext = this;
    worldDef.enableSleep = true;

    m_worldId = b3CreateWorld( &worldDef );
}

template <typename ValueType>
b3BodyId PhysicsEngine<Box3dEngine, ValueType, b3BodyId>::AddBodyAt(BodyType Type, const Vector3& Pos,
    ValueType ScaleFactor, bool bIsDynamic)
{
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

    bodyDef.position = { Pos.x, Pos.y, Pos.z };
    switch (Type)
    {
        case Box:
            {
                b3BoxHull boxHull = b3MakeBoxHull( {
                    randomFloat(0.5, 1) * ScaleFactor,
                    randomFloat(0.5, 1) * ScaleFactor,
                    randomFloat(0.5, 1) * ScaleFactor,
                });
                b3CreateHullShape( bodyId, &shapeDef, &boxHull.base );
            }
        break;

        case BodyType::Sphere:
            {
                b3Sphere sphere = { b3Vec3_zero, randomFloat(0.5, 1) * ScaleFactor };
                b3CreateSphereShape(
                    bodyId,
                    &shapeDef,
                    &sphere
                );
            }
        break;

        case Cylinder:
            {
                ValueType length = randomFloat(0.5, 1) * ScaleFactor;
                ValueType radius = randomFloat(0.5, 1) * ScaleFactor;
                b3Capsule capsule = { { -length / 2, 0.0f, 0.0f }, { length / 2, 0.0f, 0.0f }, radius };
                b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
            }
        break;
    }

    return bodyId;
}

template <typename ValueType>
void PhysicsEngine<Box3dEngine, ValueType, b3BodyId>::SetBodyPosition(b3BodyId Handle, const Vector3& Pos)
{
    b3Body_SetTransform(Handle, b3Vec3({Pos.x, Pos.y, Pos.z}), b3Quat_identity);
}

template <typename ValueType>
bool PhysicsEngine<Box3dEngine, ValueType, b3BodyId>::Raycast(const Vector3& Start, const Vector3& Direction)
{
    const ValueType length = 30000000;
    const b3Vec3 translation = b3Vec3({Direction.x * length, Direction.y * length, Direction.z * length});

    b3RayResult rayResult = b3World_CastRayClosest(m_worldId, b3Vec3({Start.x, Start.y, Start.z}), translation, b3DefaultQueryFilter());
    return rayResult.hit;
}

template <typename ValueType>
void PhysicsEngine<Box3dEngine, ValueType, b3BodyId>::OptimizeBroadPhase()
{
}
