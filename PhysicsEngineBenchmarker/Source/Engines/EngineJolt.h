#include "Engine.h"
#include "Jolt/RegisterTypes.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/Core/TempAllocator.h"
#include "Jolt/Physics/EActivation.h"
#include "Jolt/Physics/PhysicsSettings.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/CastResult.h"
#include "Jolt/Physics/Collision/ContactListener.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/Collision/RayCast.h"
#include "Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/Collision/Shape/SphereShape.h"

namespace Layers
{
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
    ObjectLayerPairFilterImpl(const ObjectLayerPairFilterImpl&) = delete;
    ObjectLayerPairFilterImpl& operator=(const ObjectLayerPairFilterImpl&) = delete;
    ObjectLayerPairFilterImpl(ObjectLayerPairFilterImpl&&) = delete;
    ObjectLayerPairFilterImpl& operator=(ObjectLayerPairFilterImpl&&) = delete;
    ObjectLayerPairFilterImpl() = default;
    virtual ~ObjectLayerPairFilterImpl() override = default;

    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
        case Layers::NON_MOVING:
            return inObject2 == Layers::MOVING; // Non moving only collides with moving
        case Layers::MOVING:
            return true; // Moving collides with everything
        default:
            return false;
        }
    }
};

namespace BroadPhaseLayers
{
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::uint NUM_LAYERS(2);
};

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl(const BPLayerInterfaceImpl&) = delete;
    BPLayerInterfaceImpl& operator=(const BPLayerInterfaceImpl&) = delete;
    BPLayerInterfaceImpl(BPLayerInterfaceImpl&&) = delete;
    BPLayerInterfaceImpl& operator=(BPLayerInterfaceImpl&&) = delete;
    virtual ~BPLayerInterfaceImpl() override = default;

    BPLayerInterfaceImpl()
    {
        // Create a mapping table from object to broad phase layer
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    virtual JPH::uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
    {
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        switch ((JPH::BroadPhaseLayer::Type)inLayer)
        {
        case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
        case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING: return "MOVING";
        default:
            return "INVALID";
        }
    }
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    ObjectVsBroadPhaseLayerFilterImpl() = default;
    ObjectVsBroadPhaseLayerFilterImpl(const ObjectVsBroadPhaseLayerFilterImpl&) = delete;
    ObjectVsBroadPhaseLayerFilterImpl& operator=(const ObjectVsBroadPhaseLayerFilterImpl&) = delete;
    ObjectVsBroadPhaseLayerFilterImpl(ObjectVsBroadPhaseLayerFilterImpl&&) = delete;
    ObjectVsBroadPhaseLayerFilterImpl& operator=(ObjectVsBroadPhaseLayerFilterImpl&&) = delete;
    virtual ~ObjectVsBroadPhaseLayerFilterImpl() override = default;

    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case Layers::NON_MOVING:
            return inLayer2 == BroadPhaseLayers::MOVING;
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

// An example contact listener
class MyContactListener : public JPH::ContactListener
{
public:
    MyContactListener() = default;
    MyContactListener(const ObjectVsBroadPhaseLayerFilterImpl&) = delete;
    MyContactListener& operator=(const ObjectVsBroadPhaseLayerFilterImpl&) = delete;
    MyContactListener(ObjectVsBroadPhaseLayerFilterImpl&&) = delete;
    MyContactListener& operator=(ObjectVsBroadPhaseLayerFilterImpl&&) = delete;
    virtual ~MyContactListener() override = default;

    // See: ContactListener
    virtual JPH::ValidateResult OnContactValidate(const JPH::Body& , const JPH::Body& , JPH::RVec3Arg ,
                                                  const JPH::CollideShapeResult& ) override
    {
        // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    virtual void OnContactAdded(const JPH::Body& , const JPH::Body& , const JPH::ContactManifold& ,
                                JPH::ContactSettings& ) override
    {
    }

    virtual void OnContactPersisted(const JPH::Body& , const JPH::Body& , const JPH::ContactManifold& ,
                                    JPH::ContactSettings& ) override
    {
    }

    virtual void OnContactRemoved(const JPH::SubShapeIDPair& ) override
    {
    }
};

template <typename ValueType>
class PhysicsEngine<JoltEngine, ValueType, JPH::BodyID>
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

    JPH::BodyID AddBodyAt(BodyType Type, const Vector3& Pos, ValueType ScaleFactor, bool bIsDynamic);
    void SetBodyPosition(JPH::BodyID Handle, const Vector3& Pos);
    bool Raycast(const Vector3& Start, const Vector3& Direction);
    void OptimizeBroadPhase();

protected:
    JPH::PhysicsSystem physics_system;

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

    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_vs_object_layer_filter;
    MyContactListener contact_listener;
};

template <typename ValueType>
void PhysicsEngine<JoltEngine, ValueType, JPH::BodyID>::Init(unsigned int seed)
{
    using namespace JPH;

    rng.seed(seed);

    RegisterDefaultAllocator();

    Factory::sInstance = new Factory();
    RegisterTypes();
    TempAllocatorImpl temp_allocator(100 * 1024 * 1024);
    JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, static_cast<int>(thread::hardware_concurrency()) - 1);

    const uint cMaxBodies = 65536;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 65536;
    const uint cMaxContactConstraints = 10240;

    // Now we can create the actual physics system.
    physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface,
                        object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

    // A contact listener gets notified when bodies (are about to) collide, and when they separate again.
    // Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    physics_system.SetContactListener(&contact_listener);
}

template <typename ValueType>
JPH::BodyID PhysicsEngine<JoltEngine, ValueType, JPH::BodyID>::AddBodyAt(BodyType Type, const Vector3& Pos, ValueType ScaleFactor, bool bIsDynamic)
{
    using namespace JPH;

    Shape *shape = nullptr;
    switch (Type)
    {
    case Box:
        shape = new BoxShape(
            Vec3::sReplicate((float)randomFloat(0.5 * ScaleFactor, 2.0 * ScaleFactor)), 0.0f);
        break;
    case BodyType::Sphere:
        shape = new SphereShape((float)randomFloat(0.5 * ScaleFactor, ScaleFactor));
        break;
    case Cylinder:
        shape = new CylinderShape((float)randomFloat(0.5 * ScaleFactor, ScaleFactor), (float)randomFloat(0.5f * ScaleFactor, ScaleFactor));
        break;
    default:
        break;
    }

    JPH::BodyCreationSettings settings(shape,
        RVec3(Pos.x, Pos.y, Pos.z), Quat::sIdentity(),
        bIsDynamic ? EMotionType::Kinematic : EMotionType::Static, bIsDynamic ? Layers::MOVING : Layers::NON_MOVING);

    JPH::Body *body = physics_system.GetBodyInterface().CreateBody(settings);
    physics_system.GetBodyInterface().AddBody(body->GetID(), EActivation::Activate);

    return body->GetID();
}

template <typename ValueType>
void PhysicsEngine<JoltEngine, ValueType, JPH::BodyID>::SetBodyPosition(JPH::BodyID Handle, const Vector3& Pos)
{
    physics_system.GetBodyInterface().SetPosition(Handle, JPH::RVec3Arg(Pos.x, Pos.y, Pos.z), JPH::EActivation::Activate);
}

template <typename ValueType>
bool PhysicsEngine<JoltEngine, ValueType, JPH::BodyID>::Raycast(const Vector3& Start, const Vector3& Direction)
{
    JPH::RayCastResult hit;
    bool had_hit = physics_system.GetNarrowPhaseQueryNoLock().CastRay(
        JPH::RRayCast(
            JPH::RVec3(Start.x, Start.y, Start.z),
            JPH::Vec3Arg(float(Direction.x), float(Direction.y), float(Direction.z))
        ),
        hit
    );

    return had_hit;
}

template <typename ValueType>
void PhysicsEngine<JoltEngine, ValueType, JPH::BodyID>::OptimizeBroadPhase()
{
    physics_system.OptimizeBroadPhase();
}
