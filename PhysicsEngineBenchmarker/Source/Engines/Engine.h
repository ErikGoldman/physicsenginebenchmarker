#pragma once

template <typename ValueType>
struct TVector3
{
    TVector3()
    : x(0), y(0), z(0)
    {}

    TVector3(ValueType X, ValueType Y, ValueType Z)
    : x(X), y(Y), z(Z)
    {}

    explicit TVector3(ValueType R)
    : x(R), y(R), z(R)
    {}

    ValueType distance(const TVector3<ValueType> &p2) const
    {
        const ValueType dx = p2.x - x;
        const ValueType dy = p2.y - y;
        const ValueType dz = p2.z - z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    void normalize()
    {
        const ValueType len = distance(TVector3<ValueType>(0, 0, 0));
        if (len > 0.0f)
        {
            x /= len;
            y /= len;
            z /= len;
        }
        else
        {
            x = y = z = 0.0f;
        }
    }

    ValueType x, y, z;
};

enum BodyType
{
    Box,
    Sphere,
    Cylinder
};

struct JoltEngine {};
struct Box3dEngine {};

template <typename EngineType, typename ValueType, typename BodyId>
class PhysicsEngine
{
    typedef TVector3<ValueType> Vector3;

public:
    void Init(unsigned int seed) {}

    BodyId AddBodyAt(BodyType Type, const Vector3& Pos, ValueType ScaleFactor, bool bIsDynamic) { return BodyId(); }
    void RemoveBody(int Handle) {}
    void SetBodyPosition(BodyId Handle, const Vector3& Pos) {}
    bool Raycast(const Vector3& Start, const Vector3& Direction) { return false; }
    void OptimizeBroadPhase() {}
};

