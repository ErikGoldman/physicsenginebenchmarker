#include <Jolt/Jolt.h>

// STL includes
#include <iostream>
#include <cstdarg>
#include <iomanip>
#include <thread>

#include "ThreadHelpers.h"
#include "box3d/id.h"
#include "Engines/Engine.h"
#include "Engines/EngineJolt.h"
#include "Engines/EngineBox3d.h"

template <typename EngineType, typename ScalarType, typename BodyId>
void TestClumpRaycasts(PhysicsEngine<EngineType, ScalarType, BodyId> &Engine)
{
	// Create a clump of bodies
	for (int i = 0; i < 1000; ++i)
	{
		Engine.AddBodyAt(Box, TVector3<ScalarType>(i, i, i), 1, false);
	}
}

template <typename ScalarType, typename BodyId>
struct BodyAndRadius
{
    BodyAndRadius(TVector3<ScalarType> inPos, ScalarType inRadius, BodyId inId) : pos(inPos), radius(inRadius), id(inId) {}

    TVector3<ScalarType> pos;
    ScalarType radius;
    BodyId id;
};

template <typename ScalarType, typename BodyId>
bool is_colliding(const TVector3<ScalarType> & new_pos, ScalarType new_scale, const std::vector<BodyAndRadius<ScalarType, BodyId>>& existing_bodies) {
    for (const auto& existing_body : existing_bodies) {
        // Collision occurs if the distance between centers is less than the sum of their radii (ScaleFactors)
        if (new_pos.distance(existing_body.pos) < new_scale + existing_body.radius) {
            return true;
        }
    }
    return false;
}

template <typename EngineType, typename ScalarType, typename BodyId>
void generate_non_colliding_bodies(
    PhysicsEngine<EngineType, ScalarType, BodyId> &engine,
    std::vector<BodyAndRadius<ScalarType, BodyId>> &generated_bodies,
    unsigned int seed,
    bool bDynamic,
    int N, // Number of clumps (centroids)
    int M, // Number of bodies per clump
    const TVector3<ScalarType>& volume_min, // Minimum bounds of the placement volume
    const TVector3<ScalarType>& volume_max, // Maximum bounds of the placement volume
    float min_clump_separation, // Minimum distance between clump centroids
    float max_clump_radius, // Maximum distance of a body from its centroid
    float min_body_scale, // Minimum body scale factor (radius)
    float max_body_scale, // Maximum body scale factor (radius)
    int max_placement_attempts // Max attempts for placing a single body/centroid
) {
    std::vector<TVector3<ScalarType>> clump_centroids;

    // Use a random device to seed the random number generator
    std::mt19937 rng(seed); // Mersenne Twister engine

    // Distribution for generating positions within the volume bounds
    std::uniform_real_distribution<ScalarType> dist_x(volume_min.x, volume_max.x);
    std::uniform_real_distribution<ScalarType> dist_y(volume_min.y, volume_max.y);
    std::uniform_real_distribution<ScalarType> dist_z(volume_min.z, volume_max.z);

    // Distribution for generating body scale factors
    std::uniform_real_distribution<ScalarType> dist_scale(min_body_scale, max_body_scale);

    std::uniform_int_distribution shapeType(0, 2);

    // 1. Generate Clump Centroids
    for (int i = 0; i < N; ++i) {
        int attempts = 0;
        bool placed = false;
        while (attempts < max_placement_attempts && !placed) {
            TVector3<ScalarType> centroid(dist_x(rng), dist_y(rng), dist_z(rng));

            bool too_close = false;
            for (const auto& existing_centroid : clump_centroids) {
                if (centroid.distance(existing_centroid) < min_clump_separation) {
                    too_close = true;
                    break;
                }
            }

            if (!too_close) {
                clump_centroids.push_back(centroid);
                placed = true;
                // std::cout << "  Placed centroid " << i + 1 << " at (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
            }
            attempts++;
        }
        if (!placed) {
            std::cerr << "Warning: Could not place clump centroid " << i + 1 << " after " << max_placement_attempts << " attempts." << std::endl;
        }
    }

    // 2. Generate Bodies around Centroids
    for (const auto& centroid : clump_centroids) {
        // Distribution for generating positions within the clump radius
        // We generate in a cube and reject points outside the sphere for simplicity
        std::uniform_real_distribution<ScalarType> dist_clump_offset(-max_clump_radius, max_clump_radius);

        for (int i = 0; i < M; ++i) {
            int attempts = 0;
            bool placed = false;
            while (attempts < max_placement_attempts && !placed) {
                TVector3<ScalarType> offset(dist_clump_offset(rng), dist_clump_offset(rng), dist_clump_offset(rng));
                // Check if the offset is within the sphere radius
                if (offset.distance(TVector3<ScalarType>(0,0,0)) > max_clump_radius) {
                    attempts++;
                    continue; // Retry if outside the sphere
                }

                TVector3<ScalarType> body_pos = centroid;
                body_pos.x += offset.x;
                body_pos.y += offset.y;
                body_pos.z += offset.z;

                ScalarType body_scale = dist_scale(rng);

                // Check for collisions with already placed bodies
                if (!is_colliding(body_pos, body_scale, generated_bodies))
                {
                    BodyId bodyId = engine.AddBodyAt(static_cast<BodyType>(shapeType(rng)), body_pos, body_scale, bDynamic);
                    generated_bodies.push_back(BodyAndRadius<ScalarType, BodyId>(body_pos, body_scale, bodyId));
                    placed = true;
                }
                attempts++;
            }
            if (!placed) {
                 std::cerr << "Warning: Could not place body " << i + 1 << " in a clump after " << max_placement_attempts << " attempts. Clump may be too dense or parameters too restrictive." << std::endl;
            }
        }
    }
}

template <typename EngineType, typename Scalar, typename BodyId>
long long PlaceAndRaycast(unsigned int seed, unsigned int num_threads, bool bIncludeStaticBodies)
{
    PhysicsEngine<EngineType, Scalar, BodyId> Engine;
    Engine.Init(seed);

    auto start_clock = std::chrono::steady_clock::now();
    std::vector<BodyAndRadius<Scalar, BodyId>> generated_bodies;
    generate_non_colliding_bodies(Engine, generated_bodies, seed, true, 30, 100,
        TVector3<Scalar>(-6000000.f, -6000000.f, 0.f),
        TVector3<Scalar>(6000000.f, 6000000.f, 120000.f),
        100000, 50000, 900, 4000, 30);

    if (bIncludeStaticBodies)
    {
        std::vector<BodyAndRadius<Scalar, BodyId>> static_bodies;
        generate_non_colliding_bodies(Engine, static_bodies, seed, false, 30, 100,
            TVector3<Scalar>(-6000000.f, -6000000.f, 0.f),
            TVector3<Scalar>(6000000.f, 6000000.f, 120000.f),
            100000, 50000, 900, 4000, 30);
    }
    Engine.OptimizeBroadPhase();

    const int num_raycasts_per_body = 5;

    auto raycast_nanos = ThreadHelpers::ParallelFor(
        num_threads, static_cast<unsigned int>(generated_bodies.size()),
        [num_raycasts_per_body, &generated_bodies, seed, &Engine](unsigned int start, unsigned int end)
    {
        std::mt19937 rng(seed); // Mersenne Twister engine
        std::uniform_real_distribution<Scalar> zero_one(0, 1);

        for (unsigned int i = start; i < end; ++i)
        {
            const BodyAndRadius<Scalar, BodyId> &body = generated_bodies[i];
            for (int rc = 0; rc < num_raycasts_per_body; ++rc)
            {
                // pick a body and raycast from it
                TVector3<Scalar> ray_origin = body.pos;
                TVector3<Scalar> ray_direction = TVector3<Scalar>(zero_one(rng), zero_one(rng), zero_one(rng));
                ray_direction.normalize();

                Scalar shiftFactor = body.radius * 1.5f; // I don't want the ray to hit the casting body every time
                ray_origin.x += shiftFactor * ray_direction.x;
                ray_origin.y += shiftFactor * ray_direction.y;
                ray_origin.z += shiftFactor * ray_direction.z;

                Engine.Raycast(body.pos, ray_direction);
            }
        }
    });

    return raycast_nanos;
}

template <typename EngineType, typename Scalar, typename BodyId>
long long PlaceAndMove(unsigned int seed, const float shift_factor, unsigned int num_threads, bool bOptimizeBroadPhase)
{
    PhysicsEngine<EngineType, Scalar, BodyId> Engine;
    Engine.Init(seed);

    std::vector<BodyAndRadius<Scalar, BodyId>> generated_bodies;
    generate_non_colliding_bodies(Engine, generated_bodies, seed, false, 30, 100,
        TVector3<Scalar>(-6000000.f, -6000000.f, 0.f),
        TVector3<Scalar>(6000000.f, 6000000.f, 120000.f),
        100000, 50000, 900, 4000, 30);
    Engine.OptimizeBroadPhase();

    auto start_clock = std::chrono::steady_clock::now();
    ThreadHelpers::ParallelFor(
        num_threads, static_cast<unsigned int>(generated_bodies.size()),
        [&generated_bodies, seed, &Engine, shift_factor](unsigned int start, unsigned int end)
    {
        std::mt19937 rng(seed); // Mersenne Twister engine
        std::uniform_real_distribution<Scalar> zero_one(0, 1);

        for (unsigned int i = start; i < end; ++i)
        {
            const BodyAndRadius<Scalar, BodyId> &body = generated_bodies[i];
            Engine.SetBodyPosition(body.id, TVector3<Scalar>(
                zero_one(rng) * shift_factor, zero_one(rng) * shift_factor, zero_one(rng) * shift_factor));
        }
    });

    if (bOptimizeBroadPhase)
    {
        Engine.OptimizeBroadPhase();
    }
    auto end_clock = std::chrono::steady_clock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(end_clock - start_clock).count();
}

void PrintLine()
{
    std::cout << "----------------------------------------\n";
}

double GetAverageMs(std::string TestName, int samples, std::function<long long()> func)
{
    long long total = 0;
    for (int i = 0; i < samples; i++)
    {
        total += func();
    }

    double ms = (static_cast<double>(total) / samples) / (1000 * 1000);
    std::cout << TestName << ": " << ms << " ms\n";
    return ms;
}

// Program entry point
int main(int , char** )
{
    constexpr int NUM_SAMPLES = 10;
    std::cout << "**Place and move**\n";
    std::cout << "Jolt\n";
    for (unsigned int num_threads = 1; num_threads < 8; ++num_threads)
    {
        std::stringstream ss;
        ss << "\tJolt[" << num_threads << "]";
        std::string title = ss.str();

        GetAverageMs(title, NUM_SAMPLES, [num_threads] { return PlaceAndMove<JoltEngine, double, JPH::BodyID>(41887, 1000.f, num_threads, true); });
    }
    std::cout << "Box3d\n";
    GetAverageMs("\tBox3d", NUM_SAMPLES, [] { return PlaceAndMove<Box3dEngine, float, b3BodyId>(41887, 1000.f, 1, false); });

    PrintLine();
    std::cout << "\n**Raycasting**\n";
    std::cout << "Jolt\n";
    for (unsigned int num_threads = 1; num_threads < 8; ++num_threads)
    {
        std::stringstream ss;
        ss << "\tJolt[" << num_threads << "]";
        std::string title = ss.str();

        GetAverageMs(title, NUM_SAMPLES, [num_threads] { return PlaceAndRaycast<JoltEngine, double, JPH::BodyID>(41887, num_threads, true); });
    }
    std::cout << "Box3d\n";
    for (unsigned int num_threads = 1; num_threads < 8; ++num_threads)
    {
        std::stringstream ss;
        ss << "\tBox3d[" << num_threads << "]";
        std::string title = ss.str();

        GetAverageMs(title, NUM_SAMPLES, [num_threads] { return PlaceAndRaycast<Box3dEngine, float, b3BodyId>(41887, num_threads, true); });
    }
    PrintLine();

	return 0;
}