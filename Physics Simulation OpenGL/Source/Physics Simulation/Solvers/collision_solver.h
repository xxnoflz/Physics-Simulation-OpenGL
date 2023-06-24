#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <tuple>
#include <vector>

#include "../Utilities/resource_manager.h"
#include "../Utilities/model.h"
#include "../Objects/physics_object.h"
#include "../Utilities/aabb.h"
#include "../Utilities/manifold.h"

namespace Solvers {

	constexpr float BIAS{ 0.1f };
	constexpr float SLOP{ 0.001f };
	constexpr float MAX_LINEAR_CORRECTION{ 0.2f };

	class CollisionSolver {
		struct Projection {
			float maxProjection;
			float minProjection;
			glm::vec3 maxPoint;
			glm::vec3 minPoint;
		};
	public:
		struct CollisionData {
			float distance;
			glm::vec3 normal;
			Utilities::Manifold manifold;
		};

		static bool CheckCollisionAABB(const Utilities::AABB& first, const Utilities::AABB& second);
		static std::tuple<bool, CollisionData> CheckCollision(Objects::PhysicsObject* first, Objects::PhysicsObject* second);
		static void ResolveCollision(CollisionData& collision_data);
		static void ResolvePenetration(CollisionData& collision_data);
	private:
		static Projection GetProjection(const glm::vec3& axis, const std::vector<glm::vec3>& objectPoints);

		static Utilities::Manifold GenerateManifold(Objects::PhysicsObject* first, Objects::PhysicsObject* second, const glm::vec3& collisionNormal);
		static glm::vec3 GetFurthestPoint(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& normal);
		static uint32_t GetSignificantFace(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& requiredVertex, const glm::vec3& normal);
		static std::vector<Utilities::Manifold::Point> ClipFace(const Utilities::Model::Face* referenceFace, const Utilities::Model::Face* incidentFace,
			const std::vector<Utilities::Model::Face>& referenceFaces, bool flip);
		static glm::vec3 ClipVector(const glm::vec3& subjectVector, const glm::vec3& subjectOrigin, const glm::vec3& clipVertex, const glm::vec3& clipNormal);

		static void SolveNormalImpulse(Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
			CollisionData& collision_data, const uint32_t current_point, const float total_inverse_mass);
		static void SolveFrictionImpulse(Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
			CollisionData& collision_data, const uint32_t current_point, const float total_inverse_mass);
		static void SolvePenetrationImpulse(Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
			CollisionData& collision_data, const uint32_t current_point, const float total_inverse_mass);
	};

}