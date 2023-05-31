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

namespace Solvers {

	constexpr float BIAS{ 0.1f };
	constexpr float SLOP{ 0.0005f };

	class CollisionSolver {
		struct Projection {
			float maxProjection;
			float minProjection;
			glm::vec3 maxPoint;
			glm::vec3 minPoint;
		};
		struct ContactPointData {
			glm::vec3 relativeFirst;
			glm::vec3 relativeSecond;

			glm::vec3 angVelocityFirst;
			glm::vec3 angVelocitySecond;

			glm::vec3 fullVelocityFirst;
			glm::vec3 fullVelocitySecond;
			glm::vec3 contactVelocity;
		};
	public:
		struct Manifold {
			std::vector<glm::vec3> vertices;
			Objects::PhysicsObject* firstObject;
			Objects::PhysicsObject* secondObject;
		};
		struct CollisionData {
			float distance;
			glm::vec3 normal;
			Manifold manifold;
		};

		static bool CheckCollisionAABB(const Utilities::AABB& first, const Utilities::AABB& second);
		static std::tuple<bool, CollisionData> CheckCollision(Objects::PhysicsObject* first, Objects::PhysicsObject* second);
		static void ResolveCollision(CollisionData collision_data, std::vector<float>& accumulatedImpulses, std::vector<float>& accumulatedFrictions, float deltaTime);
	private:
		static Projection GetProjection(const glm::vec3& axis, const std::vector<glm::vec3>& objectPoints);

		static std::vector<glm::vec3> GenerateManifold(Objects::PhysicsObject* first, Objects::PhysicsObject* second, const glm::vec3& collisionNormal);
		static glm::vec3 GetFurthestPoint(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& normal);
		static Utilities::Model::Face GetSignificantFace(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& requiredVertex, const glm::vec3& normal);
		static Utilities::Model::Face ClipFace(Utilities::Model::Face* referenceFace, Utilities::Model::Face* incidentFace,
			const std::vector<Utilities::Model::Face>& referenceFaces);
		static glm::vec3 ClipVector(const glm::vec3& subjectVector, const glm::vec3& subjectOrigin, const glm::vec3& clipVertex, const glm::vec3& clipNormal);

		static const ContactPointData GetContactPointData(const CollisionData& collision_data, uint32_t currentManifold);
		static void SolveNormalImpulse(Objects::PhysicsObject* first, Objects::PhysicsObject* second,
			const CollisionData& collision_data, const uint32_t currentManifold, const ContactPointData& data,
			const float deltaTime,
			const float totalInverseMass, std::vector<float>& accumulatedImpulses);
		static void SolveFrictionImpulse(Objects::PhysicsObject* first, Objects::PhysicsObject* second,
			const CollisionData& collision_data, const uint32_t currentManifold, const ContactPointData& data,
			const float totalInverseMass, std::vector<float>& accumulatedImpulses, std::vector<float>& accumulatedFrictions);
	};

}