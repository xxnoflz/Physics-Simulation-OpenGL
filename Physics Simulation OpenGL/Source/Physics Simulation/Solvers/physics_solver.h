#pragma once
#include <glm/glm.hpp>

#include <vector>

#include "../Objects/physics_object.h"
#include "collision_solver.h"

namespace Solvers {

	inline glm::vec3 gravityConstant = glm::vec3(0.0f, -9.8f, 0.0f);
	constexpr uint32_t ITERATION_COUNT = 4;

	class PhysicsSolver {
		struct CollisionResponseData {
			Solvers::CollisionSolver::CollisionData collisionData;
			std::vector<float> accumulatedImpulses;
			std::vector<float> accumulatedFrictions;
		};
	public:
		static void Update(std::vector<Objects::PhysicsObject*> objects, float deltaTime);
	private:
		static void ApplyGravity(std::vector<Objects::PhysicsObject*> objects);
		static void UpdatePositions(std::vector<Objects::PhysicsObject*> objects, float deltaTime);
		static void SolveCollisions(std::vector<Objects::PhysicsObject*> objects, float deltaTime);
	};

}