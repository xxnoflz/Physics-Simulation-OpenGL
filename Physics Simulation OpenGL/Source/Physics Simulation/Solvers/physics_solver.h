#pragma once
#include <glm/glm.hpp>

#include <vector>
#include <stack>

#include "../Objects/physics_object.h"
#include "../Utilities/aabb_tree.h"
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
		static void Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime);
	private:
		static void UpdateAABB(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects);
		static void ApplyGravity(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects);
		static void UpdatePositions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, float deltaTime);
		static void SolveCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime);
	};

}