#pragma once
#include <glm/glm.hpp>

#include <vector>
#include <stack>
#include <algorithm>
#include <random>

#include "../Objects/physics_object.h"
#include "../Utilities/aabb_tree.h"
#include "collision_solver.h"

namespace Solvers {

	inline glm::vec3 gravityConstant { glm::vec3(0.0f, -9.8f, 0.0f) };
	constexpr uint32_t ITERATION_COUNT{ 10 };

	class PhysicsSolver {
	public:
		static void Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime);
	private:
		static void UpdateObjects(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, float deltaTime);
		static void IntegrateObjects(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, float deltaTime);

		static void SolveCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime);
		static std::vector<Solvers::CollisionSolver::CollisionData> FindCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree);
		static void ResolveCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, std::vector<Solvers::CollisionSolver::CollisionData>& currentCollisions, const float deltaTime);

		static std::vector<Solvers::CollisionSolver::CollisionData> m_previous_collisions;
	};

}