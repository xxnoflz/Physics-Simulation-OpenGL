#include "physics_solver.h"

void Solvers::PhysicsSolver::Update(std::vector<Objects::PhysicsObject*> objects, float deltaTime) {
	ApplyGravity(objects);
	UpdatePositions(objects, deltaTime);
	SolveCollisions(objects, deltaTime);
}

void Solvers::PhysicsSolver::ApplyGravity(std::vector<Objects::PhysicsObject*> objects) {
	for (const auto& object : objects)
		object->Accelerate(gravityConstant);
}

void Solvers::PhysicsSolver::UpdatePositions(std::vector<Objects::PhysicsObject*> objects, float deltaTime) {
	for (const auto& object : objects)
		object->Integrate(deltaTime);
}

void Solvers::PhysicsSolver::SolveCollisions(std::vector<Objects::PhysicsObject*> objects, float deltaTime) {
	std::vector<CollisionResponseData> currentCollisions{};

	for (uint32_t iterator_in{}; iterator_in < objects.size(); ++iterator_in) {
		for (uint32_t iterator_out{ iterator_in + 1 }; iterator_out < objects.size(); ++iterator_out) {
			std::tuple<bool, Solvers::CollisionSolver::CollisionData> result{ Solvers::CollisionSolver::CheckCollision(objects[iterator_in], objects[iterator_out]) };

			if (std::get<0>(result)) {
				Solvers::CollisionSolver::CollisionData collisionDetecionData{ std::get<1>(result) };
				CollisionResponseData data{
					.collisionData = collisionDetecionData,
					.accumulatedImpulses = std::vector<float>(collisionDetecionData.manifold.vertices.size()),
					.accumulatedFrictions = std::vector<float>(collisionDetecionData.manifold.vertices.size())
				};
				currentCollisions.push_back(data);
			}
		}
	}
	for (uint32_t iteration_count{}; iteration_count < ITERATION_COUNT; ++iteration_count)
		for (auto& [collisionData, accumulatedImpulses, accumulatedFrictions] : currentCollisions)
			Solvers::CollisionSolver::ResolveCollision(collisionData, accumulatedImpulses, accumulatedFrictions, deltaTime);
}