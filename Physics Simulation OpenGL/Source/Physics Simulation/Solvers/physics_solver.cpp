#include "physics_solver.h"

void Solvers::PhysicsSolver::Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime) {
	ApplyGravity(objects);
	UpdatePositions(objects, deltaTime);
	UpdateAABB(objects);
	tree.Update(objects);
	SolveCollisions(objects, tree, deltaTime);
}

void Solvers::PhysicsSolver::UpdateAABB(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects) {
	for (const auto& object : objects)
		if(object->isKinematic())
			object->UpdateAABB();
}

void Solvers::PhysicsSolver::ApplyGravity(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects) {
	for (const auto& object : objects)
		object->Accelerate(gravityConstant);
}

void Solvers::PhysicsSolver::UpdatePositions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, float deltaTime) {
	for (const auto& object : objects)
		object->Integrate(deltaTime);
}

void Solvers::PhysicsSolver::SolveCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime) {
	std::vector<CollisionResponseData> currentCollisions{};
	std::vector <std::pair<Objects::PhysicsObject*, Objects::PhysicsObject*>> processedCollisions{};
	//for (uint32_t iterator_in{}; iterator_in < objects.size(); ++iterator_in) {
	//	for (uint32_t iterator_out{ iterator_in + 1 }; iterator_out < objects.size(); ++iterator_out) {
	//		std::tuple<bool, Solvers::CollisionSolver::CollisionData> result{ Solvers::CollisionSolver::CheckCollision(objects[iterator_in].get(), objects[iterator_out].get()) };

	//		if (std::get<0>(result)) {
	//			Solvers::CollisionSolver::CollisionData collisionDetecionData{ std::get<1>(result) };
	//			CollisionResponseData data{
	//				.collisionData = collisionDetecionData,
	//				.accumulatedImpulses = std::vector<float>(collisionDetecionData.manifold.vertices.size()),
	//				.accumulatedFrictions = std::vector<float>(collisionDetecionData.manifold.vertices.size())
	//			};
	//			currentCollisions.push_back(data);
	//		}
	//	}
	//}
	for (const auto& object : objects) {
		std::stack<std::shared_ptr<Utilities::AABB_Tree::Node>> currentStack{};
		currentStack.push(tree.GetRoot());

		while (!currentStack.empty()) {
			std::shared_ptr<Utilities::AABB_Tree::Node> currentNode{ currentStack.top() };
			currentStack.pop();

			if (currentNode->object == object.get())
				continue;

			if (currentNode->isLeaf) {
				Objects::PhysicsObject* first{object.get()};
				Objects::PhysicsObject* second{currentNode->object};

				if (first->GetID() > second->GetID())
					std::swap(first, second);

				if (std::find(processedCollisions.begin(), processedCollisions.end(), std::pair{ first, second }) != processedCollisions.end())
					continue;

				std::tuple<bool, Solvers::CollisionSolver::CollisionData> result{ Solvers::CollisionSolver::CheckCollision(first, second) };

				if (std::get<0>(result)) {
					Solvers::CollisionSolver::CollisionData collisionDetecionData{ std::get<1>(result) };
					CollisionResponseData data{
						.collisionData = collisionDetecionData,
						.accumulatedImpulses = std::vector<float>(collisionDetecionData.manifold.vertices.size()),
						.accumulatedFrictions = std::vector<float>(collisionDetecionData.manifold.vertices.size())
					};
					
					processedCollisions.push_back({ first, second });
					currentCollisions.push_back(data);
				}
			}
			else if (Solvers::CollisionSolver::CheckCollisionAABB(object->GetAABB(), currentNode->box)) {
				currentStack.push(currentNode->firstChild);
				currentStack.push(currentNode->secondChild);
			}
		}
	}

	for (uint32_t iteration_count{}; iteration_count < ITERATION_COUNT; ++iteration_count)
		for (auto& [collisionData, accumulatedImpulses, accumulatedFrictions] : currentCollisions)
			Solvers::CollisionSolver::ResolveCollision(collisionData, accumulatedImpulses, accumulatedFrictions, deltaTime);
}