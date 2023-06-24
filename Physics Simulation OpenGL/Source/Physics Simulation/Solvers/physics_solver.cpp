#include "physics_solver.h"

std::vector<Solvers::CollisionSolver::CollisionData> Solvers::PhysicsSolver::m_previous_collisions;

void Solvers::PhysicsSolver::Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime) {
	UpdateObjects(objects, deltaTime);
	tree.Update();
	SolveCollisions(objects, tree, deltaTime);
}

void Solvers::PhysicsSolver::UpdateObjects(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, float deltaTime) {
	for(const auto& object : objects) {
		if(!object->isKinematic())
			continue;

		object->Accelerate(gravityConstant, deltaTime);

		object->UpdateWorldPoints();
		object->UpdateWorldNormals();
		if(object->NotUpdatedFaces())
			object->ClearFaces();

		object->UpdateAABB();
	}
}

void Solvers::PhysicsSolver::IntegrateObjects(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, float deltaTime) {
	for(const auto& object : objects) {
		if(!object->isKinematic())
			continue;

		object->Integrate(deltaTime);
	}
}

void Solvers::PhysicsSolver::SolveCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects, Utilities::AABB_Tree& tree, float deltaTime) {
	std::vector<Solvers::CollisionSolver::CollisionData> currentCollisions{ FindCollisions(objects, tree) };

	ResolveCollisions(objects, currentCollisions, deltaTime);
}

std::vector<Solvers::CollisionSolver::CollisionData> Solvers::PhysicsSolver::FindCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects,
	Utilities::AABB_Tree& tree) {
	std::vector<Solvers::CollisionSolver::CollisionData> currentCollisions{};

	//Bruteforce 
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

	//Dynamic AABB Tree
	for(const auto& object : objects) {
		std::stack<std::weak_ptr<Utilities::AABB_Tree::Node>> currentStack{};
		currentStack.push(tree.GetRoot());

		while(!currentStack.empty()) {
			std::weak_ptr<Utilities::AABB_Tree::Node> currentNode{ currentStack.top() };
			currentStack.pop();

			if(currentNode.lock()->object == object.get())
				continue;

			if(currentNode.lock()->isLeaf) {
				Objects::PhysicsObject* first{ object.get() };
				Objects::PhysicsObject* second{ currentNode.lock()->object };

				if(first->GetID() > second->GetID())
					continue;

				std::tuple<bool, Solvers::CollisionSolver::CollisionData> result{ Solvers::CollisionSolver::CheckCollision(first, second) };

				if(std::get<0>(result)) {
					Solvers::CollisionSolver::CollisionData collisionDetectionData{ std::get<1>(result) };
					currentCollisions.push_back(collisionDetectionData);
				}
			}
			else if(Solvers::CollisionSolver::CheckCollisionAABB(object->GetAABB(), currentNode.lock().get()->box)) {
				currentStack.push(currentNode.lock()->firstChild);
				currentStack.push(currentNode.lock()->secondChild);
			}
		}
	}

	return currentCollisions;
}

void Solvers::PhysicsSolver::ResolveCollisions(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects,
	std::vector<Solvers::CollisionSolver::CollisionData>& currentCollisions, const float deltaTime) 
{
	for(auto& [distance, normal, manifold] : currentCollisions)
		for(auto& [old_distance, old_normal, old_manifold] : m_previous_collisions)
			if(old_manifold == manifold) {
				old_manifold.Update(manifold);
				manifold.WarmStart(normal);
			}

	std::random_device random_device{};
	std::mt19937 generator{ random_device() };
	std::shuffle(currentCollisions.begin(), currentCollisions.end(), generator); //Randomize solver order, to not prioritize one contact other another?

	for(uint32_t iteration_count{}; iteration_count < ITERATION_COUNT; ++iteration_count)
		for(auto& collision_data : currentCollisions)
			Solvers::CollisionSolver::ResolveCollision(collision_data);

	IntegrateObjects(objects, deltaTime);

	for(auto& collision_data : currentCollisions)
		Solvers::CollisionSolver::ResolvePenetration(collision_data);

	m_previous_collisions = currentCollisions;
}