#pragma once
#include <vector>
#include <algorithm>
#include <memory>

#include "../Objects/physics_object.h"
#include "aabb.h"

namespace Utilities {

	class AABB_Tree {
	public:
		struct Node {
			AABB box;

			Objects::PhysicsObject* object;
			std::shared_ptr<Node> parent;

			std::shared_ptr<Node> firstChild;
			std::shared_ptr<Node> secondChild;

			bool isLeaf;
		};

		AABB_Tree();

		void Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects);

		void InsertLeaf(Objects::PhysicsObject* object);
		void RemoveLeaf(Objects::PhysicsObject* object);

		const std::shared_ptr<Node> GetRoot() const;
		const std::vector<std::shared_ptr<Node>>& GetNodes() const;
	private:
		std::vector<std::shared_ptr<Node>> m_nodes;
		std::shared_ptr<Node> m_root;

		std::shared_ptr<Node> AllocateLeafNode(Objects::PhysicsObject* object);
		std::shared_ptr<Node> AllocateInternalNode();
		float ComputeCost();
		std::shared_ptr<Node> PickBest(std::shared_ptr<Node> leaf);
	};

}