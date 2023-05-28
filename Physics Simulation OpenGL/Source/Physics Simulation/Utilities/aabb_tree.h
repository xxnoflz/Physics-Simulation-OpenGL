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
			std::weak_ptr<Node> parent;

			std::weak_ptr<Node> firstChild;
			std::weak_ptr<Node> secondChild;

			bool isLeaf;
		};

		AABB_Tree();

		void Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects);

		void InsertLeaf(Objects::PhysicsObject* object);
		void RemoveLeaf(Objects::PhysicsObject* object);

		const std::weak_ptr<Node> GetRoot() const;
		const std::vector<std::shared_ptr<Node>>& GetNodes() const;
	private:
		std::vector<std::shared_ptr<Node>> m_nodes;
		std::weak_ptr<Node> m_root;

		std::weak_ptr<Node> AllocateLeafNode(Objects::PhysicsObject* object);
		std::weak_ptr<Node> AllocateInternalNode();
		float ComputeCost();
		std::weak_ptr<Node> PickBest(std::weak_ptr<Node> leaf);
	};

}