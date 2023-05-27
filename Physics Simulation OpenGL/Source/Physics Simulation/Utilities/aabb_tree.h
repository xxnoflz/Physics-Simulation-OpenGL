#pragma once
#include <vector>
#include <algorithm>

#include "../Objects/physics_object.h"
#include "aabb.h"

namespace Utilities {

	class AABB_Tree {
	public:
		struct Node {
			AABB box;

			Objects::PhysicsObject* object;
			Node* parent;

			Node* firstChild;
			Node* secondChild;

			bool isLeaf;
		};

		AABB_Tree();

		void Update(std::vector<Objects::PhysicsObject*>& objects);

		void InsertLeaf(Objects::PhysicsObject* object);
		void RemoveLeaf(Objects::PhysicsObject* object);

		const Node* GetRoot() const;
		const std::vector<Node*>& GetNodes() const;
	private:
		std::vector<Node*> m_nodes;
		Node* m_root;

		Node* AllocateLeafNode(Objects::PhysicsObject* object);
		Node* AllocateInternalNode();
		float ComputeCost();
		Node* PickBest(Node* leaf);
	};

}