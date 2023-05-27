#include "aabb_tree.h"

Utilities::AABB_Tree::AABB_Tree() : m_nodes(), m_root() {}

void Utilities::AABB_Tree::Update(std::vector<Objects::PhysicsObject*>& objects) {
	for (const auto& object : objects) 
		if (object->isKinematic()) {
			RemoveLeaf(object);
			InsertLeaf(object);
		}
}

void Utilities::AABB_Tree::InsertLeaf(Objects::PhysicsObject* object) {
	Node* newLeaf{ AllocateLeafNode(object) };
	if (m_nodes.size() == 1) {
		m_root = newLeaf;
		return;
	}

	Node* bestSibling{ PickBest(newLeaf) };

	Node* oldParent{ bestSibling->parent };
	Node* newParent{ AllocateInternalNode() };

	newParent->parent = oldParent;
	newParent->box = AABB::Union(newLeaf->box, bestSibling->box);

	if (oldParent != nullptr) {
		if (oldParent->firstChild == bestSibling)
			oldParent->firstChild = newParent;
		else
			oldParent->secondChild = newParent;
		newParent->firstChild = bestSibling;
		newParent->secondChild = newLeaf;

		bestSibling->parent = newParent;
		newLeaf->parent = newParent;
	}
	else {
		newParent->firstChild = bestSibling;
		newParent->secondChild = newLeaf;

		bestSibling->parent = newParent;
		newLeaf->parent = newParent;

		m_root = newParent;
	}

	Node* nodeIterator{ newLeaf->parent };
	while (nodeIterator != nullptr) {
		nodeIterator->box = AABB::Union(nodeIterator->firstChild->box, nodeIterator->secondChild->box);
		nodeIterator = nodeIterator->parent;
	}
}

void Utilities::AABB_Tree::RemoveLeaf(Objects::PhysicsObject* object) {
	Node* nodeToDelete{};
	for (const auto& node : m_nodes)
		if (node->object == object) {
			nodeToDelete = node;

			break;
		}

	if (nodeToDelete == nullptr)
		return;

	Node* parent{ nodeToDelete->parent };
	Node* grandParent{ parent->parent };
	Node* sibling{ (parent->firstChild == nodeToDelete) ? parent->secondChild : parent->firstChild };

	if (nodeToDelete == m_root) {
		m_root = nullptr;
		return;
	}

	if (grandParent != nullptr) {
		if (grandParent->firstChild == parent)
			grandParent->firstChild = sibling;
		else
			grandParent->secondChild = sibling;
		sibling->parent = grandParent;
		m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), parent), m_nodes.end());

		while (grandParent != nullptr) {
			grandParent->box = AABB::Union(grandParent->firstChild->box, grandParent->secondChild->box);
			grandParent = grandParent->parent;
		}
	}
	else {
		m_root = sibling;
		sibling->parent = nullptr;
		m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), parent), m_nodes.end());
	}

	m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), nodeToDelete), m_nodes.end());
	delete nodeToDelete;
}

Utilities::AABB_Tree::Node* Utilities::AABB_Tree::AllocateLeafNode(Objects::PhysicsObject* object) {
	Node* node{ new Node() };

	node->box = object->GetAABB();
	node->object = object;
	node->isLeaf = true;
	m_nodes.push_back(node);

	return node;
}

Utilities::AABB_Tree::Node* Utilities::AABB_Tree::AllocateInternalNode() {
	Node* node{ new Node() };
	node->isLeaf = false;
	m_nodes.push_back(node);

	return node;
}

float Utilities::AABB_Tree::ComputeCost() {
	float cost{};
	for (auto& node : m_nodes) {
		if (node->isLeaf == false)
			cost += node->box.GetArea();
	}
	return cost;
}

Utilities::AABB_Tree::Node* Utilities::AABB_Tree::PickBest(Node* leaf) {
	float bestCost{ std::numeric_limits<float>::max() };
	Node* bestSibling{ nullptr };

	for (const auto& node : m_nodes) {
		if (node == leaf)
			continue;

		AABB united{ AABB::Union(leaf->box, node->box) };

		float cost{ united.GetArea() };
		Node* parent{ node->parent };

		while (parent != nullptr) {
			AABB parentUnited{ AABB::Union(leaf->box, parent->box) };
			cost += parentUnited.GetArea() - parent->box.GetArea();
			parent = parent->parent;
		}

		if (cost < bestCost) {
			bestCost = cost;
			bestSibling = node;
		}
	}

	return bestSibling;
}

const Utilities::AABB_Tree::Node* Utilities::AABB_Tree::GetRoot() const {
	return m_root;
}

const std::vector<Utilities::AABB_Tree::Node*>& Utilities::AABB_Tree::GetNodes() const {
	return m_nodes;
}