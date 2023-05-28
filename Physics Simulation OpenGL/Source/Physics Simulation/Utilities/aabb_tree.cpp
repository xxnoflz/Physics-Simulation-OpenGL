#include "aabb_tree.h"

Utilities::AABB_Tree::AABB_Tree() : m_nodes(), m_root() {}

void Utilities::AABB_Tree::Update(std::vector<std::unique_ptr<Objects::PhysicsObject>>& objects) {
	for (const auto& object : objects) 
		if (object->isKinematic()) {
			RemoveLeaf(object.get());
			InsertLeaf(object.get());
		}
}

void Utilities::AABB_Tree::InsertLeaf(Objects::PhysicsObject* object) {
	std::shared_ptr<Node> newLeaf{ AllocateLeafNode(object) };
	if (m_nodes.size() == 1) {
		m_root = newLeaf;
		return;
	}

	std::shared_ptr<Node> bestSibling{ PickBest(newLeaf) };

	std::shared_ptr<Node> oldParent{ bestSibling->parent };
	std::shared_ptr<Node> newParent{ AllocateInternalNode() };

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

	std::shared_ptr<Node> nodeIterator{ newLeaf->parent };
	while (nodeIterator != nullptr) {
		nodeIterator->box = AABB::Union(nodeIterator->firstChild->box, nodeIterator->secondChild->box);
		nodeIterator = nodeIterator->parent;
	}
}

void Utilities::AABB_Tree::RemoveLeaf(Objects::PhysicsObject* object) {
	std::shared_ptr<Node> nodeToDelete{};
	for (const auto& node : m_nodes)
		if (node->object == object) {
			nodeToDelete = node;

			break;
		}

	if (nodeToDelete == nullptr)
		return;

	std::shared_ptr<Node> parent{ nodeToDelete->parent };
	std::shared_ptr<Node> grandParent{ parent->parent };
	std::shared_ptr<Node> sibling{ (parent->firstChild == nodeToDelete) ? parent->secondChild : parent->firstChild };

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
	nodeToDelete.reset();
}

std::shared_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::AllocateLeafNode(Objects::PhysicsObject* object) {
	m_nodes.push_back(std::make_shared<Node>());

	m_nodes.back()->box = object->GetAABB();
	m_nodes.back()->object = object;
	m_nodes.back()->isLeaf = true;

	return m_nodes.back();
}

std::shared_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::AllocateInternalNode() {
	m_nodes.push_back(std::make_shared<Node>());

	m_nodes.back()->isLeaf = false;

	return m_nodes.back();
}

float Utilities::AABB_Tree::ComputeCost() {
	float cost{};
	for (auto& node : m_nodes) {
		if (node->isLeaf == false)
			cost += node->box.GetArea();
	}
	return cost;
}

std::shared_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::PickBest(std::shared_ptr<Utilities::AABB_Tree::Node> leaf) {
	float bestCost{ std::numeric_limits<float>::max() };
	std::shared_ptr<Node> bestSibling{ nullptr };

	for (const auto& node : m_nodes) {
		if (node == leaf)
			continue;

		AABB united{ AABB::Union(leaf->box, node->box) };

		float cost{ united.GetArea() };
		std::shared_ptr<Node> parent{ node->parent };

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

const std::shared_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::GetRoot() const {
	return m_root;
}

const std::vector<std::shared_ptr<Utilities::AABB_Tree::Node>>& Utilities::AABB_Tree::GetNodes() const {
	return m_nodes;
}