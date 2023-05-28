#include "aabb_tree.h"

Utilities::AABB_Tree::AABB_Tree() : m_nodes(), m_root() {}

void Utilities::AABB_Tree::Update() {
	for (const auto& node : m_nodes) {
		if (!node.get()->isLeaf)
			continue;

		Objects::PhysicsObject* object{ node.get()->object };
		if (object->isKinematic() && !node.get()->box.Contains(object->GetAABB())) {
			RemoveLeaf(node);
			InsertLeaf(object);
		}
	}
}

void Utilities::AABB_Tree::InsertLeaf(Objects::PhysicsObject* object) {
	std::weak_ptr<Node> newLeaf{ AllocateLeafNode(object) };
	if (m_nodes.size() == 1) {
		m_root = newLeaf;
		return;
	}

	std::weak_ptr<Node> bestSibling{ PickBest(newLeaf) };

	std::weak_ptr<Node> oldParent{ bestSibling.lock()->parent };
	std::weak_ptr<Node> newParent{ AllocateInternalNode() };

	newParent.lock()->parent = oldParent;
	newParent.lock()->box = AABB::Union(newLeaf.lock()->box, bestSibling.lock()->box);

	if (!oldParent.expired()) {
		if (oldParent.lock()->firstChild.lock() == bestSibling.lock())
			oldParent.lock()->firstChild = newParent;
		else
			oldParent.lock()->secondChild = newParent;
		newParent.lock()->firstChild = bestSibling;
		newParent.lock()->secondChild = newLeaf;

		bestSibling.lock()->parent = newParent;
		newLeaf.lock()->parent = newParent;
	}
	else {
		newParent.lock()->firstChild = bestSibling;
		newParent.lock()->secondChild = newLeaf;

		bestSibling.lock()->parent = newParent;
		newLeaf.lock()->parent = newParent;

		m_root = newParent;
	}

	std::weak_ptr<Node> nodeIterator{ newLeaf.lock()->parent };
	while (!nodeIterator.expired()) {
		nodeIterator.lock()->box = AABB::Union(nodeIterator.lock().get()->firstChild.lock().get()->box, nodeIterator.lock().get()->secondChild.lock().get()->box);
		nodeIterator = nodeIterator.lock()->parent;
	}
}

void Utilities::AABB_Tree::RemoveLeaf(std::weak_ptr<Node> nodeToDelete) {
	if (nodeToDelete.lock() == m_root.lock()) 
		m_root.reset();

	std::weak_ptr<Node> parent{ nodeToDelete.lock()->parent };
	if (!parent.expired()) {
		std::weak_ptr<Node> grandParent{ parent.lock()->parent };
		std::weak_ptr<Node> sibling{ (parent.lock()->firstChild.lock() == nodeToDelete.lock()) ? parent.lock()->secondChild : parent.lock()->firstChild };

		if (!grandParent.expired()) {
			if (grandParent.lock()->firstChild.lock() == parent.lock())
				grandParent.lock()->firstChild = sibling;
			else
				grandParent.lock()->secondChild = sibling;

			sibling.lock()->parent = grandParent.lock();
			m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), parent.lock()), m_nodes.end());

			while (!grandParent.expired()) {
				grandParent.lock()->box = AABB::Union(grandParent.lock().get()->firstChild.lock().get()->box, grandParent.lock().get()->secondChild.lock().get()->box);
				grandParent = grandParent.lock()->parent;
			}
		}
		else {
			m_root = sibling;
			sibling.lock().get()->parent.reset();
			m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), parent.lock()), m_nodes.end());
		}
	}

	m_nodes.erase(std::remove(m_nodes.begin(), m_nodes.end(), nodeToDelete.lock()), m_nodes.end());
}

std::weak_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::AllocateLeafNode(Objects::PhysicsObject* object) {
	m_nodes.push_back(std::make_shared<Node>());

	m_nodes.back()->box = { object->GetAABB().GetLowerBound() - AABB_MARGIN, object->GetAABB().GetUpperBound() + AABB_MARGIN };
	m_nodes.back()->object = object;
	m_nodes.back()->isLeaf = true;

	return m_nodes.back();
}

std::weak_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::AllocateInternalNode() {
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

std::weak_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::PickBest(std::weak_ptr<Utilities::AABB_Tree::Node> leaf) {
	float bestCost{ std::numeric_limits<float>::max() };
	std::weak_ptr<Node> bestSibling{ };

	for (const auto& node : m_nodes) {
		if (node == leaf.lock())
			continue;

		AABB united{ AABB::Union(leaf.lock()->box, node->box) };

		float cost{ united.GetArea() };
		std::weak_ptr<Node> parent{ node->parent };

		while (!parent.expired()) {
			AABB parentUnited{ AABB::Union(leaf.lock()->box, parent.lock()->box) };
			cost += parentUnited.GetArea() - parent.lock()->box.GetArea();
			parent = parent.lock()->parent;
		}

		if (cost < bestCost) {
			bestCost = cost;
			bestSibling = node;
		}
	}

	return bestSibling;
}

const std::weak_ptr<Utilities::AABB_Tree::Node> Utilities::AABB_Tree::GetRoot() const {
	return m_root;
}

const std::vector<std::shared_ptr<Utilities::AABB_Tree::Node>>& Utilities::AABB_Tree::GetNodes() const {
	return m_nodes;
}