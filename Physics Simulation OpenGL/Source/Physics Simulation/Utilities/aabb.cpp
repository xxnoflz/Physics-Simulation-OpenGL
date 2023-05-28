#include "aabb.h"

Utilities::AABB::AABB() : m_lowerBound(), m_upperBound() { }
Utilities::AABB::AABB(const glm::vec3& lowerBound, const glm::vec3& upperBound) : m_lowerBound(lowerBound), m_upperBound(upperBound) {}

void Utilities::AABB::Update(const std::vector<glm::vec4>& objectVertices) {
	glm::vec3 currentMin{ std::numeric_limits<float>::max() };
	glm::vec3 currentMax{};

	for (const auto& vertex : objectVertices) {
		for (int componentIterator{}; componentIterator < glm::vec3::length(); ++componentIterator) {
			currentMax[componentIterator] = std::max(currentMax[componentIterator], vertex[componentIterator]);
			currentMin[componentIterator] = std::min(currentMin[componentIterator], vertex[componentIterator]);
		}
	}
	m_lowerBound = currentMin;
	m_upperBound = currentMax;
}

const glm::vec3 Utilities::AABB::MinimumOfTwo(const AABB& first, const AABB& second) {
	glm::vec3 result{ std::numeric_limits<float>::max() };

	for (int componentIterator{}; componentIterator < glm::vec3::length(); ++componentIterator)
		result[componentIterator] = std::min({ result[componentIterator], first.GetLowerBound()[componentIterator], second.GetLowerBound()[componentIterator] });

	return result;
}

const glm::vec3 Utilities::AABB::MaximumOfTwo(const AABB& first, const AABB& second) {
	glm::vec3 result{ };

	for (int componentIterator{}; componentIterator < glm::vec3::length(); ++componentIterator)
		result[componentIterator] = std::max({ result[componentIterator], first.GetUpperBound()[componentIterator], second.GetUpperBound()[componentIterator] });

	return result;
}

const glm::vec3& Utilities::AABB::GetLowerBound() const {
	return m_lowerBound;
}

const glm::vec3& Utilities::AABB::GetUpperBound() const {
	return m_upperBound;
}

float Utilities::AABB::GetArea() {
	glm::vec3 diagonal{ m_upperBound - m_lowerBound };
	return 2.0f * (diagonal.x * diagonal.y + diagonal.y * diagonal.z + diagonal.z * diagonal.x);
}

bool Utilities::AABB::Contains(const Utilities::AABB& object) {
	bool xCoord{ m_lowerBound.x <= object.GetLowerBound().x && m_upperBound.x >= object.GetUpperBound().x };
	bool yCoord{ m_lowerBound.y <= object.GetLowerBound().y && m_upperBound.y >= object.GetUpperBound().y };
	bool zCoord{ m_lowerBound.z <= object.GetLowerBound().z && m_upperBound.z >= object.GetUpperBound().z };
	return xCoord && yCoord && zCoord;
}

Utilities::AABB Utilities::AABB::Union(const AABB& first, const AABB& second) {
	return { MinimumOfTwo(first, second), MaximumOfTwo(first, second) };
}