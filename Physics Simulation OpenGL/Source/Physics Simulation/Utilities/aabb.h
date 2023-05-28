#pragma once
#include <glm/glm.hpp>
#include <vector>

namespace Utilities {

	class AABB {
	public:
		AABB();
		AABB(const glm::vec3& lowerBound, const glm::vec3& upperBound);

		void Update(const std::vector<glm::vec4>& objectVertices);

		static const glm::vec3 MinimumOfTwo(const AABB& first, const AABB& second);
		static const glm::vec3 MaximumOfTwo(const AABB& first, const AABB& second);

		const glm::vec3& GetLowerBound() const;
		const glm::vec3& GetUpperBound() const;

		float GetArea();
		bool Contains(const AABB& object);

		static AABB Union(const AABB& first, const AABB& second);
	private:
		glm::vec3 m_lowerBound;
		glm::vec3 m_upperBound;
	};

}