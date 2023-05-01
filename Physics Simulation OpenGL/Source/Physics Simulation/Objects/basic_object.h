#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Objects {

	class BasicObject {
	public:
		BasicObject(const glm::vec3& position = glm::vec3(), const glm::vec3& size = glm::vec3(), const glm::quat& rotation = glm::quat());

		glm::vec3& GetPosition();
		glm::vec3& GetSize();
		glm::quat& GetRotation();
	private:
		glm::vec3 m_position;
		glm::vec3 m_size;
		glm::quat m_rotation;
	};

}