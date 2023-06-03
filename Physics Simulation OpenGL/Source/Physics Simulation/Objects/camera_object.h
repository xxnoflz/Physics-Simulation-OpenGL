#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "basic_object.h"

namespace Objects {
	inline glm::vec3 worldUp = glm::vec3(0.0f, 1.0f, 0.0f);

	struct EulerAngles {
		float pitch;
		float yaw;
		float roll;
	};

	struct MouseData {
		float lastXPos;
		float lastYPos;
	};

	class CameraObject : public BasicObject {
	public:
		CameraObject(const glm::vec3& position);

		void MouseInput(float xPos, float yPos);

		const glm::mat4& GetMatrix() const;
		const glm::vec3& GetDirection() const;
		const EulerAngles& GetAngles() const;

		void UpdateMatrix();
	private:
		EulerAngles m_camera_rotation;
		MouseData m_mouse_data;
		glm::vec3 m_direction;
		glm::mat4 m_view_matrix;

		void UpdateRotation();
	};

}