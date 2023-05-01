#include "camera_object.h"

Objects::CameraObject::CameraObject(const glm::vec3& position) :
	BasicObject(position), m_camera_rotation(), m_mouse_data(), m_view_matrix()
{
	UpdateRotation();
}

void Objects::CameraObject::MouseInput(float xPos, float yPos) {
	float deltaX{ xPos - m_mouse_data.lastXPos };
	float deltaY{ m_mouse_data.lastYPos - yPos };
	m_mouse_data.lastXPos = xPos;
	m_mouse_data.lastYPos = yPos;

	m_camera_rotation.pitch += deltaY * 0.1;
	m_camera_rotation.yaw += deltaX * 0.1;

	UpdateRotation();
}

const glm::mat4& Objects::CameraObject::GetMatrix() const {
	return m_view_matrix;
}

const glm::vec3& Objects::CameraObject::GetDirection() const {
	return m_direction;
}

void Objects::CameraObject::UpdateRotation() {
	glm::vec3 newDirection{};
	newDirection.x = glm::cos(glm::radians(m_camera_rotation.pitch)) * glm::cos(glm::radians(m_camera_rotation.yaw));
	newDirection.y = glm::sin(glm::radians(m_camera_rotation.pitch));
	newDirection.z = glm::cos(glm::radians(m_camera_rotation.pitch)) * glm::sin(glm::radians(m_camera_rotation.yaw));
	m_direction = glm::normalize(newDirection);

	UpdateMatrix();
}

void Objects::CameraObject::UpdateMatrix() {
	m_view_matrix = glm::lookAt(BasicObject::GetPosition(), BasicObject::GetPosition() + m_direction, worldUp);
}