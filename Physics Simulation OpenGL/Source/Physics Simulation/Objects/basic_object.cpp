#include "basic_object.h"

Objects::BasicObject::BasicObject(const glm::vec3& position, const glm::vec3& size, const glm::quat& rotation) :
	m_position(position), m_size(size), m_rotation(rotation)
{

}

glm::vec3& Objects::BasicObject::GetPosition() {
	return m_position;
}
glm::vec3& Objects::BasicObject::GetSize() {
	return m_size;
}
glm::quat& Objects::BasicObject::GetRotation() {
	return m_rotation;
}