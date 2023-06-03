#include "spectator_object.h"

Objects::SpectatorObject::SpectatorObject(const glm::vec3& position) :
	CameraObject(position)
{

}

void Objects::SpectatorObject::KeyboardInput(PressedKey key, float deltaTime) {
	float speed{ player_speed * deltaTime };

	glm::vec3 forwardVector{ glm::cos(glm::radians(GetAngles().yaw)), 0.0f, glm::sin(glm::radians(GetAngles().yaw))};
	glm::normalize(forwardVector);

	switch(key) {
		case W:
			GetPosition() += GetDirection() * speed;
			break;
		case S:
			GetPosition() -= GetDirection() * speed;
			break;
		case A:
			GetPosition() -= glm::cross(forwardVector, worldUp) * speed;
			break;
		case D:
			GetPosition() += glm::cross(forwardVector, worldUp) * speed;
			break;
	}
	UpdateMatrix();
}