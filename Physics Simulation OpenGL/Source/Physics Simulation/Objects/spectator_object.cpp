#include "spectator_object.h"

Objects::SpectatorObject::SpectatorObject(const glm::vec3& position) :
	CameraObject(position)
{

}

void Objects::SpectatorObject::KeyboardInput(PressedKey key, float deltaTime) {
	float speed{ player_speed * deltaTime };

	if (key == W)
		GetPosition() += GetDirection() * speed;
	else if(key == S)
		GetPosition() -= GetDirection() * speed;
	else if (key == A)
		GetPosition() -= glm::cross(GetDirection(), worldUp) * speed;
	else if (key == D)
		GetPosition() += glm::cross(GetDirection(), worldUp) * speed;
	UpdateMatrix();
}