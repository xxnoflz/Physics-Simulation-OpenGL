#pragma once
#include "camera_object.h"

namespace Objects {

	constexpr float player_speed = 5.0f;

	class SpectatorObject : public CameraObject {
	public:
		enum PressedKey {
			W = 87,
			A = 65,
			S = 83,
			D = 68
		};
		SpectatorObject(const glm::vec3& position);

		void KeyboardInput(PressedKey key, float deltaTime);
	};

}