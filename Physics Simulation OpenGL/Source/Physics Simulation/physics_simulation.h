#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cstdint>
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>
#include <memory>

#include "Utilities/resource_manager.h"
#include "Render/renderer.h"
#include "Objects/physics_object.h"
#include "Utilities/aabb_tree.h"
#include "Objects/spectator_object.h"
#include "Solvers/physics_solver.h"

namespace Simulation {

	constexpr std::string_view window_title = "Physics Simulation";
	constexpr float fov_value = 45.0f;
	inline glm::vec3 spectator_start_position = glm::vec3(0.0f);

	struct TimeHandler {
		float deltaTime;
		float lastFrame;
		float currentFrame;
	};

	class PhysicsSimulation {
	public:
		PhysicsSimulation(const uint32_t window_width, const uint32_t window_height);

		void Run();

		Objects::SpectatorObject& GetSpectator();
	private:
		GLFWwindow* m_window;
		uint32_t m_window_width;
		uint32_t m_window_height;
		Render::Renderer m_render;

		TimeHandler m_time;
		bool m_hasSpawned;

		std::vector<std::unique_ptr<Objects::PhysicsObject>> m_objects;
		Objects::SpectatorObject m_spectator;

		Utilities::AABB_Tree m_tree;

		void Init();
		void InitOpenGL();

		void Update();
		void UpdateTime();
		void KeyboardInput();

		void Render();
	};

}