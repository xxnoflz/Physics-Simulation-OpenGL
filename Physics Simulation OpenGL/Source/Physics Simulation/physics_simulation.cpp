#include "physics_simulation.h"

Simulation::PhysicsSimulation::PhysicsSimulation(const uint32_t window_width, const uint32_t window_height) :
	m_window(), m_window_width(window_width), m_window_height(window_height),
	m_spectator(spectator_start_position), m_time(), m_objects(),
	m_hasSpawned(false), m_tree()
{
	Init();
}

void MouseCallback(GLFWwindow* window, double xPos, double yPos) {
	Simulation::PhysicsSimulation* pointer{ static_cast<Simulation::PhysicsSimulation*>(glfwGetWindowUserPointer(window)) };
	pointer->GetSpectator().MouseInput((float)xPos, (float)yPos);
}

void Simulation::PhysicsSimulation::KeyboardInput() {
	if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
		m_spectator.KeyboardInput(Objects::SpectatorObject::W, m_time.deltaTime);
	if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS)
		m_spectator.KeyboardInput(Objects::SpectatorObject::S, m_time.deltaTime);
	if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS)
		m_spectator.KeyboardInput(Objects::SpectatorObject::A, m_time.deltaTime);
	if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS)
		m_spectator.KeyboardInput(Objects::SpectatorObject::D, m_time.deltaTime);
	if (glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(m_window, true);

	if (glfwGetKey(m_window, GLFW_KEY_E) == GLFW_PRESS && !m_hasSpawned) {
		m_objects.push_back(std::make_unique<Objects::PhysicsObject>(m_spectator.GetPosition() + (m_spectator.GetDirection() * 3.0f),
			glm::vec3(1.0f), true, 20.0f, m_spectator.GetDirection() * 7.5f,
			"cube_model", "kinematic_texture"));
		m_hasSpawned = true;
		m_tree.InsertLeaf(m_objects.back().get());

		//for(int it{}; it < 10; ++it){
		//	m_objects.push_back(std::make_unique<Objects::PhysicsObject>(glm::vec3(5.0f, 0.5f + (it * 1.0f), 5.0f),
		//	glm::vec3(1.0f), true, 1.0f, glm::vec3(0.0f),
		//	"cube_model", "kinematic_texture"));
		//	m_hasSpawned = true;
		//	m_tree.InsertLeaf(m_objects.back().get());
		//}
	}
	else if (glfwGetKey(m_window, GLFW_KEY_Q) == GLFW_PRESS && !m_hasSpawned) {
		m_objects.push_back(std::make_unique<Objects::PhysicsObject>(m_spectator.GetPosition() + (m_spectator.GetDirection() * 3.0f),
			glm::vec3(10.0f, 0.5f, 10.0f), false, std::numeric_limits<float>::infinity(), glm::vec3(0.0f),
			"cube_model", "static_texture"));
		m_hasSpawned = true;
		m_tree.InsertLeaf(m_objects.back().get());
	}
	else if (glfwGetKey(m_window, GLFW_KEY_E) == GLFW_RELEASE && glfwGetKey(m_window, GLFW_KEY_Q) == GLFW_RELEASE)
		m_hasSpawned = false;
}

Objects::SpectatorObject& Simulation::PhysicsSimulation::GetSpectator() {
	return m_spectator;
}

void Simulation::PhysicsSimulation::Init() {
	InitOpenGL();

	m_render.CreateBuffer();

	Utilities::ResourceManager::LoadShader("Source/Physics Simulation/Shaders/basic_vertex.glsl", "Source/Physics Simulation/Shaders/basic_fragment.glsl", "basic_shader");
	Utilities::ResourceManager::UseShader("basic_shader");
	m_render.BindShader(Utilities::ResourceManager::GetCurrentShader(), "Matrices");

	Utilities::ResourceManager::LoadTexture("Source/Physics Simulation/Textures/kinematic.jpg", "kinematic_texture");
	Utilities::ResourceManager::LoadTexture("Source/Physics Simulation/Textures/static.jpg", "static_texture");

	glm::mat4 projection{ glm::perspective(fov_value, (float)m_window_width / m_window_height, 0.1f, 100.0f) };
	m_render.UpdateBuffer(projection, Render::Renderer::Offsets::Projection);

	Utilities::ResourceManager::LoadModel("Source/Physics Simulation/Models/cube_render.obj", "Source/Physics Simulation/Models/cube_collision.obj", "cube_model", "basic_shader");
}

void Simulation::PhysicsSimulation::InitOpenGL() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	m_window = glfwCreateWindow(m_window_width, m_window_height, window_title.data(), nullptr, nullptr);
	if (!m_window) {
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(m_window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		glfwTerminate();
		return;
	}

	glViewport(0, 0, m_window_width, m_window_height);
	glEnable(GL_DEPTH_TEST);

	glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPosCallback(m_window, MouseCallback);
	glfwSetWindowUserPointer(m_window, this);
}

void Simulation::PhysicsSimulation::Update() {
	UpdateTime();
	KeyboardInput();
	Solvers::PhysicsSolver::Update(m_objects, m_tree, m_time.deltaTime);
}

void Simulation::PhysicsSimulation::UpdateTime() {
	m_time.currentFrame = (float)glfwGetTime();
	m_time.deltaTime = m_time.currentFrame - m_time.lastFrame;
	m_time.lastFrame = m_time.currentFrame;
}

void Simulation::PhysicsSimulation::Render() {
	m_render.UpdateBuffer(m_spectator.GetMatrix(), Render::Renderer::Offsets::View);
	for (const auto& object : m_objects)
		object->Draw(&m_render);
}

void Simulation::PhysicsSimulation::Run() {
	while (!glfwWindowShouldClose(m_window)) {
		Update();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

		Render();

		glfwSwapBuffers(m_window);
		glfwPollEvents();
	}
	glfwTerminate();
}