#include "physics_object.h"

Objects::PhysicsObject::PhysicsObject(const glm::vec3& position, const glm::vec3& size, bool isKinematic, float mass, glm::vec3 start_linear_velocity, std::string_view model_name) :
	BasicObject(position, size), m_isKinematic(isKinematic), m_mass(mass), m_inertia_tensor(glm::boxInertia3(m_mass, size)), m_lastPos(), m_linear_velocity(start_linear_velocity),
	m_model_name(model_name)
{
	UpdateMatrix();
}

void Objects::PhysicsObject::UpdateMatrix() {
	m_model_matrix = glm::mat4(1.0f);
	m_model_matrix = glm::translate(m_model_matrix, GetPosition());
	m_model_matrix *= glm::toMat4(GetRotation());
	m_model_matrix = glm::scale(m_model_matrix, GetSize());
}

const glm::mat4& Objects::PhysicsObject::GetMatrix() const {
	return m_model_matrix;
}

const std::vector<glm::vec4>& Objects::PhysicsObject::GetVertices() const {
	return Utilities::ResourceManager::GetModel(m_model_name).GetVertices();
}

const std::vector<glm::vec3>& Objects::PhysicsObject::GetNormals() const {
	return Utilities::ResourceManager::GetModel(m_model_name).GetNormals();
}

const float Objects::PhysicsObject::GetMass() const {
	return m_mass;
}
const bool Objects::PhysicsObject::isKinematic() const {
	return m_isKinematic;
}

const glm::vec3& Objects::PhysicsObject::GetLinearVelocity() const {
	return m_linear_velocity;
}
const glm::vec3& Objects::PhysicsObject::GetAngularVelocity() const {
	return m_angular_velocity;
}
const glm::mat3& Objects::PhysicsObject::GetTensor() {
	return m_inertia_tensor;
}
const glm::mat3 Objects::PhysicsObject::GetInverseWorldTensor() {
	glm::mat3 rotation_matrix{ GetRotate() };
	glm::mat3 rotation_transpose{ glm::transpose(rotation_matrix) };
	return (rotation_matrix * glm::inverse(m_inertia_tensor) * rotation_transpose);
}
const glm::vec3& Objects::PhysicsObject::GetLastPos() const {
	return m_lastPos;
}
const glm::vec3 Objects::PhysicsObject::GetCenter() const {
	std::vector<glm::vec4> objectPoints{ GetVertices() };
	if (objectPoints.empty())
		return {};
	glm::vec4 m_box_min{};
	glm::vec4 m_box_max{};
	m_box_min = *std::min_element(objectPoints.begin(), objectPoints.end(), [=](const glm::vec3& first, const glm::vec3& smallest) {
		return glm::all(glm::lessThan(first, smallest));
		});
	m_box_max = *std::max_element(objectPoints.begin(), objectPoints.end(), [=](const glm::vec3& largest, const glm::vec3& first) {
		return glm::all(glm::greaterThan(first, largest));
		});
	return m_model_matrix * ((m_box_min + m_box_max) / 2.0f);
	//return m_model_matrix * glm::vec4(0.5f, 0.5f, 0.5f, 1.0f); //Use if not working
}
std::string_view Objects::PhysicsObject::GetModelName() const {
	return m_model_name;
}
const glm::mat4 Objects::PhysicsObject::GetRotate() {
	return glm::mat4_cast(GetRotation());
}

void Objects::PhysicsObject::Draw(Render::Renderer* render) {
	UpdateMatrix();
	std::string shaderName{ Utilities::ResourceManager::GetModel(m_model_name).GetShader() };
	Utilities::Model usedModel{ Utilities::ResourceManager::GetModel(m_model_name) };
	Utilities::ResourceManager::UseShader(shaderName);
	Utilities::ResourceManager::GetShader(shaderName).SetMat4("model", m_model_matrix);
	if(m_isKinematic)
		Utilities::ResourceManager::GetShader(shaderName).SetVec3("objectColor", glm::vec3(0.4f, 0.4f, 0.4f));
	else
		Utilities::ResourceManager::GetShader(shaderName).SetVec3("objectColor", glm::vec3(0.8f, 0.8f, 0.8f));
	usedModel.Draw(render);
}

void Objects::PhysicsObject::Integrate(float deltaTime) {
	if (!m_isKinematic || deltaTime == 0.0f)
		return;
	m_lastPos = GetPosition();
	//Semi-implicit Euler integration
	m_linear_velocity += m_acceleration * deltaTime;
	GetPosition() += m_linear_velocity * deltaTime;
	
	GetRotation() += glm::quat(0.0f, m_angular_velocity * deltaTime * 0.5f) * GetRotation();
	GetRotation() = glm::normalize(GetRotation());

	m_acceleration = glm::vec3(0.0f);
	UpdateMatrix();
}

void Objects::PhysicsObject::Accelerate(glm::vec3 acceleration) {
	if (!m_isKinematic)
		return;
	m_acceleration += acceleration;
}

void Objects::PhysicsObject::AppyLinearImpulse(const glm::vec3& impulse) {
	if (!m_isKinematic)
		return;
	m_linear_velocity += impulse * (1.0f / m_mass);
}
void Objects::PhysicsObject::AppyAngularImpulse(const glm::vec3& impulse) {
	if (!m_isKinematic)
		return;
	m_angular_velocity += GetInverseWorldTensor() * impulse;
}