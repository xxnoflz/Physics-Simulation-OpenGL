#include "physics_object.h"

Objects::PhysicsObject::PhysicsObject(const glm::vec3& position, const glm::vec3& size, 
	bool isKinematic, const float mass, const glm::vec3& start_linear_velocity, std::string_view model_name,
	std::string_view texture_name)
	: BasicObject(position, size), m_isKinematic(isKinematic), m_mass(mass), m_inertia_tensor(glm::boxInertia3(mass, size)), m_linear_velocity(start_linear_velocity),
	m_model_name(model_name), m_texture_name(texture_name), m_aabb(), m_id(id_counter++), m_world_points(), m_world_normals()
{
	UpdateMatrix();

	UpdateWorldPoints();
	UpdateWorldNormals();

	UpdateAABB();
}

void Objects::PhysicsObject::Draw(Render::Renderer* render) {
	UpdateMatrix();
	std::string shaderName{ Utilities::ResourceManager::GetModel(m_model_name).GetShader() };
	Utilities::Model usedModel{ Utilities::ResourceManager::GetModel(m_model_name) };

	Utilities::ResourceManager::UseTexture(m_texture_name);
	Utilities::ResourceManager::UseShader(shaderName);
	Utilities::ResourceManager::GetShader(shaderName).SetMat4("model", m_model_matrix);
	usedModel.Draw(render);
}

void Objects::PhysicsObject::UpdateMatrix() {
	m_model_matrix = glm::mat4(1.0f);
	m_model_matrix = glm::translate(m_model_matrix, GetPosition());

	m_model_matrix = glm::translate(m_model_matrix, 0.5f * GetSize());
	m_model_matrix *= glm::toMat4(GetRotation());
	m_model_matrix = glm::translate(m_model_matrix, -0.5f * GetSize());

	m_model_matrix = glm::scale(m_model_matrix, GetSize());
}

void Objects::PhysicsObject::Integrate(float deltaTime) {
	//Semi-implicit Euler integration
	GetPosition() += m_linear_velocity * deltaTime;
	
	GetRotation() += glm::quat(0.0f, m_angular_velocity * deltaTime * 0.5f) * GetRotation();
	GetRotation() = glm::normalize(GetRotation());

	UpdateMatrix();
}

void Objects::PhysicsObject::IntegrateImpulse(const glm::vec3& linear_impulse, const glm::vec3& angular_impulse) {
	//Semi-implicit Euler integration
	GetPosition() += (1.0f / m_mass) * linear_impulse;

	GetRotation() += glm::quat(0.0f, (GetInverseWorldTensor() * angular_impulse) * 0.5f) * GetRotation();
	GetRotation() = glm::normalize(GetRotation());

	UpdateMatrix();
}

void Objects::PhysicsObject::Accelerate(const glm::vec3& acceleration, float deltaTime) {
	m_linear_velocity += acceleration * deltaTime;
}

void Objects::PhysicsObject::AppyLinearImpulse(const glm::vec3& impulse) {
	m_linear_velocity += impulse * (1.0f / m_mass);
}

void Objects::PhysicsObject::AppyAngularImpulse(const glm::vec3& impulse) {
	m_angular_velocity += GetInverseWorldTensor() * impulse;
}

void Objects::PhysicsObject::UpdateAABB() {
	m_aabb.Update(GetWorldPoints());
}

void Objects::PhysicsObject::UpdateWorldPoints() {
	m_world_points.clear();
	const std::vector<glm::vec4> objectPoints{ GetVertices() };

	for (const auto& point : objectPoints)
		m_world_points.push_back(m_model_matrix * point);
}

void Objects::PhysicsObject::UpdateWorldNormals() {
	m_world_normals.clear();
	const std::vector<glm::vec3> objectNormals{ GetNormals() };

	for (const auto& normal : objectNormals)
		m_world_normals.push_back(glm::normalize(GetRotate() * glm::vec4(normal, 0.0f)));
}

void Objects::PhysicsObject::UpdateFaces() {
	const std::vector<Utilities::Model::Face> objectFaces{ Utilities::ResourceManager::GetModel(m_model_name).GetFaces() };

	for (const auto& [vertices, normal, edges] : objectFaces) {
		Utilities::Model::Face face{};
		face.normal = glm::normalize(GetRotate() * glm::vec4(normal, 0.0f));
		for (const auto& vertex : vertices)
			face.vertices.push_back(m_model_matrix * glm::vec4(vertex, 1.0f));
		for(const auto& edge : edges) {
			glm::vec3 first_transformed{ m_model_matrix * glm::vec4(edge.first_point, 1.0f) };
			glm::vec3 second_transformed{ m_model_matrix * glm::vec4(edge.second_point, 1.0f) };
			face.edges.push_back( {first_transformed, second_transformed} );
		}
		m_world_faces.push_back(face);
	}
}

bool Objects::PhysicsObject::NotUpdatedFaces() {
	return !m_world_faces.empty();
}

void Objects::PhysicsObject::ClearFaces() {
	m_world_faces.clear();
}

const std::vector<glm::vec3>& Objects::PhysicsObject::GetWorldPoints() {
	return m_world_points;
}

const std::vector<glm::vec3>& Objects::PhysicsObject::GetWorldNormals() {
	return m_world_normals;
}

const std::vector<Utilities::Model::Face>& Objects::PhysicsObject::GetWorldFaces() {
	if (m_world_faces.empty()) 
		UpdateFaces();

	return m_world_faces;
}

const Utilities::AABB& Objects::PhysicsObject::GetAABB() const {
	return m_aabb;
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

const glm::mat3 Objects::PhysicsObject::GetInverseWorldTensor() {
	if(m_mass == std::numeric_limits<float>::infinity())
		return glm::mat3(0.0f);
	glm::mat3 rotation_matrix{ GetRotate() };
	glm::mat3 rotation_transpose{ glm::transpose(rotation_matrix) };
	return (rotation_matrix * glm::inverse(m_inertia_tensor) * rotation_transpose);
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
	return glm::toMat4(GetRotation());
}

const uint32_t Objects::PhysicsObject::GetID() const {
	return m_id;
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
