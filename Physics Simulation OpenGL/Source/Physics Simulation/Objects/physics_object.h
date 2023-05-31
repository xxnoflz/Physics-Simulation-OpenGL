#pragma once
#include <string>
#include <string_view>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/inertia.hpp>

#include "../Utilities/resource_manager.h"
#include "../Utilities/aabb.h"
#include "../Render/renderer.h"

#include "basic_object.h"

namespace Objects {

	static uint32_t id_counter{ 0 };

	class PhysicsObject : public BasicObject {
	public:
		PhysicsObject(const glm::vec3& position, const glm::vec3& size, bool isKinematic, const float mass, const glm::vec3& start_linear_velocity, std::string_view model_name);

		void Draw(Render::Renderer* render);

		void UpdateMatrix();

		void Integrate(float deltaTime);
		void Accelerate(const glm::vec3& acceleration, float deltaTime);

		void AppyLinearImpulse(const glm::vec3& impulse);
		void AppyAngularImpulse(const glm::vec3& impulse);

		void UpdateAABB();

		void UpdateWorldPoints();
		void UpdateWorldNormals();
		void UpdateFaces();
		bool NotUpdatedFaces();
		void ClearFaces();

		const std::vector<glm::vec3>& GetWorldPoints();
		const std::vector<glm::vec3>& GetWorldNormals();
		const std::vector<Utilities::Model::Face>& GetWorldFaces();
		const Utilities::AABB& GetAABB() const;
		const float GetMass() const;
		const bool isKinematic() const;
		const glm::vec3& GetLinearVelocity() const;
		const glm::vec3& GetAngularVelocity() const;
		const glm::mat3 GetInverseWorldTensor();
		const glm::vec3 GetCenter() const;
		std::string_view GetModelName() const;
		const glm::mat4 GetRotate();
		const uint32_t GetID() const;
		const std::vector<glm::vec4>& GetVertices() const;
		const std::vector<glm::vec3>& GetNormals() const;
		const glm::mat4& GetMatrix() const;
	private:
		uint32_t m_id;
		bool m_isKinematic;
		float m_mass;

		glm::vec3 m_linear_velocity;
		glm::vec3 m_angular_velocity;
		glm::vec3 m_acceleration;
		glm::mat3 m_inertia_tensor;

		std::string m_model_name;
		glm::mat4 m_model_matrix;

		Utilities::AABB m_aabb;

		std::vector<glm::vec3> m_world_points;
		std::vector<glm::vec3> m_world_normals;
		std::vector<Utilities::Model::Face> m_world_faces;
	};

}