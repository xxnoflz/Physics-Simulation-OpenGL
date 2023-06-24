#include "manifold.h"

Utilities::Manifold::Manifold() :
	m_id(),
	m_first_object(), m_second_object(),
	m_points() {

}

Utilities::Manifold::Manifold(ManifoldID id, Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
	std::vector<Point> points, const glm::vec3& normal) : m_id(id), m_first_object(first_object), m_second_object(second_object), m_points(points) {
	InitializeData(normal);
}

float RecipSqrt(float a) {
	return 1.0f / glm::sqrt(a);
}

void GetTwoTangential(const glm::vec3& n, glm::vec3& p, glm::vec3& q) {
	//if(glm::abs(n.z) > 0.7071067811865475244008443621048490) {
	//	float a = n[1] * n[1] + n[2] * n[2];
	//	float k = RecipSqrt(a);
	//	p[0] = 0;
	//	p[1] = -n[2] * k;
	//	p[2] = n[1] * k;
	//	// set q = n x p
	//	q[0] = a * k;
	//	q[1] = -n[0] * p[2];
	//	q[2] = n[0] * p[1];
	//}
	//else {
	//	// choose p in x-y plane
	//	float a = n[0] * n[0] + n[1] * n[1];
	//	float k = RecipSqrt(a);
	//	p[0] = -n[1] * k;
	//	p[1] = n[0] * k;
	//	p[2] = 0;
	//	// set q = n x p
	//	q[0] = -n[2] * p[1];
	//	q[1] = n[2] * p[0];
	//	q[2] = a * k;
	//}
	if(glm::abs(n.x) >= 0.57735f)
		p = glm::vec3(n.y, -n.x, 0.0f);
	else
		p = glm::vec3(0.0f, n.z, -n.y);

	p = glm::normalize(p);
	q = glm::cross(n, p);
}

void Utilities::Manifold::InitializeData(const glm::vec3& normal) {
	for(uint32_t current_point{}; current_point < m_points.size(); ++current_point) {
		m_points[current_point].contact_data.position_relative_first = m_points[current_point].position_on_first - m_first_object->GetCenter();
		m_points[current_point].contact_data.position_relative_second = m_points[current_point].position_on_second - m_second_object->GetCenter();

		m_points[current_point].local_first = glm::inverse(m_first_object->GetMatrix()) *
			glm::vec4(m_points[current_point].position_on_first, 1.0f);
		m_points[current_point].local_second = glm::inverse(m_second_object->GetMatrix()) *
			glm::vec4(m_points[current_point].position_on_second, 1.0f);

		const Utilities::Manifold::ContactPointData contact_data{ GetContactPointData(current_point) };

		glm::vec3 first_tangent{ contact_data.contact_velocity - normal * glm::dot(contact_data.contact_velocity, normal) };
		glm::vec3 second_tangent{};
		float length_squared_first{ glm::length2(first_tangent) };

		if(length_squared_first > 0.0f) {
			first_tangent /= glm::sqrt(length_squared_first);
			second_tangent = glm::cross(first_tangent, normal);
			second_tangent = glm::normalize(second_tangent);
		}
		else
			GetTwoTangential(normal, first_tangent, second_tangent);

		m_points[current_point].tangents = { first_tangent, second_tangent };
	}
}

void Utilities::Manifold::Update(Manifold& new_manifold) {
	for(uint32_t new_point_iterator{}; new_point_iterator < new_manifold.m_points.size(); ++new_point_iterator) {

		Point* new_point{ &new_manifold.m_points[new_point_iterator] };
		for(uint32_t old_point_iterator{}; old_point_iterator < m_points.size(); ++old_point_iterator) {
			Point old_point{ m_points[old_point_iterator] };

			if(new_point->feature == old_point.feature) {
				new_point->accumulated_impulse = old_point.accumulated_impulse;

				glm::vec3 friction{ old_point.accumulated_friction[0] * old_point.tangents[0] + old_point.accumulated_friction[1] * old_point.tangents[1] };
				new_point->accumulated_friction[0] = glm::dot(friction, new_point->tangents[0]);
				new_point->accumulated_friction[1] = glm::dot(friction, new_point->tangents[1]);
				break;
			}
		}
	}
}

void Utilities::Manifold::WarmStart(const glm::vec3& normal) {
	for(uint32_t current_point{}; current_point < m_points.size(); ++current_point) {
		const Utilities::Manifold::ContactPointData contact_data{ GetContactPointData(current_point) };

		glm::vec3 warm_start{ m_points[current_point].accumulated_impulse * normal };
		warm_start += m_points[current_point].accumulated_friction[0] * m_points[current_point].tangents[0];
		warm_start += m_points[current_point].accumulated_friction[1] * m_points[current_point].tangents[1];

		m_first_object->AppyLinearImpulse(-warm_start);
		m_first_object->AppyAngularImpulse(-glm::cross(contact_data.position_relative_first, warm_start));

		m_second_object->AppyLinearImpulse(warm_start);
		m_second_object->AppyAngularImpulse(glm::cross(contact_data.position_relative_second, warm_start));
	}
}

const Utilities::Manifold::ContactPointData& Utilities::Manifold::GetContactPointData(uint32_t contact_iterator) {
	if(contact_iterator < 0 || contact_iterator >= m_points.size())
		return {};

	m_points[contact_iterator].contact_data.velocity_angular_first = glm::cross(m_first_object->GetAngularVelocity(),
		m_points[contact_iterator].contact_data.position_relative_first);
	m_points[contact_iterator].contact_data.velocity_angular_second = glm::cross(m_second_object->GetAngularVelocity(),
		m_points[contact_iterator].contact_data.position_relative_second);

	m_points[contact_iterator].contact_data.velocity_full_first = m_first_object->GetLinearVelocity() +
		m_points[contact_iterator].contact_data.velocity_angular_first;
	m_points[contact_iterator].contact_data.velocity_full_second = m_second_object->GetLinearVelocity() +
		m_points[contact_iterator].contact_data.velocity_angular_second;
	m_points[contact_iterator].contact_data.contact_velocity = m_points[contact_iterator].contact_data.velocity_full_second - m_points[contact_iterator].contact_data.velocity_full_first;

	return m_points[contact_iterator].contact_data;
}

std::vector<Utilities::Manifold::Point>& Utilities::Manifold::GetPoints() {
	return m_points;
}

Objects::PhysicsObject* Utilities::Manifold::GetFirstObject() {
	return m_first_object;
}

Objects::PhysicsObject* Utilities::Manifold::GetSecondObject() {
	return m_second_object;
}

Utilities::Manifold::ManifoldID Utilities::Manifold::GetID() const {
	return m_id;
}