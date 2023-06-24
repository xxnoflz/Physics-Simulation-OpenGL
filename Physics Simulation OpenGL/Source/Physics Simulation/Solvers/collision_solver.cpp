#include "collision_solver.h"

//Collision Detection
bool Solvers::CollisionSolver::CheckCollisionAABB(const Utilities::AABB& first, const Utilities::AABB& second) {
	bool xCoord{ first.GetUpperBound().x >= second.GetLowerBound().x && second.GetUpperBound().x >= first.GetLowerBound().x };
	bool yCoord{ first.GetUpperBound().y >= second.GetLowerBound().y && second.GetUpperBound().y >= first.GetLowerBound().y };
	bool zCoord{ first.GetUpperBound().z >= second.GetLowerBound().z && second.GetUpperBound().z >= first.GetLowerBound().z };
	return xCoord && yCoord && zCoord;
}

std::tuple<bool, Solvers::CollisionSolver::CollisionData> Solvers::CollisionSolver::CheckCollision(Objects::PhysicsObject* first, Objects::PhysicsObject* second) {
	const std::vector<glm::vec3> firstPoints{ first->GetWorldPoints() };
	const std::vector<glm::vec3> secondPoints{ second->GetWorldPoints()  };

	const std::vector<glm::vec3> firstNormals{ first->GetWorldNormals()  };
	const std::vector<glm::vec3> secondNormals{ second->GetWorldNormals()  };

	std::vector<glm::vec3> normal_cross_products{};
	for(const auto& first_normal : firstNormals)
		for(const auto& second_normal : secondNormals) {
			if(glm::cross(first_normal, second_normal) == glm::vec3(0.0f))
				continue;
			normal_cross_products.push_back(glm::normalize(glm::cross(first_normal, second_normal)));
		}

	std::vector<glm::vec3> axes{};
	axes.insert(axes.end(), firstNormals.begin(), firstNormals.end());
	axes.insert(axes.end(), secondNormals.begin(), secondNormals.end());
	axes.insert(axes.end(), normal_cross_products.begin(), normal_cross_products.end());

	CollisionData data{ .distance = std::numeric_limits<float>::max() };

	for(const auto& axis : axes) {
		Projection firstProjection{ GetProjection(axis, firstPoints) };
		Projection secondProjection{ GetProjection(axis, secondPoints) };

		if(firstProjection.maxProjection < secondProjection.minProjection ||
			secondProjection.maxProjection < firstProjection.minProjection)
			return std::make_tuple(false, CollisionData{});

		float overlap_test{ std::min(firstProjection.maxProjection, secondProjection.maxProjection) - std::max(firstProjection.minProjection, secondProjection.minProjection) };
		if(data.distance > overlap_test) {
			data.distance = overlap_test;
			if(firstProjection.minProjection <= secondProjection.minProjection && firstProjection.maxProjection >= secondProjection.minProjection)
				data.normal = axis;
			else if(secondProjection.minProjection <= firstProjection.minProjection && secondProjection.maxProjection >= firstProjection.minProjection)
				data.normal = -axis;
		}
	}
	data.manifold = GenerateManifold(first, second, data.normal);

	return std::make_tuple(true, data);
}

Solvers::CollisionSolver::Projection Solvers::CollisionSolver::GetProjection(const glm::vec3& axis, const std::vector<glm::vec3>& objectPoints) {
	Projection projection{ .maxProjection = -std::numeric_limits<float>::max(), .minProjection = std::numeric_limits<float>::max() };

	for(const auto& point : objectPoints) {
		float currentProjection{ glm::dot(point, axis) };
		if(projection.minProjection > currentProjection) {
			projection.minProjection = currentProjection;
			projection.minPoint = point;
		}
		if(projection.maxProjection < currentProjection) {
			projection.maxProjection = currentProjection;
			projection.maxPoint = point;
		}
	}

	return projection;
}


//Contact Manifold
Utilities::Manifold Solvers::CollisionSolver::GenerateManifold(Objects::PhysicsObject* first, Objects::PhysicsObject* second, const glm::vec3& collisionNormal) {
	const std::vector<Utilities::Model::Face> firstFaces{ first->GetWorldFaces() };
	const std::vector<Utilities::Model::Face> secondFaces{ second->GetWorldFaces() };

	const glm::vec3 furthestFirst{ GetFurthestPoint(firstFaces, collisionNormal) };
	const glm::vec3 furthestSecond{ GetFurthestPoint(secondFaces, -collisionNormal) };

	const uint32_t best_face_first_iterator{ GetSignificantFace(firstFaces, furthestFirst, collisionNormal) };
	const uint32_t best_face_second_iterator{ GetSignificantFace(secondFaces, furthestSecond, -collisionNormal) };

	const Utilities::Model::Face best_face_first{ firstFaces[best_face_first_iterator] };
	const Utilities::Model::Face best_face_second{ secondFaces[best_face_second_iterator] };

	const glm::vec3 firstCross{ glm::cross(best_face_first.normal, collisionNormal) };
	const glm::vec3 secondCross{ glm::cross(best_face_second.normal, collisionNormal) };

	bool flip{ false };

	const Utilities::Model::Face* referenceFace{ (glm::length(firstCross) <= glm::length(secondCross)) ? &best_face_first : &best_face_second };
	const std::vector<Utilities::Model::Face>* referenceFaces{ (glm::length(firstCross) <= glm::length(secondCross)) ? &firstFaces : &secondFaces };
	const Utilities::Model::Face* incidentFace{ (glm::length(secondCross) >= glm::length(firstCross)) ? &best_face_second : &best_face_first };

	if(incidentFace == &best_face_first)
		flip = true;

	std::vector<Utilities::Manifold::Point> clipped_points{ ClipFace(referenceFace, incidentFace, *referenceFaces, flip) };
	return { Utilities::Manifold::ManifoldID(best_face_first_iterator, best_face_second_iterator), first, second, clipped_points, collisionNormal };
}

glm::vec3 Solvers::CollisionSolver::GetFurthestPoint(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& normal) {
	float maxProjection{ -std::numeric_limits<float>::max() };
	glm::vec3 maxPoint{};

	for(const auto& [face_vertices, face_normal, face_edges] : objectFaces) {
		for(const auto& point : face_vertices) {
			float projection{ glm::dot(point, normal) };
			if(projection > maxProjection) {
				maxProjection = projection;
				maxPoint = point;
			}
		}
	}

	return maxPoint;
}

uint32_t Solvers::CollisionSolver::GetSignificantFace(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& requiredVertex, const glm::vec3& normal) {
	float minLength{ std::numeric_limits<float>::max() };
	uint32_t best_face{};

	for(uint32_t face_iterator{}; face_iterator < objectFaces.size(); ++face_iterator) {
		Utilities::Model::Face current_face{ objectFaces[face_iterator] };

		if(std::find(current_face.vertices.begin(), current_face.vertices.end(), requiredVertex) == current_face.vertices.end())
			continue;

		const glm::vec3 crossProduct{ glm::cross(current_face.normal, normal) };
		if(crossProduct == glm::vec3(0.0f))
			return face_iterator;

		float length{ glm::length(crossProduct) };
		if(minLength > length) {
			minLength = length;
			best_face = face_iterator;
		}
	}

	return best_face;
}

std::vector<Utilities::Manifold::Point> Solvers::CollisionSolver::ClipFace(const Utilities::Model::Face* referenceFace, const Utilities::Model::Face* incidentFace,
	const std::vector<Utilities::Model::Face>& referenceFaces, bool flip) 
{
	std::vector<Utilities::Manifold::Point> clipped_points{};
	for(int edge_iterator{}; edge_iterator < incidentFace->edges.size(); ++edge_iterator) {
		glm::vec3 point_position{ incidentFace->edges[edge_iterator].first_point };
		Utilities::Manifold::Feature point_feature{ edge_iterator, -1 };
		Utilities::Manifold::Point point{ point_position, point_position, point_feature };
		clipped_points.push_back(point);
	}

	uint32_t reference_face_iterator{};

	for(uint32_t clipping_face_iterator{}; clipping_face_iterator < referenceFaces.size(); ++clipping_face_iterator) {
		Utilities::Model::Face clipFace{ referenceFaces[clipping_face_iterator] };

		if(clipFace.normal == referenceFace->normal) {
			reference_face_iterator = clipping_face_iterator;
			continue;
		}

		const glm::vec3 referenceNormal{ -clipFace.normal };
		const glm::vec3 referenceVertex{ clipFace.vertices[0] };

		std::vector<Utilities::Manifold::Point> current_clip{};

		for(uint32_t point_iterator{}; point_iterator < clipped_points.size(); ++point_iterator) {
			const Utilities::Manifold::Point first_point{ clipped_points[point_iterator] };
			const Utilities::Manifold::Point second_point{ clipped_points[(point_iterator + 1) % clipped_points.size()] };

			const float firstDistance{ glm::dot(referenceNormal, first_point.position_on_second - referenceVertex) };
			const float secondDistance{ glm::dot(referenceNormal, second_point.position_on_second - referenceVertex) };

			if(firstDistance >= 0.0f && secondDistance >= 0.0f)
				current_clip.push_back(second_point);
			else if(firstDistance >= 0.0f && secondDistance < 0.0f) {
				const glm::vec3 edgeVector{ second_point.position_on_second - first_point.position_on_second };

				Utilities::Manifold::Point clipped_second{ second_point };
				clipped_second.position_on_second = ClipVector(edgeVector, first_point.position_on_second, referenceVertex, referenceNormal);
				clipped_second.feature.clipping_face = clipping_face_iterator;

				current_clip.push_back(clipped_second);
			}
			else if(firstDistance < 0.0f && secondDistance >= 0.0f) {

				const glm::vec3 edgeVector{ first_point.position_on_second - second_point.position_on_second };

				Utilities::Manifold::Point clipped_first{ second_point };
				clipped_first.position_on_second = ClipVector(edgeVector, second_point.position_on_second, referenceVertex, referenceNormal);
				clipped_first.feature.clipping_face = clipping_face_iterator;

				current_clip.push_back(clipped_first);
				current_clip.push_back(second_point);
			}
		}
		clipped_points = current_clip;
	}

	const glm::vec3 referenceNormal{ -referenceFace->normal };
	const glm::vec3 referenceVertex{ referenceFace->vertices[0] };

	std::vector<Utilities::Manifold::Point> current_clip{};

	for(uint32_t point_iterator{}; point_iterator < clipped_points.size(); ++point_iterator) {
		const Utilities::Manifold::Point first_point{ clipped_points[point_iterator] };
		const Utilities::Manifold::Point second_point{ clipped_points[(point_iterator + 1) % clipped_points.size()] };

		const float firstDistance{ glm::dot(referenceNormal, first_point.position_on_second - referenceVertex) };
		const float secondDistance{ glm::dot(referenceNormal, second_point.position_on_second - referenceVertex) };

		if(firstDistance >= 0.0f && secondDistance >= 0.0f) {
			Utilities::Manifold::Point clipped_second{ second_point };
			glm::vec3 delta_vector{ clipped_second.position_on_second - referenceVertex };
			float distance{ glm::dot(delta_vector, referenceNormal) };
			glm::vec3 projected_vertex{ second_point.position_on_second - (distance * referenceNormal) };

			clipped_second.position_on_first = projected_vertex;
			clipped_second.penetration_distance = distance;
			if(flip)
				std::swap(clipped_second.position_on_first, clipped_second.position_on_second);

			current_clip.push_back(clipped_second);
		}
		else if(firstDistance >= 0.0f && secondDistance < 0.0f) {
			Utilities::Manifold::Point clipped_first{ first_point };
			glm::vec3 delta_vector{ clipped_first.position_on_second - referenceVertex };
			float distance{ glm::dot(delta_vector, referenceNormal) };
			glm::vec3 projected_vertex{ clipped_first.position_on_second - (distance * referenceNormal) };

			clipped_first.position_on_first = projected_vertex;
			clipped_first.penetration_distance = distance;
			if(flip)
				std::swap(clipped_first.position_on_first, clipped_first.position_on_second);

			current_clip.push_back(clipped_first);
		}
		else if(firstDistance < 0.0f && secondDistance >= 0.0f) {
			Utilities::Manifold::Point clipped_second{ second_point };
			glm::vec3 delta_vector{ clipped_second.position_on_second - referenceVertex };
			float distance{ glm::dot(delta_vector, referenceNormal) };
			glm::vec3 projected_vertex{ second_point.position_on_second - (distance * referenceNormal) };

			clipped_second.position_on_first = projected_vertex;
			clipped_second.penetration_distance = distance;
			if(flip)
				std::swap(clipped_second.position_on_first, clipped_second.position_on_second);

			current_clip.push_back(clipped_second);
		}
	}
	clipped_points = current_clip;

	std::unordered_set<Utilities::Manifold::Point> s(clipped_points.begin(), clipped_points.end());	//
	clipped_points.assign(s.begin(), s.end());														//only unique elements

	return clipped_points;
}

glm::vec3 Solvers::CollisionSolver::ClipVector(const glm::vec3& subjectVector, const glm::vec3& subjectOrigin, const glm::vec3& clipVertex, const glm::vec3& clipNormal) {
	float enumerator{ glm::dot((clipVertex - subjectOrigin), clipNormal) };
	float denominator{ glm::dot(glm::normalize(subjectVector), clipNormal) };

	return subjectOrigin + glm::normalize(subjectVector) * (enumerator / denominator);
}


//Collision Response
void Solvers::CollisionSolver::ResolveCollision(CollisionData& collision_data) {
	Objects::PhysicsObject* first_object{ collision_data.manifold.GetFirstObject() };
	Objects::PhysicsObject* second_object{ collision_data.manifold.GetSecondObject() };

	const float total_inverse_mass{ (1.0f / first_object->GetMass()) + (1.0f / second_object->GetMass()) };

	for(uint32_t current_point{}; current_point < collision_data.manifold.GetPoints().size(); ++current_point) {
		SolveFrictionImpulse(first_object, second_object, collision_data, current_point, total_inverse_mass);
		SolveNormalImpulse(first_object, second_object, collision_data, current_point, total_inverse_mass);
	}
}

void Solvers::CollisionSolver::ResolvePenetration(CollisionData& collision_data) {
	Objects::PhysicsObject* first_object{ collision_data.manifold.GetFirstObject() };
	Objects::PhysicsObject* second_object{ collision_data.manifold.GetSecondObject() };

	const float total_inverse_mass{ (1.0f / first_object->GetMass()) + (1.0f / second_object->GetMass()) };

	for(uint32_t current_point{}; current_point < collision_data.manifold.GetPoints().size(); ++current_point) 
		SolvePenetrationImpulse(first_object, second_object, collision_data, current_point, total_inverse_mass);
}

//Normal impulse
void Solvers::CollisionSolver::SolveNormalImpulse(Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
	CollisionData& collision_data, const uint32_t current_point, const float total_inverse_mass) 
{
	const Utilities::Manifold::ContactPointData contact_data{ collision_data.manifold.GetContactPointData(current_point) };

	const float normal_impulse{ glm::dot(contact_data.contact_velocity, collision_data.normal) };

	const glm::vec3 inertia_first{ glm::cross(first_object->GetInverseWorldTensor() *
		glm::cross(contact_data.position_relative_first, collision_data.normal), contact_data.position_relative_first) };
	const glm::vec3 inertia_second{ glm::cross(second_object->GetInverseWorldTensor() *
		glm::cross(contact_data.position_relative_second, collision_data.normal), contact_data.position_relative_second) };
	const float angular_effect{ glm::dot(inertia_first + inertia_second, collision_data.normal) };

	float impulse{ -(1.0f + 0.66f) * normal_impulse / (total_inverse_mass + angular_effect) }; //Restitution

	const float temp_accumulated{ collision_data.manifold.GetPoints()[current_point].accumulated_impulse };
	collision_data.manifold.GetPoints()[current_point].accumulated_impulse = std::max(temp_accumulated + impulse, 0.0f);
	impulse = collision_data.manifold.GetPoints()[current_point].accumulated_impulse - temp_accumulated;

	const glm::vec3 result_impulse{ collision_data.normal * impulse };

	first_object->AppyLinearImpulse(-result_impulse);
	first_object->AppyAngularImpulse(-glm::cross(contact_data.position_relative_first, result_impulse));

	second_object->AppyLinearImpulse(result_impulse);
	second_object->AppyAngularImpulse(glm::cross(contact_data.position_relative_second, result_impulse));
}

//Friction impulse
void Solvers::CollisionSolver::SolveFrictionImpulse(Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
	CollisionData& collision_data, const uint32_t current_point, const float total_inverse_mass) 
{
	const Utilities::Manifold::ContactPointData contact_data{ collision_data.manifold.GetContactPointData(current_point) };

	const float friction_coefficient{ 0.6f };

	for(uint32_t current_tangent{}; current_tangent < collision_data.manifold.GetPoints()[current_point].tangents.size(); ++current_tangent) {
		const float tangent_impulse{ glm::dot(contact_data.contact_velocity, collision_data.manifold.GetPoints()[current_point].tangents[current_tangent]) };

		const glm::vec3 inertia_first{ glm::cross(first_object->GetInverseWorldTensor() *
			glm::cross(contact_data.position_relative_first, collision_data.manifold.GetPoints()[current_point].tangents[current_tangent]),
			contact_data.position_relative_first) };
		const glm::vec3 inertia_second{ glm::cross(second_object->GetInverseWorldTensor() *
			glm::cross(contact_data.position_relative_second, collision_data.manifold.GetPoints()[current_point].tangents[current_tangent]),
			contact_data.position_relative_second) };
		const float angular_effect{ glm::dot(inertia_first + inertia_second, collision_data.manifold.GetPoints()[current_point].tangents[current_tangent]) };

		float impulse{ -tangent_impulse / (total_inverse_mass + angular_effect) };

		const float max_impulse{ friction_coefficient * collision_data.manifold.GetPoints()[current_point].accumulated_impulse };

		const float temp_accumulated{ collision_data.manifold.GetPoints()[current_point].accumulated_friction[current_tangent] };
		collision_data.manifold.GetPoints()[current_point].accumulated_friction[current_tangent] = glm::clamp(temp_accumulated + impulse, -max_impulse, max_impulse);
		impulse = collision_data.manifold.GetPoints()[current_point].accumulated_friction[current_tangent] - temp_accumulated;

		const glm::vec3 result_impulse{ collision_data.manifold.GetPoints()[current_point].tangents[current_tangent] * impulse };

		first_object->AppyLinearImpulse(-result_impulse);
		first_object->AppyAngularImpulse(-glm::cross(contact_data.position_relative_first, result_impulse));

		second_object->AppyLinearImpulse(result_impulse);
		second_object->AppyAngularImpulse(glm::cross(contact_data.position_relative_second, result_impulse));
	}
}

//Position correction
void Solvers::CollisionSolver::SolvePenetrationImpulse(Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object,
	CollisionData& collision_data, const uint32_t current_point, const float total_inverse_mass) 
{
	const glm::vec3 world_first_point{ first_object->GetMatrix() *
		glm::vec4(collision_data.manifold.GetPoints()[current_point].local_first, 1.0f) };
	const glm::vec3 world_second_point{ second_object->GetMatrix() *
		glm::vec4(collision_data.manifold.GetPoints()[current_point].local_second, 1.0f) };

	const glm::vec3 delta_vector{ world_second_point - world_first_point };
	const float separation{ glm::dot(collision_data.normal, delta_vector) };

	const glm::vec3 position_relative_first{ world_first_point - first_object->GetCenter() };
	const glm::vec3 position_relative_second{ world_second_point - second_object->GetCenter() };

	const glm::vec3 inertia_first{ glm::cross(first_object->GetInverseWorldTensor() *
		glm::cross(position_relative_first, collision_data.normal), position_relative_first) };
	const glm::vec3 inertia_second{ glm::cross(second_object->GetInverseWorldTensor() *
		glm::cross(position_relative_second, collision_data.normal), position_relative_second) };
	const float angular_effect{ glm::dot(inertia_first + inertia_second, collision_data.normal) };

	const float steering_force{ glm::clamp(BIAS * (separation + SLOP), -MAX_LINEAR_CORRECTION, 0.0f) };
	const float impulse{ -steering_force / (angular_effect + total_inverse_mass)};

	const glm::vec3 result_impulse{ collision_data.normal * impulse };

	first_object->IntegrateImpulse(-result_impulse, -glm::cross(position_relative_first, result_impulse));

	second_object->IntegrateImpulse(result_impulse, glm::cross(position_relative_second, result_impulse));
}