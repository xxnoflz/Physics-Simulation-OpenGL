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

	std::vector<glm::vec3> axes{};
	axes.insert(axes.end(), firstNormals.begin(), firstNormals.end());
	axes.insert(axes.end(), secondNormals.begin(), secondNormals.end());

	CollisionData data{ std::numeric_limits<float>::max() };

	for (const auto& axis : axes) {
		Projection firstProjection{ GetProjection(axis, firstPoints) };
		Projection secondProjection{ GetProjection(axis, secondPoints) };

		if (firstProjection.maxProjection < secondProjection.minProjection ||
			secondProjection.maxProjection < firstProjection.minProjection)
			return std::make_tuple(false, CollisionData{});

		float overlap_test{ std::min(firstProjection.maxProjection, secondProjection.maxProjection) - std::max(firstProjection.minProjection, secondProjection.minProjection) };
		if (data.distance > overlap_test) {
			data.distance = overlap_test;
			if (firstProjection.minProjection <= secondProjection.minProjection && firstProjection.maxProjection >= secondProjection.minProjection)
				data.normal = axis;
			else if (secondProjection.minProjection <= firstProjection.minProjection && secondProjection.maxProjection >= firstProjection.minProjection)
				data.normal = -axis;
		}
	}

	data.manifold.vertices = GenerateManifold(first, second, data.normal);
	data.manifold.firstObject = first;
	data.manifold.secondObject = second;

	return std::make_tuple(true, data);
}

Solvers::CollisionSolver::Projection Solvers::CollisionSolver::GetProjection(const glm::vec3& axis, const std::vector<glm::vec3>& objectPoints) {
	Projection projection{};
	projection.minProjection = std::numeric_limits<float>::max();
	projection.maxProjection = -std::numeric_limits<float>::max();
	for (const auto& point : objectPoints) {
		float currentProjection{ glm::dot(point, axis) };
		if (projection.minProjection > currentProjection) {
			projection.minProjection = currentProjection;
			projection.minPoint = point;
		}
		if (projection.maxProjection < currentProjection) {
			projection.maxProjection = currentProjection;
			projection.maxPoint = point;
		}
	}
	return projection;
}


//Contact Manifold
std::vector<glm::vec3> Solvers::CollisionSolver::GenerateManifold(Objects::PhysicsObject* first, Objects::PhysicsObject* second, const glm::vec3& collisionNormal) {
	std::vector<Utilities::Model::Face> firstFaces{ first->GetWorldFaces() };
	std::vector<Utilities::Model::Face> secondFaces{ second->GetWorldFaces() };

	const glm::vec3 furthestFirst{ GetFurthestPoint(firstFaces, collisionNormal) };
	const glm::vec3 furthestSecond{ GetFurthestPoint(secondFaces, -collisionNormal) };

	Utilities::Model::Face bestFaceFirst{ GetSignificantFace(firstFaces, furthestFirst, collisionNormal) };
	Utilities::Model::Face bestFaceSecond{ GetSignificantFace(secondFaces, furthestSecond, -collisionNormal) };

	glm::vec3 firstCross{ glm::cross(bestFaceFirst.normal, collisionNormal) };
	glm::vec3 secondCross{ glm::cross(bestFaceSecond.normal, collisionNormal) };

	Utilities::Model::Face ClippedFace{};
	Utilities::Model::Face* referenceFace{ (glm::length(firstCross) <= glm::length(secondCross)) ? &bestFaceFirst : &bestFaceSecond };
	std::vector<Utilities::Model::Face>* referenceFaces{ (glm::length(firstCross) <= glm::length(secondCross)) ? &firstFaces : &secondFaces };
	Utilities::Model::Face* incidentFace{ (glm::length(secondCross) >= glm::length(firstCross)) ? &bestFaceSecond : &bestFaceFirst };
	ClippedFace = ClipFace(referenceFace, incidentFace, *referenceFaces);

	return ClippedFace.vertices;
}

glm::vec3 Solvers::CollisionSolver::GetFurthestPoint(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& normal) {
	float maxProjection{ -std::numeric_limits<float>::max() };
	glm::vec3 maxPoint{};

	for (const auto& [face_vertices, face_normal] : objectFaces) {
		for (const auto& point : face_vertices) {
			float projection{ glm::dot(point, normal) };
			if (projection > maxProjection) {
				maxProjection = projection;
				maxPoint = point;
			}
		}
	}
	return maxPoint;
}

Utilities::Model::Face Solvers::CollisionSolver::GetSignificantFace(const std::vector<Utilities::Model::Face>& objectFaces, const glm::vec3& requiredVertex, const glm::vec3& normal) {
	float minLength{ std::numeric_limits<float>::max() };
	Utilities::Model::Face bestFace{};
	for (const auto& face : objectFaces) {
		if (std::find(face.vertices.begin(), face.vertices.end(), requiredVertex) == face.vertices.end())
			continue;

		const glm::vec3 crossProduct{ glm::cross(face.normal, normal) };
		if (crossProduct == glm::vec3(0.0f))
			return face;

		float length{ glm::length(crossProduct) };
		if (minLength > length) {
			minLength = length;
			bestFace = face;
		}
	}
	return bestFace;
}

Utilities::Model::Face Solvers::CollisionSolver::ClipFace(Utilities::Model::Face* referenceFace, Utilities::Model::Face* incidentFace,
	const std::vector<Utilities::Model::Face>& referenceFaces)
{
	Utilities::Model::Face result{ *incidentFace };

	for (const auto& clipFace : referenceFaces) {
		if (clipFace.normal == referenceFace->normal)
			continue;

		const glm::vec3 referenceNormal{ -clipFace.normal };
		const glm::vec3 referenceVertex{ clipFace.vertices[0] };

		Utilities::Model::Face clipped{};
		for (uint32_t vertexIterator{}; vertexIterator < result.vertices.size(); ++vertexIterator) {

			const glm::vec3 firstVertex{ result.vertices[vertexIterator] };
			const glm::vec3 secondVertex{ result.vertices[(vertexIterator + 1) % result.vertices.size()] };

			const float firstDistance{ glm::dot(referenceNormal, firstVertex - referenceVertex) };
			const float secondDistance{ glm::dot(referenceNormal, secondVertex - referenceVertex) };

			if (firstDistance >= 0.0f && secondDistance >= 0.0f)
				clipped.vertices.push_back(secondVertex);
			else if (firstDistance >= 0.0f && secondDistance < 0.0f) {
				const glm::vec3 edgeVector{ secondVertex - firstVertex };
				clipped.vertices.push_back(ClipVector(edgeVector, firstVertex, referenceVertex, referenceNormal));
			}
			else if (firstDistance < 0.0f && secondDistance >= 0.0f) {
				const glm::vec3 edgeVector{ firstVertex - secondVertex };
				clipped.vertices.push_back(ClipVector(edgeVector, secondVertex, referenceVertex, referenceNormal));
				clipped.vertices.push_back(secondVertex);
			}
		}
		result = clipped;
	}

	const glm::vec3 referenceNormal{ -referenceFace->normal };
	const glm::vec3 referenceVertex{ referenceFace->vertices[0] };

	Utilities::Model::Face clipped{};
	for (uint32_t vertexIterator{}; vertexIterator < result.vertices.size(); ++vertexIterator) {

		const glm::vec3 firstVertex{ result.vertices[vertexIterator] };
		const glm::vec3 secondVertex{ result.vertices[(vertexIterator + 1) % result.vertices.size()] };

		const float firstDistance{ glm::dot(referenceNormal, firstVertex - referenceVertex) };
		const float secondDistance{ glm::dot(referenceNormal, secondVertex - referenceVertex) };

		if (firstDistance >= 0.0f && secondDistance >= 0.0f)
			clipped.vertices.push_back(secondVertex);
		else if (firstDistance >= 0.0f && secondDistance < 0.0f)
			clipped.vertices.push_back(firstVertex);
		else if (firstDistance < 0.0f && secondDistance >= 0.0f)
			clipped.vertices.push_back(secondVertex);
	}
	result = clipped;

	std::unordered_set<glm::vec3> s(result.vertices.begin(), result.vertices.end());	//
	result.vertices.assign(s.begin(), s.end());											//Only unique elements

	return result;
}

glm::vec3 Solvers::CollisionSolver::ClipVector(const glm::vec3& subjectVector, const glm::vec3& subjectOrigin, const glm::vec3& clipVertex, const glm::vec3& clipNormal) {
	float enumerator{ glm::dot((clipVertex - subjectOrigin), clipNormal) };
	float denominator{ glm::dot(glm::normalize(subjectVector), clipNormal) };

	return subjectOrigin + glm::normalize(subjectVector) * (enumerator / denominator);
}


//Collision Response
void Solvers::CollisionSolver::ResolveCollision(CollisionData collision_data, 
	std::vector<float>& accumulatedImpulses, std::vector<float>& accumulatedFrictions, 
	float deltaTime) 
{
	const float totalInverseMass{ (1.0f / collision_data.manifold.firstObject->GetMass()) + (1.0f / collision_data.manifold.secondObject->GetMass()) };

	for (uint32_t currentManifold{}; currentManifold < collision_data.manifold.vertices.size(); ++currentManifold) {
		const ContactPointData currentData{ GetContactPointData(collision_data, currentManifold) };

		SolveNormalImpulse(collision_data.manifold.firstObject, collision_data.manifold.secondObject, 
			collision_data, currentManifold, currentData,
			deltaTime, 
			totalInverseMass, accumulatedImpulses);
		SolveFrictionImpulse(collision_data.manifold.firstObject, collision_data.manifold.secondObject, 
			collision_data, currentManifold, currentData,
			totalInverseMass, accumulatedImpulses, accumulatedFrictions);
	}
}

const Solvers::CollisionSolver::ContactPointData Solvers::CollisionSolver::GetContactPointData(const CollisionData& collision_data, uint32_t currentManifold) {
	ContactPointData data{};

	data.relativeFirst = collision_data.manifold.vertices[currentManifold] - collision_data.manifold.firstObject->GetCenter();
	data.relativeSecond = collision_data.manifold.vertices[currentManifold] - collision_data.manifold.secondObject->GetCenter();

	data.angVelocityFirst = glm::cross(collision_data.manifold.firstObject->GetAngularVelocity(), data.relativeFirst);
	data.angVelocitySecond = glm::cross(collision_data.manifold.secondObject->GetAngularVelocity(), data.relativeSecond);

	data.fullVelocityFirst = collision_data.manifold.firstObject->GetLinearVelocity() + data.angVelocityFirst;
	data.fullVelocitySecond = collision_data.manifold.secondObject->GetLinearVelocity() + data.angVelocitySecond;
	data.contactVelocity = data.fullVelocitySecond - data.fullVelocityFirst;

	return data;
}

void Solvers::CollisionSolver::SolveNormalImpulse(Objects::PhysicsObject* first, Objects::PhysicsObject* second,
	const CollisionData& collision_data, const uint32_t currentManifold, const ContactPointData& data,
	const float deltaTime,
	const float totalInverseMass, std::vector<float>& accumulatedImpulses)
{
	const float impulseForce{ glm::dot(data.contactVelocity, collision_data.normal) };

	const glm::vec3 inertiaFirst{ glm::cross(first->GetInverseWorldTensor() * glm::cross(data.relativeFirst, collision_data.normal), data.relativeFirst) };
	const glm::vec3 inertiaSecond{ glm::cross(second->GetInverseWorldTensor() * glm::cross(data.relativeSecond, collision_data.normal), data.relativeSecond) };
	const float angularEffect{ glm::dot(inertiaFirst + inertiaSecond, collision_data.normal) };

	const float correction{ (BIAS / deltaTime) * std::max(0.0f, collision_data.distance - SLOP) };
	const float numerator{ -glm::dot(data.contactVelocity, collision_data.normal) + correction };
	const float impulse{ numerator / (totalInverseMass + angularEffect) };

	const float temp{ accumulatedImpulses[currentManifold] };
	accumulatedImpulses[currentManifold] = std::max(accumulatedImpulses[currentManifold] + impulse, 0.0f);
	const float deltaImpulse{ accumulatedImpulses[currentManifold] - temp };

	const glm::vec3 fullImpulse{ collision_data.normal * deltaImpulse };
	first->AppyLinearImpulse(-fullImpulse);
	second->AppyLinearImpulse(fullImpulse);

	first->AppyAngularImpulse(glm::cross(data.relativeFirst, -fullImpulse));
	second->AppyAngularImpulse(glm::cross(data.relativeSecond, fullImpulse));
}

void Solvers::CollisionSolver::SolveFrictionImpulse(Objects::PhysicsObject* first, Objects::PhysicsObject* second,
	const CollisionData& collision_data, const uint32_t currentManifold, const ContactPointData& data,
	const float totalInverseMass, std::vector<float>& accumulatedImpulses, std::vector<float>& accumulatedFrictions)
{
	glm::vec3 tangent{};
	if (glm::dot(data.contactVelocity, collision_data.normal) != 0.0f) {
		glm::vec3 numerator{ data.contactVelocity - collision_data.normal * glm::dot(data.contactVelocity, collision_data.normal) };
		float denominator{ glm::length(numerator) };
		if (denominator != 0.0f)
			tangent = numerator / denominator;
	}
	else
		tangent = glm::vec3(0.0f);

	float coefficientStatic{ 1.0f };
	float coefficientDynamic{ 0.8f };

	//TODO: Fix static friction
	if (glm::dot(data.contactVelocity, tangent) == 0.0f && glm::dot(data.contactVelocity, tangent) <= coefficientStatic * glm::length(accumulatedImpulses[currentManifold])) {
		const float friction{ -glm::dot(data.contactVelocity, tangent) };

		const float tempFriction{ accumulatedFrictions[currentManifold] };
		accumulatedFrictions[currentManifold] = glm::clamp(accumulatedFrictions[currentManifold] + friction,
			-coefficientStatic * accumulatedImpulses[currentManifold],
			coefficientStatic * accumulatedImpulses[currentManifold]);
		const float deltaFriction{ accumulatedFrictions[currentManifold] - tempFriction };
		const glm::vec3 fullFriction{ tangent * deltaFriction };

		first->AppyLinearImpulse(-fullFriction);
		second->AppyLinearImpulse(fullFriction);

		first->AppyAngularImpulse(glm::cross(data.relativeFirst, -fullFriction));
		second->AppyAngularImpulse(glm::cross(data.relativeSecond, fullFriction));
	}
	else {
		const float frictionNumerator{ -glm::dot(data.contactVelocity, tangent) };

		const glm::vec3 frictionInertiaFirst{ glm::cross(first->GetInverseWorldTensor() * glm::cross(data.relativeFirst, tangent), data.relativeFirst) };
		const glm::vec3 frictionInertiaSecond{ glm::cross(second->GetInverseWorldTensor() * glm::cross(data.relativeSecond, tangent), data.relativeSecond) };
		const float frictionAngularEffect{ glm::dot(frictionInertiaFirst + frictionInertiaSecond, tangent) };
		const float frictionDenominator{ frictionAngularEffect + totalInverseMass };

		const float friction{ frictionNumerator / frictionDenominator };

		const float tempFriction{ accumulatedFrictions[currentManifold] };
		accumulatedFrictions[currentManifold] = glm::clamp(accumulatedFrictions[currentManifold] + friction,
			-coefficientDynamic * accumulatedImpulses[currentManifold],
			coefficientDynamic * accumulatedImpulses[currentManifold]);
		const float deltaFriction{ accumulatedFrictions[currentManifold] - tempFriction };
		const glm::vec3 fullFriction{ tangent * deltaFriction };

		first->AppyLinearImpulse(-fullFriction);
		second->AppyLinearImpulse(fullFriction);

		first->AppyAngularImpulse(glm::cross(data.relativeFirst, -fullFriction));
		second->AppyAngularImpulse(glm::cross(data.relativeSecond, fullFriction));
	}
}