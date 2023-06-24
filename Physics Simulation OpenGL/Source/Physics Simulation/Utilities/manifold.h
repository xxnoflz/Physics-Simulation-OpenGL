#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <functional>

#include "../Objects/physics_object.h"

using FrictionImpulse = std::vector<float>;
using Tangentials = std::vector<glm::vec3>;

namespace Utilities {
	class Manifold {
	public:
		struct ManifoldID {
			uint32_t first_id;
			uint32_t second_id;

			friend bool operator==(ManifoldID first, ManifoldID second) {
				return (first.first_id == second.first_id && first.second_id == second.second_id);
			}
		};
		struct Feature {
			int clipped_edge;
			int clipping_face;

			bool operator==(const Feature& other) const {
				return (clipped_edge == other.clipped_edge &&
					clipping_face == other.clipping_face);
			}
		};
		struct ContactPointData {
			glm::vec3 position_relative_first;
			glm::vec3 position_relative_second;

			glm::vec3 velocity_angular_first;
			glm::vec3 velocity_angular_second;

			glm::vec3 velocity_full_first;
			glm::vec3 velocity_full_second;
			glm::vec3 contact_velocity;
		};
		struct Point {
			glm::vec3 position_on_first;
			glm::vec3 position_on_second;

			Feature feature;

			Tangentials tangents = Tangentials(2);

			glm::vec3 local_first;
			glm::vec3 local_second;
			float penetration_distance;
			float accumulated_impulse;
			FrictionImpulse accumulated_friction = FrictionImpulse(2);
			ContactPointData contact_data;

			bool operator==(const Point& other) const {
				return (position_on_first == other.position_on_first &&
					position_on_second == other.position_on_second &&
					feature == other.feature);
			}
		};

		Manifold();

		Manifold(ManifoldID id, Objects::PhysicsObject* first_object, Objects::PhysicsObject* second_object, std::vector<Point> points, const glm::vec3& normal);

		void Update(Manifold& new_manifold);
		void WarmStart(const glm::vec3& normal);

		std::vector<Point>& GetPoints();
		Objects::PhysicsObject* GetFirstObject();
		Objects::PhysicsObject* GetSecondObject();
		ManifoldID GetID() const;
		const ContactPointData& GetContactPointData(uint32_t contact_iterator);

		friend bool operator==(Manifold first, Manifold second) {
			return (first.GetID() == second.GetID()) && (first.m_first_object == second.m_first_object) && (first.m_second_object == second.m_second_object);
		}
	private:
		ManifoldID m_id;

		Objects::PhysicsObject* m_first_object;
		Objects::PhysicsObject* m_second_object;

		std::vector<Point> m_points;

		void InitializeData(const glm::vec3& normal);
	};

}

template <>
struct std::hash<Utilities::Manifold::Feature> {
	std::size_t operator()(const Utilities::Manifold::Feature& k) const {
		using std::hash;

		return ((hash<int>()(k.clipped_edge)
			^ (hash<int>()(k.clipping_face) << 1)) >> 1);
	}
};

template <>
struct std::hash<Utilities::Manifold::Point> {
	std::size_t operator()(const Utilities::Manifold::Point& k) const {
		using std::hash;

		return ((hash<glm::vec3>()(k.position_on_first)
			^ (hash<glm::vec3>()(k.position_on_second) << 1))
			^ (hash<Utilities::Manifold::Feature>()(k.feature) << 1));
	}
};