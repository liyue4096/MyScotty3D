
#pragma once

#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "trace.h"

struct RNG;

namespace PT
{

	template <typename Primitive>
	class BVH
	{
	public:
		class Node
		{
		public:
			BBox bbox;
			size_t start, size, l, r;

			// A node is a leaf if l == r, since all interior nodes must have distinct children
			bool is_leaf() const;
			friend class BVH<Primitive>;
		};

		struct Bin
		{
			BBox bbox;
			// Storing iterators to Primitive objects
			std::vector<typename std::vector<Primitive>::iterator> primitiveIterators;

			void add(typename std::vector<Primitive>::iterator it)
			{
				bbox.enclose(it->bbox());
				primitiveIterators.push_back(it);
			}
		};

		struct HitInfo
		{
			typename std::vector<Primitive>::const_iterator prim_it;
			float t;
			friend class BVH<Primitive>;
		};

		BVH() = default;
		BVH(std::vector<Primitive> &&primitives, size_t max_leaf_size = 1);
		void build(std::vector<Primitive> &&primitives, size_t max_leaf_size = 1);
		size_t build_my_BVH(std::vector<Primitive> &primitives, size_t max_leaf_size, size_t start, size_t size, int depth = 0);
		size_t build_my_BVH_new(
			std::vector<Primitive> &prims,
			typename std::vector<Primitive>::iterator startIter,
			typename std::vector<Primitive>::iterator endIter,
			size_t max_leaf_size,
			int depth);

		BVH(BVH &&src) = default;
		BVH &operator=(BVH &&src) = default;

		BVH(const BVH &src) = delete;
		BVH &operator=(const BVH &src) = delete;

		BBox bbox() const;
		Trace hit(const Ray &ray) const;
		void find_closest_hit(const Ray &ray, const Node &node, HitInfo &closest) const;
		void intersect(const Ray &ray, typename std::vector<Primitive>::const_iterator it, HitInfo &hit) const;
		bool intersect(const Ray &ray, const BBox &bbox, HitInfo &hit) const;

		template <typename P = Primitive>
		typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type copy() const;

		uint32_t visualize(GL::Lines &lines, GL::Lines &active, uint32_t level,
						   const Mat4 &trans) const;
		size_t n_primitives() const;

		std::vector<Primitive> destructure();
		void clear();

		Vec3 sample(RNG &rng, Vec3 from) const;
		float pdf(Ray ray, const Mat4 &T = Mat4::I, const Mat4 &iT = Mat4::I) const;

		std::vector<Primitive> primitives;
		std::vector<Node> nodes;
		size_t root_idx = 0;

	private:
		size_t new_node(BBox box = {}, size_t start = 0, size_t size = 0, size_t l = 0, size_t r = 0);
	};

} // namespace PT
