
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <iostream>
#include <stack>

namespace PT
{

	struct BVHBuildData
	{
		BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst)
		{
		}
		size_t start; ///< start index into the primitive array
		size_t range; ///< range of index into the primitive array
		size_t node;  ///< address to update
	};

	struct SAHBucketData
	{
		BBox bb;		  ///< bbox of all primitives
		size_t num_prims; ///< number of primitives in the bucket
	};

	template <typename Primitive>
	void BVH<Primitive>::build(std::vector<Primitive> &&prims, size_t max_leaf_size)
	{
		// A3T3 - build a bvh

		// Keep these
		nodes.clear();
		// info("----------prim size : %d \n", prims.size());
		primitives = std::move(prims);

		// Construct a BVH from the given vector of primitives and maximum leaf
		// size configuration.
		// TODO
		// info("primitives size %d, max_leaf_size %d", primitives.size(), max_leaf_size);
		// root_idx = build_my_BVH(primitives, max_leaf_size, 0, primitives.size(), 0);
		root_idx = build_my_BVH_new(primitives, primitives.begin(), primitives.end(), max_leaf_size, 0);
		// root_idx = buildRecursive(primitives, 0, (int)primitives.size(), max_leaf_size);
		//  for (int i = 0; i < nodes.size(); i++)
		//  {
		//  	info("%d -> %d, %d", i, nodes[i].l, nodes[i].r);
		//  }
	}

	template <typename Primitive>
	size_t BVH<Primitive>::build_my_BVH(std::vector<Primitive> &prims, size_t max_leaf_size, size_t start, size_t size, int depth)
	{
		// info("max_leaf_size %d, depth %d, start: %d, size: %d \n", max_leaf_size, depth, start, size);
		if (size <= max_leaf_size)
		{
			BBox bbox_leaf;
			for (size_t i = start; i < start + size; i++)
			{
				bbox_leaf.enclose(prims[i].bbox());
			}

			return new_node(bbox_leaf, start, size, 0, 0);
		}

		// Choose the axis to sort by based on depth
		int axis = depth % 3;

		std::sort(prims.begin() + start, prims.begin() + start + size, [axis](const Primitive &a, const Primitive &b)
				  { return a.bbox().center()[axis] < b.bbox().center()[axis]; });

		// for (const auto &p : prims)
		// {
		// 	info("prims center: %f, %f, %f", p.bbox().center()[0], p.bbox().center()[1], p.bbox().center()[2]);
		// }

		// Find the splitting point
		size_t mid = start;
		float cost, min_cost = FLT_MAX;
		BBox a, b;

		for (size_t i = start; i < start + size; i++)
		{
			a = prims[start].bbox();
			for (size_t j = start; j < i; j++)
			{
				a.enclose(prims[j].bbox());
			}

			b = prims[i].bbox();
			for (size_t j = i; j < start + size; j++)
			{
				b.enclose(prims[j].bbox());
			}
			cost = (i - start) * a.surface_area() + (start + size - i) * b.surface_area();
			// info("i: %d, cost: %f", i, cost);
			if (cost < min_cost)
			{
				min_cost = cost;
				mid = i;
			}
		}
		// info("a: %f, %f, %f, %f, %f, %f ", a.min[0], a.min[1], a.min[2], a.max[0], a.max[1], a.max[2]);
		// info("b: %f, %f, %f, %f, %f, %f ", b.min[0], b.min[1], b.min[2], b.max[0], b.max[1], b.max[2]);
		a.enclose(b);
		// info("a_new: %f, %f, %f, %f, %f, %f ", a.min[0], a.min[1], a.min[2], a.max[0], a.max[1], a.max[2]);

		size_t left_node_index = build_my_BVH(prims, max_leaf_size, start, mid, depth + 1);
		size_t right_node_index = build_my_BVH(prims, max_leaf_size, start + mid, start + size - mid, depth + 1);
		info("-------size %d, left %d, right %d", nodes.size(), left_node_index, right_node_index);
		return new_node(a, start, size, left_node_index, right_node_index);
	}

	template <typename Primitive>
	size_t BVH<Primitive>::build_my_BVH_new(
		std::vector<Primitive> &prims,
		typename std::vector<Primitive>::iterator startIter,
		typename std::vector<Primitive>::iterator endIter,
		size_t max_leaf_size,
		int depth)
	{
		size_t size = std::distance(startIter, endIter);
		if (size <= max_leaf_size)
		{
			BBox bbox_leaf;
			for (auto it = startIter; it != endIter; it++)
			{
				bbox_leaf.enclose(it->bbox());
			}
			size_t new_node_id = new_node(bbox_leaf, std::distance(prims.begin(), startIter), size, 0, 0);
			// info("new_node_id %d, size: %d", new_node_id, size);
			return new_node_id;
		}

		size_t bin_size = size;
		size_t binCount = std::min(10, (int)bin_size);
		std::vector<Bin> bins;
		bins.resize(binCount);

		int axis = depth % 3;
		std::sort(startIter, endIter, [axis](const Primitive &a, const Primitive &b)
				  { return a.bbox().center()[axis] < b.bbox().center()[axis]; });

		// Calculate bin intervals (min, max) along the chosen axis
		float minVal = std::numeric_limits<float>::max();
		float maxVal = std::numeric_limits<float>::lowest();
		for (auto it = startIter; it != endIter; ++it)
		{
			float val = it->bbox().center()[axis];
			minVal = std::min(minVal, val);
			maxVal = std::max(maxVal, val);
		}

		float binInterval = (maxVal - minVal) / binCount;
		// Assign primitives to bins
		for (auto it = startIter; it != endIter; ++it)
		{
			size_t binIndex = std::min(static_cast<size_t>((it->bbox().center()[axis] - minVal) / binInterval), binCount - 1);
			bins[binIndex].add(it);
		}
		for (auto b : bins)
		{
			if (b.primitiveIterators.size() == size) // need to change axis
			{
				// info("change another axis: next_depth %d", depth + 1);
				return build_my_BVH_new(prims, startIter, endIter, max_leaf_size, depth + 1);
			}
		}

		float bestCost = std::numeric_limits<float>::max();
		size_t bestSize = 0;
		size_t leftSize = 0, rightSize = 0;
		BBox parent;
		for (size_t i = 1; i < binCount; ++i)
		{
			BBox bboxLeft, bboxRight;
			leftSize = 0, rightSize = 0;
			for (size_t j = 0; j < i; ++j)
			{
				bboxLeft.enclose(bins[j].bbox);
				leftSize += bins[j].primitiveIterators.size();
			}
			for (size_t j = i; j < binCount; ++j)
			{
				bboxRight.enclose(bins[j].bbox);
				rightSize += bins[j].primitiveIterators.size();
			}

			float cost = leftSize * bboxLeft.surface_area() + rightSize * bboxRight.surface_area();
			if (cost < bestCost)
			{
				// info("depth %d, cost %f, leftsize %d, rightsize %d, bestSplit %d", depth, cost, leftSize, rightSize, bestSplit);
				bestCost = cost;
				// bestSplit = i;
				bestSize = leftSize;
			}
			parent.enclose(bboxLeft);
			parent.enclose(bboxRight);
		}

		auto midIter = startIter;
		// info("advance %d ", bestSize);
		std::advance(midIter, bestSize);
		auto it_begin = primitives.begin();

		// info("\n-----start_index: %d, mid_index: %d, end_index: %d",
		//	 std::distance(it_begin, startIter), std::distance(it_begin, midIter), std::distance(it_begin, endIter));

		size_t left_node_index = build_my_BVH_new(prims, startIter, midIter, max_leaf_size, depth + 1);
		size_t right_node_index = build_my_BVH_new(prims, midIter, endIter, max_leaf_size, depth + 1);

		size_t node_index = new_node(parent, std::distance(it_begin, startIter), size, left_node_index, right_node_index);
		// info("%d -> L: %d, R: %d", node_index, left_node_index, right_node_index);
		return node_index;
	}

	template <typename Primitive>
	Trace BVH<Primitive>::hit(const Ray &ray) const
	{
		// A3T3 - traverse your BVH

		// Implement ray - BVH intersection test. A ray intersects
		// with a BVH aggregate if and only if it intersects a primitive in
		// the BVH that is not an aggregate.

		// The starter code simply iterates through all the primitives.
		// Again, remember you can use hit() on any Primitive value.

		// TODO: replace this code with a more efficient traversal:
		HitInfo closest;
		closest.t = FLT_MAX;
		// std::cout << "nodes.size()" << nodes.size() << "\n";

		Trace ret;
		if (nodes.empty())
		{
			return ret;
		}

		find_closest_hit(ray, nodes[nodes.size() - 1], closest);
		// for (const Primitive &prim : primitives)
		{
			// info("---closest_prim_id %d", std::distance(primitives.begin(), closest.prim_it));
			// if (abs(std::distance(primitives.begin(), closest.prim_it)) > abs(std::distance(primitives.begin(), primitives.end())))
			if (closest.prim_it == primitives.end() || abs(std::distance(primitives.begin(), closest.prim_it)) > abs(std::distance(primitives.begin(), primitives.end())))
			{
				return ret;
			}

			Trace hit = closest.prim_it->hit(ray);
			ret = Trace::min(ret, hit);
		}
		return ret;
	}

	template <typename Primitive>
	void BVH<Primitive>::find_closest_hit(const Ray &ray, const Node &node, HitInfo &closest) const
	{
		if (node.is_leaf())
		{
			typename std::vector<Primitive>::const_iterator startIter = primitives.begin() + node.start;
			typename std::vector<Primitive>::const_iterator endIter = primitives.begin() + node.start + node.size;
			for (auto it = startIter; it != endIter; it++)
			{
				HitInfo hit_info;
				intersect(ray, it, hit_info);

				if (hit_info.prim_it != primitives.end() && hit_info.t < closest.t)
				{
					// info("new_closest.t = %f", hit_info.t);
					closest.prim_it = it;
					closest.t = hit_info.t;
				}
			}
		}
		else
		{
			HitInfo hit1, hit2;
			Vec2 hit_bbox_time1(-FLT_MAX, FLT_MAX), hit_bbox_time2(-FLT_MAX, FLT_MAX);
			bool is_hit1 = intersect(ray, nodes[node.l].bbox, hit1);
			bool is_hit2 = intersect(ray, nodes[node.r].bbox, hit2);

			if (!is_hit1 && !is_hit2)
			{
				// closest.prim_it = primitives.end();
				return;
			}
			else if (is_hit1 ^ is_hit2)
			{
				if (is_hit1)
				{
					find_closest_hit(ray, nodes[node.l], closest);
					return;
				}
				else
				{
					find_closest_hit(ray, nodes[node.r], closest);
					return;
				}
			}

			const Node &first = (hit1.t <= hit2.t) ? nodes[node.l] : nodes[node.r];
			const Node &second = (hit1.t <= hit2.t) ? nodes[node.r] : nodes[node.l];
			HitInfo secondHit = (hit1.t <= hit2.t) ? hit2 : hit1;

			find_closest_hit(ray, first, closest);
			if (secondHit.t < closest.t)
			{
				find_closest_hit(ray, second, closest);
			}
		}
	}

	template <typename Primitive>
	void BVH<Primitive>::intersect(const Ray &ray, typename std::vector<Primitive>::const_iterator it, HitInfo &hit_info) const
	{
		Trace hit_trace = it->hit(ray);
		if (!hit_trace.hit)
		{
			hit_info.prim_it = primitives.end();
			hit_info.t = FLT_MAX;
			return;
		}
		hit_info.prim_it = it;
		hit_info.t = hit_trace.distance;
		return;
	}

	template <typename Primitive>
	bool BVH<Primitive>::intersect(const Ray &ray, const BBox &bbox, HitInfo &hit) const
	{
		Vec2 times(-FLT_MAX, FLT_MAX);

		bool is_hit = bbox.hit(ray, times);
		if (is_hit)
		{
			hit.t = times.x;
			return true;
		}

		hit.prim_it = primitives.end();
		hit.t = FLT_MAX;
		return false;
	}

	template <typename Primitive>
	BVH<Primitive>::BVH(std::vector<Primitive> &&prims, size_t max_leaf_size)
	{
		build(std::move(prims), max_leaf_size);
	}

	template <typename Primitive>
	std::vector<Primitive> BVH<Primitive>::destructure()
	{
		nodes.clear();
		return std::move(primitives);
	}

	template <typename Primitive>
	template <typename P>
	typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const
	{
		BVH<Primitive> ret;
		ret.nodes = nodes;
		ret.primitives = primitives;
		ret.root_idx = root_idx;
		return ret;
	}

	template <typename Primitive>
	Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const
	{
		if (primitives.empty())
			return {};
		int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
		return primitives[n].sample(rng, from);
	}

	template <typename Primitive>
	float BVH<Primitive>::pdf(Ray ray, const Mat4 &T, const Mat4 &iT) const
	{
		if (primitives.empty())
			return 0.0f;
		float ret = 0.0f;
		for (auto &prim : primitives)
			ret += prim.pdf(ray, T, iT);
		return ret / primitives.size();
	}

	template <typename Primitive>
	void BVH<Primitive>::clear()
	{
		nodes.clear();
		primitives.clear();
	}

	template <typename Primitive>
	bool BVH<Primitive>::Node::is_leaf() const
	{
		// A node is a leaf if l == r, since all interior nodes must have distinct children
		return l == r;
	}

	template <typename Primitive>
	size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r)
	{
		Node n;
		n.bbox = box;
		n.start = start;
		n.size = size;
		n.l = l;
		n.r = r;
		nodes.push_back(n);
		return nodes.size() - 1;
	}

	template <typename Primitive>
	BBox BVH<Primitive>::bbox() const
	{
		if (nodes.empty())
			return BBox{Vec3{0.0f}, Vec3{0.0f}};
		return nodes[root_idx].bbox;
	}

	template <typename Primitive>
	size_t BVH<Primitive>::n_primitives() const
	{
		return primitives.size();
	}

	template <typename Primitive>
	uint32_t BVH<Primitive>::visualize(GL::Lines &lines, GL::Lines &active, uint32_t level,
									   const Mat4 &trans) const
	{

		std::stack<std::pair<size_t, uint32_t>> tstack;
		tstack.push({root_idx, 0u});
		uint32_t max_level = 0u;

		if (nodes.empty())
			return max_level;

		while (!tstack.empty())
		{

			auto [idx, lvl] = tstack.top();
			max_level = std::max(max_level, lvl);
			const Node &node = nodes[idx];
			tstack.pop();

			Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
			GL::Lines &add = lvl == level ? active : lines;

			BBox box = node.bbox;
			box.transform(trans);
			Vec3 min = box.min, max = box.max;

			auto edge = [&](Vec3 a, Vec3 b)
			{ add.add(a, b, color); };

			edge(min, Vec3{max.x, min.y, min.z});
			edge(min, Vec3{min.x, max.y, min.z});
			edge(min, Vec3{min.x, min.y, max.z});
			edge(max, Vec3{min.x, max.y, max.z});
			edge(max, Vec3{max.x, min.y, max.z});
			edge(max, Vec3{max.x, max.y, min.z});
			edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
			edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
			edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
			edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
			edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
			edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

			if (!node.is_leaf())
			{
				tstack.push({node.l, lvl + 1});
				tstack.push({node.r, lvl + 1});
			}
			else
			{
				for (size_t i = node.start; i < node.start + node.size; i++)
				{
					uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
					max_level = std::max(c + lvl, max_level);
				}
			}
		}
		return max_level;
	}

	template class BVH<Triangle>;
	template class BVH<Instance>;
	template class BVH<Aggregate>;
	template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
