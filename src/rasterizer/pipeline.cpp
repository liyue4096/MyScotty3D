// clang-format off
//#pragma warning(disable:4189 4244)
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "framebuffer.h"
#include "sample_pattern.h"
template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).

	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};

	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// rasterize primitives:

	std::vector<Fragment> fragments;

	// helper used to put output of rasterization functions into fragments:
	auto emit_fragment = [&](Fragment const& f) { fragments.emplace_back(f); };

	// actually do rasterization:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < clipped_vertices.size(); i += 2) {
			rasterize_line(clipped_vertices[i], clipped_vertices[i + 1], emit_fragment);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < clipped_vertices.size(); i += 3) {
			rasterize_triangle(clipped_vertices[i], clipped_vertices[i + 1], clipped_vertices[i + 2], emit_fragment);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// depth test + shade + blend fragments:
	uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 
							   // (indicates something is wrong with clipping)
	for (auto const& f : fragments) {

		// fragment location (in pixels):
		int32_t x = (int32_t)std::floor(f.fb_position.x);
		int32_t y = (int32_t)std::floor(f.fb_position.y);

		// if clipping is working properly, this condition shouldn't be needed;
		// however, it prevents crashes while you are working on your clipping functions,
		// so we suggest leaving it in place:
		if (x < 0 || (uint32_t)x >= framebuffer.width || 
		    y < 0 || (uint32_t)y >= framebuffer.height) {
			++out_of_range;
			continue;
		}

		// local names that refer to destination sample in framebuffer:
		float& fb_depth = framebuffer.depth_at(x, y, 0);
		Spectrum& fb_color = framebuffer.color_at(x, y, 0);


		// depth test:
		if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
			// "Always" means the depth test always passes.
		} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
			// "Never" means the depth test never passes.
			continue; //discard this fragment
		} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) {
			// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
			// A1T4: Depth_Less
			// TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less".
			if ( f.fb_position.z < fb_depth )
			{
				fb_depth = f.fb_position.z;
			}
			else if (f.fb_position.z > fb_depth)
			{
				continue;
			}				
		} else {
			static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
		}

		// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
		if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
			fb_depth = f.fb_position.z;
		}

		// shade fragment:
		ShadedFragment sf;
		sf.fb_position = f.fb_position;
		Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

		// write color to framebuffer if color writes aren't disabled:
		if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
			// blend fragment:
			if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
				fb_color = sf.color;
			} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
				// A1T4: Blend_Add
				// TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
				fb_color += sf.color * sf.opacity; //<-- replace this line
			} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
				// A1T4: Blend_Over
				// TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
				// 		 You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
				fb_color = sf.color * sf.opacity; //<-- replace this line
			} else {
				static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
			}
		}
	}
	if (out_of_range > 0) {
		if constexpr (primitive_type == PrimitiveType::Lines) {
			warn("Produced %d fragments outside framebuffer; this indicates something is likely "
			     "wrong with the clip_line function.",
			     out_of_range);
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			warn("Produced %d fragments outside framebuffer; this indicates something is likely "
			     "wrong with the clip_triangle function.",
			     out_of_range);
		}
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	emit_vertex(va);
	emit_vertex(vb);
	emit_vertex(vc);
}

// -------------------------------------------------------------------------
// rasterization functions
// Function to check if point q lies on line segment pr
inline bool onSegment(Vec3 p, Vec3 q, Vec3 r) {
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}


// to determine whether A in Diamond with midpoint P
inline bool isInPixelDiamond(Vec3 A, Vec3 P){
	if (std::abs(P.x - A.x) + std::abs(P.y - A.y) > 0.5f)
	{
		return false;
	}
	else if (std::abs(P.x - A.x) + std::abs(P.y - A.y) == 0.5f){
		if (A.x == P.x + 0.5f || A.y == P.y + 0.5f)
		{
			return false;
		}		
	}
	return true;
}

inline bool paintPixel(Vec3 A, Vec3 B, Vec3 P){
	if (isInPixelDiamond(B, P))
	{
		return false;
	}
	else if(!isInPixelDiamond(B, P) && isInPixelDiamond(A, P)){
		return true;
	}
	else{
		if ((A.x < P.x && P.x < B.x) || (A.x > P.x && P.x > B.x) ||
			(A.y < P.y && P.y < B.y) || (A.y > P.y && P.y > B.y))
		{
			return true;
		}
		return false;
	}
}


/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
	ClippedVertex const& va, ClippedVertex const& vb,
	std::function<void(Fragment const&)> const& emit_fragment) {
	if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
		assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
	}
	// A1T2: rasterize_line

	// TODO: Check out the block comment above this function for more information on how to fill in
	// this function!
	// The OpenGL specification section 3.5 may also come in handy.

	{ // As a placeholder, draw a point in the middle of the line:
		//(remove this code once you have a real implementation)
		// Fragment mid;
		// mid.fb_position = (va.fb_position + vb.fb_position) / 2.0f;
		// mid.attributes = va.attributes;
		// mid.derivatives.fill(Vec2(0.0f, 0.0f));
		// emit_fragment(mid);

		float dx = std::abs(va.fb_position.x - vb.fb_position.x), dy = std::abs(va.fb_position.y - vb.fb_position.y);
		int i, j;

		if (dx >= dy)
		{
			/* set index i,j */
			i = 0, j = 1;
		}
		else
		{
			j = 0, i = 1;
		}
		
		ClippedVertex va_true = va, vb_true = vb; 
		if (va.fb_position.data[i] > vb.fb_position.data[i])
		{
			/* the starting coordinate should be the smaller value along the longer axis */
			va_true = vb;
			vb_true = va;
		}
		
		Vec3 DA(floor(va_true.fb_position.data[0]) + 0.5f, floor(va_true.fb_position.data[1]) + 0.5f, 0.f);
		
		if (vb_true.fb_position.data[0] <= DA.data[0]+0.5f && vb_true.fb_position.data[1] <= DA.data[1]+0.5f)
		{
			if (isInPixelDiamond(vb.fb_position, DA))
			{
				return;		
			}
			else{
				if (paintPixel(va_true.fb_position, vb_true.fb_position, DA))
				{
					Fragment frag;
					frag.fb_position.data[0] = DA.data[0];
					frag.fb_position.data[1] = DA.data[1];
					frag.fb_position.data[2] = va_true.fb_position.data[2];
					frag.attributes = va.attributes;
					frag.derivatives.fill(Vec2(0.0f, 0.0f));
					emit_fragment(frag);
				}
				return;
			}
		}		

		int lower_bound = (int)std::floor(va_true.fb_position.data[i]),
			upper_bound = (int)std::floor(vb_true.fb_position.data[i]);
		
		float ratio_w = 0, v = 0;
		Fragment frag;

		for (size_t u = lower_bound; u <= upper_bound; u++)
		{
			 // deal with start and end point 
			if (u == lower_bound)
			{
				Vec3 edge_p;
				edge_p.data[i] = u + 1.f;
				ratio_w = ((float)u + 1.f - va_true.fb_position.data[i]) / (vb_true.fb_position.data[i] - va_true.fb_position.data[i]);
				edge_p.data[j] = ratio_w * (vb_true.fb_position.data[j] - va_true.fb_position.data[j]) + va_true.fb_position.data[j];

				if (isInPixelDiamond(vb.fb_position, DA))
				{
					continue;		
				}
				else if (!paintPixel(va_true.fb_position, edge_p, DA))
				{
					continue;
				}		
			}

			if (u == upper_bound)
			{
				Vec3 edge_p;
				edge_p.data[i] = (float)upper_bound;
				ratio_w = ((float)upper_bound - va_true.fb_position.data[i]) / (vb_true.fb_position.data[i] - va_true.fb_position.data[i]);
				edge_p.data[j] = ratio_w * (vb_true.fb_position.data[j] - va_true.fb_position.data[j]) + va_true.fb_position.data[j];
				if (isInPixelDiamond(vb.fb_position, DA))
				{
					continue;		
				}
				else if (!paintPixel(va_true.fb_position, edge_p, DA))
				{
					continue;
				}		
			}
					
			// general case
			ratio_w = ((float)u + 0.5f - va_true.fb_position.data[i]) / (vb_true.fb_position.data[i] - va_true.fb_position.data[i]);
			v = ratio_w * (vb_true.fb_position.data[j] - va_true.fb_position.data[j]) + va_true.fb_position.data[j];

			frag.fb_position.data[i] = u + 0.5f;
			frag.fb_position.data[j] = std::floor(v) + 0.5f;
			frag.fb_position.data[2] = ratio_w * (vb_true.fb_position.data[2] - va_true.fb_position.data[2]) + va_true.fb_position.data[2];
			frag.attributes = va.attributes;
			frag.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(frag);
		}	
	}

}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
	// NOTE: it is okay to restructure this function to allow these tasks to use the
	//  same code paths. Be aware, however, that all of them need to remain working!
	//  (e.g., if you break Flat while implementing Correct, you won't get points
	//   for Flat.)
	if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
		// A1T3: flat triangles
		// TODO: rasterize triangle (see block comment above this function).

		std::vector<ClippedVertex> v = {va, vb, vc};
		sortTrianglePoint(v);

		std::vector<std::vector<int>> edges = std::vector<std::vector<int>>(3, std::vector<int>(3));
		labelEdge(edges, v);

		std::vector<std::vector<float>> edgeCoef = std::vector<std::vector<float>>(3, std::vector<float>(3));
		setLineCoef(edgeCoef[0], v[0].fb_position, v[1].fb_position);
		setLineCoef(edgeCoef[1], v[0].fb_position, v[2].fb_position);
		setLineCoef(edgeCoef[2], v[1].fb_position, v[2].fb_position);


		float lower_bound, upper_bound, tmp;
		float j = std::floor(v[0].fb_position.y) + 0.5f, i = 0.f;
		std::vector<float> coorWeight(3, 0);

		Fragment frag;
		ClippedVertex tmpVec3;
		//main emit func
		while (j <= std::ceil(v[2].fb_position.y) + 0.5f)
		{
			//out of triangle
			if (j < v[0].fb_position.y)
			{
				j++;	//move to upper col
				continue;
			}
			if (j > v[2].fb_position.y)
			{
				break;	//exit while loop
			}
			
			//special cases: at the end pt
			if (j == v[0].fb_position.y) //bottom point
			{
				j++;	//move to upper col
				continue;
			}
			if (j == v[2].fb_position.y && v[2].fb_position.y != v[1].fb_position.y)
			{
				break;
			}
			else if (j == v[2].fb_position.y && v[2].fb_position.y == v[1].fb_position.y) //on top line
			{
				lower_bound = std::floor(v[1].fb_position.x);
				upper_bound = v[2].fb_position.x;
				i = lower_bound +0.5f;
				while (i < upper_bound )
				{
					if (i >= v[1].fb_position.x)
					{
						tmpVec3.fb_position.x = i, tmpVec3.fb_position.y = j;
						getBaryCoor(coorWeight, tmpVec3, v);

						frag.fb_position.x = i, frag.fb_position.y = j;
						frag.fb_position.z = coorWeight[0]*v[0].fb_position.z + coorWeight[1]*v[1].fb_position.z
											+ coorWeight[2]*v[2].fb_position.z;
						frag.attributes = va.attributes;
						//in flat case frag.derivative is always 0
						emit_fragment(frag);
					}
					i++;
				}
				break;
			}
			
			//general case
			if (j <= v[1].fb_position.y) //check edge v[0]v[1] and v[0]v[2]
			{
				if (edgeCoef[0][0] == 0 || edgeCoef[1][0] == 0) //should never happen
				{
					std::cout << "	error: divided by 0!!!!! \n";
					break;
				}
				
				lower_bound = -( j*edgeCoef[0][1] + edgeCoef[0][2]) / edgeCoef[0][0];
				upper_bound = -( j*edgeCoef[1][1] + edgeCoef[1][2]) / edgeCoef[1][0];
			}
			else	//check edge v[1]v[2] and v[0]v[2]
			{
				if (edgeCoef[2][0] == 0 || edgeCoef[1][0] == 0) //should never happen
				{
					std::cout << "	error: divided by 0!!!!! \n";
					break;
				}
				lower_bound = -( j*edgeCoef[2][1] + edgeCoef[2][2]) / edgeCoef[2][0];
				upper_bound = -( j*edgeCoef[1][1] + edgeCoef[1][2]) / edgeCoef[1][0];
			}

			if (lower_bound > upper_bound)
			{
				tmp = lower_bound;
				lower_bound = upper_bound;
				upper_bound = tmp;
			}
				
			i = std::floor(lower_bound) +0.5f;
			while (i < upper_bound )
			{
				if (i >= lower_bound)
				{
					tmpVec3.fb_position.x = i, tmpVec3.fb_position.y = j;
					getBaryCoor(coorWeight, tmpVec3, v);

					frag.fb_position.x = i, frag.fb_position.y = j;
					frag.fb_position.z = coorWeight[0]*v[0].fb_position.z + coorWeight[1]*v[1].fb_position.z
										+ coorWeight[2]*v[2].fb_position.z;
					frag.attributes = va.attributes;
					//in flat case frag.derivative is always 0
					emit_fragment(frag);
				}
				i++;
			}

			j++;	
		}
		

		// As a placeholder, here's code that draws some lines:
		//(remove this and replace it with a real solution)
		// Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(va, vb, emit_fragment);
		// Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(vb, vc, emit_fragment);
		// Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(vc, va, emit_fragment);
	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
		// A1T5: screen-space smooth triangles
		// TODO: rasterize triangle (see block comment above this function).

		// As a placeholder, here's code that calls the Flat interpolation version of the function:
		//(remove this and replace it with a real solution)
		Pipeline<PrimitiveType::Lines, P, (flags & ~PipelineMask_Interp) | Pipeline_Interp_Flat>::rasterize_triangle(va, vb, vc, emit_fragment);
	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
		// A1T5: perspective correct triangles
		// TODO: rasterize triangle (block comment above this function).

		// As a placeholder, here's code that calls the Screen-space interpolation function:
		//(remove this and replace it with a real solution)
		Pipeline<PrimitiveType::Lines, P, (flags & ~PipelineMask_Interp) | Pipeline_Interp_Smooth>::rasterize_triangle(va, vb, vc, emit_fragment);
	}
}

//bottom - left rule, only consider y, x
template<PrimitiveType p, class P, uint32_t flags>
int Pipeline<p, P, flags>::cmp_tri(ClippedVertex a, ClippedVertex b){
	if (a.fb_position.y == b.fb_position.y)
	{
		return a.fb_position.x < b.fb_position.x;
	}
	return a.fb_position.y < b.fb_position.y;
}

template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::sortTrianglePoint(std::vector<ClippedVertex> &v){
	if(v.size() != 3)
	{
		std::cout << "  sortTrianglePoint error: not a valid triangle!!!!! \n" << std::endl;
		return;
	}
	std::sort(v.begin(), v.begin()+3, cmp_tri);
	//std::cout << v[0].fb_position.x << ", " << v[0].fb_position.y << ", " << v[0].fb_position.z << " ; " 
	//	<< v[1].fb_position.x << ", " << v[1].fb_position.y << ", " << v[1].fb_position.z << std::endl;
}

//1: is top-left edges; 0: others
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>:: labelEdge(std::vector<std::vector<int>> edges, std::vector<ClippedVertex> &v){
	if (edges.size() != 3 || edges[0].size() != 3)
	{
		edges.resize(3);
		edges[0].resize(3);
		edges[1].resize(3);
		edges[2].resize(3);
	}
	
	if (v[1].fb_position.y == v[2].fb_position.y) // top edge
	{
		edges[1][2] = 1;
		edges[2][1] = 1;
	}
	else if (cross(v[1].fb_position-v[0].fb_position, v[2].fb_position-v[0].fb_position).z < 0) //left
	{
		edges[1][2] = 1;
		edges[2][1] = 1;
	}
	else
	{
		edges[1][2] = 0;
		edges[2][1] = 0;
	}

	if (v[1].fb_position.x > v[0].fb_position.x)
	{
		edges[0][2] = 1;
		edges[2][0] = 1;
	}
	else if (v[1].fb_position.x == v[0].fb_position.x && v[2].fb_position.x < v[1].fb_position.x)
	{
		edges[0][2] = 1;
		edges[2][0] = 1;
	}
	else
	{
		edges[0][2] = 0;
		edges[2][0] = 0;
	}
	
	if(v[0].fb_position.y != v[1].fb_position.y && v[1].fb_position.x < v[0].fb_position.x)
	{
		edges[0][1] = 1;
		edges[1][0] = 1;
	}
	else
	{
		edges[0][1] = 0;
		edges[1][0] = 0;
	}

	edges[0][0] = edges[0][1] && edges[0][2];
	edges[1][1] = edges[1][1] && edges[1][2];
	edges[2][2] = edges[0][2] && edges[1][2];
}

template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::getBaryCoor(std::vector<float> &weight , ClippedVertex &p0, std::vector<ClippedVertex> &v){
	if (weight.size() != 3)
	{
		weight.resize(3);
	}
	float crossArea =  v[0].fb_position.x *(v[1].fb_position.y - v[2].fb_position.y) +
					   v[1].fb_position.x *(v[2].fb_position.y - v[0].fb_position.y) +
					   v[2].fb_position.x *(v[0].fb_position.y - v[1].fb_position.y);

	float pArea[3];

	pArea[0] = p0.fb_position.x *(v[1].fb_position.y - v[2].fb_position.y) +
			   v[1].fb_position.x *(v[2].fb_position.y - p0.fb_position.y) +
			   v[2].fb_position.x *(p0.fb_position.y - v[1].fb_position.y);

	pArea[1] = v[0].fb_position.x *(p0.fb_position.y - v[2].fb_position.y) +
				p0.fb_position.x *(v[2].fb_position.y - v[0].fb_position.y) +
				v[2].fb_position.x *(v[0].fb_position.y - p0.fb_position.y);

	pArea[2] = v[0].fb_position.x *(v[1].fb_position.y - p0.fb_position.y) +
				v[1].fb_position.x *(p0.fb_position.y - v[0].fb_position.y) +
				p0.fb_position.x *(v[0].fb_position.y - v[1].fb_position.y);

	for (size_t i = 0; i < 3; i++)
	{
		weight[i] = std::abs(pArea[i] / crossArea);
	}
}

bool isInLine(Vec3 p, Vec3 a, Vec3 b){
	if( p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) &&
		p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y)){
			return true;
		}
	return false;
}

void setLineCoef(std::vector<float> &coef, Vec3 a, Vec3 b){
	if(coef.size() != 3){
		coef.resize(3);
	}
	coef[0] = b.y - a.y;
	coef[1] = a.x - b.x;
	coef[2] = b.x*a.y - a.x*b.y;
}

//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;