
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

	The local operations all return a std::optional<T> type. This is used so that your
	implementation can signify that it cannot perform an operation (i.e., because
	the resulting mesh does not have a valid representation).

	An optional can have two values: std::nullopt, or a value of the type it is
	parameterized on. In this way, it's similar to a pointer, but has two advantages:
	the value it holds need not be allocated elsewhere, and it provides an API that
	forces the user to check if it is null before using the value.

	In your implementation, if you have successfully performed the operation, you can
	simply return the required reference:

			... collapse the edge ...
			return collapsed_vertex_ref;

	And if you wish to deny the operation, you can return the null optional:

			return std::nullopt;

	Note that the stubs below all reject their duties by returning the null optional.
*/

/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius)
{
	// faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3)
		return std::nullopt;

	std::vector<VertexRef> face_vertices;
	// In order to make the first edge point in the +x direction, first vertex should
	//  be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s)
	{
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	// assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false);	   // the face to return
	FaceRef boundary = emplace_face(true); // the boundary loop around the face

	std::vector<HalfedgeRef> face_halfedges; // will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s)
	{
		// will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s + 1) % sides];

		// h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		// t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		// e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); // false: non-sharp

		// set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f, -1.0f);

		// thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0)
			face->halfedge = h;
		if (s + 1 == sides)
			boundary->halfedge = t;

		// halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s)
	{
		face_halfedges[s]->next = face_halfedges[(s + 1) % sides];
		face_halfedges[(s + 1) % sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}

/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e)
{
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); // set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; // copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); // set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); // set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
	FaceRef f_not_used = emplace_face();
	HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); // unchanged

	// n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;

	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e);	 // unchanged
	// h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	// t->face unchanged

	// Phase 4: Delete unused elements
	erase_face(f_not_used);
	erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}

/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e)
{
	// A2L2 (REQUIRED): split_edge
	// std::cout << my_describe_R();

	VertexRef vn_ref = emplace_vertex(); // add a new vertex
	vn_ref->position = e->center();
	HalfedgeRef h = e->halfedge, t = e->halfedge->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;
	interpolate_data({v1, v2}, vn_ref); // set bone_weights
	// info("origin e:length: %f", e->length());
	if (!e->on_boundary())
	{
		// change face->halfedge to prevent error
		h->face->halfedge = h, t->face->halfedge = t;

		// totally 1 vertex, 2 faces, 3 edges, 6 halfedges are added
		EdgeRef e_pi_ref = emplace_edge();	 // add a split edge
		EdgeRef h_edge_ref = emplace_edge(); // add an edge in face with halfedge h
		EdgeRef t_edge_ref = emplace_edge(); // add an edge in face with halfedge t

		// add two faces
		FaceRef f_h_pi = emplace_face();
		FaceRef f_t_pi = emplace_face();

		// add two halfedges in line e
		HalfedgeRef h_pi = emplace_halfedge(), t_pi = emplace_halfedge();

		// add four other halfedges
		HalfedgeRef post_h = emplace_halfedge(), pre_h_pi = emplace_halfedge();
		HalfedgeRef post_t = emplace_halfedge(), pre_t_pi = emplace_halfedge();

		// before update, store original data
		HalfedgeRef ori_h_next = h->next, ori_h_next_2 = h->next->next;
		HalfedgeRef ori_t_next = t->next, ori_t_next_2 = t->next->next;
		VertexRef ori_ver_h_next_2 = h->next->next->vertex,
				  ori_ver_t_next_2 = t->next->next->vertex;

		// set up v, f, e ->halfedge
		vn_ref->halfedge = h_pi;
		f_h_pi->halfedge = h_pi;
		f_t_pi->halfedge = t_pi;
		e->halfedge = h;
		e_pi_ref->halfedge = h_pi;
		h_edge_ref->halfedge = post_h;
		t_edge_ref->halfedge = post_t;

		// it is time to setup new halfedge
		h->set_tnvef(t_pi, post_h, h->vertex, h->edge, h->face);
		h_pi->set_tnvef(t, ori_h_next, vn_ref, e_pi_ref, f_h_pi);
		interpolate_data({h, t}, h_pi);

		t->set_tnvef(h_pi, post_t, t->vertex, e_pi_ref, t->face);
		t_pi->set_tnvef(h, ori_t_next, vn_ref, h->edge, f_t_pi);
		interpolate_data({h, t}, t_pi);

		post_h->set_tnvef(pre_h_pi, ori_h_next_2, vn_ref, h_edge_ref, h->face);
		interpolate_data({h_pi}, post_h);

		pre_h_pi->set_tnvef(post_h, h_pi, ori_ver_h_next_2, h_edge_ref, f_h_pi);
		interpolate_data({ori_h_next->twin}, pre_h_pi);

		post_t->set_tnvef(pre_t_pi, ori_t_next_2, vn_ref, t_edge_ref, t->face);
		interpolate_data({t_pi}, post_t);

		pre_t_pi->set_tnvef(post_t, t_pi, ori_ver_t_next_2, t_edge_ref, f_t_pi);
		interpolate_data({ori_t_next->twin}, pre_t_pi);

		// change two more half edge
		ori_h_next->next = pre_h_pi;
		ori_h_next->face = f_h_pi;
		ori_t_next->next = pre_t_pi;
		ori_t_next->face = f_t_pi;
		// info("e_length: %f, e_pi_lenght: %f", e->length(), e_pi_ref->length());
		//  std::cout << my_describe_R();
		return vn_ref;
	}
	// std::cout << my_describe_R();
	//  let h be the inner halfedge of e
	if (h->face->boundary)
	{
		h = h->twin;
		t = t->twin;
	}
	// change face->halfedge to prevent error
	h->face->halfedge = h;

	// 1 vertex, 1 face, 2 edge, 4 halfedges added
	EdgeRef e_pi_ref = emplace_edge();	 // add a split edge
	EdgeRef h_edge_ref = emplace_edge(); // add an edge in face with halfedge h

	// add one face
	FaceRef f_h_pi = emplace_face();

	// add two halfedges in line e
	HalfedgeRef h_pi = emplace_halfedge(), t_pi = emplace_halfedge();
	// add two halfedges in new line
	HalfedgeRef post_h = emplace_halfedge(), pre_h_pi = emplace_halfedge();

	// before update, store original data
	HalfedgeRef ori_h_next = h->next, ori_h_next_2 = h->next->next, ori_t_next = t->next;
	VertexRef ori_ver_h_next_2 = h->next->next->vertex;

	// set up v, f, e ->halfedge
	vn_ref->halfedge = h_pi;
	f_h_pi->halfedge = h_pi;
	e->halfedge = h;
	e_pi_ref->halfedge = h_pi;
	h_edge_ref->halfedge = post_h;

	// it is time to setup new halfedge
	h->set_tnvef(t_pi, post_h, h->vertex, h->edge, h->face);
	h_pi->set_tnvef(t, ori_h_next, vn_ref, e_pi_ref, f_h_pi);
	interpolate_data({h, t}, h_pi);

	t->set_tnvef(h_pi, t_pi, t->vertex, e_pi_ref, t->face);
	t_pi->set_tnvef(h, ori_t_next, vn_ref, h->edge, t->face);
	interpolate_data({h, t}, t_pi);

	post_h->set_tnvef(pre_h_pi, ori_h_next_2, vn_ref, h_edge_ref, h->face);
	interpolate_data({h_pi}, post_h);

	pre_h_pi->set_tnvef(post_h, h_pi, ori_ver_h_next_2, h_edge_ref, f_h_pi);
	interpolate_data({ori_h_next->twin}, pre_h_pi);

	// change extra half edge
	ori_h_next->next = pre_h_pi;
	ori_h_next->face = f_h_pi;

	// std::cout << my_describe_R();
	return vn_ref;
}

/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f)
{
	// A2Lx4 (OPTIONAL): inset vertex
	if (f->boundary) // avoid edge cross
		return std::nullopt;
	// std::cout << my_describe_R();
	//  0. set up some variables
	std::vector<FaceRef> f_new;
	std::vector<EdgeRef> e_ridge_list;
	std::vector<VertexRef> v_ref;
	std::vector<VertexCRef> v_ref_const;
	std::vector<HalfedgeRef> pre_h_list, post_h_list;
	std::vector<HalfedgeCRef> h_ref_const;
	HalfedgeRef h = f->halfedge, origin_h_next;
	int degree = f->degree();
	do
	{
		h_ref_const.emplace_back(h);
		v_ref.emplace_back(h->vertex);
		v_ref_const.emplace_back(h->vertex);
		h = h->next;
	} while (h != f->halfedge);

	// 1. create the center pt of face f as refernce pt, create a halfedge to store uv and normal.
	VertexRef vc = emplace_vertex();
	vc->position = f->center();
	interpolate_data(v_ref_const, vc);
	HalfedgeRef hc = emplace_halfedge();
	hc->vertex = vc;
	interpolate_data(h_ref_const, hc);

	// 2. create f->degree() number of vertices for extrude face(omitted)
	// 3. add 1 edges connected with vi, 1 beval face, 2 halfedges, and setup elements except halfedges
	auto add_element = [&](HalfedgeRef h)
	{
		FaceRef fi = emplace_face();
		fi->halfedge = h;
		f_new.emplace_back(fi);

		HalfedgeRef pre_h = emplace_halfedge(), post_h = emplace_halfedge();
		pre_h_list.emplace_back(pre_h);
		post_h_list.emplace_back(post_h);

		EdgeRef e_ridge = emplace_edge();
		e_ridge->halfedge = pre_h;
		e_ridge_list.emplace_back(e_ridge);

		vc->halfedge = pre_h;
	};

	// 4. setup halfedges
	auto setup_new_halfedges = [&](HalfedgeRef h)
	{
		for (int i = 0; i < degree; i++)
		{
			pre_h_list[i]->set_tnvef(post_h_list[(i - 1 + degree) % degree], h, vc, e_ridge_list[i], f_new[i]);
			post_h_list[i]->set_tnvef(pre_h_list[(i + 1) % degree], pre_h_list[i], v_ref[(i + 1) % degree], e_ridge_list[(i + 1) % degree], f_new[i]);
			origin_h_next = h->next;
			h->next = post_h_list[i];
			h->face = f_new[i];
			// interpolate_data
			interpolate_data({hc}, pre_h_list[i]);
			interpolate_data({origin_h_next}, post_h_list[i]);
			h = origin_h_next;
		}
	};
	// 5. apply add_element and setup_new_halfedges
	h = f->halfedge;
	for (int i = 0; i < degree; i++)
	{
		add_element(h);
		h = h->next;
	}
	setup_new_halfedges(h);
	// std::cout << my_describe_R();
	// 6. delete assistant halfedge and face f
	// erase_vertex(vc);
	erase_halfedge(hc);
	erase_face(f);
	// std::cout << my_describe_R();
	// info("face %d", f_in->id);
	return vc;
}

/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v)
{
	// A2Lx5 (OPTIONAL): Bevel Vertex
	//  Reminder: This function does not update the vertex positions.
	//  Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
	return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e)
{
	// A2Lx6 (OPTIONAL): Bevel Edge
	//  Reminder: This function does not update the vertex positions.
	//  remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
	return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f)
{
	// A2L4: Extrude Face
	//  Reminder: This function does not update the vertex positions.
	//  Remember to also fill in extrude_helper (A2L4h)
	if (f->boundary) // avoid edge cross
		return std::nullopt;

	// 0. set up some variables
	std::vector<FaceRef> f_new;
	std::vector<EdgeRef> e_pi_list, e_ridge_list;
	std::vector<VertexRef> v_new, v_ref;
	std::vector<VertexCRef> v_ref_const;
	std::vector<HalfedgeRef> pre_h_list, post_h_list, h_pi_list, t_pi_list;
	std::vector<HalfedgeCRef> h_ref;
	HalfedgeRef h = f->halfedge, origin_h_next;
	int degree = f->degree();
	do
	{
		// std::cout << "h id: " << h->id << ", normal: " << h->corner_normal << " , uv: " << h->corner_uv <<"\n";
		h_ref.emplace_back(h);
		v_ref.emplace_back(h->vertex);
		v_ref_const.emplace_back(h->vertex);
		h = h->next;
	} while (h != f->halfedge);

	// 1. create the center pt of face f as refernce pt, create a halfedge to store uv and normal. create the new inner face.
	VertexRef vc = emplace_vertex();
	vc->position = f->center();
	interpolate_data(v_ref_const, vc);
	HalfedgeRef hc = emplace_halfedge();
	hc->vertex = vc;
	interpolate_data(h_ref, hc);
	// FaceRef f_in = emplace_face();

	// 2. create f->degree() number of vertices for extrude face
	do
	{
		VertexRef v_i = emplace_vertex();
		// v_i->position = (vc->position + h->vertex->position) / 2.0f;
		v_i->position = h->vertex->position;
		interpolate_data({h->vertex}, v_i);
		v_new.emplace_back(v_i);
		v_ref.emplace_back(h->vertex);
		h = h->next;
	} while (h != f->halfedge);

	// 3. add 2 edges connected with vi, 1 beval face, 4 halfedges, and setup elements except halfedges
	auto add_element = [&](HalfedgeRef h, VertexRef vi)
	{
		FaceRef fi = emplace_face();
		fi->halfedge = h;
		f_new.emplace_back(fi);

		HalfedgeRef pre_h = emplace_halfedge(), post_h = emplace_halfedge(),
					h_pi = emplace_halfedge(), t_pi = emplace_halfedge();
		pre_h_list.emplace_back(pre_h);
		post_h_list.emplace_back(post_h);
		interpolate_data({h, hc}, pre_h);
		interpolate_data({h->next}, post_h);
		h_pi_list.emplace_back(h_pi);
		t_pi_list.emplace_back(t_pi);

		EdgeRef e_pi = emplace_edge(), e_ridge = emplace_edge();
		e_pi->halfedge = h_pi;
		e_pi_list.emplace_back(e_pi);
		e_ridge->halfedge = pre_h;
		e_ridge_list.emplace_back(e_ridge);

		vi->halfedge = h_pi;
		f->halfedge = h_pi;
	};

	// 4. setup halfedges
	auto setup_new_halfedges = [&](HalfedgeRef h)
	{
		for (int i = 0; i < degree; i++)
		{
			// info("i:%d, h_pi:%d, twin:%d, next:%d, vertex:%d, edge:%d, face:%d", i, h_pi_list[i]->id, t_pi_list[i]->id, h_pi_list[(i+1)%degree]->id, v_new[i]->id, e_pi_list[i]->id, f->id);
			h_pi_list[i]->set_tnvef(t_pi_list[i], h_pi_list[(i + 1) % degree], v_new[i], e_pi_list[i], f);
			// info("i:%d, t_pi:%d, twin:%d, next:%d, vertex:%d, edge:%d, face:%d", i, t_pi_list[i]->id, h_pi_list[i]->id, pre_h_list[i]->id, v_new[(i+1)%degree]->id, e_pi_list[i]->id, f_new[i]->id);
			t_pi_list[i]->set_tnvef(h_pi_list[i], pre_h_list[i], v_new[(i + 1) % degree], e_pi_list[i], f_new[i]);
			// info("i:%d, pre_h_list:%d, twin:%d, next:%d, vertex:%d, edge:%d, face:%d", i, pre_h_list[i]->id, post_h_list[(i-1+degree)%degree]->id, h->id, v_new[i]->id, e_ridge_list[i]->id, f_new[i]->id);
			pre_h_list[i]->set_tnvef(post_h_list[(i - 1 + degree) % degree], h, v_new[i], e_ridge_list[i], f_new[i]);
			// info("i:%d, pre_h_list:%d, twin:%d, next:%d, vertex:%d, edge:%d, face:%d", i, post_h_list[i]->id, pre_h_list[(i+1)%degree]->id, t_pi_list[i]->id, v_ref[(i+1)%degree]->id, e_ridge_list[(i+1)%degree]->id, f_new[i]->id);
			post_h_list[i]->set_tnvef(pre_h_list[(i + 1) % degree], t_pi_list[i], v_ref[(i + 1) % degree], e_ridge_list[(i + 1) % degree], f_new[i]);
			origin_h_next = h->next;
			h->next = post_h_list[i];
			h->face = f_new[i];
			// interpolate_data
			interpolate_data({hc, h}, h_pi_list[i]);
			interpolate_data({h_pi_list[i]}, pre_h_list[i]);
			interpolate_data({h}, post_h_list[(i - 1 + degree) % degree]);
			interpolate_data({h_pi_list[i]}, t_pi_list[(i - 1 + degree) % degree]);
			h = origin_h_next;
			// info("origin_h %d", h->id);
		}
	};
	// 5. apply add_element and setup_new_halfedges
	h = f->halfedge;
	for (int i = 0; i < degree; i++)
	{
		add_element(h, v_new[i]);
		h = h->next;
	}
	// info("h: %d", h->id);
	setup_new_halfedges(h);
	// std::cout << my_describe_R();
	// 6. delete assistant vertex and halfedge and face f
	erase_vertex(vc);
	erase_halfedge(hc);
	// std::cout << my_describe_R();
	// info("face %d", f_in->id);
	return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e)
{
	// A2L1: Flip Edge
	if (e->on_boundary())
		return std::nullopt;
	// the edge can be filped only two faces are coplane(except triangle case)
	float dotproduct = dot(e->halfedge->face->normal(), e->halfedge->twin->face->normal());
	if (std::abs(dotproduct - 1) > 0.001f && e->halfedge->face->degree() != 3 && e->halfedge->twin->face->degree() != 3)
	{
		// std::cout << e->halfedge->face->normal() << "; " << e->halfedge->twin->face->normal() << "\n";
		return std::nullopt;
	}

	HalfedgeRef h = e->halfedge;
	VertexRef tarvec = h->next->next->vertex;
	VertexRef srcvec = h->twin->next->next->vertex;

	// change face
	h->face->halfedge = h;
	h->twin->face->halfedge = h->twin;
	h->next->face = h->twin->face;
	h->twin->next->face = h->face;

	// change vertex->halfedge
	h->vertex->halfedge = h->twin->next;
	h->twin->vertex->halfedge = h->next;

	// change halfedge
	HalfedgeRef pre_h = h, pre_twin = h->twin, post_h = h->next, post_twin = h->twin->next;

	while (pre_h->next != h)
	{
		pre_h = pre_h->next; // to get true pre_h
	}
	while (pre_twin->next != h->twin)
	{
		pre_twin = pre_twin->next; // to get true pre_twin
	}

	h->twin->next = h->twin->next->next;
	h->next = h->next->next;

	pre_h->next = post_twin;
	pre_twin->next = post_h;
	post_h->next = h->twin;
	post_twin->next = h;

	// change vertex
	h->vertex = srcvec;
	h->twin->vertex = tarvec;

	// std::cout << my_describe_R();
	return e;
}

/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face)
{
	// A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; // TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v)
{
	// A2Lx1 (OPTIONAL): Dissolve Vertex
	// non-boundary case:
	if (!v->on_boundary())
	{
		std::vector<HalfedgeRef> post_h_list;
		std::vector<VertexRef> post_h_vertex_list;
		int degree = v->degree();
		HalfedgeRef origin_next_vertex_h, tmp_h = v->halfedge;

		auto dissolve_halfedge = [&](HalfedgeRef h)
		{
			post_h_list.emplace_back(h->next);
			post_h_vertex_list.emplace_back(h->next->vertex);
			h->next->vertex->halfedge = h->next;
			erase_face(h->face);
			erase_edge(h->edge);
			erase_halfedge(h->twin);
			erase_halfedge(h);
		};

		auto update_next = [&]()
		{
			for (int i = 0; i < degree; i++)
			{
				post_h_list[i]->next = post_h_list[(i - 1 + degree) % degree];
			}
		};

		for (int i = 0; i < degree; i++)
		{
			if (i < degree - 1)
			{
				origin_next_vertex_h = tmp_h->twin->next;
			}
			dissolve_halfedge(tmp_h);
			if (i < degree - 1)
			{
				tmp_h = origin_next_vertex_h;
			}
		}
		update_next();
		erase_vertex(v);
		// add merged face
		FaceRef f = emplace_face();
		f->halfedge = post_h_list[0];
		for (auto h : post_h_list)
		{
			h->face = f;
		}

		// std::cout << my_describe_R();
		return f;
	}
	// std::cout << my_describe_R();
	return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e)
{
	// A2Lx2 (OPTIONAL): dissolve_edge

	// Reminder: use interpolate_data() to merge corner_uv / corner_normal data

	return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e)
{
	// A2L3: Collapse Edge

	// Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	//  (also works for bone_weights data on vertices!)
	// std::cout << my_describe_R();

	// single triangle case:
	if (edges.size() == 3)
		return std::nullopt;

	// Three pyramid case:
	if (edges.size() == 6 && vertices.size() == 4)
	{
		return std::nullopt;
	}

	// invalid case: e is the only connected edge of either face
	HalfedgeRef h = e->halfedge;

	HalfedgeRef t = h->twin;
	// get connectivity
	int connectivity_edge0 = 0, connectivity_edge1 = 0;
	do
	{
		if (!h->edge->on_boundary())
		{
			connectivity_edge0++;
		}
		h = h->next;
	} while (h != e->halfedge);

	do
	{
		if (!t->edge->on_boundary())
		{
			connectivity_edge1++;
		}
		t = t->next;
	} while (t != e->halfedge->twin);

	if (!e->on_boundary())
	{
		if (connectivity_edge0 == 1 || connectivity_edge1 == 1)
		{
			// std::cout << "1111\n";
			return std::nullopt;
		}
	}

	if (h->face->degree() == 3 && t->face->degree() == 3)
	{
		// isolated triangle case:
		if (connectivity_edge0 == 0 && connectivity_edge1 == 0)
		{
			// std::cout << "0000\n";
			return std::nullopt;
		}
		if (!e->on_boundary() && connectivity_edge0 < 3 && connectivity_edge1 < 3)
		{
			// std::cout << "2222\n";
			return std::nullopt;
		}
	}
	//  std::cout << my_describe_R();

	//   start to collapse edge
	// std::optional<Halfedge_Mesh::VertexRef> v0 = bisect_edge(e);
	// 1. create a new middle pt
	VertexRef vn = emplace_vertex(); // add a new vertex
	vn->position = e->center();
	HalfedgeRef tmp_halfedge;
	VertexRef vh = h->vertex; // will delete later
	VertexRef vt = t->vertex;
	interpolate_data({vh, vt}, vn); // set bone_weights

	// 2. all h->vertex = vh or vt change to vn
	tmp_halfedge = h;
	do
	{
		if (tmp_halfedge->vertex == vh)
		{
			tmp_halfedge->vertex = vn;
		}
		tmp_halfedge = tmp_halfedge->twin->next;
	} while (tmp_halfedge != h);

	tmp_halfedge = t;
	do
	{
		if (tmp_halfedge->vertex == vt)
		{
			tmp_halfedge->vertex = vn;
		}
		tmp_halfedge = tmp_halfedge->twin->next;
	} while (tmp_halfedge != t);

	// define a func to deal with halfside case
	auto update = [&](HalfedgeRef h)
	{
		// define and init variables
		HalfedgeRef post_h = h->next, pre_h = h;
		while (pre_h->next != h)
		{
			pre_h = pre_h->next;
		}

		// 3. no collapse case: i.e. face of pre, post, h is not a triangle
		if (post_h->next != pre_h)
		{
			h->face->halfedge = pre_h;
			pre_h->next = post_h;
			vn->halfedge = post_h;
			return;
		}
		// 4. collapse case: add 1 edge, 2 halfedge; delete 2 edges, 4 halfedges, 1 face
		// define some useful variables
		VertexRef v1 = pre_h->vertex;
		FaceRef f1 = post_h->twin->face, f2 = pre_h->twin->face, f0 = h->face; // f0 will be deleted
		EdgeRef e_post_h = post_h->edge, e_pre_h = pre_h->edge;				   // these two will be deleted
		// create 2 hldgs, 1 edge
		HalfedgeRef h_pi = emplace_halfedge(), t_pi = emplace_halfedge();
		EdgeRef e_pi = emplace_edge();
		interpolate_data({pre_h}, h_pi);
		interpolate_data({h, post_h}, t_pi);

		h_pi->set_tnvef(t_pi, post_h->twin->next, v1, e_pi, f1);
		t_pi->set_tnvef(h_pi, pre_h->twin->next, vn, e_pi, f2);
		// update f1, f2 halfedges that points to e1
		tmp_halfedge = f1->halfedge;
		while (tmp_halfedge->next != post_h->twin)
		{
			tmp_halfedge = tmp_halfedge->next;
		}
		tmp_halfedge->next = h_pi;

		tmp_halfedge = f2->halfedge;
		while (tmp_halfedge->next != pre_h->twin)
		{
			tmp_halfedge = tmp_halfedge->next;
		}
		tmp_halfedge->next = t_pi;
		// update f, e, v ->halfedge
		// if (f1->halfedge == post_h->twin)
		{
			f1->halfedge = h_pi;
		}
		// if (f2->halfedge == pre_h->twin)
		{
			f2->halfedge = t_pi;
		}
		e_pi->halfedge = h_pi;
		v1->halfedge = h_pi;
		vn->halfedge = t_pi;
		// delete elements
		erase_face(f0);
		erase_edge(e_post_h);
		erase_edge(e_pre_h);
		erase_halfedge(post_h->twin);
		erase_halfedge(post_h);
		erase_halfedge(pre_h->twin);
		erase_halfedge(pre_h);
	};

	update(h);
	update(t);

	// delete vh, vt, e, t, h
	erase_vertex(vt);
	erase_vertex(vh);
	erase_edge(e);
	erase_halfedge(t);
	erase_halfedge(h);
	// std::cout << my_describe_R();
	return vn;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f)
{
	// A2Lx3 (OPTIONAL): Collapse Face

	// Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	//  (also works for bone_weights data on vertices!)

	return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2)
{
	// A2Lx8: Weld Edges

	// Reminder: use interpolate_data() to merge bone_weights data on vertices!

	return std::nullopt;
}

/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance)
{
	// A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper

	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink)
{
	// A2L4h: Extrude Positions Helper

	// General strategy:
	//  use mesh navigation to get starting positions from the surrounding faces,
	//  compute the centroid from these positions + use to shrink,
	//  offset by move

	// info("face id:%d", face->id);
	std::vector<VertexRef> v_f;
	HalfedgeRef h = face->halfedge;
	do
	{
		// info("h id:%d", h->id);
		v_f.emplace_back(h->vertex);
		h = h->next;
	} while (h != face->halfedge);
	// add a center pt for convinience
	VertexRef vc = emplace_vertex();
	vc->position = face->center();
	// shrink
	for (auto v : v_f)
	{
		v->position = shrink * vc->position + (1 - shrink) * v->position;
	}
	// offset by move
	for (auto v : v_f)
	{
		v->position += move;
	}
	// delete vc
	erase_vertex(vc);
}
