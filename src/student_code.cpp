#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

	/**
	 * Evaluates one step of the de Casteljau's algorithm using the given points and
	 * the scalar parameter t (class member).
	 *
	 * @param points A vector of points in 2D
	 * @return A vector containing intermediate points or the final interpolated vector
	 */
	std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const& points)
	{
		// TODO Part 1.
		if (points.size() == 1) return points;
		std::vector<Vector2D> new_points;
		for (int i = 0; i < points.size() - 1; ++i) {
			Vector2D new_point;
			new_point.x = (1 - this->t) * points[i].x + this->t * points[i + 1].x;
			new_point.y = (1 - this->t) * points[i].y + this->t * points[i + 1].y;
			new_points.push_back(new_point);
		}
		return new_points;
	}



	/**
	 * Evaluates one step of the de Casteljau's algorithm using the given points and
	 * the scalar parameter t (function parameter).
	 *
	 * @param points    A vector of points in 3D
	 * @param t         Scalar interpolation parameter
	 * @return A vector containing intermediate points or the final interpolated vector
	 */
	std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const& points, double t) const
	{
		// TODO Part 2.
		if (points.size() == 1) return points;
		std::vector<Vector3D> new_points;
		for (int i = 0; i < points.size() - 1; ++i) {
			Vector3D new_point;
			new_point.x = (1 - t) * points[i].x + t * points[i + 1].x;
			new_point.y = (1 - t) * points[i].y + t * points[i + 1].y;
			new_point.z = (1 - t) * points[i].z + t * points[i + 1].z;
			new_points.push_back(new_point);
		}
		return new_points;
	}

	/**
	 * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
	 *
	 * @param points    A vector of points in 3D
	 * @param t         Scalar interpolation parameter
	 * @return Final interpolated vector
	 */
	Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const& points, double t) const
	{
		// TODO Part 2.
		std::vector<Vector3D> intermediate_points = points;
		while (intermediate_points.size() != 1) {
			intermediate_points = evaluateStep(intermediate_points, t);
		}
		return intermediate_points.front();
	}

	/**
	 * Evaluates the Bezier patch at parameter (u, v)
	 *
	 * @param u         Scalar interpolation parameter
	 * @param v         Scalar interpolation parameter (along the other axis)
	 * @return Final interpolated vector
	 */
	Vector3D BezierPatch::evaluate(double u, double v) const
	{
		// TODO Part 2.
		std::vector<Vector3D> intermediate_points;
		for (int i = 0; i < this->controlPoints.size(); i++) {
			intermediate_points.push_back(evaluate1D(this->controlPoints[i], u));
		}
		Vector3D point = evaluate1D(intermediate_points, v);
		return point;
	}

	Vector3D Vertex::normal(void) const
	{
		// TODO Part 3.
		// Returns an approximate unit normal at this vertex, computed by
		// taking the area-weighted average of the normals of neighboring
		// triangles, then normalizing.
		Vector3D averageNormal(0, 0, 0);

		HalfedgeIter h = _halfedge;
		do
		{
			FaceIter f = h->face();

			double edge1 = h->edge()->length();
			double edge2 = h->next()->edge()->length();

			double area = edge1 * edge2 / 2.0;

			averageNormal += f->normal() * area;

			h = h->twin()->next();
		} while (h != _halfedge);

		averageNormal.normalize();

		return averageNormal;
	}

	EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
	{
		// Check if the edge is a boundary edge
		if (e0->isBoundary()) return e0;

		// Get the halfedges of both triangles
		HalfedgeIter h0 = e0->halfedge();
		HalfedgeIter h1 = h0->next();
		HalfedgeIter h2 = h1->next();

		HalfedgeIter h3 = h0->twin();
		HalfedgeIter h4 = h3->next();
		HalfedgeIter h5 = h4->next();

		// Get the vertices
		VertexIter v0 = h0->vertex();
		VertexIter v1 = h1->vertex();
		VertexIter v2 = h2->vertex();
		VertexIter v3 = h5->vertex();

		// Get the faces
		FaceIter f0 = h0->face();
		FaceIter f1 = h3->face();

		// Get the edges
		EdgeIter e1 = h1->edge();
		EdgeIter e2 = h2->edge();

		EdgeIter e3 = h4->edge();
		EdgeIter e4 = h5->edge();

		// Update main halfedges
		h0->setNeighbors(h5, h3, v2, e0, f0);
		h3->setNeighbors(h2, h0, v3, e0, f1);

		// Update other halfedges
		h1->setNeighbors(h0, h1->twin(), v1, e1, f0);
		h2->setNeighbors(h4, h2->twin(), v2, e2, f1);
		h4->setNeighbors(h3, h4->twin(), v0, e3, f1);
		h5->setNeighbors(h1, h5->twin(), v3, e4, f0);

		// Update faces
		f0->halfedge() = h0;
		f1->halfedge() = h3;

		// Update vertices
		v0->halfedge() = h4;
		v1->halfedge() = h1;
		v2->halfedge() = h2;
		v3->halfedge() = h5;

		return e0;
	}


	VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
	{
		// TODO Part 5.
		// This method should split the given edge and return an iterator to the newly inserted vertex.
		// The halfedge of this vertex should point along the edge that was split, rather than the new edges.
		// Check if the edge is a boundary edge
		if (e0->isBoundary()) return newVertex();

		// Get the halfedges of both triangles
		HalfedgeIter h0 = e0->halfedge();
		HalfedgeIter h1 = h0->next();
		HalfedgeIter h2 = h1->next();

		HalfedgeIter h3 = h0->twin();
		HalfedgeIter h4 = h3->next();
		HalfedgeIter h5 = h4->next();

		// Get the vertices
		VertexIter v0 = h0->vertex();
		VertexIter v1 = h1->vertex();
		VertexIter v2 = h2->vertex();
		VertexIter v3 = h5->vertex();

		// Get the faces
		FaceIter f0 = h0->face();
		FaceIter f1 = h3->face();

		// Get the edges
		EdgeIter e1 = h1->edge();
		EdgeIter e2 = h2->edge();

		EdgeIter e3 = h4->edge();
		EdgeIter e4 = h5->edge();

		// Create new half edges
		HalfedgeIter h6 = newHalfedge();
		HalfedgeIter h7 = newHalfedge();
		HalfedgeIter h8 = newHalfedge();
		HalfedgeIter h9 = newHalfedge();
		HalfedgeIter h10 = newHalfedge();
		HalfedgeIter h11 = newHalfedge();

		// Create new vertex
		VertexIter v4 = newVertex();
		v4->position = (v0->position + v1->position) / 2.0;

		// Create new edges
		EdgeIter e5 = newEdge();
		EdgeIter e6 = newEdge();
		EdgeIter e7 = newEdge();

		// Create new faces
		FaceIter f2 = newFace();
		FaceIter f3 = newFace();


		// Update half edges
		h0->setNeighbors(h6, h8, v0, e0, f0);
		h6->setNeighbors(h2, h10, v4, e6, f0);
		h2->setNeighbors(h0, h2->twin(), v2, e2, f0);

		h1->setNeighbors(h10, h1->twin(), v1, e1, f2);
		h10->setNeighbors(h11, h6, v2, e6, f2);
		h11->setNeighbors(h1, h3, v4, e5, f2);

		h3->setNeighbors(h9, h11, v1, e5, f1);
		h9->setNeighbors(h5, h7, v4, e7, f1);
		h5->setNeighbors(h3, h5->twin(), v3, e4, f1);

		h4->setNeighbors(h7, h4->twin(), v0, e3, f3);
		h7->setNeighbors(h8, h9, v3, e7, f3);
		h8->setNeighbors(h4, h0, v4, e0, f3);

		// Update vector half edges
		v0->halfedge() = h0;
		v1->halfedge() = h1;
		v2->halfedge() = h2;
		v3->halfedge() = h5;
		v4->halfedge() = h8;

		// Update face half edges
		f0->halfedge() = h0;
		f1->halfedge() = h3;
		f2->halfedge() = h1;
		f3->halfedge() = h4;

		// Update edge half edges
		e0->halfedge() = h0;
		e1->halfedge() = h1;
		e2->halfedge() = h2;
		e3->halfedge() = h4;
		e4->halfedge() = h5;
		e5->halfedge() = h3;
		e6->halfedge() = h6;
		e7->halfedge() = h7;

		// Set edges to be new
		e6->isNew = true;
		e7->isNew = true;

		return v4;
	}



	void MeshResampler::upsample(HalfedgeMesh& mesh)
	{
		printf("Step 1\n");
		// Step 1: Compute new positions for all the vertices in the input mesh
		// using the Loop subdivision rule, and mark each vertex as a vertex of the original mesh.
		for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
			v->isNew = false;

			int n = v->degree();
			float u;
			if (n <= 3) u = 3.0 / 16.0;
			else u = 3.0 / (8.0 * n);

			Vector3D newEdgePos = v->position * (1.0 - (n * u));

			HalfedgeIter he = v->halfedge()->next();
			for (int i = 0; i < n; ++i) {
				newEdgePos += he->vertex()->position * u;
				he = he->next()->twin()->next();
			}

			v->newPosition = newEdgePos;
		}


		printf("Step 2\n");
		// Step 2: Compute the updated vertex positions associated with edges and store them in Edge::newPosition.
		for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
			HalfedgeIter he0 = e->halfedge();
			HalfedgeIter he1 = he0->twin();

			VertexIter a = he0->vertex();
			VertexIter b = he1->vertex();

			// Find the vertices adjacent to 'a' and 'b' around the edge
			VertexIter c = he0->next()->next()->vertex();
			VertexIter d = he1->next()->next()->vertex();

			// Compute the new edge position
			Vector3D newEdgePos = (a->position * 3.0 / 8.0) +
				(b->position * 3.0 / 8.0) +
				(c->position * 1.0 / 8.0) +
				(d->position * 1.0 / 8.0);

			// Update edge properties
			e->isNew = false;
			e->newPosition = newEdgePos;
		}



		printf("Step 3\n");
		// Step 3: Split every edge in the mesh, in any order.
		int numSplits = 0;
		int nEdges = mesh.nEdges();
		auto e = mesh.edgesBegin();

		for (int i = 0; i < nEdges; ++e, ++i) {
				numSplits++;
				VertexIter newVertex = mesh.splitEdge(e);
				newVertex->isNew = true;
				newVertex->newPosition = e->newPosition;
		}
		printf("splits %d\n", numSplits);

		// 18 edges before

		// Step 4: Flip any new edge that connects an old and new vertex.
		int numFlipped = 0;
		for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
			if (e->isNew) {
				HalfedgeIter h0 = e->halfedge();
				HalfedgeIter h1 = h0->twin();
				if (h0->vertex()->isNew != h1->vertex()->isNew) {
					numFlipped++;
					mesh.flipEdge(e);
				}
			}
		}
		printf("flips %d\n", numFlipped);

		printf("Step 5\n");
		// Step 5: Copy the new vertex positions into final Vertex::position.
		for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
			v->position = v->newPosition;
		}
	}


}
