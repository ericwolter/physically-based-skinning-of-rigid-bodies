#include "Edge.h"

Edge::Edge(Vector3f vertex1, Vector3f vertex2)
{
	vertices.push_back(vertex1);
	vertices.push_back(vertex2);

	direction = vertex2 - vertex1;
}
Edge::~Edge() {}