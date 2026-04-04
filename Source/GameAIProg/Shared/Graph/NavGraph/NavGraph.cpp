#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

bool GameAI::NavGraph::IsEdgeSharedByTwoTriangles(const TriPolygon::Edge& edge) const
{
	int count = 0;

	for (auto const& triangle : pNavPoly->GetTriangles())
	{
		if (triangle.HasEdge(edge))
		{
			++count;

			if (count > 2)
				return false;
		}
	}

	return count == 2;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
			// Create node here
	const auto& edges = pNavPoly->GetEdges();

	for (int lineIdx = 0; lineIdx < edges.size(); ++lineIdx)
	{
		const auto& edge = edges[lineIdx];

		// Only edges shared by 2 triangles → valid navigation edge
		if (!IsEdgeSharedByTwoTriangles(edge))
			continue;

		// Create node at midpoint of the edge
		FVector midpoint = (edge.GetP1(*pNavPoly) + edge.GetP2(*pNavPoly)) * 0.5f;

		auto node = std::make_unique<NavGraphNode>(FVector2D(midpoint.X, midpoint.Y), lineIdx);
		AddNode(std::move(node));
	}

	//2. Create connections now that every node is created	
	const auto& triangles = pNavPoly->GetTriangles();

	for (const auto& triangle : triangles)
	{
		std::vector<int> nodeIds;

		// Loop over triangle edges
		for (const auto& edge : triangle.GetEdges())
		{
			auto const& edgeIdx = pNavPoly->FindEdgeIndex(edge);

			if (edgeIdx.has_value())
			{
				int nodeId = GetNodeIdFromEdgeIndex(edgeIdx.value());
				if (nodeId != Graphs::InvalidNodeId)
				{
					nodeIds.push_back(nodeId);
				}
			}
		}
		
		// Connect nodes inside this triangle
		if (nodeIds.size() == 2)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
		}
		else if (nodeIds.size() == 3)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[2]);
			AddConnection(nodeIds[2], nodeIds[0]);
		}
	}
		
	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}
