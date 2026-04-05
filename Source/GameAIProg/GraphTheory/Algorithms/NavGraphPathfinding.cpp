#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, bool useSmoothing ,std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	const auto* startTriangle{ pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true) };
	const auto* endTriangle{ pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true) };
	
	//We have valid start/end triangles and they are not the same
	if (!startTriangle || !endTriangle)
	{
		debugNodePositions = finalPath;
		return finalPath;
	}
	
	if (startTriangle == endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		debugNodePositions = finalPath;

		return finalPath;
	}
	
	//=> Start looking for a path
	//Copy the graph
	auto pNavGraphClone = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position
	auto startNode = std::make_unique<NavGraphNode>(startPos, -1);
	const int startNodeId = pNavGraphClone->AddNode(std::move(startNode));
	
	for (const auto& edge : startTriangle->GetEdges())
	{
		std::optional<int> edgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (!edgeIdx.has_value()) continue;
		
		int nodeId = pNavGraphClone->GetNodeIdFromEdgeIndex(*edgeIdx);
		if (nodeId == Graphs::InvalidNodeId) continue;
			
		pNavGraphClone->AddConnection(startNodeId, nodeId);
	}
	
	//Create extra node for the endNode
	auto endNode = std::make_unique<NavGraphNode>(endPos, -1);
	int const endNodeId = pNavGraphClone->AddNode(std::move(endNode));

	for (auto const& edge : endTriangle->GetEdges())
	{
		std::optional<int> edgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (!edgeIdx.has_value()) continue;
		
		int nodeId = pNavGraphClone->GetNodeIdFromEdgeIndex(*edgeIdx);
		if (nodeId == Graphs::InvalidNodeId) continue;
		
		pNavGraphClone->AddConnection(endNodeId, nodeId);
	}
	
	pNavGraphClone->SetConnectionCostsToDistances();
	
	//Run A star on new graph
	AStar AStar(pNavGraphClone.get(), HeuristicFunctions::Euclidean);

	auto nodePath = AStar.FindPath(
		pNavGraphClone->GetNode(startNodeId).get(),
		pNavGraphClone->GetNode(endNodeId).get()
	);
	
	finalPath.reserve(nodePath.size());
	for (const auto pPathNode : nodePath)
	{
		const auto& nodePosition = pPathNode->GetPosition();

		finalPath.emplace_back(nodePosition);
	}
	
	//Debug Visualisation
	debugNodePositions = finalPath;
	
	
	if (useSmoothing)
	{
		debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
		finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	}
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph, bool useSmoothing)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, useSmoothing, debugNodePositions, debugPortals);
}