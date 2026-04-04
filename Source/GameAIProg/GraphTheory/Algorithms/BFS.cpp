#include "BFS.h"

#include <map>
#include <queue>
#include <set>


#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	
	std::queue<Node*> openList;
	std::set<Node*> visited;
	std::map<Node*, Node*> parent;
	
	openList.push(pStartNode);
	visited.insert(pStartNode);
	
	while(!openList.empty())
	{
		Node* currentNode = openList.front();
		openList.pop();
		
		if (currentNode == pDestinationNode)
		{
			//Return Reconstruct path
			currentNode = pDestinationNode;
			
			while (currentNode != pStartNode)
			{
				path.push_back(currentNode);
				currentNode = parent[currentNode];
			}
			
			path.push_back(pStartNode);
			std::reverse(path.begin(), path.end());
			
			return path;
		}
		
		for (auto connection : pGraph->FindConnectionsFrom(currentNode->GetId()))
		{
			Node* neighbor = pGraph->GetNode(connection->GetToId()).get();
			
			if (visited.find(neighbor) == visited.end())
			{
				visited.insert(neighbor);
				parent[neighbor] = currentNode;
				openList.push(neighbor);
			}
		}
	}
	
	return path;
}
