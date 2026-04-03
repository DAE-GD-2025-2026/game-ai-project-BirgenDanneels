#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		if (!IsConnected())
			return Eulerianity::notEulerian;

		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();

		int oddCount = 0;

		for (auto* node : nodes)
		{
			int degree = m_pGraph->FindConnectionsFrom(node->GetId()).size();
			

			if (degree % 2 != 0)
				oddCount++;
		}

		if (oddCount > 2)
			return Eulerianity::notEulerian;

		if (oddCount == 2)
			return Eulerianity::semiEulerian;

		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		std::vector<Node*> Path;

		eulerianity = IsEulerian();
		if (eulerianity == Eulerianity::notEulerian)
			return Path;

		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> nodes = graphCopy.GetActiveNodes();

		std::stack<int> stack;

		// 2. Choose starting node
		int currentNodeId = nodes[0]->GetId();

		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (auto* node : nodes)
			{
				int degree = graphCopy.FindConnectionsFrom(node->GetId()).size() / 2;
				if (degree % 2 != 0)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}

		// 3. Main loop
		while (!stack.empty() || 
			   !graphCopy.FindConnectionsFrom(currentNodeId).empty())
		{
			auto connections = graphCopy.FindConnectionsFrom(currentNodeId);

			if (!connections.empty())
			{
				// 4.I push current node
				stack.push(currentNodeId);

				// 4.II pick neighbor
				Connection* conn = connections[0];
				int nextNodeId = conn->GetToId();

				// 4.IV remove edge
				graphCopy.RemoveConnection(conn);

				// 4.III move to neighbor
				currentNodeId = nextNodeId;
			}
			else
			{
				// backtrack
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());

				currentNodeId = stack.top();
				stack.pop();
			}
		}

		// 5. add last node
		Path.push_back(m_pGraph->GetNode(currentNodeId).get());

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		visited[startIndex] = true;

		int nodeId = Nodes[startIndex]->GetId();
		auto connections = m_pGraph->FindConnectionsFrom(nodeId);

		for (auto* conn : connections)
		{
			int toId = conn->GetToId();

			// Find index of that node
			for (int i = 0; i < Nodes.size(); ++i)
			{
				if (Nodes[i]->GetId() == toId && !visited[i])
				{
					VisitAllNodesDFS(Nodes, visited, i);
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		std::vector<bool> visited(Nodes.size(), false);

		// Find a node with at least 1 connection
		int startIndex = -1;
		for (int i = 0; i < Nodes.size(); ++i)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}

		// No edges at all → considered connected
		if (startIndex == -1)
			return true;

		VisitAllNodesDFS(Nodes, visited, startIndex);

		// Check if all nodes with edges were visited
		for (int i = 0; i < Nodes.size(); ++i)
		{
			if (!visited[i] &&
				!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				return false;
			}
		}

		return true;
	}
}