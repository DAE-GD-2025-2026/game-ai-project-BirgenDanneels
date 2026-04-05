#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		if (Path.empty()) return Portals;

		//Start portal
		FVector2D start = Path.front()->GetPosition();
		Portals.emplace_back(start, start);

		for (size_t i = 0; i < Path.size(); ++i)
		{
			const auto* node = static_cast<const NavGraphNode*>(Path[i]);
			if (!node) continue;

			int edgeIdx = node->GetEdgeIdx();
			if (edgeIdx < 0) continue;

			const auto& edge = NavPoly.GetEdges()[edgeIdx];

			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
			FVector2D p1{ edge.GetP1(NavPoly).X, edge.GetP1(NavPoly).Y };
			FVector2D p2{ edge.GetP2(NavPoly).X, edge.GetP2(NavPoly).Y };

			FVector2D movementDir;
			if (i + 1 < Path.size())
				movementDir = Path[i + 1]->GetPosition() - Path[i]->GetPosition();
			else if (i > 0)
				movementDir = Path[i]->GetPosition() - Path[i - 1]->GetPosition();
			else
				continue;
			
			FVector2D edgeDir = p2 - p1;
			float cross = FVector2D::CrossProduct(movementDir, edgeDir);

			// Ensure consistent winding (e.g. P1 = right, P2 = left)
			if (cross < 0.0f)
				std::swap(p1, p2);

			//Store portal
			Portals.emplace_back(p1, p2);
		}

		//Add degenerate portal to force end evaluation
		FVector2D end = Path.back()->GetPosition();
		Portals.emplace_back(end, end);

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path;
	    if (Portals.empty()) return Path;

	    int apexIndex = 0;
	    int leftIndex = 0;
	    int rightIndex = 0;

		FVector2D apexPos = Portals[apexIndex].P1;
		FVector2D leftLeg = Portals[leftIndex].P2 - apexPos;
		FVector2D rightLeg = Portals[rightIndex].P1 - apexPos;
		
	    Path.push_back(apexPos);

		int portalIdx = 1;
		while (portalIdx < Portals.size())
		{

			//--- RIGHT CHECK ---
	    	FVector2D newRightLeg = Portals[portalIdx].P1 - apexPos;
	    	
	        if (FVector2D::CrossProduct(rightLeg, newRightLeg) >= 0.0f)
	        {
	            if (FVector2D::CrossProduct(leftLeg, newRightLeg) > 0.0f)
	            {
	                apexPos = leftLeg + apexPos;

	                apexIndex = leftIndex;
	            	portalIdx = apexIndex + 1;

	                leftIndex = portalIdx;
	                rightIndex = portalIdx;

	            	leftLeg = Portals[leftIndex].P2 - apexPos;
	            	rightLeg = Portals[rightIndex].P1 - apexPos;

	            	Path.push_back(apexPos);
	            	
	                continue;
	            }
	        	
	            rightLeg = newRightLeg;
	            rightIndex = portalIdx;
	        }

			//--- LEFT CHECK ---
	    	FVector2D newLeftLeg  = Portals[portalIdx].P2 - apexPos;
	    	
	        if (FVector2D::CrossProduct(leftLeg, newLeftLeg) <= 0.0f)
	        {
	            if (FVector2D::CrossProduct(rightLeg, newLeftLeg) < 0.0f)
	            {
	            	apexPos = rightLeg + apexPos;

	            	apexIndex = rightIndex;
	            	portalIdx = apexIndex + 1;

	            	leftIndex = portalIdx;
	            	rightIndex = portalIdx;

	            	leftLeg = Portals[leftIndex].P2 - apexPos;
	            	rightLeg = Portals[rightIndex].P1 - apexPos;

	            	Path.push_back(apexPos);
	            	
	                continue;
	            }
	        	
	            leftLeg = newLeftLeg;
	            leftIndex = portalIdx;
	        }
			
			portalIdx++;
	    }

	    Path.push_back(Portals.back().P1);
	    return Path;
	}
		
private:
	SSFA() {};
	~SSFA() {};
};
}
