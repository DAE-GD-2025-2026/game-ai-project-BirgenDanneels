#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	//calculate bounds of a cell
	CellWidth = Width / Cols * 2;
	CellHeight = Height / Rows * 2;

	//Create the cells
	Cells.reserve(NrOfCols * NrOfRows);

	float HalfOffsetX = -SpaceWidth;
	float HalfOffsetY = -SpaceHeight;
	
	for (int yIdx{ 0 }; yIdx < NrOfCols; ++yIdx)
	{
		for (int xIdx{ 0 }; xIdx < NrOfRows; ++xIdx)
		{
			Cells.emplace_back(Cell( HalfOffsetX + xIdx * CellWidth, HalfOffsetY + yIdx * CellHeight, CellWidth, CellHeight));
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int cellIdx{ PositionToIndex(Agent.GetPosition()) };

	Cells[cellIdx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int CurrentCellIdx{ PositionToIndex(Agent.GetPosition()) };
	int OldCellIdx{ PositionToIndex(OldPos) };

	if (OldCellIdx != CurrentCellIdx)
	{
		Cells[OldCellIdx].Agents.remove(&Agent);
		Cells[CurrentCellIdx].Agents.push_back(&Agent);
	}
}

void CellSpace::RenderNeighborHood(const ASteeringAgent& Agent, float QueryRadius)
{
	FRect NeighborhoodRect{ 
		FVector2D{Agent.GetPosition().X - QueryRadius, 
			Agent.GetPosition().Y - QueryRadius},
		FVector2D{Agent.GetPosition().X + QueryRadius, 
			Agent.GetPosition().Y + QueryRadius}};
	
	for (Cell& CurrentCell : Cells)
	{
		if (DoRectsOverlap(NeighborhoodRect, CurrentCell.BoundingBox))
		{
			//Fill In Partition
			auto Points = CurrentCell.GetRectPoints();

			FVector Center(
				(Points[0].X + Points[2].X) * 0.5f,
				(Points[0].Y + Points[2].Y) * 0.5f,
				0
			);

			FVector Extent(
				FMath::Abs(Points[2].X - Points[0].X) * 0.5f,
				FMath::Abs(Points[2].Y - Points[0].Y) * 0.5f,
				1
			);

			DrawDebugSolidBox(pWorld, Center, Extent, FColor(255,0,0,100), false, 0);

			//Draw Circle On Agents
			for (ASteeringAgent* currentAgent : CurrentCell.Agents)
			{
				DrawDebugCircle(
					pWorld,
					FVector(currentAgent->GetPosition().X, currentAgent->GetPosition().Y, 90),
					20,
					32,
					FColor::Green,
					false,
					0.f,
					0,
					6.f,
					FVector(1,0,0),
					FVector(0,1,0),
					false );
			}
		}
	}
	
	//Draw Neighborhood Rectangle
	std::vector<FVector2D> Points
	{
		FVector2D(NeighborhoodRect.Min.X, NeighborhoodRect.Min.Y), // Bottom-left
		FVector2D(NeighborhoodRect.Max.X, NeighborhoodRect.Min.Y), // Bottom-right
		FVector2D(NeighborhoodRect.Max.X, NeighborhoodRect.Max.Y), // Top-right
		FVector2D(NeighborhoodRect.Min.X, NeighborhoodRect.Max.Y)  // Top-left
	};

	for (int i = 0; i < Points.size(); ++i)
	{
		DrawDebugLine(
			pWorld,
			FVector(Points[i], 50.f),
			FVector(Points[(i + 1) % Points.size()], 50.f),
			FColor::Black,
			false,
			0.f,
			0,
			6.f
		);
	}
	
	//Draw Neighborhood Circle
	DrawDebugCircle(
	pWorld,
	FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 90),
	QueryRadius,
	32,
	FColor::Black,
	false,
	0.f,
	0,
	6.f,
	FVector(1,0,0),
	FVector(0,1,0),
	false );
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	FRect NeighborhoodRect{ 
		FVector2D{Agent.GetPosition().X - QueryRadius, 
			Agent.GetPosition().Y - QueryRadius},
		FVector2D{Agent.GetPosition().X + QueryRadius, 
			Agent.GetPosition().Y + QueryRadius}};
	
	NrOfNeighbors = 0;

	for (Cell& CurrentCell : Cells)
	{
		if (DoRectsOverlap(NeighborhoodRect, CurrentCell.BoundingBox))
		{
			for (ASteeringAgent* CurrentAgent : CurrentCell.Agents)
			{
				Neighbors[NrOfNeighbors] = CurrentAgent;
				++NrOfNeighbors;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	for (const auto& CurrentCell : Cells)
	{
		const std::vector<FVector2D>& Points = CurrentCell.GetRectPoints();

		for (int i = 0; i < Points.size(); ++i)
		{
			DrawDebugLine(
				pWorld,
				FVector(Points[i], 50.f),
				FVector(Points[(i + 1) % Points.size()], 50.f),
				FColor::Red,
				false,
				0.f,
				0,
				6.f
			);
		}

		DrawDebugString(
			pWorld,
			FVector(Points[3], 50.f),
			FString::FromInt(CurrentCell.Agents.size()),
			nullptr,
			FColor::Black,
			0.f
		);
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	float xOffset{ SpaceWidth };
	float yOffset{ SpaceHeight };
	
	int Collum = int((Pos.X + xOffset) / CellWidth);
	int Row = int((Pos.Y + yOffset) / CellHeight);			

	if (Row >= NrOfRows)Row = NrOfRows -1;
	if (Collum >= NrOfCols)Collum = NrOfCols - 1;
	
	return Row * (NrOfCols) + Collum;;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}