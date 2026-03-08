# Game AI Project
This project, built in Unreal Engine 5 with mainly C++, explores classic Game AI patterns. 
It includes basic and combined steering behaviors and flocking.
It was created as part of the course Algorithms 2.

# Features
## Steering Behaviors

- **Seeking**: Move towards a target smoothly.

- **Fleeing**: Escape from a threat dynamically.

- **Arriving**: Approach a target and slow down near it.

- **Facing**: Turn to face a target (overrides the auto-rotation).

- **Pursuing**: Predict and chase a moving target.

- **Evading**: Avoid an incoming threat using prediction.

- **Wandering**: Random but natural-looking movement based on a projected point on a circle.
  
![Basic Steering](https://github.com/user-attachments/assets/df44cd2c-a00f-4913-bbe4-0d2a9134b41c)

## Combined Steering

- **Blended**: Mix multiple behaviors for smoother results.

- **Prioritized**: Resolve multiple behaviors by priority.
  
![Combined Steering](https://github.com/user-attachments/assets/89829524-5b2c-4444-8d86-f2d0a07afcbd)

## Boids Flocking

- **Cohesion**: Stay close to neighbors.

- **Separation**: Create space between neighbors.

- **Velocity Matching**: Align direction with neighbors.

- **Evade Agent**: Avoid a wandering agent while flocking.

## Spatial Partitioning

- Divides the area into square partitions storing agent pointers.

- Automatically updates agents when they move between partitions.

- Queries only nearby partitions (dictated by a radius) for efficient neighbor searches.

- Significantly reduces the number of agents checked for behaviors, improving performance in large flocks.

![Flock With Spatial Partitioning](https://github.com/user-attachments/assets/80a7d32b-4eec-4f6e-a2b7-ddef066d34b2)

## Debug Drawing
- Visualizes steering directions, wandering circle and spatial partitions with number of agents for better understanding and debugging.

# Notes
- When a lot of agents go outside of the trimmed area in the same spot, they get put on the otherside, which can cause agents to get stuck on the edge if there is already one there.
