using System;
using System.Collections.Generic;
using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;


namespace Pathfinding
{
    public class Astar
    {
        private readonly List<Vector3> _astarExploredNodes = new();
        private List<Vector3> _astarPath = new();
        
        /// <summary>
        /// Run the A* algorithm. 
        /// </summary>
        /// <param name="start">Start position. </param>
        /// <param name="goal">Goal position. </param>
        /// <returns>The planned path. </returns>
        public List<Vector3> PlanPathAStar(Vector3 start, Vector3 goal)
        {
            const float gridSize = 2.0f;
            const float carRadius = 0.9f;
            
            start = RoundToGrid(start, gridSize);
            goal = RoundToGrid(goal, gridSize);
            
            List<AStarNode> openSet = new();
            HashSet<Vector3> closedSet = new();
            
            AStarNode startNode = new AStarNode(start);
            startNode.GCost = 0;
            startNode.HCost = Vector3.Distance(start, goal);
            openSet.Add(startNode);
            
            int maxIterations = 50000;
            int iter = 0;
            
            while (openSet.Count > 0 && iter < maxIterations)
            {
                iter++;
                
                AStarNode currentNode = openSet.OrderBy(n => n.FCost).First();
                openSet.Remove(currentNode);
                closedSet.Add(currentNode.Position);
                
                _astarExploredNodes.Add(currentNode.Position);
                
                if (Vector3.Distance(currentNode.Position, goal) < gridSize * 1.5f)
                {
                    Debug.Log($"A* found path in {iter} iterations");
                    List<Vector3> path = ReconstructPath(currentNode);
                    _astarPath = path;  // Store for visualization
                    return path;
                }
                
                foreach (Vector3 neighborPos in GetNeighbors(currentNode.Position, gridSize))
                {
                    if (closedSet.Contains(neighborPos))
                        continue;
                    
                    if (!IsTraversableAStar(neighborPos, carRadius))
                        continue;
                    
                    float tentativeGCost = currentNode.GCost + Vector3.Distance(currentNode.Position, neighborPos);
                    
                    AStarNode neighborNode = openSet.FirstOrDefault(n => n.Position == neighborPos);
                    
                    if (neighborNode == null)
                    {
                        neighborNode = new AStarNode(neighborPos);
                        neighborNode.GCost = tentativeGCost;
                        neighborNode.HCost = Vector3.Distance(neighborPos, goal);
                        neighborNode.Parent = currentNode;
                        openSet.Add(neighborNode);
                    }
                    else if (tentativeGCost < neighborNode.GCost)
                    {
                        neighborNode.GCost = tentativeGCost;
                        neighborNode.Parent = currentNode;
                    }
                }
            }
            
            Debug.LogError($"A* failed after {iter} iterations. Explored {_astarExploredNodes.Count} nodes, OpenSet empty: {openSet.Count == 0}");
            return new List<Vector3> { start, goal };
        }
        
        
        /// <summary>
        /// The nodes of the A* algorithm. Each node stores its position, gCost (cost from start),
        /// hCost (heuristic to goal), and a reference to its parent node for path reconstruction.
        /// </summary>
        private class AStarNode
        {
            public float GCost;
            public float HCost;
            public AStarNode Parent;
            public Vector3 Position;

            /// <summary>
            /// Initializes a new AStarNode with the given position. gCost and hCost are set to infinity by default, and parent is null.
            /// </summary>
            /// <param name="pos">The position of the node. </param>
            public AStarNode(Vector3 pos)
            {
                Position = pos;
            }

            public float FCost => GCost + HCost;
        }
        
        
        /// <summary>
        /// Rounds a position to the nearest grid point based on the specified grid size. This helps to discretize the search space for A*.
        /// </summary>
        /// <param name="pos">The position we want to round. </param>
        /// <param name="gridSize">The grid size. </param>
        /// <returns>The rounded position. </returns>
        private static Vector3 RoundToGrid(Vector3 pos, float gridSize)
        {
            return new Vector3(
                Mathf.Round(pos.x / gridSize) * gridSize,
                pos.y,
                Mathf.Round(pos.z / gridSize) * gridSize
            );
        }
        
        
        /// <summary>
        /// Returns the path from the start node to the given end node. 
        /// </summary>
        /// <param name="endNode">The end node. </param>
        /// <returns>The path to the end node. </returns>
        private static List<Vector3> ReconstructPath(AStarNode endNode)
        {
            List<Vector3> path = new List<Vector3>();
            AStarNode current = endNode;
        
            while (current != null)
            {
                path.Add(current.Position);
                current = current.Parent;
            }
        
            path.Reverse();
            return path;
        }
        
        
        /// <summary>
        /// Get the neighboring positions around the given position based on the specified grid size. This generates 8-connected neighbors (including diagonals).
        /// </summary>
        /// <param name="pos">The node position we base the neighbors on. </param>
        /// <param name="gridSize">The grid size. </param>
        /// <returns>List of the neighbor positions. </returns>
        private static List<Vector3> GetNeighbors(Vector3 pos, float gridSize)
        {
            List<Vector3> neighbors = new List<Vector3>();
    
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dz == 0) continue;
            
                    Vector3 neighbor = new Vector3(
                        pos.x + dx * gridSize,
                        pos.y,
                        pos.z + dz * gridSize
                    );
                    neighbors.Add(neighbor);
                }
            }
    
            return neighbors;
        }
        
        
        /// <summary>
        /// Checks if a position is traversable. 
        /// </summary>
        /// <param name="position">The position we want to check. </param>
        /// <param name="radius">Radius of the vehicle. </param>
        /// <returns>True if the position is traversable and false otherwise. </returns>
        private static bool IsTraversableAStar(Vector3 position, float radius)
        {
            int obstacleLayer = LayerMask.GetMask("Obstacle");
    
            // Check a box at this position with the given radius
            bool isBlocked = Physics.CheckBox(
                position,
                new Vector3(radius, 0.5f, radius), 
                Quaternion.identity,
                obstacleLayer
            );
    
            return !isBlocked;
        }
    }
}