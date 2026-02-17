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


/*
[RequireComponent(typeof(CarController))]
public class CarAI : Agent
{
    public CarController car; // the car controller we want to use
    private List<Vector3> astarExploredNodes = new List<Vector3>();
    private List<Vector3> astarPath = new List<Vector3>();
    private Controller controller;

    private HybridAStar hybridAStar;
    private Transform initialCarState;
    private CGSmoother smoother;
    private List<Node> waypoints;

    private void OnDrawGizmos()
    {

         // if (this.hybridAStar == null || this.hybridAStar.exploredNodes == null)
         //    return;

        // Hybrid A* search debug
        // foreach (var node in this.hybridAStar.exploredNodes)
        // {
        //     Gizmos.color = Color.red;
        //     Gizmos.DrawSphere(new Vector3(node.continuousState.x, 0.5f, node.continuousState.y), 0.3f);
        //
        //     // Draw heading arrow
        //     float arrowLen = 1f;
        //     Vector3 start = new Vector3(node.continuousState.x, 0.5f, node.continuousState.y);
        //     Vector3 end = start + new Vector3(
        //         Mathf.Cos(node.continuousState.z) * arrowLen,
        //         0,
        //         Mathf.Sin(node.continuousState.z) * arrowLen
        //     );
        //     Gizmos.DrawLine(start, end);
        // }


        if (this.waypoints != null && this.waypoints.Count > 0)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < this.waypoints.Count - 1; i++)
            {
                Vector3 start = new Vector3(this.waypoints[i].position.x, 1f, this.waypoints[i].position.y);
                Vector3 end = new Vector3(this.waypoints[i + 1].position.x, 1f, this.waypoints[i + 1].position.y);
                Gizmos.DrawLine(start, end);
                Gizmos.DrawSphere(start, 0.5f);
            }
            // Draw last waypoint
            Vector3 lastPos = new Vector3(this.waypoints[this.waypoints.Count - 1].position.x, 1f, this.waypoints[this.waypoints.Count - 1].position.y);
            Gizmos.DrawSphere(lastPos, 0.5f);
        }

        if (this.controller != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(
                new Vector3(this.controller.closestPoint.x, this.initialCarState.position.y,
                    this.controller.closestPoint.y), 0.5f);
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(
                new Vector3(this.controller.targetPoint.x, this.initialCarState.position.y,
                    this.controller.targetPoint.y), 0.5f);
        }



        //bool showDistMap = true;
        //if (this.smoother != null && showDistMap)
        //{
        //    for (int i = 0; i < CGSmoother.DISTANCE_MAP_RESOLUTION; i+=2)
        //    {
        //        for (int j = 0; j < CGSmoother.DISTANCE_MAP_RESOLUTION; j+=2)
        //        {
        //            Vector2 obsPos = this.smoother.distMap[i][j];
        //            float xPos = this.smoother.distStart.x + i * this.smoother.stepX;
        //            float zPos = this.smoother.distStart.z + j * this.smoother.stepZ;
        //            float dist = Vector2.Distance(obsPos, new Vector2(xPos, zPos));
        //            if (dist < CGSmoother.D_MAX)
        //            {
        //                Gizmos.color = Color.red;
        //            }
        //            else
        //            {
        //                Gizmos.color = Color.green;
        //            }
        //            Gizmos.DrawSphere(new Vector3(xPos, this.initialCarState.position.y + 5, zPos), 0.3f);
        //        }
        //    }
        //}
    }
    public override void Initialize() // Runs before game starts
    {
        // Gets relevant game data
        this.initialCarState = gameObject.transform.Find("Colliders/ColliderBody").transform;
        GameObject groundPlane = GameObject.Find("GroundPlane");
        Collider groundCollider = groundPlane.GetComponent<Collider>();

        this.hybridAStar = new HybridAStar(  
            MapManager.GetGlobalStartPosition(),
            MapManager.GetGlobalGoalPosition(),
            groundCollider,
            this.initialCarState,
            ObstacleMap
        );
        
        // TRY HYBRID A* FIRST
        this.hybridAStar.RunHybridAStar();
    
        List<Node> nodes;
    
        if (!this.hybridAStar.foundPath || this.hybridAStar.waypoints.Count == 0)
        {
            Debug.LogWarning("Hybrid A* failed - falling back to simple A*");
        
            // FALLBACK: Use simple A*
            List<Vector3> astarPath = PlanPathAStar(
                MapManager.GetGlobalStartPosition(),
                MapManager.GetGlobalGoalPosition()
            );
        
            if (astarPath.Count < 2)
            {
                Debug.LogError("Both Hybrid A* and A* failed - no path found");
                return;
            }
        
            Debug.Log($"A* found path with {astarPath.Count} waypoints");
        
            // Convert Vector3 path to Node list
            nodes = new List<Node>();
            foreach (Vector3 pos in astarPath)
            {
                nodes.Add(new Node(pos.x, pos.z));
            }
        }
        else
        {
            // Hybrid A* succeeded
            Debug.Log("Hybrid A* succeeded!");
            nodes = this.hybridAStar.waypoints;
        }
    
        // Smoothes the waypoints
        this.smoother = new CGSmoother(this.initialCarState.position.y, groundCollider);
        nodes = this.smoother.GetSmoothedPath(nodes);
        this.waypoints = nodes;
        
        // Creates the PD Controller
        this.controller = new Controller(this.waypoints, MapManager.GetGlobalGoalPosition(), this.initialCarState);

    }

    public override void Step() // Runs every step of the physics simulation
    {
        // Gets current car state
        Transform carTransform = gameObject.transform.Find("Colliders/ColliderBody").transform;

        // Calculates the move
        this.controller.PDCalculateMove(carTransform);

        // Executes the move
        car.Move(controller.steering, controller.acceleration, controller.footbrake, controller.handbrake);
        //car.Move(controller.steering, 1, 0, 0);
    }

    private class Node
    {
        public Node(float x, float y)
        {
            this.position = new Vector2(x, y);
            this.parent = null;
        }

        public Vector2 position { get; set; }
        public Node parent { get; set; }
    }

    public class HybridAStarNode
    {
        public Vector3 continuousState;
        public Vector3 discreteCell;
        public float gCost;
        public float hCost;
        public bool isReverse;
        public HybridAStarNode parent;
        public float steeringAngle;
        public float velocity;
    }
    
    private class HybridAStar
    {
        // Car kinematic parameters
        private const float WHEELBASE = 2.87f;
        private const float MAX_STEER_ANGLE = 25f * Mathf.Deg2Rad;
        private const float VELOCITY = 2f;
        private const float DT = 1f;
        private const float TURN_MULTIPLIER = 1.8f; 

        // Grid discretization
        private const float GRID_XY_RES = 3f;
        private const float GRID_THETA_RES = 15f;
        
        // Search parameters
        private const float GOAL_TOLERANCE = 5f;
        private const float COLLISION_RADIUS = 1.35f; // Car Width = 1.21
        private const int MAX_ITERATIONS = 300000;
        
        // Steering options
        private static readonly float[] FORWARD_STEERING = { -1f, -0.75f, -0.5f, 0f, 0.5f, 0.75f, 1f };
        
        // Holonomic heuristic data
        private float[,] holonomicDistances;
        private int gridWidth;
        private int gridHeight;
        private Vector3 mapMin;

        // Instance data
        private Vector3 startPos;
        private Vector3 goalPos;
        private Collider ground;
        private Transform carTransform;
        private int obstacleLayer;
        
        public List<Node> waypoints { get; set; }
        public bool foundPath { get; set; }
        public List<HybridAStarNode> exploredNodes { get; set; }

        public HybridAStar(Vector3 startPos, Vector3 goalPos, Collider ground, Transform carTransform, ObstacleMap obstacleMap)
        {
            this.startPos = startPos;
            this.goalPos = goalPos;
            this.ground = ground;
            this.carTransform = carTransform;
            this.foundPath = false;
            this.waypoints = new List<Node>();
            this.exploredNodes = new List<HybridAStarNode>();
            this.obstacleLayer = LayerMask.GetMask("Obstacle");
            
            PrecomputeHolonomicHeuristic(obstacleMap);
        }

        private void PrecomputeHolonomicHeuristic(ObstacleMap obstacleMap)
        {
            mapMin = ground.bounds.center - ground.bounds.extents;
            Vector3 mapMax = ground.bounds.center + ground.bounds.extents;
            
            gridWidth = Mathf.CeilToInt((mapMax.x - mapMin.x) / GRID_XY_RES);
            gridHeight = Mathf.CeilToInt((mapMax.z - mapMin.z) / GRID_XY_RES);
            
            holonomicDistances = new float[gridWidth, gridHeight];
            
            // Initialize all distances to infinity
            for (int x = 0; x < gridWidth; x++)
                for (int y = 0; y < gridHeight; y++)
                    holonomicDistances[x, y] = float.MaxValue;
            
            // Convert goal to grid coordinates
            int goalX = Mathf.Clamp(Mathf.RoundToInt((goalPos.x - mapMin.x) / GRID_XY_RES), 0, gridWidth - 1);
            int goalZ = Mathf.Clamp(Mathf.RoundToInt((goalPos.z - mapMin.z) / GRID_XY_RES), 0, gridHeight - 1);
            
            // Run Dijkstra's algorithm from goal (backward search)
            PriorityQueue<Vector2Int> openSet = new PriorityQueue<Vector2Int>();
            holonomicDistances[goalX, goalZ] = 0;
            openSet.Enqueue(new Vector2Int(goalX, goalZ), 0);
            
            // 8-connected grid expansion
            int[] dx = { -1, 1, 0, 0, -1, -1, 1, 1 };
            int[] dy = { 0, 0, -1, 1, -1, 1, -1, 1 };
            float[] costs = { 1f, 1f, 1f, 1f, 1.414f, 1.414f, 1.414f, 1.414f };
            
            while (openSet.Count > 0)
            {
                Vector2Int current = openSet.Dequeue();
                float currentDist = holonomicDistances[current.x, current.y];
                
                for (int i = 0; i < 8; i++)
                {
                    int nx = current.x + dx[i];
                    int ny = current.y + dy[i];
                    
                    if (nx < 0 || nx >= gridWidth || ny < 0 || ny >= gridHeight)
                        continue;
                    
                    Vector3 worldPos = new Vector3(mapMin.x + nx * GRID_XY_RES, 0f, mapMin.z + ny * GRID_XY_RES);
                    
                    if (!IsTraversable(worldPos, obstacleMap))
                        continue;
                    
                    float newDist = currentDist + costs[i] * GRID_XY_RES;
                    
                    if (newDist < holonomicDistances[nx, ny])
                    {
                        holonomicDistances[nx, ny] = newDist;
                        openSet.Enqueue(new Vector2Int(nx, ny), newDist);
                    }
                }
            }
            
            Debug.Log("Holonomic heuristic precomputed");
        }
        
        private bool IsTraversable(Vector3 worldPos, ObstacleMap obstacleMap)
        {
            if (!ground.bounds.Contains(worldPos))
                return false;
            
            return obstacleMap.GetGlobalPointTravesibility(worldPos) == ObstacleMap.Traversability.Free;
        }

        public void RunHybridAStar()
        {
            // Face the direction of the car when starting
            Vector3 carForward = carTransform.forward;
            float initialTheta = Mathf.Atan2(carForward.z, carForward.x);

            HybridAStarNode startNode = new HybridAStarNode
            {
                continuousState = new Vector3(startPos.x, startPos.z, initialTheta),
                discreteCell = StateToCell(new Vector3(startPos.x, startPos.z, initialTheta)),
                velocity = VELOCITY,
                isReverse = false,
                steeringAngle = 0f,
                parent = null,
                gCost = 0f,
                hCost = Heuristic(new Vector3(startPos.x, startPos.z, initialTheta))
            };

            List<HybridAStarNode> openSet = new List<HybridAStarNode> { startNode };
            Dictionary<Vector3Int, HybridAStarNode> closedSet = new Dictionary<Vector3Int, HybridAStarNode>();

            int iter = 0;
            while (openSet.Count > 0 && iter < MAX_ITERATIONS)
            {
                HybridAStarNode current = GetLowestFCost(openSet);
                openSet.Remove(current);

                if (IsGoalReached(current.continuousState))
                {
                    Debug.Log($"Hybrid A* found path in {iter} iterations");
                    foundPath = true;
                    ReconstructPath(current);
                    return;
                }

                Vector3Int cellKey = Vector3Int.RoundToInt(current.discreteCell);
                
                if (closedSet.ContainsKey(cellKey))
                    continue;
                    
                closedSet[cellKey] = current;
                exploredNodes.Add(current);

                foreach (var child in ExpandNode(current))
                {
                    Vector3Int childCellKey = Vector3Int.RoundToInt(child.discreteCell);
                    if (!closedSet.ContainsKey(childCellKey))
                        openSet.Add(child);
                }

                iter++;
            }

            Debug.LogWarning($"Hybrid A* failed after {iter} iterations");
            foundPath = false;
        }

        private List<HybridAStarNode> ExpandNode(HybridAStarNode current)
        {
            List<HybridAStarNode> children = new List<HybridAStarNode>();

            // Forward motions
            foreach (float steering in FORWARD_STEERING)
            {
                Vector3 newState = SimulateMotion(current.continuousState, steering, false);
                if (IsStateValid(newState))
                {
                    children.Add(CreateChildNode(current, newState, steering, false));
                }
            }

            return children;
        }

        private float EstimateSpeed(HybridAStarNode parent, Vector3 newState)
        {
            List<Node> path = new List<Node>();
            path.Add(new Node(newState.x, newState.y));
            HybridAStarNode curr = parent;
            while (true)
            {
                Node node = new Node(curr.continuousState.x, curr.continuousState.y);
                path.Add(node);
                if (curr.parent == null)
                {
                    break;
                }
                curr = curr.parent;
            }

            for (int i = 0; i < path.Count-1; i++)
            {
                path[i].parent = path[i + 1];
            }
            path.Reverse();
            Debug.Log(path.Count);

            if (path.Count <= 3)
            {
                return 15f;
            }
            return Controller.GetFinalSpeed(path);
        }
        
        private float EstimateSpeedForMotion(HybridAStarNode parent, Vector3 newState)
        {
            // Need three points for curvature: grandparent → parent → child
            if (parent == null || parent.parent == null)
                return 15f;  // Default speed at start of path
    
            Vector2 prev = new Vector2(parent.parent.continuousState.x, parent.parent.continuousState.y);
            Vector2 curr = new Vector2(parent.continuousState.x, parent.continuousState.y);
            Vector2 next = new Vector2(newState.x, newState.y);
    
            Vector2 enterDir = (curr - prev).normalized;
            Vector2 leaveDir = (next - curr).normalized;
            float angle = Vector2.Angle(enterDir, leaveDir);
    
            float dist = Mathf.Max(
                (Vector2.Distance(curr, prev) + Vector2.Distance(next, curr)) / 2f, 
                0.001f
            );
    
            float curvature = angle * Mathf.Deg2Rad / dist;
    
            if (curvature < 0.001f)
                return 30f;  // Straight sections - high speed
    
            // Kapania et al. formula
            float maxSpeed = Controller.CURV_CONST * Mathf.Sqrt(Controller.FRICTION * Controller.G / curvature);
    
            return Mathf.Clamp(maxSpeed, 1f, 30f);
        }
        
        private HybridAStarNode CreateChildNode(HybridAStarNode parent, Vector3 newState, float steering, bool isReverse)
        {
            // Estimate speed based on path curvature
            float estimatedSpeed = EstimateSpeed(parent, newState);
    
            // Calculate time cost 
            float distance = VELOCITY * DT;  // Distance traveled in this step
            float time = distance / estimatedSpeed;  // Time = distance / speed
    
            // Penalty for sharp steering 
            time += Mathf.Abs(steering) * 0.2f;  // Reduced penalty since speed already accounts for it
            
            return new HybridAStarNode 
            {
                continuousState = newState,
                discreteCell = StateToCell(newState),
                velocity = estimatedSpeed,
                isReverse = isReverse,
                steeringAngle = steering,
                parent = parent,
                gCost = parent.gCost + time,
                hCost = Heuristic(newState)
            };
        }

        private Vector3 SimulateMotion(Vector3 state, float steering, bool reverse)
        {
            float x = state.x;
            float y = state.y;
            float theta = state.z;

            float v = reverse ? -VELOCITY : VELOCITY;
            float delta = steering * MAX_STEER_ANGLE;

            // Bicycle kinematic model
            float x_new = x + v * Mathf.Cos(theta) * DT;
            float y_new = y + v * Mathf.Sin(theta) * DT;
            float theta_new = theta + (v / WHEELBASE) * Mathf.Tan(delta) * DT * TURN_MULTIPLIER;

            // Normalize angle to [-π, π]
            theta_new = Mathf.Atan2(Mathf.Sin(theta_new), Mathf.Cos(theta_new));

            return new Vector3(x_new, y_new, theta_new);
        }

        private bool IsStateValid(Vector3 state)
        {
            Vector3 pos3D = new Vector3(state.x, 0f, state.y);

            if (!ground.bounds.Contains(pos3D))
                return false;

            // Direct physics collision check
            return !Physics.CheckSphere(pos3D, COLLISION_RADIUS, obstacleLayer);
        }

        private Vector3 StateToCell(Vector3 state)
        {
            int x_cell = Mathf.RoundToInt(state.x / GRID_XY_RES);
            int y_cell = Mathf.RoundToInt(state.y / GRID_XY_RES);
            int theta_cell = Mathf.RoundToInt(state.z * Mathf.Rad2Deg / GRID_THETA_RES);

            return new Vector3(x_cell, y_cell, theta_cell);
        }

        private float Heuristic(Vector3 state)
        {
            int x = Mathf.Clamp(Mathf.RoundToInt((state.x - mapMin.x) / GRID_XY_RES), 0, gridWidth - 1);
            int y = Mathf.Clamp(Mathf.RoundToInt((state.y - mapMin.z) / GRID_XY_RES), 0, gridHeight - 1);

            float holonomicDist = holonomicDistances[x, y];
            
            // Convert distance to time estimate
            // Assume average speed of 15 m/s
            float estimatedTime = holonomicDist / 15f;
    
            return estimatedTime;
        }

        private bool IsGoalReached(Vector3 state)
        {
            return Vector2.Distance(new Vector2(state.x, state.y), new Vector2(goalPos.x, goalPos.z)) < GOAL_TOLERANCE;
        }

        private HybridAStarNode GetLowestFCost(List<HybridAStarNode> nodes)
        {
            HybridAStarNode lowest = nodes[0];
            float lowestF = lowest.gCost + lowest.hCost;

            for (int i = 1; i < nodes.Count; i++)
            {
                float f = nodes[i].gCost + nodes[i].hCost;
                if (f < lowestF)
                {
                    lowest = nodes[i];
                    lowestF = f;
                }
            }

            return lowest;
        }

        private void ReconstructPath(HybridAStarNode goalNode)
        {
            List<Node> path = new List<Node>();
            HybridAStarNode current = goalNode;

            while (current != null)
            {
                path.Add(new Node(current.continuousState.x, current.continuousState.y));
                current = current.parent;
            }

            path.Reverse();
            waypoints = path;
            Debug.Log($"Path reconstructed with {path.Count} waypoints");
        }
    }

    private class CGSmoother
    {
        private const float ALPHA = 0.00005f;
        public const float D_MAX = 2.4f;
        private const float W_COLLISION = 10000f;


        private const float W_CURVATURE = 5f;
        private const float W_SMOOTHNESS = 1f;
        public const int DISTANCE_MAP_RESOLUTION = 800;
        private const int MAX_OUTER_ITER = 5;
        private const int MAX_INNER_ITER = 50000;
        private const float MAX_GRADIENT = 100f;

        public CGSmoother(float carHeight, Collider map)
        {
            this.carHeight = carHeight;
            CreateDistanceMap(map);
        }

        private float carHeight { get; set; }
        public Vector2[][] distMap { get; set; }
        public Vector3 distStart { get; set; }
        public float stepX { get; set; }
        public float stepZ { get; set; }

        private void CreateDistanceMap(Collider map)
        {
            Vector3 mapOrigin = map.bounds.center;
            Vector3 mapExtents = map.bounds.extents;
            this.distStart = mapOrigin - mapExtents;
            this.stepX = mapExtents.x * 2 / (DISTANCE_MAP_RESOLUTION - 1);
            this.stepZ = mapExtents.z * 2 / (DISTANCE_MAP_RESOLUTION - 1);
            int obstacles = LayerMask.GetMask("Obstacle");

            this.distMap = new Vector2[DISTANCE_MAP_RESOLUTION][];
            for (int i = 0; i < DISTANCE_MAP_RESOLUTION; i++)
            {
                distMap[i] = new Vector2[DISTANCE_MAP_RESOLUTION];
            }

            for (int i = 0; i < DISTANCE_MAP_RESOLUTION; i++)
            {
                for (int j = 0; j < DISTANCE_MAP_RESOLUTION; j++)
                {
                    Vector3 pos = this.distStart + new Vector3(i * stepX, this.carHeight, j * stepZ);
                    distMap[i][j] = GetClosestObject(pos, obstacles);
                }
            }

            Debug.Log("Distance Map Done, Resolution = " + DISTANCE_MAP_RESOLUTION);
        }

        private Vector2 GetClosestObject(Vector3 pos, int obstacles)
        {
            float searchDist = 50f;
            float closestDistance = 50f;
            Vector2 closestPos = Vector2.positiveInfinity;
            Collider[] colliders = Physics.OverlapSphere(pos, searchDist, obstacles);

            foreach (Collider collider in colliders)
            {
                Vector3 obsPos;
                if (collider is MeshCollider) // ClosestPoint() Does not work, might need more work later
                {
                    Vector3 estimatedClosest = collider.bounds.center;
                    Vector3 estClosestFlat = new Vector3(estimatedClosest.x, this.carHeight, estimatedClosest.z);
                    Vector3 dir = (estClosestFlat - pos).normalized;
                    // Shoots a 2D line at the estimated position to increase likelihood of actually hitting the real position
                    if (Physics.BoxCast(pos, new Vector3(0.5f, 0.5f, 0.5f), dir,out RaycastHit hit, Quaternion.LookRotation(dir), 50f))
                    {
                        if (hit.collider.gameObject.name == "CarA1(Clone)" || hit.collider.gameObject.name == "startOverhead")
                        {
                            continue;
                        }
                        obsPos = hit.point;
                    }
                    else
                    {
                        obsPos = estimatedClosest;
                    }

                }
                else {
                    obsPos = collider.ClosestPoint(pos);
                }

                float distance = Vector3.Distance(obsPos, pos);
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPos = V3ToV2(obsPos);
                }
            }

            return closestPos;
        }

        private Vector2 GetDistMapEntry(Vector2 v)
        {
            int indexX = Mathf.RoundToInt(Mathf.Clamp((v.x - this.distStart.x) / this.stepX, 1f, DISTANCE_MAP_RESOLUTION-1));
            int indexZ = Mathf.RoundToInt(Mathf.Clamp((v.y - this.distStart.z) / this.stepZ, 1f, DISTANCE_MAP_RESOLUTION-1));
            return this.distMap[indexX][indexZ];
        }

        public List<Node> GetResampledPath(List<Node> path, float spacing)
        {
            List<Node> newPath = new List<Node>();
            newPath.Add(new Node(path[0].position.x, path[0].position.y));
            float currSpace = spacing;
            for (int i = 0; i < path.Count - 1; i++)
            {
                Vector2 curr = path[i].position;
                Vector2 next = path[i + 1].position;
                while (true)
                {
                    float dist = Vector2.Distance(curr, next);
                    if (dist < currSpace)
                    {
                        currSpace -= dist;
                        break;
                    }
                    Vector2 dir = (next - curr).normalized;
                    Vector2 newPos = curr + dir * currSpace;
                    Node newNode = new Node(newPos.x, newPos.y);
                    newPath.Add(newNode);
                    
                    currSpace = spacing;
                    curr = newNode.position;
                }
            }
            
            newPath.Add(new Node(path[path.Count - 1].position.x, path[path.Count - 1].position.y));
            
            for (int i = 1; i < newPath.Count; i++)
            {
                newPath[i].parent = newPath[i - 1];
            }
            
            return newPath;
        }

        public List<Node> GetSmoothedPath(List<Node> path)
        {
            int maxIter = MAX_OUTER_ITER;
            int iter = 0;
            List<float> targetSpeed;
            while (maxIter > iter)
            {
                path = GetResampledPath(path, 2);
                targetSpeed = Controller.GenerateTargetSpeeds(path);
                path = RunGradientDescent(path, targetSpeed);
                iter++;
            }

            return path;
        }

        public List<Node> RunGradientDescent(List<Node> path, List<float> targetSpeed)
        {
            // Gradient Descent
            int maxIter = MAX_INNER_ITER;
            float convergenceThreshold = 0.001f;
            for (int i = 0; i < maxIter; i++)
            {
                Vector2[] gradients = new Vector2[path.Count];
                for (int j = 1; j < path.Count - 1; j++)
                {
                    Vector2[] jGradient = CalculateGradient(path, j, targetSpeed[j]);
                    for (int k = 0; k < 3; k++)
                    {
                        gradients[k + j - 1] += jGradient[k];
                    }
                }

                float totalGradientMagnitude = 0;
                for (int j = 1; j < path.Count - 1; j++)
                {
                    totalGradientMagnitude += gradients[j].magnitude;

                     if (gradients[j].magnitude > MAX_GRADIENT)
                     {
                         gradients[j] = gradients[j].normalized*MAX_GRADIENT;
                     }
                    path[j].position -= ALPHA * (gradients[j]);
                }
                
                if (i % 1000 == 0)
                {
                    Debug.Log("Iteration " + i);
                    Debug.Log("Magnitude " + totalGradientMagnitude);
                }

                if (totalGradientMagnitude < convergenceThreshold)
                {
                    Debug.Log("Convergence");
                    Debug.Log("Iterations: " + i);
                    return path;
                }
            }

            Debug.Log("No Convergence");
            return path;
        }

        private Vector2[] CalculateGradient(List<Node> path, int index, float targetSpeed)
        {
            // Dolgov Paper
            Vector2 x0 = path[index - 1].position;
            Vector2 x1 = path[index].position;
            Vector2 x2 = path[index + 1].position;


            Vector2[] gradients = new Vector2[3];

            // Collision
            Vector2 o = GetNearestObstacle(x1);
            if ((!float.IsPositiveInfinity(o.x) && Vector2.Distance(o, x1) < D_MAX))
            {
                if (Vector2.Distance(o, x1) < 0.001f)
                {
                    o += new Vector2(0.01f, 0.01f);
                }

                gradients[1] += W_COLLISION * (2 * ((x1 - o).magnitude - D_MAX) * (x1 - o) / ((x1 - o).magnitude));
            }
            
            // Much simpler, also no singularity so more stable :)
            Vector2 laplace = x0 - 2 * x1 + x2;
            Vector2 dir = laplace.normalized;
            float adjustment = ((float)Math.Pow(laplace.magnitude*5, 5) + 4f * laplace.magnitude);
            
            // Adjust curvature weight based on target speed, dont need curvature if we are slow
            float wCurvature = W_CURVATURE-2.5f + Mathf.Sqrt(targetSpeed)/2;
            
            gradients[1] += wCurvature * -4 * dir * adjustment;
            gradients[0] += wCurvature * 2 * dir* adjustment;
            gradients[2] += wCurvature * 2 * dir* adjustment;


            // Smoothness (simple derivation, sign flipped due to negative x1)
            gradients[1] += W_SMOOTHNESS * (2 * (x1 - x0) + 2 * (x1 - x2));

            return gradients;
        }

        private Vector2 CalculateOrthogonal(Vector2 v1, Vector2 v2)
        {
            if (Vector2.Dot(v1, v2) < 0.0001)
            {
                // Avoid division by zero
                return v1;
            }

            // Orthogonal = v1 - projection(v1, v2)
            Vector2 projection = (Vector2.Dot(v1, v2) / (Vector2.Dot(v2, v2))) * v2;
            return v1 - projection;
        }

        private Vector2 GetNearestObstacle(Vector2 pos)
        {
            return GetDistMapEntry(pos);
        }

        private Vector2 V3ToV2(Vector3 v3)
        {
            return new Vector2(v3.x, v3.z);
        }
    }

    private class Controller
    {
        private const float K_P_STEERING = 0.6f;
        private const float K_D_STEERING = 0.3f;
        private const float K_P_SPEED = 1f;
        private const float K_D_SPEED = 0f;

        public const float FRICTION = 1f;
        public const float CURV_CONST = 1f;
        public const float G = 9.81f;

        public Controller(List<Node> waypoints, Vector3 goal, Transform carState)
        {
            this.steering = 0f;
            this.acceleration = 0.5f;
            this.footbrake = 1f;
            this.handbrake = 0f;
            this.waypoints = waypoints;
            this.targetDistance = 1;
            this.lastSteeringError = 0f;
            this.goal = goal;
            this.lastSpeedError = 0f;
            this.prevCarPos = carState.position;
            this.prevCarState = carState;
            this.targetSpeeds = GenerateTargetSpeeds(waypoints);
        }

        public float steering { get; set; }
        public float acceleration { get; set; }
        public List<Node> waypoints { get; set; }
        public float targetDistance { get; set; }
        public float footbrake { get; set; }
        public float handbrake { get; set; }
        public Vector2 closestPoint { get; set; }
        public Vector2 targetPoint { get; set; }
        public Vector3 prevCarPos { get; set; }
        public Transform prevCarState { get; set; }
        public float lastSteeringError { get; set; }
        public Vector3 goal { get; set; }
        public int bestStartIndex { get; set; }
        public float lastSpeedError { get; set; }
        public Transform currCarState { get; set; }
        public List<float> targetSpeeds { get; set; }

        public void StanleyCalculateMove(Transform carTransform)
        {
            this.currCarState = carTransform;
            StanleyCalculateSteer();
            // Still use PD for acceleration
            CalculateAcceleration();
            this.prevCarPos = carTransform.position;
        }

        private void StanleyCalculateSteer()
        {
            float distToFront = 4f;
            Vector3 frontAxlePos = this.currCarState.position + this.currCarState.forward*distToFront;
            (Vector2 closestPoint, int index) = GetClosestPointOnPath(frontAxlePos);

            index = Math.Clamp(index, 0, this.waypoints.Count - 2);
            Vector2 direction = (this.waypoints[index + 1].position - this.waypoints[index].position).normalized;

            float error = Vector3.SignedAngle(
                this.currCarState.forward,
                new Vector3(direction.x, 0, direction.y),
                new Vector3(0, 1, 0));
            
            Vector2 relativeTarget = this.currCarState.InverseTransformPoint(new Vector3(closestPoint.x, this.currCarState.position.y, closestPoint.y));
            float crossTrackError = relativeTarget.x;
            float carSpeed = Mathf.Max((this.currCarState.position-this.prevCarPos).magnitude/Time.fixedDeltaTime, 1f);
            float controlGain = 5f;
            float steeringContr = Mathf.Atan(crossTrackError * controlGain / carSpeed)*Mathf.Rad2Deg;
            
            float stanleySteer = error + steeringContr;
            
            float currentRotationDeriv = Mathf.DeltaAngle(this.currCarState.eulerAngles.y, this.prevCarState.eulerAngles.y)/Time.fixedDeltaTime;

            float damping = 0.5f;
            this.steering = stanleySteer + damping*currentRotationDeriv;
        }

        public void PDCalculateMove(Transform carTransform)
        {
            this.currCarState = carTransform;
            // Effectively two seperate PD Controllers
            UpdateTargetDistance();
            PDCalculateSteer();
            CalculateAcceleration();
            this.prevCarPos = carTransform.position;
            this.prevCarState = carTransform;
        }

        private void UpdateTargetDistance()
        {
            float currentSpeed = Vector3.Distance(this.currCarState.position, this.prevCarPos) / Time.fixedDeltaTime;
            this.targetDistance = Mathf.Clamp(currentSpeed / 10, 5, 20);
        }

        private void CalculateAcceleration()
        {
            float currentSpeed = Vector3.Distance(this.currCarState.position, prevCarPos) / Time.fixedDeltaTime;
            float targetSpeed = GetTargetSpeed(this.closestPoint, this.bestStartIndex, this.waypoints);

            targetSpeed *= 1.05f; // Want to avoid constantly accelerating and braking
            targetSpeed += 1;
            
            float error = (targetSpeed - currentSpeed);
            float acceleration = K_P_SPEED * error + K_D_SPEED * (error - this.lastSpeedError) / Time.fixedDeltaTime;

            if (acceleration < 0)
            {
                this.footbrake = 1f;
                this.acceleration = 0f;
            }
            else
            {
                this.acceleration = 1f;
                this.footbrake = 0f;
            }
            
            Debug.Log("Target Speed: " + targetSpeed);
            Debug.Log("Current Speed: " + currentSpeed);
            Debug.Log("Diff: " + (currentSpeed - targetSpeed));
            
            this.lastSpeedError = error;
        }

        private float GetTargetSpeed(Vector2 pos, int index, List<Node> path)
        {
            Vector2 prev = path[index].position;
            Vector2 next = path[index+1].position;
            float nextDist = Vector2.Distance(pos, next);
            float totalDist = Vector2.Distance(prev, next);
            float prevSpeed = this.targetSpeeds[index];
            float nextSpeed = this.targetSpeeds[index+1];
            // Linear interpolation
            float targetSpeed = nextDist/totalDist*prevSpeed + (1 - nextDist / totalDist)*nextSpeed;
            return targetSpeed;
        }

        public static float GetFinalSpeed(List<Node> path)
        {
            // Gets speed of the last node of a path
            List<float> speeds = GenerateTargetSpeeds(path);
            return speeds[speeds.Count - 1];
        }

        public static List<float> GenerateTargetSpeeds(List<Node> path)
        {
            // Kapania, Subosits, Gerdes "A Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories"
            List<float> speeds = new List<float>();
            speeds.Add(0);
            for (int i = 1; i < path.Count - 1; i++)
            {
                Vector2 prev = path[i - 1].position;
                Vector2 curr = path[i].position;
                Vector2 next = path[i + 1].position;
                
                Vector2 enterDir = (curr - prev).normalized;
                Vector2 leaveDir = (next - curr).normalized;
                float angle = Vector2.Angle(enterDir, leaveDir);
                float dist = Math.Max(Vector2.Distance(curr, prev) + Vector2.Distance(next, curr) / 2, 0.001f);
                float curvature = angle * Mathf.Deg2Rad / dist;
                curvature = Mathf.Max(Mathf.Pow(curvature+0.95f, 2f) - 1, curvature);
                if (curvature < 0.001f)
                {
                    speeds.Add(1000);
                    continue;
                }
                
                float maxSpeed = CURV_CONST*Mathf.Sqrt(FRICTION * G / curvature);
                speeds.Add(maxSpeed);
            }
            speeds[0] = 50;
            speeds.Add(1000);
            
            // Backwards
            for (int i = path.Count - 2; i >= 0; i--)
            {
                Vector2 curr = path[i].position;
                Vector2 next = path[i + 1].position;
                float dist =  Vector2.Distance(curr, next);
                
                float vNext = speeds[i + 1];

                // Values derived from AccelerationRecorder, Linear relationship
                float a = 0.103786f;
                float b = 0f;
                float decel = a * vNext + b;
                
                float vCurr = (float) Math.Sqrt(Math.Pow(vNext, 2) + 2*decel*dist);

                speeds[i] = Math.Min(vCurr, speeds[i]);
            }
            for (int i = 1; path.Count > i; i++)
            {
                Vector2 curr = path[i].position;
                Vector2 prev = path[i - 1].position;
                float dist =  Vector2.Distance(curr, prev);
                
                float vPrev = speeds[i - 1];
                
                float accel;
                if (vPrev <= 5.5)
                {
                    accel = 2.298f * Mathf.Pow(vPrev, 0.6582f);
                }
                else
                {
                    float a = -0.1041002f;
                    float b = 6.78387f;
                    accel = a * vPrev + b;
                }
                float vCurr = (float) Math.Sqrt(Math.Pow(vPrev, 2) + 2*accel*dist);

                speeds[i] = Math.Min(vCurr, speeds[i]);
            }
            
            return speeds;
        }

        private void PDCalculateSteer()
        {
            Vector2 currentPosition2D = new Vector2(this.currCarState.position.x, this.currCarState.position.z);
            Vector2 targetPoint = GetTargetPoint(this.currCarState.position);
            Vector2 direction = Vector2.Normalize(targetPoint - currentPosition2D);
            float error = Vector3.SignedAngle(
                this.currCarState.forward,
                new Vector3(direction.x, 0, direction.y),
                new Vector3(0, 1, 0));

            float derivative = (error - lastSteeringError) / Time.fixedDeltaTime;
            this.lastSteeringError = error;
            this.steering = error * K_P_STEERING + derivative * K_D_STEERING;
        }

        private Vector2 GetTargetPoint(Vector3 currentPosition)
        {
            var (closestPoint, startIndex) = GetClosestPointOnPath(currentPosition);

            // Choose correct line segment for the target
            float remainingDistance = this.targetDistance;
            remainingDistance -= Vector2.Distance(this.waypoints[startIndex + 1].position, closestPoint);
            while (true)
            {
                if (startIndex >= this.waypoints.Count - 2)
                {
                    // Reached the end
                    return this.waypoints[this.waypoints.Count - 1].position;
                }

                if (remainingDistance < 0)
                {
                    break;
                }

                startIndex++;
                remainingDistance -= Vector2.Distance(this.waypoints[startIndex].position,
                    this.waypoints[startIndex + 1].position);
            }

            remainingDistance += Vector2.Distance(this.waypoints[startIndex].position,
                this.waypoints[startIndex + 1].position);
            Node start = this.waypoints[startIndex];
            Node end = this.waypoints[startIndex + 1];
            Vector2 direction = Vector2.Normalize(end.position - start.position);
            Vector2 targetPoint = start.position + (direction * remainingDistance);
            this.targetPoint = targetPoint;
            return targetPoint;
        }

        private (Vector2, int) GetClosestPointOnPath(Vector3 currentPosition)
        {
            Vector2 closestPointPath = new Vector2(0f, 0f);
            float closestDistance = float.MaxValue;
            int bestIndex = 0;
            for (int i = 0; i < this.waypoints.Count - 1; i++)
            {
                Node start = this.waypoints[i];
                Node end = this.waypoints[i + 1];
                Vector2 closestPointLine = GetClosestPointToLine(start.position, end.position,
                    new Vector2(currentPosition.x, currentPosition.z));
                float distance = Vector2.Distance(closestPointLine, new Vector2(currentPosition.x, currentPosition.z));
                if (distance < closestDistance && !CollisionCheck(closestPointLine, currentPosition))
                {
                    closestPointPath = closestPointLine;
                    closestDistance = distance;
                    bestIndex = i;
                }
            }

            this.bestStartIndex = bestIndex;
            this.closestPoint = closestPointPath;
            return (closestPointPath, bestIndex);
        }

        private bool CollisionCheck(Vector2 start, Vector3 currentPosition)
        {
            // TODO: Might implement later
            return false;
            //return Physics.Linecast(
            //new Vector3(start.x, currentPosition.y, start.y),
            //currentPosition);
        }

        private Vector2 GetClosestPointToLine(Vector2 start, Vector2 end, Vector2 pos)
        {
            // Simple projection along the two nodes
            Vector2 segmentDirection = Vector2.Normalize(end - start);
            Vector2 relativePostion = pos - start;
            float magnitude = Vector2.Dot(segmentDirection, relativePostion);
            float cappedMagnitude = Mathf.Clamp(magnitude, 0f, (end - start).magnitude);
            return start + (segmentDirection * cappedMagnitude);
        }
    }

    private class PriorityQueue<T>
    {
        private List<(T item, float priority)> elements = new List<(T, float)>();

        public int Count => elements.Count;

        public void Enqueue(T item, float priority)
        {
            elements.Add((item, priority));
        }

        public T Dequeue()
        {
            int bestIndex = 0;
            for (int i = 1; i < elements.Count; i++)
            {
                if (elements[i].priority < elements[bestIndex].priority)
                    bestIndex = i;
            }
        
            T bestItem = elements[bestIndex].item;
            elements.RemoveAt(bestIndex);
            return bestItem;
        }
    }
    
    private List<Vector3> PlanPathAStar(Vector3 start, Vector3 goal)
{
    float gridSize = 2.0f;
    float carRadius = 0.9f;
    
    start = RoundToGrid(start, gridSize);
    goal = RoundToGrid(goal, gridSize);
    
    List<AStarNode> openSet = new List<AStarNode>();
    HashSet<Vector3> closedSet = new HashSet<Vector3>();
    
    AStarNode startNode = new AStarNode(start);
    startNode.gCost = 0;
    startNode.hCost = Vector3.Distance(start, goal);
    openSet.Add(startNode);
    
    int maxIterations = 50000;
    int iter = 0;
    
    while (openSet.Count > 0 && iter < maxIterations)
    {
        iter++;
        
        AStarNode currentNode = openSet.OrderBy(n => n.fCost).First();
        openSet.Remove(currentNode);
        closedSet.Add(currentNode.position);
        
        astarExploredNodes.Add(currentNode.position);
        
        if (Vector3.Distance(currentNode.position, goal) < gridSize * 1.5f)
        {
            Debug.Log($"A* found path in {iter} iterations");
            List<Vector3> path = ReconstructPath(currentNode);
            astarPath = path;  // Store for visualization
            return path;
        }
        
        foreach (Vector3 neighborPos in GetNeighbors(currentNode.position, gridSize))
        {
            if (closedSet.Contains(neighborPos))
                continue;
            
            if (!IsTraversableAStar(neighborPos, carRadius))
                continue;
            
            float tentativeGCost = currentNode.gCost + Vector3.Distance(currentNode.position, neighborPos);
            
            AStarNode neighborNode = openSet.FirstOrDefault(n => n.position == neighborPos);
            
            if (neighborNode == null)
            {
                neighborNode = new AStarNode(neighborPos);
                neighborNode.gCost = tentativeGCost;
                neighborNode.hCost = Vector3.Distance(neighborPos, goal);
                neighborNode.parent = currentNode;
                openSet.Add(neighborNode);
            }
            else if (tentativeGCost < neighborNode.gCost)
            {
                neighborNode.gCost = tentativeGCost;
                neighborNode.parent = currentNode;
            }
        }
    }
    
    Debug.LogError($"A* failed after {iter} iterations. Explored {astarExploredNodes.Count} nodes, OpenSet empty: {openSet.Count == 0}");
    return new List<Vector3> { start, goal };
}
    
    private Vector3 RoundToGrid(Vector3 pos, float gridSize)
{
    return new Vector3(
        Mathf.Round(pos.x / gridSize) * gridSize,
        pos.y,
        Mathf.Round(pos.z / gridSize) * gridSize
    );
}

    private List<Vector3> GetNeighbors(Vector3 pos, float gridSize)
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

    private bool IsTraversableAStar(Vector3 position, float radius)
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

    private List<Vector3> ReconstructPath(AStarNode endNode)
    {
        List<Vector3> path = new List<Vector3>();
        AStarNode current = endNode;
        
        while (current != null)
        {
            path.Add(current.position);
            current = current.parent;
        }
        
        path.Reverse();
        return path;
    }
}

public class AStarNode
{
    public float gCost;
    public float hCost;
    public AStarNode parent;
    public Vector3 position;

    public AStarNode(Vector3 pos)
    {
        position = pos;
    }

    public float fCost => gCost + hCost;
}
*/