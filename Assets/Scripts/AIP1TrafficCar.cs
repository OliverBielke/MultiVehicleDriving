using System.Collections.Generic;
using System.Linq;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using UnityEngine;
using Pathfinding;
using Pathfollowing;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : Agent
{
    public CarController car; // the car controller we want to use. Assigned in prefab

    public int priority = 0;
    private static int priorityCounter = 0;
    
    private GameObject[] _mOtherCars;
    private List<MultiVehicleGoal> _mCurrentGoals;

    public bool drawTargets;
    public bool drawAllCars;
    public bool drawTeamCars;

    public float steering;
    public float acceleration;
    public List<GameObject> targetObjects;
    public List<GameObject> teamVehicles;

    private Transform _initialCarState;
    private Controller _controller;
    private List<Node> _waypoints;
    
    // For reversal:
    private float stuckTimer = 0f;
    private bool isReversing = false;
    private float reverseTimer = 0f;
    private float stuckThreshold = 1.5f; // Seconds of stillness before reversing
    private float reverseDuration = 2.0f; // How long to reverse
    private Vector3 _lastPosition; // Used to check if we are actually stuck
    
    
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


        if (_waypoints != null && _waypoints.Count > 0)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < _waypoints.Count - 1; i++)
            {
                Vector3 start = new Vector3(_waypoints[i].position.x, 1f, _waypoints[i].position.y);
                Vector3 end = new Vector3(_waypoints[i + 1].position.x, 1f, _waypoints[i + 1].position.y);
                Gizmos.DrawLine(start, end);
                Gizmos.DrawSphere(start, 0.5f);
            }
            // Draw last waypoint
            Vector3 lastPos = new Vector3(_waypoints[_waypoints.Count - 1].position.x, 1f, _waypoints[_waypoints.Count - 1].position.y);
            Gizmos.DrawSphere(lastPos, 0.5f);
        }

        if (_controller != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(
                new Vector3(_controller.closestPoint.x, _initialCarState.position.y,
                    _controller.closestPoint.y), 0.5f);
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(
                new Vector3(_controller.targetPoint.x, _initialCarState.position.y,
                    _controller.targetPoint.y), 0.5f);
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
    
    
    public override void Initialize()
    {
        this.priority = priorityCounter;
        priorityCounter++;
        
        
        var swTotal = new Stopwatch();
        var swLocal = new Stopwatch();
        
        swTotal.Start();
        swLocal.Start();
        
        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        
        _mCurrentGoals = gameManagerA2.GetGoals(gameObject); // This car's goals. Can be multiple per vehicle!
        teamVehicles = gameManagerA2.GetGroupVehicles(gameObject); //Other vehicles in a Group with this vehicle
        _mOtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles
        GameObject groundPlane = GameObject.Find("GroundPlane");
        _initialCarState = gameObject.transform.Find("Colliders/ColliderBody").transform;
        
        Collider groundCollider = groundPlane.GetComponent<Collider>();
        Transform initialCarState = gameObject.transform.Find("Colliders/ColliderBody").transform;
        
        // Note that this array will have "holes" when objects are destroyed
        // But for initial planning they should work
        // If you dont like the "holes", you can re-fetch this during fixed update.

        // Where to go?
       
        // Equivalent ways to find all the targets in the scene
        targetObjects = _mCurrentGoals.Select(goal => goal.GetTargetObject()).ToList();

        // You can also fetch other types of objects using tags, assuming the objects you are looking for have tags assigned :).

        // Feel free to refer to any examples from previous assignments.

        Vector3 startPos = _initialCarState.position;
        Vector3 goalPos = targetObjects[0].transform.position;
        
        swLocal.Stop();
        Debug.Log($"Before anything: {swLocal.ElapsedMilliseconds} ms");
        
        swLocal.Restart();
        Astar astar = new();
        List<Vector3> astarPath = astar.PlanPathAStar(startPos, goalPos);
        
        
        if (astarPath.Count < 2)
        {
            Debug.LogError("A* failed - no path found");
            return;
        }
        
        Debug.Log($"A* found path with {astarPath.Count} waypoints");
        
        
        // Convert Vector3 path to Node list
        List<Node> nodes = new();
        foreach (Vector3 pos in astarPath)
        {
            nodes.Add(new Node(pos.x, pos.z));
        }
        swLocal.Stop();
        Debug.Log($"A* planning time: {swLocal.ElapsedMilliseconds} ms");
        
        swLocal.Restart();
        // Smoothes the waypoints
        CGSmoother smoother = new CGSmoother(_initialCarState.position.y, groundCollider);
        nodes = smoother.GetSmoothedPath(nodes);

        _waypoints = nodes;
        
        _lastPosition = transform.position; //For reversal
        
        // Creates the PD Controller
        _controller = new Controller(nodes, MapManager.GetGlobalGoalPosition(), initialCarState);
        _controller.priority = this.priority;
        swLocal.Stop();
        Debug.Log($"Smoothing and controller setup time: {swLocal.ElapsedMilliseconds} ms");

    }


    public override void Step()
    {
        // Execute your path and collision checking here
        // ...

        // Feel free to refer to any examples from previous assignments.

        //Example of cars moving into the centre of the field.
        /*Vector3 avg_pos = _mOtherCars.Aggregate(Vector3.zero, (sum, car) => sum + car.transform.position) / _mOtherCars.Length;


        Vector3 goal_pos = targetObjects[0].transform.position;
        
        (steering, acceleration) = ControlsTowardsPoint(avg_pos);
        (steering, acceleration) = ControlsTowardsPoint(goal_pos);

        car.Move(steering, acceleration, acceleration, 0f);*/
        
        float currentSpeed = (transform.position - _lastPosition).magnitude / Time.fixedDeltaTime;
        _lastPosition = transform.position;

        // if we are moving slow AND not already reversing
        if (currentSpeed < 0.5f && !_controller.isReversing) 
        {
            stuckTimer += Time.fixedDeltaTime;
            if (stuckTimer > stuckThreshold)
            {
                // initiate reverse then
                _controller.isReversing = true;
                _controller.reverseTimer = reverseDuration;
                stuckTimer = 0f;
            }
        }
        else if (currentSpeed >= 0.5f)
        {
            stuckTimer = 0f; // reset if we are moving normally
        }
        
        // Gets current car state
        Transform carTransform = gameObject.transform.Find("Colliders/ColliderBody").transform;
        
        // Calculates the move
        _controller.PDCalculateMove(carTransform);
        
        float finalSteering = _controller.steering;
        float finalAccel = _controller.acceleration;
        float finalBrake = _controller.footbrake;
        
        if (!_controller.isReversing)
        {
            float panicRadius = 10.0f;
            var (avoidSteer, avoidBrake) = LocalAvoidance.CalculateSeparation(carTransform, this.priority, _mOtherCars, panicRadius);
            
            finalSteering = Mathf.Clamp(_controller.steering + avoidSteer, -1f, 1f);
            
            finalAccel = avoidBrake > 0.1f ? 0f : _controller.acceleration;
            
            finalBrake = avoidBrake > 0.1f ? -1f : _controller.footbrake; 
        }
        
        car.Move(finalSteering, finalAccel, finalBrake, _controller.handbrake);    }

    private (float steering, float acceleration) ControlsTowardsPoint(Vector3 avg_pos)
    {
        Vector3 direction = (avg_pos - transform.position).normalized;

        bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
        bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

        float steering = 0f;
        float acceleration = 0;

        if (is_to_the_right && is_to_the_front)
        {
            steering = 1f;
            acceleration = 1f;
        }
        else if (is_to_the_right && !is_to_the_front)
        {
            steering = -1f;
            acceleration = -1f;
        }
        else if (!is_to_the_right && is_to_the_front)
        {
            steering = -1f;
            acceleration = 1f;
        }
        else if (!is_to_the_right && !is_to_the_front)
        {
            steering = 1f;
            acceleration = -1f;
        }

        float alpha = Mathf.Asin(Vector3.Dot(direction, transform.right));
        if (is_to_the_front && Mathf.Abs(alpha) < 1f)
        {
            steering = alpha;
        }

        return (steering, acceleration);
    }


    private void Update()
    {
        if (drawTargets)
        {
            foreach (var item in targetObjects)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.red);
            }
        }

        if (drawTeamCars)
        {
            foreach (var item in teamVehicles)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.blue);
            }
        }

        if (drawAllCars)
        {
            foreach (var item in _mOtherCars)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.yellow);
            }
        }
        //Debug.DrawLine(Vector3.zero, new Vector3(1, 0, 0), Color.red);
    }
}