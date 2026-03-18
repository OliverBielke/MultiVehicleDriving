using System.Collections.Generic;
using System.Linq;
using FormationGame;
using Scripts.Game;
using Scripts.Vehicle;
using UnityEngine;
using PathFinding;
using PathFollowing;
using Debug = UnityEngine.Debug;

[RequireComponent(typeof(DroneControlling))]
public class AIP2TrafficDrone : Agent
{
    public DroneController mDrone; // the drone controller we want to use. Assigned in prefab

    public int priority;
    private static int _priorityCounter;
    private float _maxAcceleration;
    
    private GameObject[] _mOtherVehicles;
    private List<MultiVehicleGoal> _mCurrentGoals;

    public List<GameObject> targetObjects;
    public List<GameObject> teamVehicles;

    private Transform _initialDroneState;
    private DroneControlling _droneControlling;
    private List<Node> _waypoints;
    
    // For reversal/stuck logic:
    private float stuckTimer = 0f;
    private float reverseTimer = 0f;
    private float stuckThreshold = 1.5f; // Seconds of stillness before reversing
    private float reverseDuration = 2.0f; // How long to reverse
    private Vector3 _lastPosition; // Used to check if we are actually stuck
    
    // Staggered start
    private float _startupDelay;
    private float _startupTimer;
    private const float StaggeredDelay = 5f;
    
    
    private void OnDrawGizmos()
    {
        if (_waypoints != null && _waypoints.Count > 0)
        {
            Gizmos.color = Color.cyan;
            for (int i = 0; i < _waypoints.Count - 1; i++)
            {
                Vector3 start = new Vector3(_waypoints[i].position.x, transform.position.y, _waypoints[i].position.y);
                Vector3 end = new Vector3(_waypoints[i + 1].position.x, transform.position.y, _waypoints[i + 1].position.y);
                Gizmos.DrawLine(start, end);
                Gizmos.DrawSphere(start, 0.3f);
            }
            // Draw last waypoint
            Vector3 lastPos = new Vector3(_waypoints[_waypoints.Count - 1].position.x, transform.position.y, _waypoints[_waypoints.Count - 1].position.y);
            Gizmos.DrawSphere(lastPos, 0.4f);
        }

        if (_droneControlling != null && _initialDroneState != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(
                new Vector3(_droneControlling.closestPoint.x, _initialDroneState.position.y,
                    _droneControlling.closestPoint.y), 0.5f);
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(
                new Vector3(_droneControlling.targetPoint.x, _initialDroneState.position.y,
                    _droneControlling.targetPoint.y), 0.5f);
        }
    }
    
    public override void Initialize()
    {
        priority = _priorityCounter;
        _priorityCounter++;
        _maxAcceleration = 5f; 
        
        _startupDelay = this.priority * StaggeredDelay;
        _startupTimer = 0f;
        
        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        
        _mCurrentGoals = gameManagerA2.GetGoals(gameObject); 
        teamVehicles = gameManagerA2.GetGroupVehicles(gameObject); 
        _mOtherVehicles = GameObject.FindGameObjectsWithTag("Player"); 
        
        GameObject groundPlane = GameObject.Find("GroundPlane");
        Collider groundCollider = groundPlane.GetComponent<Collider>();

        // Fallback to standard transform if Colliders/ColliderBottom doesn't exist on the drone
        Transform colliderBottom = gameObject.transform.Find("Colliders/ColliderBottom");
        _initialDroneState = colliderBottom != null ? colliderBottom : gameObject.transform;
        
        targetObjects = _mCurrentGoals.Select(goal => goal.GetTargetObject()).ToList();

        if (targetObjects.Count == 0) return;

        Vector3 startPos = _initialDroneState.position;
        //Vector3 goalPos = targetObjects[0].transform.position;
        var goalChoose = new GoalChoosing(targetObjects, teamVehicles, _initialDroneState);
        var goalPos = goalChoose.GetGoalPosition();
        
        Astar astar = new();
        List<Vector3> astarPath = astar.PlanPathAStar(startPos, goalPos);
        
        if (astarPath.Count < 2)
        {
            Debug.LogError("A* failed - no path found for drone");
            return;
        }
        
        // Convert Vector3 path to Node list
        List<Node> nodes = new();
        foreach (Vector3 pos in astarPath)
        {
            nodes.Add(new Node(pos.x, pos.z));
        }
        
        // Smoothes the waypoints
        CGSmoother smoother = new CGSmoother(_initialDroneState.position.y, groundCollider);
        nodes = smoother.GetSmoothedPath(nodes);

        _waypoints = nodes;
        _lastPosition = transform.position; 
        
        // Creates the PD Controller
        _droneControlling = new DroneControlling(nodes, goalPos, _initialDroneState);
    }


    public override void Step()
    {
        _startupTimer += Time.fixedDeltaTime;
        
        
        Vector3 currentVelocity = (transform.position - _lastPosition) / Time.fixedDeltaTime;
        _lastPosition = transform.position;

        float finalH;
        float finalV;
        
        if (_droneControlling.HasReachedGoal || _startupTimer < _startupDelay)
        {
            // Force a complete stop, ignoring everything else
            finalH = 0f;
            finalV = 0f;
        }
        else
        {
            // Calculates the move
            _droneControlling.PDCalculateMove(droneTransform:_initialDroneState, drone:mDrone);
        
            finalH = _droneControlling.h;
            finalV = _droneControlling.v;
        }
        
        
        var vo = new VO(vehicleTransform:_initialDroneState, mDrone.max_acceleration);
        
        Collider[] obstacles = Physics.OverlapSphere(transform.position, 20f, LayerMask.GetMask("Obstacle"));
        GameObject[] pedestrians = GameObject.FindGameObjectsWithTag("Searcher");
        
        (finalH, finalV) = vo.GetSafeAcceleration(myTransform:_initialDroneState, currentVelocity:currentVelocity, 
            intendedH:finalH, intendedV:finalV, otherDrones:_mOtherVehicles, pedestrians:pedestrians, staticObstacles:obstacles);
        
        // Drones only take 2 variables: Steering (turn) and Acceleration (forward)
        mDrone.Move(finalH, finalV);
    }
}