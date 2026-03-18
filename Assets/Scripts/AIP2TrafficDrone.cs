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

    public GameObject currentTargetObject;
    
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
        
        _startupDelay = priority * StaggeredDelay;
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
        

        AssignNextGoal();
    }


    public override void Step()
    {
        _startupTimer += Time.fixedDeltaTime;
        
        
        Vector3 currentVelocity = (transform.position - _lastPosition) / Time.fixedDeltaTime;
        _lastPosition = transform.position;

        float finalH;
        float finalV;
        
        //Staggered start + if goals reached
        if (_startupTimer < _startupDelay || (_droneControlling != null && _droneControlling.HasReachedGoal && targetObjects.Count == 0))
        {
            // Force a complete stop, ignoring everything else
            finalH = 0f;
            finalV = 0f;
        }
        //If reached goal and needs to go to the next
        else if (_droneControlling != null && _droneControlling.HasReachedGoal && targetObjects.Count > 0)
        {
            // Goal reached, but we have more targets. Remove the completed one and reassign.
            if (currentTargetObject != null)
            {
                // 1. Remove from MY list
                targetObjects.Remove(currentTargetObject);
        
                // 2. Remove from ALL TEAMMATES' lists so they know it's done
                foreach (GameObject teammate in teamVehicles)
                {
                    if (teammate == this.gameObject) continue;
            
                    AIP2TrafficDrone mateScript = teammate.GetComponent<AIP2TrafficDrone>();
                    if (mateScript != null)
                    {
                        mateScript.targetObjects.Remove(currentTargetObject);
                    }
                }
            }
            AssignNextGoal();
            
            // Calculates the move
            _droneControlling.PDCalculateMove(droneTransform:_initialDroneState, drone:mDrone);
        
            finalH = _droneControlling.h;
            finalV = _droneControlling.v;
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
    
    
    // ADDED: Extracted goal choosing and pathfinding into a reusable method
    private void AssignNextGoal()
    {
        if (targetObjects == null || targetObjects.Count == 0) return;

        // Find out which targets are already claimed by teammates
        List<GameObject> claimedTargets = new();
        foreach (var teammate in teamVehicles)
        {
            if (teammate == this.gameObject) continue; // Don't check ourselves
        
            var mateScript = teammate.GetComponent<AIP2TrafficDrone>();
            if (mateScript != null && mateScript.currentTargetObject != null)
            {
                claimedTargets.Add(mateScript.currentTargetObject);
            }
        }

        // Filter our list to only include UNCLAIMED targets
        var availableTargets = targetObjects.Except(claimedTargets).ToList();
    
        // Fallback: If there are more drones than targets, availableTargets will be empty. 
        // In that case, just use the full list so they don't get stuck doing nothing.
        if (availableTargets.Count == 0)
        {
            availableTargets = targetObjects;
        }
        
        var startPos = _initialDroneState.position;
        var goalChoose = new GoalChoosing(availableTargets, teamVehicles, _initialDroneState);
        var goalPos = goalChoose.GetGoalPosition();
        
        // Identify which object was chosen so we can remove it once we reach it
        currentTargetObject = targetObjects.OrderBy(t => Vector3.Distance(t.transform.position, goalPos)).FirstOrDefault();
        
        Astar astar = new();
        List<Vector3> astarPath = astar.PlanPathAStar(startPos, goalPos);
        
        if (astarPath.Count < 2)
        {
            Debug.LogError("A* failed - no path found for drone");
            return;
        }
        
        List<Node> nodes = new();
        foreach (Vector3 pos in astarPath)
        {
            nodes.Add(new Node(pos.x, pos.z));
        }
        
        GameObject groundPlane = GameObject.Find("GroundPlane");
        Collider groundCollider = groundPlane.GetComponent<Collider>();
        CGSmoother smoother = new CGSmoother(_initialDroneState.position.y, groundCollider);
        nodes = smoother.GetSmoothedPath(nodes);

        _waypoints = nodes;
        
        // Creates the new PD Controller for the new path
        _droneControlling = new DroneControlling(nodes, goalPos, _initialDroneState);
    }
}