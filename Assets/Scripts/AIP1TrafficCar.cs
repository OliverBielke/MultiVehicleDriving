using System.Collections.Generic;
using System.Linq;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using UnityEngine;
using PathFinding;
using PathFollowing;
using Debug = UnityEngine.Debug;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : Agent
{
    public CarController car; // the car controller we want to use. Assigned in prefab
    public GameObject currentTargetObject;
    
    public int priority = 0;
    private static int priorityCounter = 0;
    private float _maxAcceleration;
    
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
    private CarControlling _carControlling;
    public List<Node> Waypoints { get; private set; }
    
    // For reversal:
    private float stuckTimer = 0f;
    private bool isReversing = false;
    private float reverseTimer = 0f;
    private float stuckThreshold = 1.5f; // Seconds of stillness before reversing
    private float reverseDuration = 2.0f; // How long to reverse
    private Vector3 _lastPosition; // Used to check if we are actually stuck
    
    // Staggered start
    private float _startupDelay;
    private float _startupTimer = 0f;
    
    private void OnDrawGizmos()
    {
        if (Waypoints != null && Waypoints.Count > 0)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < Waypoints.Count - 1; i++)
            {
                Vector3 start = new Vector3(Waypoints[i].position.x, 1f, Waypoints[i].position.y);
                Vector3 end = new Vector3(Waypoints[i + 1].position.x, 1f, Waypoints[i + 1].position.y);
                Gizmos.DrawLine(start, end);
                Gizmos.DrawSphere(start, 0.5f);
            }
            // Draw last waypoint
            Vector3 lastPos = new Vector3(Waypoints[Waypoints.Count - 1].position.x, 1f, Waypoints[Waypoints.Count - 1].position.y);
            Gizmos.DrawSphere(lastPos, 0.5f);
        }

        if (_carControlling != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(
                new Vector3(_carControlling.closestPoint.x, _initialCarState.position.y,
                    _carControlling.closestPoint.y), 0.5f);
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(
                new Vector3(_carControlling.targetPoint.x, _initialCarState.position.y,
                    _carControlling.targetPoint.y), 0.5f);
        }
    }
    
    
    public override void Initialize()
    {
        priority = priorityCounter;
        priorityCounter++;
        _maxAcceleration = 5f;
        
        _startupDelay = priority * 1.5f;
        _startupTimer = 0f;
        
        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        
        _mCurrentGoals = gameManagerA2.GetGoals(gameObject); // This car's goals. Can be multiple per vehicle!
        teamVehicles = gameManagerA2.GetGroupVehicles(gameObject); //Other vehicles in a Group with this vehicle
        _mOtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles
        
        _initialCarState = gameObject.transform.Find("Colliders/ColliderBottom").transform;
        
        // Equivalent ways to find all the targets in the scene
        targetObjects = _mCurrentGoals.Select(goal => goal.GetTargetObject()).ToList();
        
        _lastPosition = transform.position; //For reversal
        
        AssignNextGoal();
    }


    public override void Step()
    {
        
        _startupTimer += Time.fixedDeltaTime;
        if (_startupTimer < _startupDelay)
        {
            // Keep the car completely stationary
            car.Move(0f, 0f, 1f, 1f);
            
            // We also need to keep updating _lastPosition so the "stuck" reversing logic 
            // doesn't trigger immediately after the delay finishes.
            _lastPosition = transform.position; 
            return;
        }
        
        
        float currentSpeed = (transform.position - _lastPosition).magnitude / Time.fixedDeltaTime;
        Vector3 currentVelocity = (transform.position - _lastPosition) / Time.fixedDeltaTime;
        _lastPosition = transform.position;

        // if we are moving slow AND not already reversing
        if (currentSpeed < 0.1f && !_carControlling.isReversing && !_carControlling.HasReachedGoal) 
        {
            stuckTimer += Time.fixedDeltaTime;
            if (stuckTimer > stuckThreshold)
            {
                // initiate reverse then
                _carControlling.isReversing = true;
                _carControlling.reverseTimer = reverseDuration;
                stuckTimer = 0f;
            }
        }
        else if (currentSpeed >= 0.5f || stuckTimer > 10f) // if we are moving or have been stuck for a long time, reset the timer and stop reversing
        {
            stuckTimer = 0f; // reset if we are moving normally
        }
        
        // Gets current car state
        var carTransform = gameObject.transform.Find("Colliders/ColliderBottom").transform;
        
        // Calculates the move
        _carControlling.PDCalculateMove(carTransform);
        
        var finalSteering = _carControlling.steering;
        var finalAccel = _carControlling.acceleration;
        var finalBrake = _carControlling.footbrake;
        var finalHandbrake = _carControlling.handbrake;
        
        if (_carControlling.HasReachedGoal && targetObjects.Count == 0)
        {
            // Force a complete stop, ignoring everything else
            finalSteering = 0f;
            finalAccel = 0f;
            finalBrake = 0f;
            finalHandbrake = 0f;
        }
        else if (_carControlling.HasReachedGoal && targetObjects.Count > 0)
        {
            // Goal reached, but we have more targets. Remove the completed one and reassign.
            if (currentTargetObject != null)
            {
                targetObjects.Remove(currentTargetObject);
    
                foreach (GameObject teammate in teamVehicles)
                {
                    if (teammate == gameObject) continue;
        
                    AIP1TrafficCar mateScript = teammate.GetComponent<AIP1TrafficCar>();
                    if (mateScript != null)
                    {
                        mateScript.targetObjects.Remove(currentTargetObject);
                    }
                }
            }
            AssignNextGoal();
        
            // Recalculate move for the newly generated path
            _carControlling.PDCalculateMove(carTransform);
            finalSteering = _carControlling.steering;
            finalAccel = _carControlling.acceleration;
            finalBrake = _carControlling.footbrake;
            finalHandbrake = _carControlling.handbrake;
        }
        /*if (!_carControlling.isReversing && !_carControlling.HasReachedGoal)
        {
            // Pass the ego car (this) so PathStop can read the Waypoints
            var pathStop = new PathStop(carTransform, this, _mOtherCars);
    
            (finalAccel, finalSteering, finalBrake) = pathStop.GetAdjustedControls(
                currentVelocity, 
                finalSteering, 
                finalBrake, 
                finalAccel
            );
    
            finalSteering = Mathf.Clamp(finalSteering, -1f, 1f);
            
        }*/
        
        car.Move(finalSteering, finalAccel, finalBrake, finalHandbrake);
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
    }
    
    private void AssignNextGoal()
    {
        if (targetObjects == null || targetObjects.Count == 0) return;

        // Find out which targets are already claimed by teammates
        List<GameObject> claimedTargets = new();
        foreach (var teammate in teamVehicles)
        {
            if (teammate == this.gameObject) continue;
    
            var mateScript = teammate.GetComponent<AIP1TrafficCar>();
            if (mateScript != null && mateScript.currentTargetObject != null)
            {
                claimedTargets.Add(mateScript.currentTargetObject);
            }
        }

        // Filter our list to only include UNCLAIMED targets
        var availableTargets = targetObjects.Except(claimedTargets).ToList();

        if (availableTargets.Count == 0)
        {
            availableTargets = targetObjects;
        }
    
        var startPos = _initialCarState.position;
        var goalChoose = new GoalChoosing(availableTargets, teamVehicles, _initialCarState);
        var goalPos = goalChoose.GetGoalPosition();
    
        currentTargetObject = targetObjects.OrderBy(t => Vector3.Distance(t.transform.position, goalPos)).FirstOrDefault();
    
        Astar astar = new();
        List<Vector3> astarPath = astar.PlanPathAStar(startPos, goalPos);
    
        if (astarPath.Count < 2)
        {
            Debug.LogError("A* failed - no path found for car");
            return;
        }
    
        List<Node> nodes = new();
        foreach (Vector3 pos in astarPath)
        {
            nodes.Add(new Node(pos.x, pos.z));
        }
    
        GameObject groundPlane = GameObject.Find("GroundPlane");
        Collider groundCollider = groundPlane.GetComponent<Collider>();
        CGSmoother smoother = new CGSmoother(_initialCarState.position.y, groundCollider);
        nodes = smoother.GetSmoothedPath(nodes);

        Waypoints = nodes;
    
        // Creates the new PD Controller for the new path
        _carControlling = new CarControlling(nodes, goalPos, _initialCarState);
        _carControlling.priority = this.priority;
    }
}