using UnityEngine;
using System.Collections.Generic;
using Scripts.Vehicle;

namespace PathFollowing
{

    /// <summary>
    /// The state of a vehicle for the VO algorithm.
    /// </summary>
    public struct VehicleState
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public Vector2 Forward;
        public float Radius;
    }

    public class VOStop
    {
        // How many seconds into the future we look for collisions
        private const float TimeHorizon = 10f;
        private const float SpaceMargin = 1f; // Extra radius to add to each vehicle to create a safety buffer. Adjust based on your vehicle sizes and desired safety margin.
        private const float MaxSpeed = 5f;

        private readonly Transform _myTransform;
        private readonly GameObject[] _otherCars;

        /// <summary>
        /// Initializes the VOStop module with the necessary parameters.
        /// </summary>
        /// <param name="vehicleTransform">The transform of the vehicle. </param>
        /// <param name="otherCars">List of all the other cars.</param>
        public VOStop(Transform vehicleTransform, GameObject[] otherCars)
        {
            _myTransform = vehicleTransform;
            _otherCars = otherCars;
        }


        /// <summary>
        /// Adjusts intended controls based on Velocity Obstacle collision checks.
        /// </summary>
        /// <param name="currentVelocity">The current velocity of our vehicle. </param>
        /// <param name="intendedSteer">The intended steering input (before adjustment). </param>
        /// <param name="intendedBrake">The intended brake input (before adjustment). </param>
        /// <param name="intendedAccel">The intended acceleration input (before adjustment). </param>
        /// <returns>Adjusted controls as a tuple: (finalAccel, finalSteering, finalBrake). </returns>
        public (float finalAccel, float finalSteering, float finalBrake) GetAdjustedControls(
            Vector3 currentVelocity,
            float intendedSteer,
            float intendedBrake,
            float intendedAccel
            )
        {
            
            // Build the nodes for our own vehicle
            var egoNodes = GetVehicleNodes(_myTransform, currentVelocity);
            var surroundingNodes = GetSurroundingVehicleNodes();

            bool shouldStop = false;

            // If ANY of our vehicle's circles are on a collision course with ANY obstacle circle, we stop.
            foreach (var egoNode in egoNodes)
            {
                if (EvaluateShouldStop(egoNode, surroundingNodes))
                {
                    shouldStop = true;
                    break;
                }
            }

            if (shouldStop || currentVelocity.magnitude > MaxSpeed)
            {
                if (currentVelocity.magnitude < 0.1f)
                {
                    return (0f, intendedBrake, 0f);
                }
                
                return (0f, intendedSteer, -1f);
            }

            return (intendedAccel, intendedSteer, intendedBrake);
        }

        
        
        /// <summary>
        /// Adjusts intended controls for a drone (omnidirectional) based on Velocity Obstacles.
        /// </summary>
        /// <param name="currentVelocity">The current velocity of the drone.</param>
        /// <param name="intendedH">The intended horizontal input (X-axis).</param>
        /// <param name="intendedV">The intended vertical input (Z-axis).</param>
        /// <returns>Adjusted controls as a tuple: (finalH, finalV).</returns>
        public (float finalH, float finalV) GetAdjustedDroneControls(
            Vector3 currentVelocity,
            float intendedH,
            float intendedV, 
            DroneController drone
        )
        {
            var egoNodes = GetVehicleNodes(_myTransform, currentVelocity);
            var surroundingNodes = GetSurroundingVehicleNodes();

            bool shouldStop = false;

            // Check for collisions
            foreach (var egoNode in egoNodes)
            {
                if (EvaluateShouldStop(egoNode, surroundingNodes))
                {
                    shouldStop = true;
                    break;
                }
            }

            if (shouldStop)
            {
                // If we are already moving very slowly, just hover (0 input)
                if (currentVelocity.magnitude < 0.1f)
                {
                    return (0f, 0f);
                }
                
                // Active Braking: Apply thrust in the exact opposite direction of our velocity
                Vector3 stoppingDir = -currentVelocity.normalized * drone.max_acceleration;
                return (stoppingDir.x, stoppingDir.z); 
            }

            // Path is clear, proceed with intended inputs
            return (intendedH, intendedV);
        }
        
        
        /// <summary>
        /// Get the states of all surrounding vehicles based on the provided radius. This assumes all vehicles are circles with the same radius for simplicity.
        /// </summary>
        /// <returns>A list of all surrounding vehicle states. </returns>
        private List<VehicleState> GetSurroundingVehicleNodes()
        {
            var allNodes = new List<VehicleState>();

            foreach (var car in _otherCars)
            {
                // Safety check: Skip ourselves if we accidentally ended up in the _otherCars array!
                if (car.transform == _myTransform) continue;
                
                var rb = car.GetComponent<Rigidbody>();
                var velocity = rb != null ? rb.linearVelocity : Vector3.zero;

                // Generate the 1 or 2 nodes for this specific obstacle
                var obstacleNodes = GetVehicleNodes(car.transform, velocity);
                allNodes.AddRange(obstacleNodes);
            }

            return allNodes;
        }
        
        
        /// <summary>
        /// Approximates a vehicle's shape using 1 or more circular nodes.
        /// </summary>
        /// <param name="vehicleTransform">The transform of the vehicle. </param>
        /// <param name="currentVelocity">Current velocity of the vehicle. </param>
        private List<VehicleState> GetVehicleNodes(Transform vehicleTransform, Vector3 currentVelocity)
        {
            var nodes = new List<VehicleState>();
            var vel2D = new Vector2(currentVelocity.x, currentVelocity.z);
            
            Vector2 forward2D;
            if (currentVelocity.sqrMagnitude > 0.1f)
            {
                forward2D = vel2D.normalized; //Velocity direction as forward direction
            }
            else
            {
                forward2D = new Vector2(vehicleTransform.forward.x, vehicleTransform.forward.z).normalized; //Fallback to transform forward if we are nearly stationary
            }
        
            var pos2D = new Vector2(vehicleTransform.position.x, vehicleTransform.position.z);

            // Get the local dimensions (avoids the world-space rotation bounds bug)
            var boxCol = vehicleTransform.GetComponent<BoxCollider>();
            float halfWidth, halfLength;

            if (boxCol != null)
            {
                // For rectangular cars
                halfWidth = (boxCol.size.x * vehicleTransform.lossyScale.x) / 2f;
                halfLength = (boxCol.size.z * vehicleTransform.lossyScale.z) / 2f;
            }
            else
            {
                // Fallback for drones (assuming SphereCollider or roughly circular)
                var capsuleCol = vehicleTransform.GetComponent<CapsuleCollider>();
                halfWidth = capsuleCol != null ? (capsuleCol.radius * vehicleTransform.lossyScale.x) : 1f;
                halfLength = halfWidth; 
            }

            // Now our radius is strictly based on width, keeping us in our lane!
            float radius = halfWidth + SpaceMargin;

            if (halfLength <= halfWidth * 1.2f)
            {
                // Shape is roughly square/circular (Drone). One node is sufficient.
                nodes.Add(new VehicleState { Position = pos2D, Velocity = vel2D, Forward = forward2D, Radius = radius });
            }
            else
            {
                // Shape is a rectangle (Car). Create a front and back node.
                float offset = halfLength - halfWidth; 
                
                // Front Circle Node
                nodes.Add(new VehicleState { 
                    Position = pos2D + forward2D * offset, 
                    Velocity = vel2D, Forward = forward2D, Radius = radius 
                });
                
                // Back Circle Node
                nodes.Add(new VehicleState { 
                    Position = pos2D - forward2D * offset, 
                    Velocity = vel2D, Forward = forward2D, Radius = radius 
                });
                
                // Note: If you have massive trucks, you could add a 3rd middle circle here.
            }

            return nodes;
        }
        
        
        /// <summary>
        /// Evaluates surrounding traffic to determine if this vehicle must stop.
        /// </summary>
        /// <param name="thisVehicle">The state of the vehicle. </param>
        /// <param name="surroundingVehicles">The states of all surrounding vehicles. </param>
        /// <returns>True if an imminent collision is detected and this vehicle must yield; otherwise, false.</returns>
        private static bool EvaluateShouldStop(VehicleState thisVehicle, List<VehicleState> surroundingVehicles)
        { 
            
            foreach (var obstacle in surroundingVehicles)
            {
                // Calculate relative position.
                // This is the vector pointing exactly from our center to the obstacle's center.
                var relativePosition = obstacle.Position - thisVehicle.Position;

                // Calculate relative velocity. 
                // By subtracting the obstacle's velocity from ours, we can treat the obstacle as stationary.
                var relativeVelocity = thisVehicle.Velocity - obstacle.Velocity;

                // Calculate the combined radius. 
                // This inflates the obstacle's size so we can mathematically treat ThisVehicle as a single point.
                var combinedRadius = thisVehicle.Radius + obstacle.Radius;

                if (HasRightOfWay(thisVehicle, obstacle))
                {
                    // We have right of way over this obstacle, so we can ignore it for collision checking.
                    DrawDebugVO(thisVehicle, relativePosition, relativeVelocity, combinedRadius, Color.cyan);
                    continue;
                }

                // Calculate Time to Collision (TTC)
                // Starting from the equation ||V*t - P|| = R, we derive a quadratic formula to solve for t (time until collision).
                // Quadratic equation: a*t^2 + b*t + c = 0
                var a = relativeVelocity.sqrMagnitude;
                var b = -2f * Vector2.Dot(relativeVelocity, relativePosition);
                var c = relativePosition.sqrMagnitude - combinedRadius * combinedRadius;

                // If a is near zero, relative velocity is zero (we are matching speeds perfectly)
                if (a < 0.0001f) continue;

                // Convert to p-q form: t^2 + (b/a)*t + (c/a) = 0
                var p = b / a;
                var q = c / a;

                var inSqrt = (p * p / 4f) - q;

                if (inSqrt < 0)
                {
                    // No real roots means no collision
                    DrawDebugVO(thisVehicle, relativePosition, relativeVelocity, combinedRadius, Color.green);
                    continue;
                }

                var sqrtTerm = Mathf.Sqrt(inSqrt);

                // Get the collision times
                var t1 = (-p / 2f) - sqrtTerm;
                var t2 = (-p / 2f) + sqrtTerm;

                // We want the smallest positive time
                var t = -1f;
                if (t1 >= 0 && (t2 < 0 || t1 < t2)) t = t1;
                else if (t2 >= 0) t = t2;

                // If the collision happens in the future, and within our time horizon
                if (t is >= 0 and <= TimeHorizon)
                {
                    DrawDebugVO(thisVehicle, relativePosition, relativeVelocity, combinedRadius, Color.red);
                    return true; // Imminent collision detected!
                }
                if (t > TimeHorizon)
                {
                    // Collision course, but outside the Time Horizon (Safe for now)
                    DrawDebugVO(thisVehicle, relativePosition, relativeVelocity, combinedRadius, Color.yellow);
                }
                
            }

            return false;
        }


        private static bool HasRightOfWay(VehicleState thisVehicle, VehicleState obstacle)
        {
            // Dot Product to check if the obstacle is in front of us
            var relativePosition = obstacle.Position - thisVehicle.Position;
            var dotProduct = Vector2.Dot(thisVehicle.Forward, relativePosition);
            var isInFront = dotProduct > 0;

            // We never brake for cars behind us; it is their responsibility to brake for us.
            if (!isInFront)
            {
                return true; // We have right of way over cars behind us
            }
            
            // 2D Cross Product to check if the obstacle is to our right
            var crossProduct = (thisVehicle.Forward.x * relativePosition.y) - (thisVehicle.Forward.y * relativePosition.x);
            var isToRight = crossProduct < 0;

            return isToRight; // We have right of way if the other car is to our right
        }
        
        
        /// <summary>
        /// Draws the VO for debugging. 
        /// </summary>
        /// <param name="thisVehicle">The state of the current vehicle. </param>
        /// <param name="relativePosition">The relative positions between the vehicles. </param>
        /// <param name="relativeVelocity">The relative velocity between the vehicles. </param>
        /// <param name="combinedRadius">The combined radius of the vehicles. </param>
        private static void DrawDebugVO(VehicleState thisVehicle, Vector2 relativePosition, 
            Vector2 relativeVelocity, float combinedRadius, Color color)
        {
            const float drawHeight = 1f; 
            const float furthestDrawDistance = 60f;
            const float drawScale = 0.5f;
    
            var dist = relativePosition.magnitude;
            if (dist <= combinedRadius || dist > furthestDrawDistance) return;
    
            var egoPos3D = new Vector3(thisVehicle.Position.x, drawHeight, thisVehicle.Position.y);
        
            // 1. Draw the VO Cone to the obstacle's distance
            var angle = Mathf.Asin(combinedRadius / dist) * Mathf.Rad2Deg;
            var centerLine = new Vector3(relativePosition.x, 0, relativePosition.y);
        
            var leftTangent = Quaternion.Euler(0, -angle, 0) * centerLine;
            var rightTangent = Quaternion.Euler(0, angle, 0) * centerLine;

            // Draw the cone stretching exactly to the distance of the obstacle
            Vector3 leftPoint = egoPos3D + leftTangent*drawScale;
            Vector3 rightPoint = egoPos3D + rightTangent*drawScale;

            Debug.DrawLine(egoPos3D, leftPoint, color);
            Debug.DrawLine(egoPos3D, rightPoint, color);
            Debug.DrawLine(leftPoint, rightPoint, color); // Cap the cone

            // 2. Draw the Relative Velocity Vector scaled by TimeHorizon
            if (relativeVelocity.sqrMagnitude > 0.01f)
            {
                var relVel3D = new Vector3(relativeVelocity.x, 0, relativeVelocity.y);

                // The length of this line is exactly how much relative distance 
                // will be covered in 'TimeHorizon' seconds.
                Debug.DrawRay(egoPos3D, relVel3D * (TimeHorizon*drawScale), Color.blue);
            }
        }
    }
}