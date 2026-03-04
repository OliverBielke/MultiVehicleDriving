using UnityEngine;
using System.Collections.Generic;

namespace Pathfollowing
{

    /// <summary>
    /// The state of a vehicle for the VO algorithm.
    /// </summary>
    public struct VehicleState
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public float Radius;
    }

    public class VOStop
    {
        // How many seconds into the future we look for collisions
        private const float TimeHorizon = 3.0f;

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
            
            // Build the state for this ego vehicle
            var thisVehicle = new VehicleState
            {
                Position = new Vector2(_myTransform.position.x, _myTransform.position.z),
                Velocity = new Vector2(currentVelocity.x, currentVelocity.z),
                Radius = _myTransform.GetComponent<Collider>().bounds.extents.z //Approximate the radius by the width
            };
            
            var otherStateList = GetSurroundingVehicleStates(thisVehicle.Radius);

            var shouldStop = EvaluateShouldStop(thisVehicle, otherStateList);

            if (shouldStop)
            {
                // Imminent collision! Override controls to stop immediately.
                // You could expand this later to proportionally brake based on TTC.
                return (0f, intendedSteer, 1f);
            }

            // Safe to proceed, return intended controls
            return (intendedAccel, intendedSteer, intendedBrake);
        }

        
        /// <summary>
        /// Get the states of all surrounding vehicles based on the provided radius. This assumes all vehicles are circles with the same radius for simplicity.
        /// </summary>
        /// <param name="radius">Radius of the cars. </param>
        /// <returns>A list of all surrounding vehicle states. </returns>
        private List<VehicleState> GetSurroundingVehicleStates(float radius)
        {
            var states = new List<VehicleState>();

            foreach (var car in _otherCars)
            {
                var rb = car.GetComponent<Rigidbody>();
                var velocity = rb != null ? rb.linearVelocity : Vector3.zero;

                states.Add(new VehicleState
                {
                    Position = new Vector2(car.transform.position.x, car.transform.position.z),
                    Velocity = new Vector2(velocity.x, velocity.z),
                    Radius = radius
                });
            }

            return states;
        }
        
            
        /// <summary>
        /// Evaluates surrounding traffic to determine if this vehicle must stop.
        /// </summary>
        /// <param name="thisVehicle">The state of the vehicle. </param>
        /// <param name="surroundingVehicles">The states of all surrounding vehicles. </param>
        /// <returns>True if an imminent collision is detected and this vehicle must yield; otherwise, false.</returns>
        private static bool EvaluateShouldStop(VehicleState thisVehicle, List<VehicleState> surroundingVehicles)
        {
            // If we are practically stopped, we don't have a forward direction and don't need to stop further.
            if (thisVehicle.Velocity.sqrMagnitude < 0.0001f)
            {
                return false;
            }

            // Normalize our velocity to get a pure directional vector (length of 1)
            var forwardDir = thisVehicle.Velocity.normalized;

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

                // Dot Product to check if the obstacle is in front of us
                var dotProduct = Vector2.Dot(forwardDir, relativePosition);
                var isInFront = dotProduct > 0;

                // 2D Cross Product to check if the obstacle is to our right
                // Formula: (Forward.X * Target.Y) - (Forward.Y * Target.X)
                // Note: Assumes standard math coordinates (X is right, Y is up). 
                var crossProduct = (forwardDir.x * relativePosition.y) - (forwardDir.y * relativePosition.x);
                var isToRight = crossProduct < 0;

                // Step 3c: Apply Right-Of-Way rules. 
                // If it is NOT in front AND NOT to the right, we have the right of way. Skip it!
                if (!isInFront && !isToRight)
                {
                    continue;
                }

                // Calculate Time to Collision (TTC)
                // Startin from the equation ||V*t - P|| = R, we derive a quadratic formula to solve for t (time until collision).
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

            }

            return false;
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
            //Variables
            const float drawHeight = 1f; // Height at which to draw the debug lines (adjust as needed)
            const float furthestDrawDistance = 50f; // Maximum distance to draw the VO cone and velocity vector for better visibility
            const float drawSizeMultiplier = 0.2f; // Multiplier to scale the size of the drawn elements for better visibility
            
            // Only draw if the obstacle is within a reasonable distance to prevent clutter
            if (relativePosition.magnitude > furthestDrawDistance)
            {
                return;
            }
            
            // 3D Start position for our debug lines
            var egoPos3D = new Vector3(thisVehicle.Position.x, drawHeight, thisVehicle.Position.y);
            
            var dist = relativePosition.magnitude;
                
            // 1. Draw the VO Cone
            if (dist > combinedRadius)
            {
                // Calculate the angle of the tangent lines
                var angle = Mathf.Asin(combinedRadius / dist) * Mathf.Rad2Deg;
                    
                var centerLine = new Vector3(relativePosition.x, 0, relativePosition.y);
                    
                // Rotate the center line by +/- angle to get the cone edges
                var leftTangent = Quaternion.Euler(0, -angle, 0) * centerLine;
                var rightTangent = Quaternion.Euler(0, angle, 0) * centerLine;

                // Draw the edges of the cone
                Debug.DrawRay(egoPos3D, leftTangent.normalized * (dist*drawSizeMultiplier), color);
                Debug.DrawRay(egoPos3D, rightTangent.normalized * (dist*drawSizeMultiplier), color);
                
                // 1. Calculate the exact world positions of the left and right endpoints
                Vector3 leftPoint = egoPos3D + (leftTangent.normalized * (dist * drawSizeMultiplier));
                Vector3 rightPoint = egoPos3D + (rightTangent.normalized * (dist * drawSizeMultiplier));

                // 2. Use DrawLine to connect point A to point B
                Debug.DrawLine(leftPoint, rightPoint, color);
            }

            // 2. Draw the Relative Velocity Vector
            // Only draw it if there is a meaningful relative velocity to prevent screen clutter
            if (relativeVelocity.sqrMagnitude > 0.1f)
            {
                var relVel3D = new Vector3(relativeVelocity.x, 0, relativeVelocity.y);
                Debug.DrawRay(egoPos3D, relVel3D * drawSizeMultiplier, Color.blue);
            }
        }
    }
}