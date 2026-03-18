using UnityEngine;
using System.Linq;
using System.Collections.Generic;

namespace PathFollowing
{
    public class VO
    {
        // --- 1. Agent Capabilities ---
        private readonly float _maxAcceleration;
        private readonly float _vehicleRadius;
        
        // --- 2. Safety Parameters ---
        private const float TimeHorizon = 2.0f;           // How far ahead to predict (seconds)
        private const float PedestrianRadius = 2.0f;      // Approximate size of a pedestrian
        private const int AccelSamples = 100;               // How many points to check on our acceleration grid
        private const float VehicleRadiusPadding =  0.5f;
        private const float StaticObstacleRadiusPadding = 1.5f;
        
        // Add this line:
        private const float PedestrianTurnAnticipationTime = 1.0f; // How many seconds before a turn to assume v=0
        
        public VO(Transform vehicleTransform, float maxAcceleration)
        {
            _maxAcceleration = maxAcceleration;
            _vehicleRadius = vehicleTransform.GetComponent<Collider>().bounds.extents.z + VehicleRadiusPadding; 
        }

        public (float, float) GetSafeAcceleration(Transform myTransform, Vector3 currentVelocity, 
            float intendedH, float intendedV, GameObject[] otherDrones, GameObject[] pedestrians, 
            Collider[] staticObstacles
            )
        {
            var currentVelocity2 = new Vector2(currentVelocity.x, currentVelocity.z);
            
            // Visualize Agent's Current State & Intent
            VisualizeAgentAndObstacles(myTransform, currentVelocity2, intendedH, intendedV, pedestrians);
            
            // Clamp intent to the physical capabilities of this specific drone model
            var intendedAccel2D = GetRealAcceleration(intendedH, intendedV);
            
            //Get candidate accelerations
            var candidates = GetCandidateAccelerations(intendedAccel2D);
            
            // Sort the candidates
            candidates =  SortCandidates(candidates, intendedAccel2D);
            
            //Get the obstacles
            var obstacleList = new List<VehicleState>();

            foreach (var drone in otherDrones)
            {
                // Safety check: Skip ourselves if we accidentally ended up in the _otherDrone array!
                if (drone.transform == myTransform) continue;
                obstacleList.Add(ConvertDroneToState(drone));
            }
            foreach (var pedestrian in pedestrians) obstacleList.Add(ConvertPedestrianToState(pedestrian));
            foreach (var obstacle in staticObstacles) obstacleList.Add(ConvertStaticObstacleToState(obstacle, myTransform));
            
            var bestAccel = Vector2.zero;
            var maxColTime = 0f;
            
            float h;
            float v;
            
            var mostThreateningObstacleIndex = -1; // Track the threat for the chosen candidate
            
            // Evaluate each candidate to find the best one
            foreach (var candidate in candidates)
            {
                var newVel = currentVelocity2 + candidate * Time.fixedDeltaTime;
                
                var (colTime, obsIndex) = LowestTimeToCollision(newVel, new Vector2(myTransform.position.x, myTransform.position.z), obstacleList);
                
                if (colTime > TimeHorizon)
                {
                    // If perfectly safe, optionally draw line to the furthest tracked threat (if any exist)
                    if (obsIndex != -1)
                    {
                        var threat = obstacleList[obsIndex];
                        var threatPos3D = new Vector3(threat.Position.x, myTransform.position.y, threat.Position.y);
                        
                        Debug.DrawLine(myTransform.position, threatPos3D, Color.red);
                        
                        // Draw red circle if the threat is static (velocity near zero)
                        if (threat.Velocity.sqrMagnitude < 0.0001f)
                        {
                            DrawDebugCircle(threatPos3D, threat.Radius, Color.red);
                        }
                    }
                    (h, v) = GetAccelerationOutput(candidate);
                    //Debug.Log($"Input accel: {intendedH:F2}, {intendedV:F2}; Output accel: {h:F2}, {v:F2}. Found perfectly safe acceleration with time to collision of {colTime:F2} seconds. Using this acceleration.");
                    return (h, v);
                }

                if (colTime <= maxColTime)//If worse than best time
                {
                    continue;
                }
                
                maxColTime = colTime;
                bestAccel = candidate;
                mostThreateningObstacleIndex = obsIndex;
            }
            
            // Draw the debug line for the best fallback candidate we are forced to use
            if (mostThreateningObstacleIndex != -1)
            {
                var threatPos = obstacleList[mostThreateningObstacleIndex].Position;
                Debug.DrawLine(myTransform.position, new Vector3(threatPos.x, myTransform.position.y, threatPos.y), Color.red);
            }
            
            // Visualize the finalized, chosen acceleration in magenta
            Debug.DrawRay(myTransform.position + currentVelocity, new Vector3(bestAccel.x, 0, bestAccel.y), Color.magenta);
            
            (h, v) = GetAccelerationOutput(bestAccel);
            Debug.Log($"Input accel: {intendedH:F2}, {intendedV:F2}; Output accel: {h:F2}, {v:F2}. No perfectly safe acceleration found. Best candidate has time to collision of {maxColTime:F2} seconds.");

            return (h, v);
        }


        private VehicleState ConvertStaticObstacleToState(Collider obstacle, Transform myTransform)
        {
            var obsPos3D = obstacle.ClosestPoint(myTransform.position);
            var obsPos = new Vector2(obsPos3D.x, obsPos3D.z);
            
            return new VehicleState
            {
                Position = obsPos,
                Velocity = Vector2.zero, 
                Forward = Vector2.zero, 
                Radius = 0f + StaticObstacleRadiusPadding
            };
        }
        
        
        private VehicleState ConvertDroneToState(GameObject drone)
        {
            var droneVel = drone.GetComponent<Rigidbody>().linearVelocity;
                
            var dronePos = new Vector2(drone.transform.position.x, drone.transform.position.z);
            
            return new VehicleState
            {
                Position = dronePos,
                Velocity = new Vector2(droneVel.x, droneVel.z), 
                Forward = new Vector2(droneVel.x, droneVel.z).normalized, 
                Radius = _vehicleRadius
            };
        }
        
        
        private VehicleState ConvertPedestrianToState(GameObject pedestrian)
        {
            var pedAI = pedestrian.GetComponent<ObstacleAI>();
            var pedVel = new Vector2(pedAI.direction.x, pedAI.direction.z) * pedAI.speed;
            
            // --- Anticipation Logic ---
            // Use the helper method to check if we should freeze their velocity
            if (IsPedestrianAnticipatingTurn(pedAI, pedestrian.transform.position))
            {
                pedVel = Vector2.zero; 
            }
            
            var pedPos = new Vector2(pedestrian.transform.position.x, pedestrian.transform.position.z);
            
            return new VehicleState
            {
                Position = pedPos,
                Velocity = pedVel, 
                Forward = new Vector2(pedAI.direction.x, pedAI.direction.z).normalized, 
                Radius = PedestrianRadius
            };
        }
        
        
        private bool IsPedestrianAnticipatingTurn(ObstacleAI pedAI, Vector3 position)
        {
            Vector3 p1 = position + pedAI.direction;
    
            if (Physics.SphereCast(p1, pedAI.characterRadius, pedAI.direction, out RaycastHit hit, 20f))
            {
                float distanceUntilTurn = hit.distance - 2f;
        
                // Return true if they are within the anticipation time window
                if (distanceUntilTurn < (pedAI.speed * PedestrianTurnAnticipationTime))
                {
                    return true;
                }
            }
            return false;
        }
        
        
        /// <summary>
        /// Sort candidates based on how close they are to the intended acceleration. 
        /// </summary>
        /// <param name="candidates">The list to be sorted. </param>
        /// <param name="intendedAccel2D">Intended acceleration. </param>
        /// <returns>The sorted list. </returns>
        private Vector2[] SortCandidates(Vector2[] candidates, Vector2 intendedAccel2D)
        {
            return candidates
                .OrderBy(c => (c - intendedAccel2D).sqrMagnitude)
                .ToArray();
        }
        

        private (float, int) LowestTimeToCollision(Vector2 velocity, Vector2 position,
            List<VehicleState> obstacles)
        {
            int closestObstacleIndex = -1; // -1 means no collision found
            var t = float.MaxValue;
            for (int i = 0; i < obstacles.Count; i++)
            {
                var tNew = TimeToCollision(velocity, position, obstacles[i]);
                if (tNew >= 0f && tNew < t)
                {
                    t = tNew;
                    closestObstacleIndex = i;
                }
            }
            
            return (t,  closestObstacleIndex);
        }
        
        
        /// <summary>
        /// Returns to collision with an object. Negative if no collision. 
        /// </summary>
        /// <param name="velocity">Velocity of the drone. </param>
        /// <param name="position">Position of the drone. </param>
        /// <param name="obstacle">Obstacle we want to avoid. </param>
        /// <returns>Time to collision, negative if no collision. </returns>
        private float TimeToCollision(Vector2 velocity, Vector2 position, VehicleState obstacle)
        {
            var relativePosition = position - obstacle.Position;
            var relativeVelocity = velocity - obstacle.Velocity;
            
            var combinedRadius = _vehicleRadius + obstacle.Radius;
            
            // Calculate Time to Collision (TTC)
            // Starting from the equation ||V*t + P|| = R, we derive a quadratic formula to solve for t (time until collision).
            // Quadratic equation: a*t^2 + b*t + c = 0
            var a = relativeVelocity.sqrMagnitude;
            var b = 2f * Vector2.Dot(relativeVelocity, relativePosition);
            var c = relativePosition.sqrMagnitude - combinedRadius * combinedRadius;

            // --- Handle agents that are already intersecting ---
            if (c < 0f)
            {
                // b represents the direction of relative velocity compared to relative position.
                // If b <= 0, the agents are moving towards each other (or perfectly parallel).
                // If b > 0, they are moving apart.
                if (b <= 0f) 
                {
                    // Penalize moving deeper by returning 0 (immediate collision)
                    return 0f; 
                }
                // Moving apart! Treat this as a perfectly an escape route.
                // Calculate the time it will take to EXIT the circle (t2)
                var pEsc = b / a;
                var qEsc = c / a;
                var tExit = (-pEsc / 2f) + Mathf.Sqrt((pEsc * pEsc / 4f) - qEsc);
                
                // Trick the evaluation loop: smaller exit time -> larger "safe" time score.
                // We cap it just below TimeHorizon so it doesn't early-exit the candidate search,
                // forcing the algorithm to evaluate all options and pick the FASTEST escape route.
                return Mathf.Min(0.5f / tExit, TimeHorizon - 0.01f);
            }
            // --------------------------------------------------------
            
            // If a is near zero, relative velocity is zero (we are matching speeds perfectly)
            if (a < 0.0001f) return -1f;

            // Convert to p-q form: t^2 + (b/a)*t + (c/a) = 0
            var p = b / a;
            var q = c / a;

            var inSqrt = (p * p / 4f) - q;

            if (inSqrt < 0)
            {
                // No real roots means no collision
                return -1f;
            }

            var sqrtTerm = Mathf.Sqrt(inSqrt);

            // Get the collision times
            var t1 = (-p / 2f) - sqrtTerm;
            var t2 = (-p / 2f) + sqrtTerm;

            // We want the smallest positive time
            var t = -1f;
            if (t1 >= 0 && (t2 < 0 || t1 < t2)) t = t1;
            else if (t2 >= 0) t = t2;
            
            return t;
        }
        
        
        /// <summary>
        /// Get the candidate accelerations to evaluate. Candidate 0 is always the exact intended acceleration, and the rest are distributed in a circle around it.
        /// </summary>
        /// <param name="intendedAccel2D">Intended acceleration. </param>
        /// <returns>Candidate accelerations. </returns>
        private Vector2[] GetCandidateAccelerations(Vector2 intendedAccel2D)
        {
            // Generate our candidate accelerations
            var candidates = new Vector2[AccelSamples];
            
            // We ALWAYS include the exact intended acceleration as candidate 0
            candidates[0] = intendedAccel2D;
            
            // Distribute the remaining samples in a circle around the drone
            // (These are our escape options if the intended path is blocked)
            for (var i = 1; i < AccelSamples; i++)
            {
                var angle = (i - 1) * (Mathf.PI * 2f) / (AccelSamples - 1);
                candidates[i] = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * _maxAcceleration;
            }
            
            return candidates;
        }
        
        
        /// <summary>
        /// Convert the accel input in the same way the Move function does, to ensure we're evaluating the same physical acceleration that the drone will actually experience.
        /// </summary>
        /// <param name="h">Horizontal acceleration. </param>
        /// <param name="v">Vertical acceleration. </param>
        /// <returns>The physical acceleration. </returns>
        private Vector2 GetRealAcceleration(float h, float v)
        {
            var acceleration = (Vector3.right * h + Vector3.forward * v) * _maxAcceleration;
            if (acceleration.magnitude > _maxAcceleration)
            {
                acceleration = acceleration.normalized * _maxAcceleration;
            }
            
            return new Vector2(acceleration.x, acceleration.z);
        }


        /// <summary>
        /// Convert the physical acceleration to the one used in Move. 
        /// </summary>
        /// <param name="outputAccel2D">Our best physical acceleration. </param>
        /// <returns>The output h and v. </returns>
        private (float, float) GetAccelerationOutput(Vector2 outputAccel2D)
        {
            var acceleration = outputAccel2D /  _maxAcceleration;
            return (acceleration.x, acceleration.y);
        }
        
        
        private void VisualizeAgentAndObstacles(Transform myTransform, Vector3 currentVelocity, 
            float intendedH, float intendedV, GameObject[] pedestrians)
        {
            // Green line: Current Velocity
            Debug.DrawRay(myTransform.position, currentVelocity, Color.green);
            
            // Blue line: Intended Acceleration (scaled for visibility, applied at the tip of velocity)
            var intendedAccelVector = new Vector3(intendedH, 0, intendedV);
            Debug.DrawRay(myTransform.position + currentVelocity, intendedAccelVector, Color.blue);
            
            // Visualize the obstacles in the scene.
            foreach (var ped in pedestrians)
            {
                var pedAI = ped.GetComponent<ObstacleAI>();
        
                // Check if the pedestrian is anticipating a turn
                bool isTurning = IsPedestrianAnticipatingTurn(pedAI, ped.transform.position);
        
                // Choose red if turning, otherwise yellow
                Color circleColor = isTurning ? Color.red : Color.yellow;
                DrawDebugCircle(ped.transform.position, PedestrianRadius, circleColor);
                
                // If pedestrians have rigidbodies, draw their velocity in red
                if (ped.TryGetComponent<Rigidbody>(out var rb))
                {
                    Debug.DrawRay(ped.transform.position, rb.linearVelocity, Color.red);
                }
            }
            
            // Draw this vehicle radius
            DrawDebugCircle(myTransform.position, _vehicleRadius, Color.cyan); 
        }
        
        
        private void DrawDebugCircle(Vector3 center, float radius, Color color)
        {
            int segments = 24;
            float angle = 0f;
            float step = Mathf.PI * 2f / segments;
            
            // Start point 
            Vector3 prevPoint = center + new Vector3(Mathf.Cos(angle) * radius, 0, Mathf.Sin(angle) * radius);
            
            for (int i = 1; i <= segments; i++)
            {
                angle += step;
                Vector3 nextPoint = center + new Vector3(Mathf.Cos(angle) * radius, 0, Mathf.Sin(angle) * radius);
                Debug.DrawLine(prevPoint, nextPoint, color);
                prevPoint = nextPoint;
            }
        }
    }
}