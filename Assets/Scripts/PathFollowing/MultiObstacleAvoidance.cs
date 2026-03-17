using UnityEngine;

namespace PathFollowing
{
    
    /// <summary>
    /// Obstacle avoidance from the paper "Multi-Agent Obstacle Avoidance using
    /// Velocity Obstacles and Control Barrier Functions"
    /// /// Supports both Car and Drone vehicles.
    /// </summary>
    public class MultiObstacleAvoidance
    {
        //Parameters
        private const float SafetyMargin = 0.5f;            // Strict distance margin (delta)
        private const float SafetyMarginDrone = 0.6f;       // Stricter margin for drones with pedestrians (omnidirectional)
        private const float SafetyMarginPedestrian = 0.8f;  // Extra safety margin specifically for pedestrians
        private const float TimeHorizon = 3.0f;           // How far ahead to check for VO (seconds)
        private const float SteerToNeighborPenalty = 10f; // Penalty for steering towards neighbors in the VO penalty function
        private const float SideBySidePenalty = 10f ;     // Penalty for driving side-by-side with neighbors in the VO penalty function
        
        private const float WeightReference = 1.0f;       // Weight for following intended input
        private const float WeightVO = 2.0f;              // Weight for avoiding Velocity Obstacles
        private const float WeightLateralAvoidance = 5.0f; // Weight for encouraging lateral movement to resolve deadlocks
        private const float WeightPedestrianVO = 5.0f;    // Extra weight for pedestrian avoidance
        
        private readonly float _pedestrianRadius = 1f; // Approximate radius of a pedestrian
        private readonly float _maxAcceleration;     // Max accel capability (m/s^2), data from previous assignment
        private readonly float _maxDeceleration = 5f;     // Max braking capability (m/s^2), data from previous assignment
        private readonly float _maxSteeringAngle = 25f;   // Max wheel turn in degrees
        private readonly float _wheelbase = 2.5f;         // Distance between front and rear axles
        
        private int _steerSamples = 7;           // Number of steering angles to sample
        private int _accelSamples = 5;           // Number of acceleration values to sample
        private int _droneAccelSamples = 9;      // Higher sampling for drone omnidirectional control
        
        //Non-changable
        private readonly float _vehicleRadius;   // Approximate radius of the car
        
        
        public MultiObstacleAvoidance(Transform vehicleState, float maxAcceleration)
        {
            _vehicleRadius = vehicleState.GetComponent<Collider>().bounds.extents.z; //Approximate the radius by the width
            //Debug.Log($"Acceleration set to {maxAcceleration}");
            _maxAcceleration = maxAcceleration;
        }
        
        public (float newAccel, float newSteer, float newBrake) getAdjustedControls(Transform myTransform, Vector3 currentVelocity, 
            float intendedSteer, float intendedAccel, float intendedBrake, GameObject[] otherCars, Collider[] staticObstacles, 
            GameObject[] pedestrians = null)
        {
            //Initialize the outputs
            float safeAccel;
            float safeSteer;
            float safeBrake;
            
            intendedSteer = Mathf.Clamp(intendedSteer, -1f, 1f);
            intendedAccel = Mathf.Clamp(intendedAccel, 0f, 1f);
            intendedBrake = Mathf.Clamp(intendedBrake, 0f, 1f);
            
            var pos = myTransform.position;

            // Convert intended Unity inputs to a unified scalar reference (Acceleration: [-maxDecel, maxAccel])
            var refAccelValue = intendedAccel * _maxAcceleration - intendedBrake * _maxDeceleration;

            var bestCost = float.MaxValue;
            var bestSteer = intendedSteer;
            var bestAccelValue = refAccelValue;
            var foundSafeSolution = false;

            // Extract the forward speed from the true velocity vector
            var currentForwardSpeed = Vector3.Dot(currentVelocity, myTransform.forward);
            
            // Iterate through possible control spaces
            for (var i = 0; i < _steerSamples; i++)
            {
                // Normalize i to [-1, 1]
                var sampledSteer = Mathf.Lerp(-1f, 1f, (float)i / (_steerSamples - 1));

                for (var j = 0; j < _accelSamples; j++)
                {
                    // Normalize j to [-maxDeceleration, maxAcceleration]
                    var sampledAccelValue = Mathf.Lerp(-_maxDeceleration, _maxAcceleration, (float)j / (_accelSamples - 1));

                    // 1. Predict new velocity vector roughly based on steer and accel
                    // (Assuming steer directly affects angular velocity, simplified for kinematic prediction)
                    Vector3 predictedForward = Quaternion.Euler(0, sampledSteer * _maxSteeringAngle, 0) * myTransform.forward;
                    float predictedSpeed = Mathf.Max(0, currentForwardSpeed + sampledAccelValue * Time.fixedDeltaTime);
                    Vector3 predictedVelocity = predictedForward * predictedSpeed;

                    // 2. Check strict CBF Safety (Hard Constraint)
                    if (!IsCBFSafe(pos, myTransform, predictedVelocity, otherCars, staticObstacles, pedestrians))
                        continue; // Discard this control input completely

                    // 3. Compute Objective Cost (Minimize deviation + VO penalty)
                    float costReference = WeightReference * (
                        Mathf.Pow(sampledSteer - intendedSteer, 2) + 
                        Mathf.Pow((sampledAccelValue - refAccelValue) / _maxAcceleration, 2));

                    float costVO = WeightVO * CalculateVOPenalty(pos, predictedVelocity, otherCars, staticObstacles, myTransform, pedestrians);

                    float totalCost = costReference + costVO;

                    // 4. Keep the control with the minimum cost
                    if (totalCost < bestCost)
                    {
                        bestCost = totalCost;
                        bestSteer = sampledSteer;
                        bestAccelValue = sampledAccelValue;
                        foundSafeSolution = true;
                    }
                }
            }

            // If no mathematically safe solution is found, trigger maximum braking
            if (!foundSafeSolution)
            {
                //Debug.Log("No safe control found, applying emergency brake!");
                safeSteer = intendedSteer; 
                safeAccel = 0f;
                safeBrake = 1f;
                return (safeAccel, safeSteer, safeBrake);
            }
            
            // Convert unified accel value back to Unity inputs
            safeSteer = bestSteer;
            safeAccel = bestAccelValue > 0 ? bestAccelValue / _maxAcceleration : 0f;
            safeBrake = bestAccelValue < 0 ? Mathf.Abs(bestAccelValue) / _maxDeceleration : 0f;
            
            return (safeAccel, safeSteer, safeBrake);
        }
        
        
        public (float finalH, float finalV) GetAdjustedDroneControls(Transform myTransform, Vector3 currentVelocity, 
            float intendedH, float intendedV, GameObject[] otherCars, Collider[] staticObstacles, GameObject[] pedestrians)
        {
            float safeH = 0f;
            float safeV = 0f;

            
            Vector2 intendedInput = new Vector2(intendedH, intendedV);

            Vector3 pos = myTransform.position;
            Vector3 intendedPredictedVelocity = currentVelocity + new Vector3(intendedH, 0, intendedV) * _maxAcceleration * Time.fixedDeltaTime;

            float bestCost = float.MaxValue;
            bool foundSafeSolution = false;
            Vector3 bestPredictedVelocity = Vector3.zero;

            // Sample a 2D grid for omnidirectional thrust mapping
            for (int i = 0; i < _droneAccelSamples; i++)
            {
                float sampledH = Mathf.Lerp(-1f, 1f, (float)i / (_droneAccelSamples - 1));
                
                for (int j = 0; j < _droneAccelSamples; j++)
                {
                    float sampledV = Mathf.Lerp(-1f, 1f, (float)j / (_droneAccelSamples - 1));
                    
                    Vector2 sampledInput = new Vector2(sampledH, sampledV);
                    if (sampledInput.magnitude > 1f) sampledInput.Normalize(); // Keep thrust circular

                    // 1. Predict new velocity vector for drone (omnidirectional acceleration)
                    Vector3 accelVector = new Vector3(sampledInput.x, 0, sampledInput.y) * _maxAcceleration;
                    Vector3 predictedVelocity = currentVelocity + (accelVector * Time.fixedDeltaTime);

                    // 2. Check strict CBF Safety (Hard Constraint)
                    if (!IsCBFSafe(pos, myTransform, predictedVelocity, otherCars, staticObstacles, pedestrians))
                        continue;

                    // 3. Compute Objective Cost (Minimize deviation + VO penalty + Lateral incentive)
                    float costReference = WeightReference * Mathf.Pow(Vector2.Distance(sampledInput, intendedInput), 2);
                    float costVO = WeightVO * CalculateVOPenalty(pos, predictedVelocity, otherCars, staticObstacles, myTransform, pedestrians);
                    
                    // Add lateral movement incentive to break deadlocks when drones are too close
                    float costLateral = CalculateLateralAvoidanceCost(pos, sampledInput, myTransform, otherCars, pedestrians);

                    float totalCost = costReference + costVO + (WeightLateralAvoidance * costLateral);

                    // 4. Keep the control with the minimum cost
                    if (totalCost < bestCost)
                    {
                        bestCost = totalCost;
                        safeH = sampledInput.x;
                        safeV = sampledInput.y;
                        bestPredictedVelocity = predictedVelocity;
                        foundSafeSolution = true;
                    }
                }
            }

            // 5. Fallback: Active Braking (If mathematically boxed in)
            if (!foundSafeSolution)
            {
                if (currentVelocity.magnitude < 0.1f)
                {
                    return (0f, 0f); // Just hover
                }
                
                // Active Braking: Apply thrust in the exact opposite direction of our velocity
                Vector3 stoppingDir = -currentVelocity.normalized;
                return (stoppingDir.x, stoppingDir.z); 
            }

            // Visualization
            Debug.DrawLine(pos, pos + intendedPredictedVelocity, Color.yellow, 0.1f);
            Debug.DrawLine(pos, pos + bestPredictedVelocity, Color.green, 0.1f);

            return (safeH, safeV);
        }
        
        
        /// <summary>
        /// Strict safety check using Control Barrier Function logic.
        /// Ensures we have enough distance to break before collision.
        /// </summary>
        private bool IsCBFSafe(Vector3 myPos, Transform myTransform, Vector3 predictedVelocity, GameObject[] otherCars, Collider[] staticObstacles, 
            GameObject[] pedestrians)
        {
            // 1. Check against other agents
            foreach (var car in otherCars)
            {
                
                // Get the Rigidbody for the ego car (do this once outside the loop if possible for performance, 
                // but doing it here works perfectly for fixing the logic)
                Rigidbody myRb = myTransform.GetComponentInParent<Rigidbody>();
                Rigidbody otherRb = car.GetComponent<Rigidbody>(); // Or GetComponentInParent depending on your setup

                // If both parts belong to the exact same physics body, they are the same car.
                if (myRb != null && myRb == otherRb) continue;
                
                
                Vector3 relativePos = car.transform.position - myPos;
                
                // Approximate other car's velocity
                Vector3 otherVelocity = Vector3.zero; 
                Rigidbody rb = car.GetComponent<Rigidbody>();
                if (rb != null) otherVelocity = rb.linearVelocity;

                Vector3 relativeVel = otherVelocity - predictedVelocity;
                
                float distance = relativePos.magnitude - (_vehicleRadius * 2);
                
                Vector3 dir = relativePos.normalized;

                // Velocity projected onto the relative direction
                float relSpeedProjected = Mathf.Min(0, Vector3.Dot(relativeVel, dir));
                
                //Debug.Log($"Dist: {distance:F2} | MyVel: {predictedVelocity.magnitude:F2} | OtherVel: {otherVelocity.magnitude:F2} | ClosingSpeed: {relSpeedProjected:F2}");
                // CBF Function: h_c = distance - margin - (v_rel^2 / 2*a_max) >= 0
                float hc = distance - SafetyMargin - (Mathf.Pow(relSpeedProjected, 2) / (2f * _maxDeceleration));
                
                if (hc < 0)
                {
                    //Debug.Log("Predicted collision with other car");
                    Debug.DrawLine(myPos, car.transform.position, Color.red, 0.1f);
                    return false; // This control breaks the safety barrier
                }
            }
            
            
            // 2. Check against pedestrians
            if (pedestrians != null)
            {
                foreach (var ped in pedestrians)
                {
                    Vector3 relativePos = ped.transform.position - myPos;
                    Vector3 pedVelocity = Vector3.zero;

                    // Extract velocity from ObstacleAI
                    var ai = ped.GetComponent<ObstacleAI>(); 
                    if (ai != null)
                    {
                        pedVelocity = ai.direction.normalized * ai.speed;
                    }
                    else
                    {
                        Rigidbody pedRb = ped.GetComponent<Rigidbody>();
                        if (pedRb != null) pedVelocity = pedRb.linearVelocity;
                    }

                    Vector3 relativeVel = pedVelocity - predictedVelocity;
        
                    // Drone radius + Pedestrian radius
                    float distance = relativePos.magnitude - (_vehicleRadius + _pedestrianRadius);
                    Vector3 dir = relativePos.normalized;

                    float relSpeedProjected = Mathf.Min(0, Vector3.Dot(relativeVel, dir));
                    float hc = distance - SafetyMarginPedestrian - (Mathf.Pow(relSpeedProjected, 2) / (2f * _maxDeceleration));

                    if (hc < 0) {
                        Debug.DrawLine(myPos, ped.transform.position, Color.red, 0.1f);
                        return false;
                    }
                }
            }
            
            
            // 2. Check against static obstacles (simplified as spheres)
            foreach (var obs in staticObstacles)
            {
                // 1. Find the exact point on the wall closest to the car
                Vector3 closestPoint = obs.ClosestPoint(myPos);
                Vector3 relativePos = closestPoint - myPos;
                
                Vector3 relativeVel = -predictedVelocity; // Obstacle is stationary
                
                float distance = relativePos.magnitude - _vehicleRadius;
                
                if (distance <= 0.001f) 
                {
                    //Debug.Log("Already touching static obstacle!");
                    return false;
                }
                
                Vector3 dir = relativePos.normalized;

                float relSpeedProjected = Mathf.Min(0, Vector3.Dot(relativeVel, dir));
                float hc = distance - SafetyMargin - (Mathf.Pow(relSpeedProjected, 2) / (2f * _maxDeceleration));

                if (hc < 0)
                {
                    //Debug.unityLogger.Log("Predicted collision with static obstacle");
                    Debug.DrawLine(myPos, closestPoint, Color.red, 0.1f);
                    return false;
                }
            }
            
            return true;
        }

        /// <summary>
        /// Calculates the soft VO penalty (Slack Variable equivalent in the paper's objective function).
        /// </summary>
        private float CalculateVOPenalty(Vector3 myPos, Vector3 predictedVelocity, GameObject[] otherCars, Collider[] staticObstacles,
            Transform myTransform, GameObject[] pedestrians)
        {
            float totalPenalty = 0f;

            // Calculate time-to-collision for dynamic agents
            foreach (var car in otherCars)
            {
                // Get the Rigidbody for the ego car (do this once outside the loop if possible for performance, 
                // but doing it here works perfectly for fixing the logic)
                Rigidbody myRb = myTransform.GetComponentInParent<Rigidbody>();
                Rigidbody otherRb = car.GetComponent<Rigidbody>(); // Or GetComponentInParent depending on your setup

                // If both parts belong to the exact same physics body, they are the same car.
                if (myRb != null && myRb == otherRb) continue;
                
                Rigidbody rb = car.GetComponent<Rigidbody>();
                Vector3 otherVelocity = rb != null ? rb.linearVelocity : Vector3.zero;
                Vector3 relativeVel = predictedVelocity - otherVelocity;
                Vector3 relativePos = car.transform.position - myPos;
                
                float distance = relativePos.magnitude;

                // Dynamic Spatial Penalty ---
                if (distance > 0.1f && distance < _vehicleRadius * 15f) 
                {
                    Vector3 dirToOther = relativePos.normalized;
                    float sideAlignment = Mathf.Abs(Vector3.Dot(myTransform.right, dirToOther));

                    // If they are side-by-side
                    if (sideAlignment > 0.5f) 
                    {
                        // 1. Penalize steering TOWARDS the neighbor
                        // If Dot > 0, the predicted velocity is pushing them closer
                        float steerTowards = Vector3.Dot(predictedVelocity.normalized, dirToOther);
                        if (steerTowards > 0) 
                        {
                            totalPenalty += (steerTowards * SteerToNeighborPenalty) / distance;
                        }

                        // 2. Penalize matching speeds
                        // This forces the optimizer to pick a braking or accelerating sample to break symmetry
                        float speedDifference = relativeVel.magnitude;
                        totalPenalty += SideBySidePenalty / (speedDifference + 0.1f);
                    }
                }

                float ttc = ComputeTimeToCollision(relativePos, relativeVel, _vehicleRadius * 2);
                if (ttc > 0 && ttc < TimeHorizon)
                {
                    // Inverse time-to-collision as the penalty weight (w_ij = 1 / T_col)
                    totalPenalty += 1.0f / (ttc + 0.1f); 
                }
            }
            
            // VO Penalty for Pedestrians
            if (pedestrians != null)
            {
                foreach (var ped in pedestrians)
                {
                    Vector3 pedVelocity = Vector3.zero;
                    var ai = ped.GetComponent<ObstacleAI>(); 
                    if (ai != null) pedVelocity = ai.direction.normalized * ai.speed;
                    else
                    {
                        Rigidbody pedRb = ped.GetComponent<Rigidbody>();
                        if (pedRb != null) pedVelocity = pedRb.linearVelocity;
                    }

                    Vector3 relativeVel = predictedVelocity - pedVelocity;
                    Vector3 relativePos = ped.transform.position - myPos;

                    float ttc = ComputeTimeToCollision(relativePos, relativeVel, _vehicleRadius + _pedestrianRadius);
                    if (ttc > 0 && ttc < TimeHorizon)
                    {
                        // Pedestrians get extra weight - they are vulnerable and must be prioritized
                        totalPenalty += WeightPedestrianVO * (1.0f / (ttc + 0.1f));
                    }
                }
            }
            
            // Calculate time-to-collision for static obstacles
            foreach (var obs in staticObstacles)
            {
                // 1. Find the exact point on the wall closest to the car
                Vector3 closestPoint = obs.ClosestPoint(myPos);
                Vector3 relativePos = closestPoint - myPos;
                
                Vector3 relativeVel = predictedVelocity; //Stationary obstacle

                float ttc = ComputeTimeToCollisionStatic(relativePos, relativeVel, _vehicleRadius);
                if (ttc > 0 && ttc < TimeHorizon)
                {
                    totalPenalty += 10f / (ttc + 0.1f);
                }
            }

            return totalPenalty;
        }

        /// <summary>
        /// Computes ray-sphere intersection time to collision.
        /// </summary>
        private float ComputeTimeToCollision(Vector3 relativePos, Vector3 relativeVel, float combinedRadius)
        {
            float a = Vector3.Dot(relativeVel, relativeVel);
            if (a < 0.001f) return -1f; // Not moving relative to each other

            float b = -2f * Vector3.Dot(relativeVel, relativePos);
            float c = Vector3.Dot(relativePos, relativePos) - (combinedRadius * combinedRadius);

            float discriminant = (b * b) - (4 * a * c);
            if (discriminant < 0) return -1f; // No collision course

            float t1 = (b - Mathf.Sqrt(discriminant)) / (2 * a);
            return t1 > 0 ? t1 : -1f; // Return positive time or -1 if passed
        }
        
        
        /// <summary>
        /// Computes simple 1D time to collision for flat static obstacles.
        /// </summary>
        private float ComputeTimeToCollisionStatic(Vector3 relativePos, Vector3 relativeVel, float vehicleRadius)
        {
            // Direction straight from the car to the closest point on the wall
            Vector3 dirToWall = relativePos.normalized;
    
            // Project our velocity onto that direction. How fast are we closing the gap?
            float closingSpeed = Vector3.Dot(relativeVel, dirToWall);
    
            // If closing speed is zero or negative, we are moving away from or parallel to the wall. Safe!
            if (closingSpeed <= 0.001f) return -1f;
    
            // True distance from the edge of our car's bumper to the wall's surface
            float distanceToWall = relativePos.magnitude - vehicleRadius;
    
            // If we are already inside the wall, collision time is 0
            if (distanceToWall <= 0) return 0f;
    
            // Simple physics: Time = Distance / Speed
            return distanceToWall / closingSpeed;
        }

        /// <summary>
        /// Calculates cost to encourage lateral movement when drones are too close.
        /// This helps break deadlocks by rewarding sideways or diagonal movement.
        /// </summary>
        private float CalculateLateralAvoidanceCost(Vector3 myPos, Vector2 sampledInput, Transform myTransform, GameObject[] otherCars, GameObject[] pedestrians)
        {
            float lateralCost = 0f;

            // Check proximity to other drones
            if (otherCars != null)
            {
                foreach (var car in otherCars)
                {
                    Rigidbody myRb = myTransform.GetComponentInParent<Rigidbody>();
                    Rigidbody otherRb = car.GetComponent<Rigidbody>();
                    if (myRb != null && myRb == otherRb) continue;

                    Vector3 relativePos = car.transform.position - myPos;
                    float distance = relativePos.magnitude;

                    // Only care about nearby drones
                    if (distance < _vehicleRadius * 10f && distance > 0.1f)
                    {
                        // Convert input to direction (2D horizontal plane)
                        Vector3 moveDirection = new Vector3(sampledInput.x, 0, sampledInput.y).normalized;
                        Vector3 dirToOther = relativePos.normalized;

                        // Calculate how much the sampled input moves laterally relative to the other drone
                        float headOnAlignment = Vector3.Dot(moveDirection, dirToOther);

                        // Reward lateral movement (perpendicular to head-on direction)
                        // If headOnAlignment is 0, we're moving perpendicular (good!)
                        // If headOnAlignment is 1, we're moving directly at them (bad!)
                        float lateralComponent = 1f - Mathf.Abs(headOnAlignment);

                        // Stronger incentive for closer drones
                        float proximityWeight = 1f / (distance + 0.5f);
                        lateralCost -= lateralComponent * proximityWeight; // Negative because we want to minimize cost
                    }
                }
            }

            // Similar check for pedestrians
            if (pedestrians != null)
            {
                foreach (var ped in pedestrians)
                {
                    Vector3 relativePos = ped.transform.position - myPos;
                    float distance = relativePos.magnitude;

                    // Larger detection range for pedestrians since they're vulnerable
                    if (distance < _vehicleRadius * 12f && distance > 0.1f)
                    {
                        Vector3 moveDirection = new Vector3(sampledInput.x, 0, sampledInput.y);
                        if (moveDirection.magnitude > 0.01f) moveDirection.Normalize();
                        else moveDirection = Vector3.zero;
                        
                        Vector3 dirToOther = relativePos.normalized;

                        float headOnAlignment = moveDirection.magnitude > 0.01f ? Vector3.Dot(moveDirection, dirToOther) : 0f;
                        float lateralComponent = 1f - Mathf.Abs(headOnAlignment);

                        // Stronger incentive for pedestrians
                        float proximityWeight = 1.5f / (distance + 0.2f);
                        lateralCost -= lateralComponent * proximityWeight;
                    }
                }
            }

            // Return positive cost (minimize it = prefer lateral movement)
            return -lateralCost;
        }

    }
}