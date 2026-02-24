using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine;

namespace Pathfollowing
{
    
    /// <summary>
    /// Obstacle avoidance from the paper "Multi-Agent Obstacle Avoidance using
    /// Velocity Obstacles and Control Barrier Functions"
    /// </summary>
    public class MultiObstacleAvoidance
    {
        //Parameters
        private readonly float _safetyMargin = 0f;      // Strict distance margin (delta)
        private readonly float _timeHorizon = 3.0f;       // How far ahead to check for VO (seconds)
        
        private readonly float _maxAcceleration = 5f;     // Max accel capability (m/s^2), data from previous assignment
        private readonly float _maxDeceleration = 5f;     // Max braking capability (m/s^2), data from previous assignment
        private readonly float _maxSteeringAngle = 35f;   // Max wheel turn in degrees
        private readonly float _wheelbase = 2.5f;         // Distance between front and rear axles
        
        private float _weightReference = 1.0f;   // Weight for following intended input
        private float _weightVO = 2.0f;          // Weight for avoiding Velocity Obstacles
        
        private int _steerSamples = 7;           // Number of steering angles to sample
        private int _accelSamples = 5;           // Number of acceleration values to sample
        
        //Non-changable
        private readonly float _vehicleRadius;   // Approximate radius of the car
        
        
        public MultiObstacleAvoidance(Transform vehicleState)
        {
            _vehicleRadius = vehicleState.GetComponent<Collider>().bounds.extents.z; //Approximate the radius by the width
        }
        
        public (float newAccel, float newSteer, float newBrake) getAdjustedControls(Transform myTransform, Vector3 currentVelocity, 
            float intendedSteer, float intendedAccel, float intendedBrake, GameObject[] otherCars, Collider[] staticObstacles)
        {
            //Initialize the outputs
            float safeAccel;
            float safeSteer;
            float safeBrake;
            
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
                    Vector3 predictedForward = Quaternion.Euler(0, sampledSteer * 30f * Time.fixedDeltaTime, 0) * myTransform.forward;
                    float predictedSpeed = Mathf.Max(0, currentForwardSpeed + sampledAccelValue * Time.fixedDeltaTime);
                    Vector3 predictedVelocity = predictedForward * predictedSpeed;

                    // 2. Check strict CBF Safety (Hard Constraint)
                    if (!IsCBFSafe(pos, myTransform, predictedVelocity, otherCars, staticObstacles))
                        continue; // Discard this control input completely

                    // 3. Compute Objective Cost (Minimize deviation + VO penalty)
                    float costReference = _weightReference * (
                        Mathf.Pow(sampledSteer - intendedSteer, 2) + 
                        Mathf.Pow((sampledAccelValue - refAccelValue) / _maxAcceleration, 2));

                    float costVO = _weightVO * CalculateVOPenalty(pos, predictedVelocity, otherCars, staticObstacles, myTransform);

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
        
        /// <summary>
    /// Strict safety check using Control Barrier Function logic.
    /// Ensures we have enough distance to break before collision.
    /// </summary>
    private bool IsCBFSafe(Vector3 myPos, Transform myTransform, Vector3 predictedVelocity, GameObject[] otherCars, Collider[] staticObstacles)
    {
        // 1. Check against other agents
        foreach (var car in otherCars)
        {
            if (car.transform.root == myTransform.root) continue; // Skip self
            
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
            
            // CBF Function: h_c = distance - margin - (v_rel^2 / 2*a_max) >= 0
            float hc = distance - _safetyMargin - (Mathf.Pow(relSpeedProjected, 2) / (2f * _maxDeceleration));
            
            if (hc < 0)
            {
                Debug.Log("Predicted collision with other car");
                return false; // This control breaks the safety barrier
            }
        }

        // 2. Check against static obstacles (simplified as spheres)
        foreach (var obs in staticObstacles)
        {
            Vector3 relativePos = obs.transform.position - myPos;
            Vector3 relativeVel = -predictedVelocity; // Obstacle is stationary
            
            float distance = relativePos.magnitude - _vehicleRadius - obs.bounds.extents.magnitude;
            Vector3 dir = relativePos.normalized;

            float relSpeedProjected = Mathf.Min(0, Vector3.Dot(relativeVel, dir));
            float hc = distance - _safetyMargin - (Mathf.Pow(relSpeedProjected, 2) / (2f * _maxDeceleration));

            if (hc < 0)
            {
                return false;
            }
        }
        
        return true;
    }

    /// <summary>
    /// Calculates the soft VO penalty (Slack Variable equivalent in the paper's objective function).
    /// </summary>
    private float CalculateVOPenalty(Vector3 myPos, Vector3 predictedVelocity, GameObject[] otherCars, Collider[] staticObstacles,
        Transform myTransform)
    {
        float totalPenalty = 0f;

        // Calculate time-to-collision for dynamic agents
        foreach (var car in otherCars)
        {
            if (car.transform.root == myTransform.root) continue;

            Rigidbody rb = car.GetComponent<Rigidbody>();
            Vector3 otherVelocity = rb != null ? rb.linearVelocity : Vector3.zero;
            Vector3 relativeVel = predictedVelocity - otherVelocity;
            Vector3 relativePos = car.transform.position - myPos;

            float ttc = ComputeTimeToCollision(relativePos, relativeVel, _vehicleRadius * 2);
            if (ttc > 0 && ttc < _timeHorizon)
            {
                // Inverse time-to-collision as the penalty weight (w_ij = 1 / T_col)
                totalPenalty += 1.0f / (ttc + 0.1f); 
            }
        }

        // Calculate time-to-collision for static obstacles
        foreach (var obs in staticObstacles)
        {
            Vector3 relativeVel = predictedVelocity;
            Vector3 relativePos = obs.transform.position - myPos;

            float ttc = ComputeTimeToCollision(relativePos, relativeVel, _vehicleRadius + obs.bounds.extents.magnitude);
            if (ttc > 0 && ttc < _timeHorizon)
            {
                totalPenalty += 1.0f / (ttc + 0.1f);
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

    }
}