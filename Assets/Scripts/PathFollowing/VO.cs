using UnityEngine;

namespace PathFollowing
{
    public class VO
    {
        // --- 1. Agent Capabilities ---
        private readonly float _maxAcceleration;
        private readonly float _vehicleRadius;
        
        // --- 2. Safety Parameters ---
        private const float TimeHorizon = 3.0f;           // How far ahead to predict (seconds)
        private const float PedestrianRadius = 1.0f;      // Approximate size of a pedestrian
        private const int AccelSamples = 9;               // How many points to check on our acceleration grid
        private const float VehicleRadiusPadding =  0.5f;
        
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
            // 1. Visualize Agent's Current State & Intent
            // Green line: Current Velocity
            Debug.DrawRay(myTransform.position, currentVelocity, Color.green);
            
            // Blue line: Intended Acceleration (scaled for visibility, applied at the tip of velocity)
            var intendedAccelVector = new Vector3(intendedH, 0, intendedV);
            Debug.DrawRay(myTransform.position + currentVelocity, intendedAccelVector, Color.blue);
            
            // Visualize the obstacles in the scene.
            foreach (var ped in pedestrians)
            {
                DrawDebugCircle(ped.transform.position, PedestrianRadius, Color.yellow);
                
                // If pedestrians have rigidbodies, draw their velocity in red
                if (ped.TryGetComponent<Rigidbody>(out var rb))
                {
                    Debug.DrawRay(ped.transform.position, rb.linearVelocity, Color.red);
                }
            }
            
            // Generate and Visualize the Search Space
            // We will eventually test these points to see which are safe, 
            // and pick the safe one closest to intendedAccelVector.
            int gridSide = Mathf.CeilToInt(Mathf.Sqrt(AccelSamples));
            float step = (_maxAcceleration * 2f) / Mathf.Max(1, gridSide - 1);

            for (int i = 0; i < gridSide; i++)
            {
                for (int j = 0; j < gridSide; j++)
                {
                    float candH = -_maxAcceleration + (i * step);
                    float candV = -_maxAcceleration + (j * step);
                    
                    Vector3 candAccel = new Vector3(candH, 0, candV);
                    
                    // Discard points outside our maximum acceleration capability (keep it circular)
                    if (candAccel.magnitude > _maxAcceleration) continue;

                    // Draw the candidate acceleration points in white at the tip of our velocity vector
                    DrawDebugCross(myTransform.position + currentVelocity + candAccel, 0.15f, Color.white);
                }
            }

            DrawDebugCircle(myTransform.position, _vehicleRadius, Color.cyan); 
            

            // Still just returning the intended acceleration for now
            return (intendedH, intendedV);
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
        
        // Helper method to draw a small cross for points in space
        private void DrawDebugCross(Vector3 position, float size, Color color)
        {
            Debug.DrawLine(position - Vector3.right * size, position + Vector3.right * size, color);
            Debug.DrawLine(position - Vector3.forward * size, position + Vector3.forward * size, color);
        }
    }
}