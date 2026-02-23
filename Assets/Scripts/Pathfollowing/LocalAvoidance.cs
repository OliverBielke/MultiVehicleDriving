using UnityEngine;

namespace Pathfollowing
{
    public static class LocalAvoidance
    {
        public static (float steerAdjust, float brakeAdjust) CalculateSeparation(Transform myTransform, int myPriority, GameObject[] otherCars, float panicRadius)
        {
            float avoidSteering = 0f;
            float avoidBraking = 0f;

            foreach (var otherCar in otherCars)
            {
                //.root, otherwise we also care about our own care (not good)
                if (otherCar == null || otherCar.transform.root == myTransform.root) continue;
                
                AIP1TrafficCar otherCarScript = otherCar.GetComponent<AIP1TrafficCar>();
                int otherPriority = otherCarScript != null ? otherCarScript.priority : 0;
                
                if (myPriority > otherPriority)
                {
                    continue; 
                }
                // flatten the Y-axis so we only care about 2D distance
                Vector3 myPos = new Vector3(myTransform.position.x, 0, myTransform.position.z);
                Vector3 otherPos = new Vector3(otherCar.transform.position.x, 0, otherCar.transform.position.z);
                
                Vector3 toOtherCar = otherPos - myPos;
                float distance = toOtherCar.magnitude;

                if (distance > 0.1f && distance < panicRadius) 
                {
                    Vector3 dirToOther = toOtherCar.normalized;
                    float forwardDot = Vector3.Dot(myTransform.forward, dirToOther);
                    float rightDot = Vector3.Dot(myTransform.right, dirToOther);

                    // If the car is in front of us (or slightly to the side)
                    if (forwardDot > -0.2f) 
                    {
                        float urgency = 1f - (distance / panicRadius);

                        float steerDirection = rightDot > 0.3f ? -1f : 1f;
                        
                        avoidSteering += steerDirection * urgency * 3.0f;

                        if (distance < panicRadius * 0.5f && forwardDot > 0.7f)
                        {
                            avoidBraking = Mathf.Max(avoidBraking, urgency); 
                        }
                    }
                }
            }

            return (avoidSteering, avoidBraking);
        }
    }
}