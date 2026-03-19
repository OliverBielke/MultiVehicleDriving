using UnityEngine;
using System.Collections.Generic;
using FormationGame; // Adjust based on where Node is located
using PathFinding;

namespace PathFollowing
{
    public class PathStop
    {
        private const float LookaheadDistance = 15f; 
        private const float ArrivalTimeThreshold = 2.5f; // Seconds. If cars arrive at intersection within this time of each other, trigger yield logic
        private const float MaxSpeed = 5f;
        private const float StopDistance = 3.5f; // Distance to maintain behind a blocked car

        private readonly Transform _myTransform;
        private readonly AIP1TrafficCar _egoCar;
        private readonly GameObject[] _otherCars;

        public PathStop(Transform vehicleTransform, AIP1TrafficCar egoCar, GameObject[] otherCars)
        {
            _myTransform = vehicleTransform;
            _egoCar = egoCar;
            _otherCars = otherCars;
        }

        public (float finalAccel, float finalSteering, float finalBrake) GetAdjustedControls(
            Vector3 currentVelocity,
            float intendedSteer,
            float intendedBrake,
            float intendedAccel)
        {
            bool shouldStop = false;
            
            if (_egoCar.Waypoints == null || _egoCar.Waypoints.Count < 2)
                return (intendedAccel, intendedSteer, intendedBrake);

            List<Vector2> egoPath = ExtractPathToLookahead(_egoCar.Waypoints, _myTransform.position);

            foreach (var otherObj in _otherCars)
            {
                if (otherObj == _egoCar.gameObject) continue;

                var otherCarScript = otherObj.GetComponent<AIP1TrafficCar>();
                if (otherCarScript == null || otherCarScript.Waypoints == null || otherCarScript.Waypoints.Count < 2) 
                    continue;

                // Basic distance optimization - don't calculate for cars far away
                if (Vector3.Distance(_myTransform.position, otherObj.transform.position) > LookaheadDistance * 1.5f)
                    continue;

                List<Vector2> otherPath = ExtractPathToLookahead(otherCarScript.Waypoints, otherObj.transform.position);
                Vector3 otherVelocity = otherObj.GetComponent<Rigidbody>()?.linearVelocity ?? Vector3.zero;

                if (EvaluateShouldYield(egoPath, currentVelocity, otherPath, otherVelocity, otherObj.transform))
                {
                    shouldStop = true;
                    Debug.DrawLine(_myTransform.position, otherObj.transform.position, Color.red);
                    break;
                }
            }

            if (shouldStop)
            {
                if (currentVelocity.magnitude < 0.1f) return (0f, intendedSteer, 1f); // Held brake
                return (0f, intendedSteer, 1f); // Active braking
            }

            return (intendedAccel, intendedSteer, intendedBrake);
        }

        private bool EvaluateShouldYield(List<Vector2> egoPath, Vector3 egoVelocity, List<Vector2> otherPath, Vector3 otherVelocity, Transform otherTransform)
        {
            Vector2 egoPos = new Vector2(_myTransform.position.x, _myTransform.position.z);
            Vector2 otherPos = new Vector2(otherTransform.position.x, otherTransform.position.z);

            // RULE 1 & 2: Car blocked by other cars ahead yields / Car behind yields
            // We check this by seeing if the other car is close, in front of us, and heading roughly the same way.
            Vector2 toOther = otherPos - egoPos;
            Vector2 egoForward = new Vector2(_myTransform.forward.x, _myTransform.forward.z).normalized;
            
            if (toOther.magnitude < StopDistance)
            {
                float dotForward = Vector2.Dot(egoForward, toOther.normalized);
                if (dotForward > 0.7f) // Other car is directly in front of us
                {
                    DrawDebugCross(otherTransform.position, Color.magenta);
                    return true;
                }
            }

            // Check for Path Intersections (Rules 3 & 4)
            for (int i = 0; i < egoPath.Count - 1; i++)
            {
                for (int j = 0; j < otherPath.Count - 1; j++)
                {
                    if (LineSegmentsIntersect(egoPath[i], egoPath[i+1], otherPath[j], otherPath[j+1], out Vector2 intersectPoint))
                    {
                        float distEgo = GetDistanceAlongPath(egoPos, egoPath, intersectPoint, i);
                        float distOther = GetDistanceAlongPath(otherPos, otherPath, intersectPoint, j);

                        float speedEgo = Mathf.Max(egoVelocity.magnitude, 1f); // Avoid divide by zero
                        float speedOther = Mathf.Max(otherVelocity.magnitude, 1f);

                        float timeEgo = distEgo / speedEgo;
                        float timeOther = distOther / speedOther;

                        // If we don't arrive at the intersection at roughly the same time, no collision risk
                        if (Mathf.Abs(timeEgo - timeOther) > ArrivalTimeThreshold)
                            continue;

                        // Determine if Merge or Cross by comparing segment directions after the intersection
                        Vector2 egoNextDir = (egoPath[i+1] - intersectPoint).normalized;
                        Vector2 otherNextDir = (otherPath[j+1] - intersectPoint).normalized;
                        
                        float pathDot = Vector2.Dot(egoNextDir, otherNextDir);
                        bool isMerging = pathDot > 0.8f; // Paths are converging to go the same way

                        DrawDebugCross(new Vector3(intersectPoint.x, 0.5f, intersectPoint.y), isMerging ? Color.yellow : Color.cyan);

                        if (isMerging)
                        {
                            // RULE 3: When merging paths, car making sharper turn yields
                            Vector2 egoApproachDir = (intersectPoint - egoPath[i]).normalized;
                            Vector2 otherApproachDir = (intersectPoint - otherPath[j]).normalized;

                            // Angle between approach and exit. Closer to 1 dot product = straighter line. 
                            // Lower dot product = sharper turn.
                            float egoTurnSharpness = Vector2.Dot(egoApproachDir, egoNextDir);
                            float otherTurnSharpness = Vector2.Dot(otherApproachDir, otherNextDir);

                            if (egoTurnSharpness < otherTurnSharpness)
                            {
                                return true; // Ego turn is sharper (lower dot product), so Ego yields
                            }
                        }
                        else
                        {
                            // RULE 4: At crossing, car with shorter path to go yields
                            if (distEgo < distOther)
                            {
                                return true; // Ego has shorter path to intersection, so Ego yields
                            }
                        }
                    }
                }
            }
            return false;
        }

        // --- Geometric Helpers ---

        private List<Vector2> ExtractPathToLookahead(List<Node> waypoints, Vector3 currentPos)
        {
            List<Vector2> path = new List<Vector2>();
            Vector2 pos2D = new Vector2(currentPos.x, currentPos.z);
            path.Add(pos2D);

            float accumulatedDistance = 0f;
            int closestIndex = 0;
            float closestDist = float.MaxValue;

            // Find where we currently are on the path
            for (int i = 0; i < waypoints.Count; i++)
            {
                float d = Vector2.Distance(pos2D, new Vector2(waypoints[i].position.x, waypoints[i].position.y));
                if (d < closestDist)
                {
                    closestDist = d;
                    closestIndex = i;
                }
            }

            // Extract forward path up to LookaheadDistance
            for (int i = closestIndex; i < waypoints.Count; i++)
            {
                Vector2 wp = new Vector2(waypoints[i].position.x, waypoints[i].position.y);
                accumulatedDistance += Vector2.Distance(path[path.Count - 1], wp);
                path.Add(wp);

                if (accumulatedDistance > LookaheadDistance) break;
            }

            // Visualization
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(new Vector3(path[i].x, 0.5f, path[i].y), new Vector3(path[i+1].x, 0.5f, path[i+1].y), Color.green);
            }

            return path;
        }

        private float GetDistanceAlongPath(Vector2 startPos, List<Vector2> path, Vector2 intersectPoint, int intersectSegmentIndex)
        {
            float dist = Vector2.Distance(startPos, path[1]); // Distance to first actual waypoint ahead
            for (int i = 1; i < intersectSegmentIndex; i++)
            {
                dist += Vector2.Distance(path[i], path[i+1]);
            }
            dist += Vector2.Distance(path[intersectSegmentIndex], intersectPoint);
            return dist;
        }

        private bool LineSegmentsIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, out Vector2 intersection)
        {
            intersection = Vector2.zero;
            float d = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
            if (Mathf.Abs(d) < 0.0001f) return false; // Parallel lines

            float u = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / d;
            float v = ((p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x)) / d;

            if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) return false; // Intersection outside segments

            intersection.x = p1.x + u * (p2.x - p1.x);
            intersection.y = p1.y + u * (p2.y - p1.y);
            return true;
        }

        private void DrawDebugCross(Vector3 position, Color color)
        {
            float size = 1f;
            Debug.DrawLine(position - Vector3.right * size, position + Vector3.right * size, color);
            Debug.DrawLine(position - Vector3.forward * size, position + Vector3.forward * size, color);
        }
    }
}