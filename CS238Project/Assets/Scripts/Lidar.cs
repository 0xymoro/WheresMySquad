using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
    // The angle between two lidar points
    public float lidar_precision_degrees = 20f;
    // Farthest distance the lidar can reach
    public float lidar_max_distance = 5f;

    private float _lidar_precision_radians;
    private int _num_rays;
    private float[] _distance_observation;
    private bool _lidar_initialized = false;

    // The frequence at which scans are performed
    static int UPDATE_FREQ = 5;
    int updateCounter = 0;

    // Start is called before the first frame update
    void Start()
    {
        _lidar_precision_radians = Mathf.PI * lidar_precision_degrees / 180.0f;
        _num_rays = Mathf.RoundToInt(2 * Mathf.PI / _lidar_precision_radians);
        _distance_observation = new float[_num_rays];
    }

    // Update is called once per frame
    void Update()
    {
        if (updateCounter % UPDATE_FREQ == 0) {
            _distance_observation = Scan(transform.position, false, true);
            _lidar_initialized = true;
        }
        updateCounter++;
    }

    // Performs one 360 lidar scan of the environment
    // Takes in the origin of the scan
    // Returns an array containing the hit distance for each ray
    public float[] Scan(Vector3 scanOrigin, bool includeAgents=false, bool displayRays=false)
    {
        // Configure layermask to ignore agents if includeAgents is false, otherwise include everything
        int layermask = ~0;
        if (!includeAgents) {
            layermask = ~(1 << 8);
        }
        float[] distance_observation = new float[_num_rays];
        // For each ray compute the hit distance
        for (int i = 0; i < _num_rays; i++) {
            float angle = i * _lidar_precision_radians;
            Vector3 direction = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0);
            RaycastHit hit;
            // If hit, draw the ray and store the distance
            if (Physics.Raycast(scanOrigin, direction, out hit, lidar_max_distance, layermask)) {
                // Add noise to the detected distance
                float hitDistance = hit.distance + Random.Range(-0.5f, 0.5f);
                if (displayRays) {
                    Debug.DrawRay(scanOrigin, direction * hitDistance, Color.red);
                }
                distance_observation[i] = hitDistance;
            } else {
                // -1 to indicate it was not a hit
                distance_observation[i] = -1;
            }
        }
        return distance_observation;
    }

    public float[] GetDistanceObservation()
    {
        return _distance_observation;
    }

    public float GetLidarPrecisionRadians()
    {
        return _lidar_precision_radians;
    }

    public bool GetLidarInitialized()
    {
        return _lidar_initialized;
    }
}
