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

    // The frequence at which scans are performed
    static int UPDATE_FREQ = 30;

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
        int updateCounter = 0;
        if (updateCounter % UPDATE_FREQ == 0) {
            Scan();
        }
        updateCounter++;
    }

    // Performs one 360 lidar scan of the environment
    void Scan()
    {
        // For each ray compute the hit distance
        for (int i = 0; i < _num_rays; i++) {
            float angle = i * _lidar_precision_radians;
            Vector3 direction = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0);
            RaycastHit hit;
            // If hit, draw the ray and store the distance
            if (Physics.Raycast(transform.position, direction, out hit, lidar_max_distance)) {
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                _distance_observation[i] = hit.distance;
            } else {
                // -1 to indicate it was not a hit
                _distance_observation[i] = -1;
            }
        }
    }

    public float[] GetDistanceObservation()
    {
        return _distance_observation;
    }

    public float GetLidarPrecisionRadians()
    {
        return _lidar_precision_radians;
    }
}
