using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
    // TESTS
    private float _testObservation = 10f;

    // The angle between two lidar points
    public float lidar_precision_degrees = 20f;
    private float _lidar_precision_radians;

    // Start is called before the first frame update
    void Start()
    {
        _lidar_precision_radians = Mathf.PI * lidar_precision_degrees / 180.0f;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // Performs one 360 lidar scan of the environment
    void scan()
    {
        float angle = 0;
        // Vector3 direction = new Vector3(Cos(angle), Sin(angle), 0);
    }

    public float getTestObservation()
    {
        return _testObservation;
    }
}
