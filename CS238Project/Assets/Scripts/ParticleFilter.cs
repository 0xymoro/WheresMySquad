using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticleFilter : MonoBehaviour
{
    // Random seed
    const int SEED = 1337; 
    static int UPDATE_FREQ = 5;
    int updateCounter = 0;

    // World boundary
    public Camera main_camera;
    static float X_LENGTH;
    static float Y_LENGTH;

    // Particle properties
    static int NUM_PARTICLES;
    public float observation_threshold = 1;
    public GameObject particlePrefab;
    private GameObject[] _beliefStates;
    private GameObject _particleParent;
    private float _lidar_precision_radians;
    static float NOISE_MEAN = 0f;
    static float NOISE_STD = 0.1f;

    // Current state of the agent
    private Vector3 _agentPosition;

    // Lidar
    private Lidar _lidar_script;

    // Start is called before the first frame update
    void Start()
    {
        // Define the world boundary based on camera projection
        X_LENGTH = 2 * main_camera.orthographicSize * Screen.width / Screen.height;
        Y_LENGTH = 2 * main_camera.orthographicSize;

        // Number of particles is the amount of particles that can cover the map
        NUM_PARTICLES = (int)X_LENGTH * (int)Y_LENGTH;
        Debug.Log("Number of particles: " + NUM_PARTICLES);

        // Set random seed
        Random.InitState(SEED);

        // Set initial position
        _agentPosition = transform.position;

        // Get Lidar script
        _lidar_script = GetComponent<Lidar>();
        _lidar_precision_radians = _lidar_script.GetLidarPrecisionRadians();

        // Initialize particles
        _beliefStates = new GameObject[NUM_PARTICLES];
        _particleParent = new GameObject();
        _particleParent.name = gameObject.name + "_particleParent";

        for (int i = 0; i < (int)X_LENGTH; i++) {
            for (int j = 0; j < (int)Y_LENGTH; j++) {
                float x = i - (int)X_LENGTH / 2;
                float y = j - (int)Y_LENGTH / 2;
                GameObject particle = Instantiate(particlePrefab, new Vector3(x, y, -2), Quaternion.identity);
                particle.transform.SetParent(_particleParent.transform);
                _beliefStates[i + j * (int)X_LENGTH] = particle;
            }    
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (!_lidar_script.GetLidarInitialized()) {
            return;
        }
     
        if (updateCounter % UPDATE_FREQ == 0) {
            UpdateBelief();
        }
        updateCounter++;
    }

    // Update belief state with new particles
    void UpdateBelief() {
        // Get observation from Lidar and action(displacement) of the agent
        float[] agent_observation = _lidar_script.GetDistanceObservation();
        Vector3 action = GetAction();
        
        // Create weights for each sampled particle
        float[] weights = new float[_beliefStates.Length];
        Vector3[] sampledPositions = new Vector3[_beliefStates.Length];

        // Sample from belief states
        for (int i = 0; i < _beliefStates.Length; i++) {
            int randomParticleIndex = Random.Range(0, _beliefStates.Length);
            GameObject randomParticle = _beliefStates[randomParticleIndex];
            // Update the sampled particle's position by taking the action performed by the agent
            Vector3 newParticlePosition = randomParticle.transform.position + action;
            // sampledPositions[i] = randomParticle.transform.position;
            sampledPositions[i] = newParticlePosition;
            float[] particle_observation = _lidar_script.Scan(newParticlePosition + new Vector3(0, 0, 2));

            // Assume at least 1 ray matched so weight won't ever be 0
            int numRayMatches = 1;
            // Compare particle observation with agent observation
            for (int j = 0; j < agent_observation.Length; j++) {
                // If the observation is positive and within threshold then increment match counts
                if (Mathf.Abs(agent_observation[j] - particle_observation[j]) < observation_threshold && 
                    agent_observation[j] > 0 && particle_observation[j] > 0) {
                    numRayMatches++;
                }
            }
            // Weight is the percentage of rays that matched between agent and particle observations
            weights[i] = (float)numRayMatches / agent_observation.Length;
        }
        
        // Resample using the weights and update the belief
        for (int i = 0; i < _beliefStates.Length; i++) {
            int randomIndex = WeightedRandomIndex(ref weights);
            //Vector3 noise = new Vector3(Random.Range(-NOISE_BOUND, NOISE_BOUND), Random.Range(-NOISE_BOUND, NOISE_BOUND), 0);
            Vector3 noise = new Vector3(SampleNormal(NOISE_MEAN, NOISE_STD), SampleNormal(NOISE_MEAN, NOISE_STD), 0);
            _beliefStates[i].transform.position = sampledPositions[randomIndex] + noise;
        }
    }

    // Returns the action that was taken to get from the previous state(position) to the current state
    // TODO: Currently using the position difference over time to estimate the action. IRL it should be
    // integrated acceleration to get displacement. Currently, this should not be cheating for localization
    // but it may be better to not use transform.position. That involves changing how the agent is moved and
    // it's hard to deal with collisions if we use Translate() instead of AddForce().
    Vector3 GetAction()
    {
        Vector3 action = transform.position - _agentPosition;
        _agentPosition = transform.position;
        return action;
    }

    // Takes in an array of floats containing weights
    // Return a random index with the probability given by weights
    int WeightedRandomIndex(ref float[] weights)
    {
        float totalWeight = 0;
        foreach (float weight in weights) {
            totalWeight += weight;
        }
        float randomWeight = Random.Range(0, totalWeight);
        float currentWeight = 0;
        for (int i = 0; i < weights.Length; i++) {
            currentWeight += weights[i];
            if (randomWeight <= currentWeight) {
                return i;
            }
        }
        // Placeholding return statement, should never be reached
        return -1;
    }


    //Sample from normal
    float SampleNormal(float mean, float stdDev)
    {
        float u1 = 1.0f-Random.Range(0.0f, 1.0f); //uniform(0,1] random doubles
        float u2 = 1.0f-Random.Range(0.0f, 1.0f);
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
             Mathf.Sin(2.0f * Mathf.PI * u2); //random normal(0,1)
        float randNormal =
             mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)
        return randNormal;
    }


}
