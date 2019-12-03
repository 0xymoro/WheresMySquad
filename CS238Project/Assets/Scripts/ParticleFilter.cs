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
    public float observation_threshold = 1;
    public GameObject particlePrefab;
    public GameObject beliefPositionPrefab;
    public float upper_confidence_threshold = 0.7f; // Particles are near the agent if its weight is higher than this threshold

    static int NUM_PARTICLES;
    private GameObject[] _beliefStates;
    private float[] _weights;
    private GameObject _beliefPosition;
    private GameObject _particleParent;
    private float _lidar_precision_radians;
    static float NOISE_MEAN = 0f;
    static float NOISE_STD = 0.1f;
    public float noise_multiplier = 10.0f;

    // Current state of the agent
    private Vector3 _agentPosition;

    // Lidar
    private Lidar _lidar_script;


    // Other agent variables
    const float RADIO_RANGE = 10f;
    GameObject[] _agents;

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
        _weights = new float[NUM_PARTICLES];
        _particleParent = new GameObject();
        _particleParent.name = gameObject.name + "_particleParent";

        for (int i = 0; i < (int)X_LENGTH; i++) {
            for (int j = 0; j < (int)Y_LENGTH; j++) {
                float x = i - (int)X_LENGTH / 2;
                float y = j - (int)Y_LENGTH / 2;
                GameObject particle = Instantiate(particlePrefab, new Vector3(x, y, -2), Quaternion.identity);
                particle.transform.SetParent(_particleParent.transform);
                _beliefStates[i + j * (int)X_LENGTH] = particle;
                _weights[i + j * (int)X_LENGTH] = 0.01f;
            }    
        }

        // Create belief position
        _beliefPosition = Instantiate(beliefPositionPrefab, new Vector3(0, 0, -2), Quaternion.identity);
        
        // Find other agents for cooperative localization
        _agents = GameObject.FindGameObjectsWithTag("agents");
    }


    // Update is called once per frame
    void Update()
    {
        if (!_lidar_script.GetLidarInitialized()) {
            return;
        }
     
        if (updateCounter % UPDATE_FREQ == 0) {
            UpdateBelief();
            _beliefPosition.transform.position = GetBeliefAgentPosition();
        }
        updateCounter++;
    }

    // Update belief state with new particles
    void UpdateBelief() {
        // Get observation from Lidar and action(displacement) of the agent
        float[] agent_observation = _lidar_script.GetDistanceObservation();
        Vector3 action = GetAction();
        
        // Create _weights for each sampled particle
        _weights = new float[_beliefStates.Length];
        Vector3[] sampledPositions = new Vector3[_beliefStates.Length];

        bool has_high_confidence = false;
        // Sample from belief states
        for (int i = 0; i < _beliefStates.Length; i++) {
            int randomParticleIndex = Random.Range(0, _beliefStates.Length);
            GameObject randomParticle = _beliefStates[randomParticleIndex];
            // Update the sampled particle's position by taking the action performed by the agent
            Vector3 newParticlePosition = randomParticle.transform.position + action;
            // sampledPositions[i] = randomParticle.transform.position;
            sampledPositions[i] = newParticlePosition;
            float[] particle_observation = _lidar_script.Scan(newParticlePosition + new Vector3(0, 0, 2));

            int numRayMatches = 0;
            int numRayObserved_agent = 0;
            int numRayObserved_particle = 0;
            // Compare particle observation with agent observation
            for (int j = 0; j < agent_observation.Length; j++) {
                // If the observation is positive and within threshold then increment match counts
                if ((agent_observation[j] > 0 && particle_observation[j] > 0 && 
                    Mathf.Abs(agent_observation[j] - particle_observation[j]) < observation_threshold)) {
                    numRayMatches++;
                }
                // Count how many rays are observed by the agent
                if (agent_observation[j] != -1) {
                    numRayObserved_agent++;
                }
                // Count how many rays are observed by the particle
                if (particle_observation[j] != -1) {
                    numRayObserved_particle++;
                }

            }
            // Heavily pentalize particle weight if the particle observed more than the agent
            if (numRayObserved_particle > numRayObserved_agent) {
                numRayObserved_agent = 2 * Mathf.Max(numRayObserved_agent, numRayObserved_particle);
            }
            // Weight is the percentage of rays that matched between agent and particle observations
            _weights[i] = (float)(numRayMatches + 1) / (numRayObserved_agent + 1);
            if (_weights[i] > upper_confidence_threshold) {
                has_high_confidence = true;
            }
        }


        // If there's at least one particle with high confidence or all particles are low confidence, keep noise level normal. Otherwise increase noise.
        // Noise is increased when there are observations but none of the particles match the observations.
        float multiplier = has_high_confidence ? 1 : noise_multiplier;
        // Resample using the _weights and update the belief
        for (int i = 0; i < _beliefStates.Length; i++) {
            int randomIndex = WeightedRandomIndex(ref _weights);
            Vector3 noise = new Vector3(SampleNormal(NOISE_MEAN, multiplier * NOISE_STD), SampleNormal(NOISE_MEAN, multiplier * NOISE_STD), 0);
            _beliefStates[i].transform.position = sampledPositions[randomIndex] + noise;
        }

        //Update beliefs with information from other agents
        MultiAgentUpdate();


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

    // Takes in an array of floats containing _weights
    // Return a random index with the probability given by _weights
    int WeightedRandomIndex(ref float[] _weights)
    {
        float totalWeight = 0;
        foreach (float weight in _weights) {
            totalWeight += weight;
        }
        float randomWeight = Random.Range(0, totalWeight);
        float currentWeight = 0;
        for (int i = 0; i < _weights.Length; i++) {
            currentWeight += _weights[i];
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
        float u1 = 1.0f-Random.Range(0.0f, 0.999f); //uniform[0,0.999] random doubles
        float u2 = 1.0f-Random.Range(0.0f, 1.0f);
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
             Mathf.Sin(2.0f * Mathf.PI * u2); //random normal(0,1)
        float randNormal =
             mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)
        return randNormal;
    }

    // Returns where the agent believes it's at.
    public Vector3 GetBeliefAgentPosition()
    {
        Vector3 beliefPosition = new Vector3(0, 0, 0);
        float totalWeight = 0;
        for (int i = 0; i < _beliefStates.Length; i++) {
            Vector3 particlePosition = _beliefStates[i].transform.position;
            float particleWeight = _weights[i];
            beliefPosition += particlePosition * particleWeight;
            totalWeight += particleWeight;
        }
        beliefPosition /= totalWeight;
        return beliefPosition;
    }
    // public Vector3 GetBeliefAgentPosition()
    // {
    //     int highestIndex = 0;
    //     float highestWeight = 0;
    //     for (int i = 0; i < _beliefStates.Length; i++) {
    //         if (_weights[i] > highestWeight) {
    //             highestWeight = _weights[i];
    //             highestIndex = i;
    //         }
    //     }
    //     return _beliefStates[highestIndex].transform.position;
    // }

    // Returns true if the agent is considered localized
    // An agent is considered localized if a percentage of the particles are within range of the agent
    public bool AgentLocalized()
    {
        Evaluation eval = GameObject.Find("Evaluation").GetComponent<Evaluation>();
        float localizationDistance = eval.GetLocalizationDistance();
        float localizationPercentage = eval.GetLocalizationPercentage();

        int numLocalizedParticles = 0;
        // Check if each particle is localized
        foreach (GameObject particle in _beliefStates) {
            float dist = Vector3.Distance(particle.transform.position + new Vector3(0, 0, 2), transform.position);
            if (dist <= localizationDistance) {
                numLocalizedParticles++;
            }
        }
        // Check the percentage of particles localized
        if ((float)numLocalizedParticles / _beliefStates.Length >= localizationPercentage) {
            return true;
        }
        return false;
    }


    // Updates agent's weights 
    public void MultiAgentUpdate()
    {
        // Get list of low weight particles that will be shifted
        ArrayList lowWeightParticles = new ArrayList();
        for (int i = 0; i < _beliefStates.Length; i++) {
            // Pick particles for multi-agent update - by low weight and also by fixed selection
            if (i % 10 == 0) {//_weights[i] < 0.7f) { // || i%10==0) {
                lowWeightParticles.Add(_beliefStates[i]);
            }
        }

        // Count neighbors
        int neighborCount = 0;
        ArrayList neighbors = new ArrayList();
        foreach (GameObject agent in _agents) {
            // Check against self
            if (agent == gameObject) {
                continue;
            }
            // Distance is given noiseless - radio simulation
            float distance = Vector3.Distance(transform.position, agent.transform.position);
            if (distance < RADIO_RANGE)
            {
                neighbors.Add(agent);
                neighborCount++;
            }
        }

        if (neighborCount == 0) return;

        // Partition low weight particles into |neighborCount| items in 2D list
        ArrayList[] particlesForEachNeighbor = new ArrayList[neighborCount];
        for (int i = 0; i < neighborCount; i++)
        {
            particlesForEachNeighbor[i] = new ArrayList();
        }

        for (int particleIdx = 0; particleIdx < lowWeightParticles.Count; particleIdx++) {
            int whichNeighbor = particleIdx % neighborCount;
            particlesForEachNeighbor[whichNeighbor].Add(lowWeightParticles[particleIdx]);
        }


        // Update the particles
        for (int i = 0; i < neighbors.Count; i++) { 
            GameObject neighbor = (GameObject)neighbors[i];
            ArrayList particles = particlesForEachNeighbor[i];
            float distance = Vector3.Distance(transform.position, neighbor.transform.position);
            Vector3 otherBeliefPosition = neighbor.GetComponent<ParticleFilter>().GetBeliefAgentPosition();
            Vector3 direction = Vector3.Normalize(neighbor.transform.position - transform.position);
            UpdateParticlesUsingOtherAgent(distance, otherBeliefPosition, particles, direction);
            //UpdateParticlesUsingOtherAgent(distance, otherBeliefPosition, particles);

        }
    }

    //Get some particles with small weights, and shift them based on distance readings from other agents
    public void UpdateParticlesUsingOtherAgent(float distance, Vector3 otherBeliefPosition, ArrayList particles, Vector3 direction)
    {
        foreach (GameObject particle in particles) {
            //Vector3 direction = Random.insideUnitCircle * distance;
            //Vector3 noise = new Vector3(SampleNormal(NOISE_MEAN, NOISE_STD), SampleNormal(NOISE_MEAN, NOISE_STD), 0);
            Vector3 particleLocation = otherBeliefPosition - direction*distance;// + noise;
            particle.transform.position = particleLocation;
        }
    }


}
