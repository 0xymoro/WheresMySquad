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
    float cluster_radius = 5.0f;

    // Current state of the agent
    private Vector3 _agentPosition;

    // Lidar
    private Lidar _lidar_script;


    // Other agent variables
    const float RADIO_RANGE = 6f;
    GameObject[] _agents;
    const float MULTI_AGENT_THRESHOLD = 0.3f; // if confidence < threshold, reposition particle based on other agent

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
        _beliefPosition.GetComponent<Renderer>().materials = GetComponent<Renderer>().materials;


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

    // This struct contains the position of a particle and which cluster it belongs to.
    // Used for clustering belief particles to determine the belief position
    struct ParticleCluster {
        Vector3 position;
        int clusterNumber;
    }

    // Returns where the agent believes it's at.
    // public Vector3 GetBeliefAgentPosition()
    // {
    //     // Initially all particles are unclustered and are put into the unclustered set
    //     HashSet<Vector3> unclusteredPoints = new HashSet<Vector3>();
    //     for (int i = 0; i < _beliefStates.Length; i++) {
    //         unclusteredPoints.Add(_beliefStates[i].transform.position);
    //     }

    //     List<Vector3> maxCluster = new List<Vector3>();
    //     // Computation is only done on unclustered points and eventually all points should be clustered
    //     // Start with one random unclustered point and add all its neighbors to the cluster then iterate through all the added
    //     // neighbors and add more points to be cluster if applicable
    //     while (unclusteredPoints.Count != 0) {
    //         // Take one unclustered point and put it in a new cluster
    //         Vector3 startPoint = unclusteredPoints.GetEnumerator().Current;
    //         unclusteredPoints.Remove(startPoint);
    //         List<Vector3> cluster = new List<Vector3>();
    //         cluster.Add(startPoint);

    //         int referenceIndex = 0;
    //         // Loop through each point in the cluster. 
    //         // Number of points in the cluster is changed dynamically as more points are found and added
    //         while (referenceIndex < cluster.Count) {
    //             // Point in the cluster that all unassigned points are compared to
    //             Vector3 referencePoint = cluster[referenceIndex];
    //             // If any unassigned points are within radius of the reference point, that new point is assigned to this cluster
    //             foreach (Vector3 point in unclusteredPoints) {
    //                 if (Vector3.Distance(point, referencePoint) <= cluster_radius) {
    //                     cluster.Add(point);
    //                 }
    //             }
    //             // Remove all the newly clustered points from the set of unclustered points
    //             for (int i = referenceIndex + 1; i < cluster.Count; i++) {
    //                 unclusteredPoints.Remove(cluster[i]);
    //             }
    //             referenceIndex++;
    //         }
    //         // Keep this cluster if it's the biggest cluster found
    //         if (cluster.Count > maxCluster.Count) {
    //             maxCluster = cluster;
    //         }
    //     }
        
    //     // Compute average of all the points in the largest cluster to get the belief position
    //     Vector3 beliefPosition = new Vector3(0, 0, 0);
    //     foreach (Vector3 position in maxCluster) {
    //         beliefPosition += position;
    //     }
    //     beliefPosition /= maxCluster.Count;
    
    //     return beliefPosition;
    // }

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
        List<GameObject> lowWeightParticles = new List<GameObject>();
        for (int i = 0; i < _beliefStates.Length; i++) {
            // Pick particles for multi-agent update - by low weight
            if (_weights[i] < MULTI_AGENT_THRESHOLD) {
                lowWeightParticles.Add(_beliefStates[i]);
            }
        }

        // Count neighbors
        int neighborCount = 0;
        List<GameObject> neighbors = new List<GameObject>();
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

        if (neighborCount == 0) {
            return;
        }

        // Partition low weight particles into |neighborCount| items in 2D list
        List<GameObject>[] particlesForEachNeighbor = new List<GameObject>[neighborCount];
        for (int i = 0; i < neighborCount; i++)
        {
            particlesForEachNeighbor[i] = new List<GameObject>();
        }

        for (int particleIdx = 0; particleIdx < lowWeightParticles.Count; particleIdx++) {
            int whichNeighbor = particleIdx % neighborCount;
            particlesForEachNeighbor[whichNeighbor].Add(lowWeightParticles[particleIdx]);
        }

        if (neighborCount != 2) {
            // Update the particles
            for (int i = 0; i < neighbors.Count; i++) { 
                GameObject neighbor = neighbors[i];
                List<GameObject> particles = particlesForEachNeighbor[i];
                float distance = Vector3.Distance(transform.position, neighbor.transform.position);
                Vector3 otherBeliefPosition = neighbor.GetComponent<ParticleFilter>().GetBeliefAgentPosition();
                UpdateParticlesUsingOtherAgent(distance, otherBeliefPosition, particles);
            }
        } 
        else {
            Vector3 beliefPosition1 = neighbors[0].GetComponent<ParticleFilter>().GetBeliefAgentPosition();
            Vector3 beliefPosition2 = neighbors[1].GetComponent<ParticleFilter>().GetBeliefAgentPosition();
            float r1 = Vector3.Distance(transform.position, neighbors[0].transform.position);
            float r2 = Vector3.Distance(transform.position, neighbors[1].transform.position);
            List<Vector3> intersections = GetCircleIntersection(beliefPosition1, beliefPosition2, r1, r2);
            Debug.Log(intersections.Count);
            if (intersections.Count == 2) {
                foreach (GameObject particle in particlesForEachNeighbor[0]) {
                    particle.transform.position = intersections[0];
                }
                foreach (GameObject particle in particlesForEachNeighbor[1]) {
                    particle.transform.position = intersections[1];
                }
            }
        }
        
    }

    //Get some particles with small weights, and shift them based on distance readings from other agents
    public void UpdateParticlesUsingOtherAgent(float distance, Vector3 otherBeliefPosition, List<GameObject> particles)
    {
        foreach (GameObject particle in particles) {
            Vector3 direction = Vector3.Normalize(Random.insideUnitCircle);
            Vector3 particleLocation = otherBeliefPosition + direction * distance;
            particle.transform.position = particleLocation;
        }
    }

    List<Vector3> GetCircleIntersection(Vector3 center1, Vector3 center2, float r1, float r2)
    {
        // There can be at most two intersections with two circles
        List<Vector3> intersections = new List<Vector3>();
        float centersDistance = Vector3.Distance(center1, center2);
        // No solutions when the two circles don't touch or the two circles are within one another or when they're the same circle
        if ((centersDistance > r1 + r2) || (centersDistance < Mathf.Abs(r1 - r2)) || centersDistance == 0) {
            return intersections;
        }

        // Assume there are two intersections. Draw a line between the intersections and draw another line between the two centers. 
        // Where theses two lines cross is defined as point p. 
        // The distance from center 1 to p is distance center1_to_p
        float center1_to_p = (Mathf.Pow(r1, 2) - Mathf.Pow(r2, 2) + Mathf.Pow(centersDistance, 2)) / (2 * centersDistance);
        Vector3 p = center1 + center1_to_p * (center2 - center1) / centersDistance;
        // If the two circles touch at one point, then center1_to_p should be r1
        if (center1_to_p == r1) {
            intersections.Add(center1 + p);
            return intersections;
        }

        // h is the distance from p to the intersection along the line between the two intersections
        float h = Mathf.Sqrt(Mathf.Pow(r1, 2) - Mathf.Pow(center1_to_p, 2));
        intersections.Add(new Vector3(p.x + h * (center2.y - center1.y) / centersDistance, p.y - h * (center2.x - center1.x) / centersDistance, -2));
        intersections.Add(new Vector3(p.x - h * (center2.y - center1.y) / centersDistance, p.y + h * (center2.x - center1.x) / centersDistance, -2));
        return intersections;
    }

}
