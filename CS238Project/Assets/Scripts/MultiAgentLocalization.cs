using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MultiAgentLocalization : MonoBehaviour
{

/*
    const float RADIO_RANGE = 10f;
    GameObject[] _agents;
    // Start is called before the first frame update
    void Start()
    {
        _agents = GameObject.FindGameObjectsWithTag("agents");
    }

    public void MultiAgentWeightUpdate()
    {
        foreach (GameObject agent in _agents) {
            // Check against self
            if (agent == gameObject) {
                continue;
            }
            // Distance is given noiseless - radio simulation
            float distance = Vector3.Distance(transform.position, agent.transform.position);
            if (distance < RADIO_RANGE)
            {
                // Now we can update our agent's weights and cooperatively localize
                Vector3 otherBeliefPosition = agent.GetBeliefAgentPosition();
                UpdateWeightsFromOtherAgent(distance, otherBeliefPosition);
            }
        }
    }

    public void UpdateWeightsFromOtherAgent(float distance, Vector3 otherBeliefPosition)
    {

    }

*/
}
