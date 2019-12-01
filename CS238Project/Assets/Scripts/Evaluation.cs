using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// The performance of the localization algorithm is evaluated based on the time it takes to localize all the agents.
// Once all the agents are localized, the time is printed.
public class Evaluation : MonoBehaviour
{
    static int UPDATE_FREQ = 5;
    int updateCounter = 0;
    bool allLocalized = false;
    float startTime;
    GameObject[] agents;
    // A particle correctly localizes an agent if it's within this distance from the agent
    public float localizationDistance = 1.5f;
    // If at least this percentage of the particles are considered localized then the agent is considered localized
    public float localizationPercentage = 0.9f;

    // Start is called before the first frame update
    void Start()
    {
        startTime = Time.time;
        agents = GameObject.FindGameObjectsWithTag("agents");
        Debug.Log("Number of agents detected: " + agents.Length);
    }

    // Update is called once per frame
    void Update()
    {
        if (updateCounter % UPDATE_FREQ == 0 && !allLocalized) {
            allLocalized = AllAgentsLocalized();
            if (allLocalized) {
                Debug.Log("All localized in: " + (Time.time - startTime) + "s");
            }
        }
        updateCounter++;
    }

    // Returns true if all agents are localized. False otherwise
    bool AllAgentsLocalized()
    {
        bool allLocalized = true;
        foreach (GameObject agent in agents) {
            bool isLocalied = agent.GetComponent<ParticleFilter>().AgentLocalized();
            allLocalized = allLocalized && isLocalied;
        }
        return allLocalized;
    }

    public float GetLocalizationDistance()
    {
        return localizationDistance;
    }

    public float GetLocalizationPercentage()
    {
        return localizationPercentage;
    }
}
