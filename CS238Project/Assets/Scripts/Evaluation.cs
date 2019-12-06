using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// The performance of the localization algorithm is evaluated based on the time it takes to localize all the _agents.
// Once all the _agents are localized, the time is printed.
public class Evaluation : MonoBehaviour
{
    static int UPDATE_FREQ = 5;
    int updateCounter = 0;
    bool allLocalized = false;
    bool _evalInitialized = false;
    public int _evalTypeIndex = 0;
    string[] _evalType = {"naive", "advanced", "advanced_triangulate"};
    float startTime;
    GameObject[] _agents;
    // A particle correctly localizes an agent if it's within this distance from the agent
    public float localizationDistance = 1.5f;
    // If at least this percentage of the particles are considered localized then the agent is considered localized
    public float localizationPercentage = 0.9f;

    // Start is called before the first frame update
    void Start()
    {
        startTime = Time.time;
        _agents = GameObject.FindGameObjectsWithTag("agents");
        Debug.Log("Number of _agents detected: " + _agents.Length);
        _evalInitialized = true;
    }

    // Update is called once per frame
    void Update()
    {
        if (updateCounter % UPDATE_FREQ == 0 && !allLocalized) {
            allLocalized = All_agentsLocalized();
            if (allLocalized) {
                Debug.Log("All localized in: " + (Time.time - startTime) + "s");
            }
        }
        updateCounter++;
    }

    // Returns true if all _agents are localized. False otherwise
    bool All_agentsLocalized()
    {
        bool allLocalized = true;
        foreach (GameObject agent in _agents) {
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

    // Returns true if evaluation script is initialized
    public bool EvalScriptInitialized()
    {
        return _evalInitialized;
    }

    // Get the type of algorithm that's being evaluated
    public string GetEvalType()
    {
        return _evalType[_evalTypeIndex];
    }
}
