using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MultiAgentLocalization : MonoBehaviour
{
    GameObject[] _agents;
    // Start is called before the first frame update
    void Start()
    {
        _agents = GameObject.FindGameObjectsWithTag("agents");
    }

    // Update is called once per frame
    void LateUpdate()
    {
        foreach (GameObject agent in _agents) {
            // Check against self
            if (agent == gameObject) {
                continue;
            }

            Vector3 beliefPosition = agent.
        }
    }
}
