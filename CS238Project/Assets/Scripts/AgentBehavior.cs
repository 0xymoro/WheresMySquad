using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class AgentBehavior : MonoBehaviour
{
    const int SEED = 1337;
    const int UPDATE_FREQ = 30;
    const float FORCE_UPDATE_MAG = 30f;



    int updateCounter = 0;

    // Start is called before the first frame update
    void Start()
    {
        Random.InitState(SEED);
    }

    // Update is called once per frame
    void Update()
    {
        if (updateCounter % UPDATE_FREQ == 0)
        {
            //Uniform distrib centered around 0, range is [-0.5, 0.5)
            float noiseX = Random.Range(-FORCE_UPDATE_MAG, FORCE_UPDATE_MAG);
            float noiseY = Random.Range(-FORCE_UPDATE_MAG, FORCE_UPDATE_MAG);
            GetComponent<Rigidbody>().AddForce(new Vector2(noiseX, noiseY));

        }

        updateCounter++;

    }
}
