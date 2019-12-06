using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class AgentBehavior : MonoBehaviour
{
    public int SEED = 0;
    private const int UPDATE_FREQ = 30;
    private const float FORCE_UPDATE_MAG = 30f;
    Random.State oldState;

    int updateCounter = 0;

    // Start is called before the first frame update
    void Start()
    {
        Random.InitState(SEED);
        oldState = Random.state;
    }

    // Update is called once per frame
    void Update()
    {
        if (updateCounter % UPDATE_FREQ == 0)
        {
            // Make sure other calls to Random in other scipts doesn't mess up the deterministic random here
            Random.state = oldState;
            //Uniform distrib centered around 0, range is [-0.5, 0.5)
            float noiseX = Random.Range(-FORCE_UPDATE_MAG, FORCE_UPDATE_MAG);
            float noiseY = Random.Range(-FORCE_UPDATE_MAG, FORCE_UPDATE_MAG);
            oldState = Random.state;
            GetComponent<Rigidbody>().AddForce(new Vector2(noiseX, noiseY));
        }
        updateCounter++;
    }
}
