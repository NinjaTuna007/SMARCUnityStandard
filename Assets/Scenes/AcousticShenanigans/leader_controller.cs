using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VehicleComponents.Actuators;
using VehicleComponents.ROS.Core;
// simple script to make both leaders just move straight ahead with fixed rpm

public class leader_controller : MonoBehaviour
{
    public int group_id = 0;
    
    public float rpm = 1000f;
    public Propeller frontProp, backProp;

    // Start is called before the first frame update
    void Start()
    {
        // pass
    }

    // FixedUpdate is called once per Physics frame
    void FixedUpdate()
    {
        frontProp.SetRpm(rpm);
        backProp.SetRpm(rpm);
    }
}
