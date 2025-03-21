using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleComponents.Actuators;

using DefaultNamespace; // ResetArticulationBody() extension

// class to control the formation based on acoustic signals from leaders
public class acousticformation : Agent
{

    public int group_id = 0;


    // methods to define
    public Transform leader0, leader1;

    // initialize class variables
    public float desired_dist_0_, desired_dist_1_;

    // stuff for the editor to actually set actions: thrusters x2, thrust vector x2, vbs, lcg
    public Propeller frontProp, backProp;
    public Hinge thrustorYaw, thrustorPitch;
    public VBS vbs;
    public Prismatic lcg;

    // history of distance measurements
    private Queue<float> dist0History;
    private Queue<float> dist1History;
    public int historyLength = 5; // length of the history

    // action frequency
    public int act_per_second;

    private Vector3 leader_0_pos, leader_0_vel, leader_1_pos, leader_1_vel, agent_pos, agent_vel;
    private int reset_in_progress = 0;

    private Vector3 agent_pos_noise;
    private Quaternion agent_rot_noise;

    private float episodeStartTime;

    // array of size 2 to store latest measurements from both leaders: initialize both as -1
    private float[] dists = new float[2] { -1, -1 };
    private float[][] dists_hist;

    private int init_frame = 0;

    public override void Initialize()
    {
        // define desired distances from leaders 0 and 1: 10 sqrt 2
        // desired_dist_0_ = 15;
        // desired_dist_1_ = 15f;

        // initialize the history queues
        dist0History = new Queue<float>(historyLength);
        dist1History = new Queue<float>(historyLength);

        // define action frequency
        act_per_second = 5;

        // store jointPositions and jointVelocities of all agents
        // leader_0_pos = leader0.GetComponent<ArticulationBody>().GetJointPositions();
        // leader_0_vel = leader0.GetComponent<ArticulationBody>().GetJointVelocities();
        // initialize the dists_hist array
        
        // initialize the history queues
        // leader_1_vel = leader1.GetComponent<ArticulationBody>().GetJointVelocities();
        // agent_pos = GetComponentInChildren<ArticulationBody>().GetJointPositions();
        // agent_vel = GetComponentInChildren<ArticulationBody>().GetJointVelocities();

        leader_0_pos = new Vector3(-10f,0f,0f) + new Vector3(30f,0f,0f) * group_id;
        leader_0_vel = new Vector3(0f,0f,0f);
        leader_1_pos = new Vector3(10f,0f,0f) + new Vector3(30f,0f,0f) * group_id;
        leader_1_vel = new Vector3(0f,0f,0f);
        agent_pos = new Vector3(0f,0f,-10f) + new Vector3(30f,0f,0f) * group_id;
        agent_vel = new Vector3(0f,0f,0f);

        episodeStartTime = Time.time;

        // call the on episode begin method to reset the environment
        reset_in_progress = 1;
        OnEpisodeBegin();
        OnEpisodeBegin();

    }

    public override void OnEpisodeBegin()
    {
        // reset the environment
        Debug.Log("Episode started");

        if (reset_in_progress != 0)
        {         
            // Reset leaders
            RelocateArticulationBody(leader0.GetComponent<ArticulationBody>(), leader_0_pos, Quaternion.identity);
            RelocateArticulationBody(leader1.GetComponent<ArticulationBody>(), leader_1_pos, Quaternion.identity);

            // Reset agent

            ArticulationBody agentArticulationBody = GetComponentInChildren<ArticulationBody>();

            // Perturb position of agent to make the task harder
            float pos_noise = 2f;
            float rot_noise = 10f;

            Vector3 perturbedPosition = new Vector3(0, 0, 0);

            if (reset_in_progress == 1){
                agent_pos_noise = new Vector3(Random.Range(-pos_noise, pos_noise), 0, Random.Range(-pos_noise, pos_noise)); // don't perturb the y position: depth
                // zero option
                // agent_pos_noise = Vector3.zero;
                
                // agent_rot_noise = Quaternion.Euler(0, Random.Range(-rot_noise, rot_noise), 0);
                //  identity option
                agent_rot_noise = Quaternion.identity;
            }

            // Perturb orientation of agent to make the task harder
            RelocateArticulationBody(agentArticulationBody, agent_pos + agent_pos_noise, Quaternion.identity * agent_rot_noise);

            reset_in_progress += 1;

            reset_in_progress = reset_in_progress % 3;


        }
        // Clear the history queues
        dist0History.Clear();
        dist1History.Clear();

        // Reset episode start time
        episodeStartTime = Time.time;

        dists_hist = new float[2][];
        for (int i = 0; i < 2; i++)
        {
            dists_hist[i] = new float[historyLength];
            for (int j = 0; j < historyLength; j++)
            {
                dists_hist[i][j] = -1;
            }
        }



    }


    private void RelocateArticulationBody(ArticulationBody articulationBody, Vector3 position, Quaternion rotation)
    {
        articulationBody.immovable = true;

        articulationBody.TeleportRoot(position, rotation);
        
        articulationBody.linearVelocity = Vector3.zero;
        articulationBody.angularVelocity = Vector3.zero;

        foreach (ArticulationBody child in articulationBody.GetComponentsInChildren<ArticulationBody>())
        {
            ArticulationReducedSpace zeroPos = new ArticulationReducedSpace(child.dofCount);
            child.jointPosition = zeroPos;

            ArticulationReducedSpace zeroVel = new ArticulationReducedSpace(child.dofCount);
            child.jointVelocity = zeroVel;
            // child.jointAcceleration = zeroVel;
            child.jointForce = zeroVel;

            child.linearVelocity = Vector3.zero;
            child.angularVelocity = Vector3.zero;
            child.ResetArticulationBody();
        }

        foreach (Rigidbody rb in articulationBody.GetComponentsInChildren<Rigidbody>())
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        articulationBody.immovable = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {

        // collect observations

        //agent articulation body
        ArticulationBody agentArticulationBody = GetComponentInChildren<ArticulationBody>();

        // use game object transform and leader transform to get distance measurements from both leaders
        float dist_0 = Vector3.Distance(agentArticulationBody.transform.position, leader0.position);
        float dist_1 = Vector3.Distance(agentArticulationBody.transform.position, leader1.position);

        // optional: add noise to the distance measurements to make the task harder
        // dist_0 += Random.Range(-1, 1);
        // dist_1 += Random.Range(-1, 1);

        // debug log
        // Debug.Log("Distance from leader 0: " + dist_0);
        // Debug.Log("Distance from leader 1: " + dist_1);

        // add the current distances to the history queues

        // if (Time.time % 2 == 0) // even seconds: leader 0 measurements are known
        float elapsedTime = Time.time - episodeStartTime;

        if (Mathf.Abs(elapsedTime - (int)elapsedTime) < Time.fixedDeltaTime) // integer seconds: leader 0 measurements are known
        {
            // Debug.Log("Elapsed time: " + elapsedTime);
            int num_seconds = (int)elapsedTime;

            if (num_seconds % 2 == 0) // even seconds: leader 0 measurements are known
            {
                if (dist0History.Count >= historyLength)
                {
                    dist0History.Dequeue();
                }
                dist0History.Enqueue(dist_0);

                dists[0] = dist_0; // leader 0 distance is known

                // populate dists_hist [0]: look for the first empty (-1) slot and fill it with dist_0
                bool is_full = true;
                for (int i = 0; i < historyLength; i++)
                {
                    if (dists_hist[0][i] == -1)
                    {
                        dists_hist[0][i] = dist_0;
                        is_full = false;
                        break;
                    }
                }

                // if dists_hist [0] is full, shift all elements to the left and add dist_0 to the last slot
                if (is_full)
                {
                    for (int i = 0; i < historyLength - 1; i++)
                    {
                        dists_hist[0][i] = dists_hist[0][i + 1];
                    }
                    dists_hist[0][historyLength - 1] = dist_0;
                }

            }

            // else if (Time.time % 2 == 1) // odd seconds: leader 1 measurements are known
            else // odd seconds: leader 1 measurements are known
            {
                if (dist1History.Count >= historyLength)
                {
                    dist1History.Dequeue();
                }
                dist1History.Enqueue(dist_1);

                dists[1] = dist_1; // leader 1 distance is known

                // populate dists_hist [1]: look for the first empty (-1) slot and fill it with dist_1
                bool is_full = true;
                for (int i = 0; i < historyLength; i++)
                {
                    if (dists_hist[1][i] == -1)
                    {
                        dists_hist[1][i] = dist_1;
                        is_full = false;
                        break;
                    }
                }

                // if dists_hist [1] is full, shift all elements to the left and add dist_1 to the last slot
                if (is_full)
                {
                    for (int i = 0; i < historyLength - 1; i++)
                    {
                        dists_hist[1][i] = dists_hist[1][i + 1];
                    }
                    dists_hist[1][historyLength - 1] = dist_1;

                }
            }
        }


        // simpler version: just add dists to observations
        // Debug.Log("Sensor readings: " + dists[0] + ", " + dists[1]);
        // Debug.Log("Frame count: " + frame_count);
        // sensor.AddObservation(dists);

        // flatten the dists_hist array and add it to the observations
        float[] dists_hist_flat = new float[2 * historyLength];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < historyLength; j++)
            {
                dists_hist_flat[i * historyLength + j] = dists_hist[i][j];
            }
        }

        sensor.AddObservation(dists_hist_flat);


        // Queue<float> full_history_queue = new Queue<float>(dist0History);

        // for (int i = 0; i < dist1History.Count; i++)
        // {
        //     full_history_queue.Enqueue(dist1History.Dequeue());
        // }

        // IEnumerable<float> full_history = full_history_queue;

        // add the history of distances to the observations
        // sensor.AddObservation(full_history_queue);
    
        // // add the history of distances to the observations
        // foreach (float d in dist0History)
        // {
        //     sensor.AddObservation(d);
        // }
        // foreach (float d in dist1History)
        // {
        //     sensor.AddObservation(d);
        // }

        // Debug.Log("Sensor readings: " + full_history);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Clamp actions to the range [-1, 1]
        float clampedRpmBack = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float clampedRpmFront = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float clampedYaw = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        // float clampedPitch = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);
        // float clampedVbsVal = Mathf.Clamp(actions.ContinuousActions[4], -1f, 1f);
        // float clampedLcgVal = Mathf.Clamp(actions.ContinuousActions[5], -1f, 1f);

        // Scale actions to the correct range
        float rpm_back = (clampedRpmBack + 1) * 500;
        float rpm_front = (clampedRpmFront + 1) * 500;
        float yaw_control = clampedYaw * 0.05f;
        // float pitch = clampedPitch;
        // float vbs_val = (clampedVbsVal + 1) * 50;
        // float lcg_val = (clampedLcgVal + 1) * 50;

        // articulation body var definitions
        ArticulationBody agentArticulationBody = GetComponentInChildren<ArticulationBody>();

        // Apply actions
        frontProp.SetRpm(rpm_front);
        backProp.SetRpm(rpm_back);
        thrustorYaw.SetAngle(yaw_control);
        // thrustorPitch.SetAngle(pitch);
        // vbs.SetPercentage(vbs_val);
        // lcg.SetPercentage(lcg_val);

        // Debug log
        Debug.Log("RPM front: " + rpm_front);
        // Debug.Log("RPM back: " + rpm_back);
        Debug.Log("Yaw: " + yaw_control);
        // Debug.Log("Pitch: " + pitch);
        // Debug.Log("VBS: " + vbs_val);
        // Debug.Log("LCG: " + lcg_val);

        // Store the last action
        // last_action_ = actions;

        // set reward for the action taken

        // reward: 1 - tanh(delta_dist_0 + delta_dist_1)
        // reward is 1 when the agent is at the desired distance from both leaders, and decays to 0 as the agent moves away from the desired distance
        float flat_param = 15f;

        float delta_dist_0 = Mathf.Abs(dists[0] - desired_dist_0_);
        float delta_dist_1 = Mathf.Abs(dists[1] - desired_dist_1_);

        agentArticulationBody = GetComponentInChildren<ArticulationBody>();


        // if z value of follower is less than z value of leader, reward exists. Otherwise 0
        double ts_reward = (1 - System.Math.Tanh((delta_dist_0 + delta_dist_1)/flat_param)) * 1f;

        if (agentArticulationBody.transform.position.z < leader0.position.z)
        {
            AddReward((float)ts_reward);
        }

        // reward for minimizing roll, pitch, and yaw
        float roll = agentArticulationBody.transform.rotation.eulerAngles.x;
        float pitch = agentArticulationBody.transform.rotation.eulerAngles.y;
        float yaw = agentArticulationBody.transform.rotation.eulerAngles.z;

        // orientation reward: 1 - tanh(roll + pitch + yaw)
        // reward is 1 when the agent is level, and decays to 0 as the agent rolls, pitches, or yaws
        double orientation_reward = (1 - System.Math.Tanh((roll + pitch + yaw)/flat_param)) * 0.01f;

        // AddReward((float)orientation_reward);

    }

    // fixed update calling collect observations
    void FixedUpdate()
    {

        if (reset_in_progress != 0)
        {
            OnEpisodeBegin();
        }

        // Check if the "O" key is pressed to reset the environment
        if (Input.GetKeyDown(KeyCode.O) || init_frame < 2)
        {
            reset_in_progress = 1;
            OnEpisodeBegin();
            init_frame += 1;
        }
    
        // debug log
        // Debug.Log("Distance from leader 0: " + dist_0);
        // Debug.Log("Distance from leader 1: " + dist_1);

        // check for death by distance
        DeathByDistance();

        // collect observations only act_per_second times every second
        float elapsedTime = Time.time - episodeStartTime;
        if ((int)(elapsedTime * act_per_second) % act_per_second == 0)
        {
            // RequestDecision();
            RequestAction();
        }


        // survival reward: 0.0001f
        AddReward(0.01f);

        // debug log
        // Debug.Log("Reward: " + ts_reward);

        // Debug.Log("Frame count: " + frame_count);

        // update frame count
        // frame_count += 1;

    }

    void DeathByDistance()
    {
        // if the distance from either leader is too large, end the episode

        ArticulationBody agentArticulationBody = GetComponentInChildren<ArticulationBody>();

        float dist_0 = Vector3.Distance(agentArticulationBody.transform.position, leader0.position);
        float dist_1 = Vector3.Distance(agentArticulationBody.transform.position, leader1.position);

        float dist_tolerance = 5f;

        if (Mathf.Abs(dist_0 - desired_dist_0_) > dist_tolerance || Mathf.Abs(dist_1 - desired_dist_1_) > dist_tolerance)
        {
            Debug.Log("Distance threshold exceeded, ending episode");
            AddReward(-1f);

            reset_in_progress = 1;
            
            EndEpisode();
        }
    }
}