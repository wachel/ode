using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sample
{
    public float[] input;
    public float[] output;
}

[System.Serializable]
public class RewardPair
{
    public RewardPair(float reward, Neuron.Network network)
    {
        this.reward = reward;
        this.network = network;
    }
    public float reward;
    public Neuron.Network network;
}

[System.Serializable]
public class RobotInfo
{
    public Robot robot;
    public Neuron.Network network;
    public float reward;
    public float startTime;
}

[System.Serializable]
public class SaveData
{
    public RewardPair[] bestNetworks;
}

public class TrainManager2 : MonoBehaviour {
    public RobotInfo leader;
    public List<RobotInfo> robots = new List<RobotInfo>();
    public RobotState robotState;
    const int BestNum = 10;
    RewardPair[] bestNetworks = new RewardPair[BestNum];
    public void Start()
    {
        Application.runInBackground = true;
        int dx = 8;
        int dy = 8;
        float distance = 5.0f;

        leader = new RobotInfo();
        leader.robot = CreateRobot();
        leader.robot.transform.position = new Vector3(0 , 0, - (dy+1) * distance / 2.0f);
        leader.robot.DoStart();

        robotState = new RobotState(leader.robot.bodies.Length,leader.robot.muscles.Length);

        int inputNum = leader.robot.inputs.Length;
        int hideNum = 50;
        int outNum = leader.robot.muscles.Length;

        leader.network = new Neuron.Network(inputNum, hideNum, outNum);
        leader.network.SetRandom(-0.5f, 0.5f);

        for (int i = 0; i < dx; i++) {
            for (int j = 0; j < dy; j++) {
                RobotInfo robotInfo = new RobotInfo();
                Robot robot = CreateRobot();
                robot.transform.position = new Vector3(i * distance - (dx - 1) * distance / 2.0f, 0, j * distance - (dy - 1) * distance / 2.0f);
                robot.DoStart();
                robotInfo.robot = robot;
                robotInfo.network = new Neuron.Network(inputNum,hideNum,outNum);
                robotInfo.network.SetRandom(-0.5f, 0.5f);
                robotInfo.startTime = Time.time;
                robots.Add(robotInfo);
            }
        }

       for (int i = 0; i < BestNum; i++) {
           Neuron.Network bn = new Neuron.Network(inputNum, hideNum, outNum);
           bestNetworks[i] = (new RewardPair(0, bn));
       }

        Load();
    }

    Robot CreateRobot()
    {
        GameObject prefab = Resources.Load<GameObject>("Robot");
        GameObject obj = GameObject.Instantiate(prefab);
        obj.SetActive(true);
        Robot robot = obj.GetComponent<Robot>();
        return robot;
    }


    public void Load()
    {
        if (System.IO.File.Exists("best5.json")) {
            string json = System.IO.File.ReadAllText("best5.json");
            SaveData data = JsonUtility.FromJson<SaveData>(json);
            bestNetworks = data.bestNetworks;
        }
    }

    public void OnDestroy()
    {
        SaveData data = new SaveData();
        data.bestNetworks = bestNetworks;
        string json = JsonUtility.ToJson(data);
        System.IO.File.WriteAllText("best5.json", json);
    }


    float[] tempDisturbance;

    public void FixedUpdate()
    {
        leader.robot.UpdateInputs();
        leader.robot.UpdateMuscle(leader.network.Forward(leader.robot.inputs));
        leader.reward += leader.robot.GetReward();

        if (tempDisturbance == null) {
            tempDisturbance = new float[leader.robot.muscles.Length];
        }

        for (int r = 0; r < robots.Count; r++) {
            robots[r].robot.UpdateInputs();
            float[] input = robots[r].robot.inputs;
            float[] activity = robots[r].network.Forward(input);
            robots[r].robot.UpdateMuscle(activity);
            robots[r].reward += robots[r].robot.GetReward();
        }

        if (!leader.robot.IsAlive()) {
            leader.network.Copy(bestNetworks[0].network);
            leader.robot.Reset();
        }

        for (int r = 0; r < robots.Count; r++) {
            if (!robots[r].robot.IsAlive()) {
                bool changed = false;
                int lastIndex = bestNetworks.Length - 1;
                if (bestNetworks.Length > 0 && robots[r].reward > bestNetworks[lastIndex].reward) {
                    bestNetworks[lastIndex].network.Copy(robots[r].network);
                    bestNetworks[lastIndex].reward = robots[r].reward;
                    changed = true;
                }
                System.Array.Sort(bestNetworks, (RewardPair v0, RewardPair v1) => {
                    return v0.reward > v1.reward ? -1 : 1;
                });
                if (changed) {
                    string log = "rewards:[";
                    for (int i = 0; i < bestNetworks.Length; i++) {
                        log += ((int)bestNetworks[i].reward).ToString() + ",";
                    }
                    log += "]";
                    Debug.Log(log);
                }
                robots[r].robot.Reset();
                robots[r].reward = 0;
                robots[r].startTime = Time.time;
                int templateIndex = Random.Range(0, bestNetworks.Length);
                robots[r].network.MapEachWeight(bestNetworks[templateIndex].network,(float weight) => {
                    float rand = Random.Range(-0.5f, 0.5f);
                    return weight + rand * rand * rand;
                });
            }
        }

        for (int r = 0; r < robots.Count; r++) {
            Vector3 force = new Vector3(Random.Range(-1,1),Random.Range(-1,1),Random.Range(-1,1));
            float age = Time.time - robots[r].startTime;
            robots[r].robot.head.body.AddForce(force * age * 2);
        }

        for (int i = 0; i < bestNetworks.Length; i++) {
            bestNetworks[i].reward *= 0.9996f;
            //bestNetworks[i].reward -= 0.1f;
        }
    }

    public void Update()
    {

    }
}
