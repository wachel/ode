//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//[System.Serializable]
//public class NNPair
//{
//    public NNPair(float reward, Neuron.Network network, Neuron.Network speed)
//    {
//        this.reward = reward;
//        this.network = network;
//        this.speed = speed;
//    }
//    public float reward;
//    public Neuron.Network network;
//    public Neuron.Network speed;
//    public CriticRecord[] record;
//}

//[System.Serializable]
//public class NNSaveData
//{
//    public NNPair[] bestNetworks;
//}

//public class TrainManager : MonoBehaviour {
//    public List<Robot> robots = new List<Robot>();
//    const int BestNum = 10;
//    NNPair[] bestNetworks = new NNPair[BestNum];
//    Neuron.Network actor;
//    Neuron.Network critic;
//    public void Start()
//    {
//        Application.runInBackground = true;
//        int dx = 8;
//        int dy = 8;
//        float distance = 5.0f;
//        for (int i = 0; i < dx; i++) {
//            for (int j = 0; j < dy; j++) {
//                GameObject prefab = Resources.Load<GameObject>("Robot");
//                GameObject obj = GameObject.Instantiate(prefab);
//                obj.SetActive(true);
//                obj.transform.position = new Vector3(i * distance - dx * distance / 2, 0, j * distance - dy * distance / 2);
//                Robot robot = obj.GetComponent<Robot>();
//                robot.network.SetRandom(-0.5f, 0.5f);
//                robots.Add(robot);
//            }
//        }
//        int inputNum = robots[0].network.layers[0].inputNum;
//        int hideNum = robots[0].network.layers[0].neuronNum;
//        int outNum = robots[0].network.layers[1].neuronNum;
//        for (int i = 0; i < BestNum; i++) {
//            Neuron.Network network = new Neuron.Network(inputNum, hideNum, outNum);
//            Neuron.Network speed = new Neuron.Network(inputNum, hideNum, outNum);
//            bestNetworks[i] = (new NNPair(0, network, speed));
//        }
//        critic = new Neuron.Network(inputNum, 50, 1);
//        critic.SetRandom(-0.5f, 0.5f);
//        actor = new Neuron.Network(inputNum, hideNum, outNum);
//        Load();
//    }

//    public void Load()
//    {
//        if (System.IO.File.Exists("best5.json")) {
//            string json = System.IO.File.ReadAllText("best5.json");
//            NNSaveData data  = JsonUtility.FromJson<NNSaveData>(json);
//            bestNetworks = data.bestNetworks;
//        }
//    }

//    public void OnDestroy()
//    {
//        NNSaveData data = new NNSaveData();
//        data.bestNetworks = bestNetworks;
//        string json = JsonUtility.ToJson(data);
//        System.IO.File.WriteAllText("best5.json", json);
//    }

//    public void FixedUpdate()
//    {
//        for (int r = 0; r < robots.Count; r++) {
//            Vector3 force = new Vector3(Random.Range(-1, 1), Random.Range(-1, 1), Random.Range(-1, 1));
//            robots[r].bodies[0].body.AddForce(force * 0.05f);
//        }
//    }

//    public void Update()
//    {
//        for (int r = 0; r < robots.Count; r++) {
//            if (robots[r].IsDead()) {
//                bool changed = false;
//                int lastIndex = bestNetworks.Length - 1;
//                if (bestNetworks.Length > 0 && robots[r].reward > bestNetworks[lastIndex].reward) {
//                    bestNetworks[lastIndex].network.MapEachWeight(robots[r].network, (float otherWeight) => { 
//                        return otherWeight; 
//                    });
//                    bestNetworks[lastIndex].speed.MapEachWeight(robots[r].speed, (float otherWeight) => {
//                        return otherWeight;
//                    });
//                    bestNetworks[lastIndex].reward = robots[r].reward;
//                    changed = true;
//                }
//                System.Array.Sort(bestNetworks,(NNPair v0, NNPair v1) => {
//                    return v0.reward > v1.reward ? -1 : 1; }
//                );
//                if (changed) {
//                    string log = "rewards:[";
//                    for (int i = 0; i < bestNetworks.Length; i++) {
//                        log += ((int)bestNetworks[i].reward).ToString() + ",";
//                    }
//                    log += "]";
//                    Debug.Log(log);
//                }
//                robots[r].Reset();
//                int templateIndex = Random.Range(0, bestNetworks.Length);
//                robots[r].network.ZipEachWeight(bestNetworks[templateIndex].network, bestNetworks[templateIndex].speed, (float weight1, float weight2) => {
//                    float rand = Random.Range(-0.5f, 0.5f);
//                    return weight1 + rand * rand * rand + weight2;
//                });
//                //get change
//                robots[r].speed.ZipEachWeight(bestNetworks[templateIndex].network, robots[r].network, (float weight1, float weight2) => {
//                    return weight2 - weight1;
//                });
//                //smooth change speed
//                robots[r].speed.ZipEachWeight(bestNetworks[templateIndex].speed, robots[r].speed, (float weight1, float weight2) => {
//                    return Mathf.Lerp(weight1, weight2, 0.1f);
//                });
                
//            }
//        }
//        for (int i = 0; i < bestNetworks.Length; i++) {
//            bestNetworks[i].reward *= 0.9999f;
//            bestNetworks[i].speed.ForEachWeight((float weight) => {
//                return weight * 0.999f;
//            });
//        }
//    }
//}
