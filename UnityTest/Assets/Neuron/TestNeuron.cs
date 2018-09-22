using UnityEngine;
using System.Collections;

public class TestNeuron : MonoBehaviour {
    Neuron.Network nn;
	void Start () {
        //nn = new Neuron.Network(2,5,1);
        //nn.SetRandom(-0.5f, 0.5f);
        nn = new Neuron.Network(256,64,8);
        nn.SetRandom(-0.5f, 0.5f);
	}
	
	// Update is called once per frame
	void Update () {
        float error = 0;
        error = Train();

       // Debug.Log("error : " + error);
	}

    float[] input = new float[256];
    float[] expected = new float[8];
    float[] error = new float[8];
    float Train()
    {
        //float[] input = {1,1, 1,0, 0,1, 0,0};
        //float[] target = {0,1,1,0};
        //int index = Random.Range(0, 4);
        //float[] a = nn.Forward(new float[] {input[index * 2],input[index * 2 + 1]});
        //float[] error = new float[] { target[index] - a[0]};
        byte num = (byte)Random.Range(0, 256);
        for (int i = 0; i < 256; i++) {
            input[i] = num == i ? 1 : 0;
        }
        for (int i = 0; i < 8; i++) {
            expected[i] = ((1 << i) & num) != 0?1:0;
        }

        float[] a = nn.Forward(input);
        VMath.Sub(expected, a, error);
        nn.BackPropagate(error, 0.1f);
        return VMath.Sum(error);
    }
}
