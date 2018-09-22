using UnityEngine;
using System.Collections;

namespace Neuron{
    public delegate float FunChangeWeight(float weight);
    public delegate float FunMap(float weight);
    public delegate float FunZip(float weight1, float weight2);


    [System.Serializable]
    public class Neuron
    {
        public float[] w;
    }
    
    [System.Serializable]
    public class Layer
    {
        public int inputNum;
        public int neuronNum;
        public Neuron[] weights;
        public float[] bias;
        float[] input;
        float[] active;
        float[] delta;
        float[] inputError;
        public Layer(int inputNum,int neuronNum)
        {
            this.inputNum = inputNum;
            this.neuronNum = neuronNum;
            weights = new Neuron[neuronNum];
            for (int i = 0; i < neuronNum; i++) {
                weights[i] = new Neuron();
                weights[i].w = new float[inputNum];
            }
            bias = new float[neuronNum];
            active = new float[neuronNum];
            delta = new float[neuronNum];
            inputError = new float[inputNum];
            input = new float[inputNum];
        }
        public void SetRandom(float min, float max)
        {
            for (int i = 0; i < neuronNum; i++) {
                VMath.Random(weights[i].w, min, max);
            }
            VMath.Random(bias, min, max);
        }
        public void ForEachWeight(FunChangeWeight fun)
        {
            for (int i = 0; i < neuronNum; i++) {
                for (int j = 0; j < inputNum; j++) {
                    weights[i].w[j] = fun(weights[i].w[j]);
                }
                {
                    bias[i] = fun(bias[i]);
                }
            }
        }
        public void MapEachWeight(FunMap fun, Layer other)
        {
            for (int i = 0; i < neuronNum; i++) {
                for (int j = 0; j < inputNum; j++) {
                    weights[i].w[j] = fun(other.weights[i].w[j]);
                }
                {
                    bias[i] = fun(other.bias[i]);
                }
            }
        }
        public void ZipEachWeight(FunZip fun, Layer layer1, Layer layer2)
        {
            for (int i = 0; i < neuronNum; i++) {
                for (int j = 0; j < inputNum; j++) {
                    weights[i].w[j] = fun(layer1.weights[i].w[j], layer2.weights[i].w[j]);
                }
                {
                    bias[i] = fun(layer1.bias[i], layer2.bias[i]);
                }
            }
        }
        public float[] Forward(float[] input)
        {
            System.Array.Copy(input, this.input, inputNum);
            for (int i = 0; i < neuronNum; i++) {
                active[i] = VMath.Tanh(VMath.Dot(weights[i].w, input) + bias[i]);
            }
            return active;
        }
        private float Dtanh(float y)
        {
            return 1.0f - y * y;
        }
        public float[] BackPropagate(float[] errors,float rate){
            for (int i = 0; i < inputError.Length; i++) {
                inputError[i] = 0;
            }
            for (int k = 0; k < neuronNum; k++) {
                delta[k] = errors[k] * Dtanh(active[k]);
                for (int j = 0; j < inputNum; j++) {
                    inputError[j] += delta[k] * weights[k].w[j];
                    float change = rate * delta[k] * input[j];
                    weights[k].w[j] += change;
                }
                float biasChange = rate * delta[k];
                bias[k] += biasChange;
            }
            return inputError;
        }
    }

    [System.Serializable]
    public class Network
    {
        public Layer[] layers;
        public Network(params int[] layerNeuronNum)
        {
            layers = new Layer[layerNeuronNum.Length - 1];
            for (int i = 0; i < layers.Length; i++) {
                layers[i] = new Layer(layerNeuronNum[i], layerNeuronNum[i + 1]);
            }
        }
        public void SetRandom(float min, float max)
        {
            for (int i = 0; i < layers.Length; i++) {
                layers[i].SetRandom(min, max);
            }
        }
        public float[] Forward(float[] input)
        {
            float[] result = input;
            for (int i = 0; i < layers.Length; i++) {
                result = layers[i].Forward(result);
            }
            return result;
        }
        public float[] BackPropagate(float[] error,float rate)
        {
            float[] result = error;
            for(int i = layers.Length - 1; i >= 0; i--){
                result = layers[i].BackPropagate(result,rate);
            }
            return result;
        }

        public void ForEachWeight(FunChangeWeight fun)
        {
            for (int i = 0; i < layers.Length; i++) {
                layers[i].ForEachWeight(fun);
            }
        }
        public void MapEachWeight(Network other,FunMap fun)
        {
            for (int i = 0; i < layers.Length; i++) {
                layers[i].MapEachWeight(fun, other.layers[i]);
            }
        }
        public void ZipEachWeight(Network network1,Network network2,FunZip fun)
        {
            for (int i = 0; i < layers.Length; i++) {
                layers[i].ZipEachWeight(fun, network1.layers[i], network2.layers[i]);
            }
        }
        public void Copy(Network other)
        {
            MapEachWeight(other, (float weight) => { return weight; });
        }
    }
}