using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VMath
{
    public static void Random(float[][] m, float min, float max)
    {
        for (int i = 0; i < m.Length; i++) {
            for (int j = 0; j < m[i].Length; j++) {
                m[i][j] = UnityEngine.Random.Range(min, max);
            }
        }
    }
    public static void Random(float[] v, float min, float max)
    {
        for (int i = 0; i < v.Length; i++) {
            v[i] = UnityEngine.Random.Range(min, max);
        }
    }
    public delegate float FunForEach(float value);
    public static void ForEach(float[] v, FunForEach fun)
    {
        for (int i = 0; i < v.Length; i++) {
            v[i] = fun(v[i]);
        }
    }
    public static float Dot(float[] v0, float[] v1)
    {
        float sum = 0;
        for (int i = 0; i < v0.Length; i++) {
            sum += v0[i] * v1[i];
        }
        return sum;
    }
    public static float Tanh(float v)
    {
        return (float)System.Math.Tanh(v);
    }
    public static void Tanh(float[] v, float[] outValue)
    {
        for (int i = 0; i < v.Length; i++) {
            outValue[i] = Tanh(v[i]);
        }
    }
    public static float Artanh(float v)
    {
        return (float)(0.5 * System.Math.Log((1 + v) / (1 - v)));
    }
    public static void Artanh(float[] v, float[] outValue)
    {
        for (int i = 0; i < v.Length; i++) {
            outValue[i] = Artanh(v[i]);
        }
    }
    public static float Sum(float[] v)
    {
        float result = 0;
        for (int i = 0; i < v.Length; i++) {
            result += v[i];
        }
        return result;
    }
    public static void Add(float[] v0,float[] v1,float[] outValue)
    {
        for (int i = 0; i < v0.Length; i++) {
            outValue[i] = v0[i] + v1[i];
        }
    }
    public static void Sub(float[] v0, float[] v1, float[] outValue)
    {
        for (int i = 0; i < v0.Length; i++) {
            outValue[i] = v0[i] - v1[i];
        }
    }
    public static void Scale(float[] v0, float scale, float[] outValue)
    {
        for (int i = 0; i < v0.Length; i++) {
            outValue[i] = v0[i] * scale;
        }
    }
    public static void Set(float[] v, float value)
    {
        for (int i = 0; i < v.Length; i++) {
            v[i] = value;
        }
    }
}