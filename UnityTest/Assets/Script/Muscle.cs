using UnityEngine;
using System.Collections;

namespace Phy {
    public class Muscle
    {
        public Body body0;
        public Body body1;
        public Vector3 relPos0;
        public Vector3 relPos1;

        public float force;
        public float maxForce = 10.0f;
        public float slackLength = 1.0f;
        public float activity;

        public float lastLength;
        public void Reset()
        {
            activity = 0;
            force = 0;
            lastLength = Length;
        }
        public void Update(float deltaTime)
        {
            Vector3 globalPos0 = body0.rotation * relPos0 + body0.position;
            Vector3 globalPos1 = body1.rotation * relPos1 + body1.position;

            Vector3 dir01 = (globalPos1 - globalPos0).normalized;

            if (slackLength < 0.01f) {
                slackLength = 0.01f;
            }
            float length = (globalPos0 - globalPos1).magnitude / slackLength;
            if (lastLength == 0) {
                lastLength = length;
            }
            float velocity = (length - lastLength) / deltaTime;
            velocity = Mathf.Clamp(velocity, -10, 10);
            force = GetForce(activity, length, velocity) * maxForce;
            lastLength = length;

            body0.AddForceAtPos(dir01 * force, globalPos0);
            body1.AddForceAtPos(-dir01 * force, globalPos1);
        }

        public float Length{get{return (body0.position - body1.position).magnitude;}}

        float GetForce(float activity,float length,float velocity)
        {
            float a = activity;
            float lm = length;
            float flce = fvce_T03(lm);
            float fpelm = fpelm_T03(lm);
            fpelm = Mathf.Clamp(fpelm, 0, maxForce);
            float fvce = fvce_T03(velocity, flce, 1, a);
            float force = flce * fvce + fpelm;
            return force;
        }

        //http://nbviewer.jupyter.org/github/demotu/BMC/blob/master/notebooks/MuscleModeling.ipynb
        //force of the contractile element as function of muscle length.
        static float flce_T03(float lm = 1f, float gammal = 0.45f)//主动部分
        {
            float fl = Mathf.Exp(-(lm - 1) * (lm - 1) / gammal);
            return fl;
        }
        // force of the muscle parallel element as function of muscle length.
        static float fpelm_T03(float lm = 1, float kpe = 5, float epsm0 = 0.6f)//并联被动部分
        {
            float fpe = 0;
            if (lm < 1) {
                fpe = 0;
            }
            else {
                fpe = (Mathf.Exp(kpe * (lm - 1) / epsm0) - 1) / (Mathf.Exp(kpe) - 1);
            }
            return fpe;
        }
        //velocity of the muscle
        static float vmfce_T03(float fm, float flce = 1, float lmopt = 1, float a = 1, float vmmax = 1, float fmlen = 1.4f, float af = 0.25f)
        {
            vmmax = vmmax * lmopt;
            float b = 0;
            if (fm <= a * flce) {  //isometric and concentric activation
                b = a * flce + fm / af;
            }
            else {             //eccentric activation;
                b = (2 + 2 / af) * (a * flce * fmlen - fm) / (fmlen - 1);
            }
            float vm = (0.25f + 0.75f * a) * vmmax * (fm - a * flce) / b;
            return vm;
        }

        //normalized force of the muscle contractile element
        static float fvce_T03(float vm = 0, float flce = 1, float lmopt = 1, float a = 1, float vmmax = 1, float fmlen = 1.4f, float af = 0.25f)
        {
            vmmax = vmmax * lmopt;
            float fvce = 0;
            if (vm <= 0) {  //isometric and concentric activation
                fvce = af * a * flce * (4 * vm + vmmax * (3 * a + 1)) / (-4 * vm + vmmax * af * (3 * a + 1));
            }
            else {        //eccentric activation
                fvce = a * flce * (af * vmmax * (3 * a * fmlen - 3 * a + fmlen - 1) + 8 * vm * fmlen * (af + 1)) / (af * vmmax * (3 * a * fmlen - 3 * a + fmlen - 1) + 8 * vm * (af + 1));
            }
            return fvce;
        }
    }
}