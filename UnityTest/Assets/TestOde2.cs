using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TestOde2 : MonoBehaviour
{
    Phy.World world;
    Phy.Space space;

    List<Phy.Geom> geoms = new List<Phy.Geom>();
    List<Phy.Body> bodies = new List<Phy.Body>();
    List<Phy.Muscle> muscles = new List<Phy.Muscle>();

    public bool CallGC = false;

    static OdeDynamic FindFirstDynamicFromSelfOrParent(Transform trans)
    {
        OdeDynamic dyn = trans.GetComponent<OdeDynamic>();
        if (dyn != null) {
            return dyn;
        }else if(trans.parent != null) { 
            return FindFirstDynamicFromSelfOrParent(trans.parent);
        } 
        return null;
    }

    static List<T> FindAllCommant<T>(Transform trans)
    {
        List<T> result = new List<T>();
        if (trans == null) {
            List<UnityEngine.GameObject> rootObjs = new List<GameObject>();
            UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects(rootObjs);
            for(int i = 0; i<rootObjs.Count; i++) {
                result.AddRange(FindAllCommant<T>(rootObjs[i].transform));
            }
        } else if(trans.gameObject.activeSelf){
            T c = trans.GetComponent<T>();
            if(c != null) {
                result.Add(c);
            }
            for(int i= 0; i< trans.childCount; i++) {
                result.AddRange(FindAllCommant<T>(trans.GetChild(i)));
            }
        }
        return result;
    }

    void Start()
    {
        ode.dInitODE2();
        world = new Phy.World();
        space = new Phy.Space(world);

        world.gravity = new Vector3(0, -10, 0);

        List<OdeDynamic> dyns = FindAllCommant<OdeDynamic>(null);
        for(int i = 0; i<dyns.Count; i++) {
            OdeDynamic dyn = dyns[i];
            if (dyn.body == null) {
                dyn.body = new Phy.Body(world);
                dyn.body.position = dyn.transform.position;
                dyn.body.rotation = dyn.transform.rotation;
                dyn.body.target = dyn.transform;
                dyn.body.mass = new Phy.Mass();
                if (dyn.isKinematic) {
                    dyn.body.SetKinematic();
                }
                bodies.Add(dyn.body);
            }
        }

        List<OdeJoint> joints = FindAllCommant<OdeJoint>(null);
        for(int i =0; i<joints.Count; i++) {
            OdeJoint jnt = joints[i];
            Phy.Joint joint = Phy.Joint.Create(world,jnt.jointType);
            if (jnt.body0 && jnt.body0.body != null && jnt.body1 && jnt.body1.body != null) {
                joint.Attach(jnt.body0.body, jnt.body1.body);
                jnt.startTargetDir = jnt.transform.InverseTransformDirection((jnt.body1.transform.position - jnt.transform.position).normalized);
            }
            if (jnt.jointType == OdeJoint.JointType.Hinge) {
                Phy.HingeJoint hinge = (Phy.HingeJoint)joint;
                hinge.anchor = jnt.transform.position;
                hinge.axis = jnt.transform.TransformDirection(Vector3.up);
                hinge.SetParam(ode.dParam.LoStop, Mathf.Min(jnt.loStop, jnt.hiStop) * Mathf.Deg2Rad);
                hinge.SetParam(ode.dParam.HiStop, Mathf.Max(jnt.loStop, jnt.hiStop) * Mathf.Deg2Rad);
            }
            else if (jnt.jointType == OdeJoint.JointType.Ball) {
                Phy.BallJoint ball = (Phy.BallJoint)joint;
                ball.anchor = jnt.transform.position;
            }
            else if (jnt.jointType == OdeJoint.JointType.Universal) {
                Phy.UniversalJoint universal = (Phy.UniversalJoint)joint;
                universal.anchor = jnt.transform.position;
                universal.anchor = jnt.transform.position;
                universal.axis1 = jnt.transform.TransformDirection(Vector3.up);
                universal.axis2 = jnt.transform.TransformDirection(Vector3.forward);
                universal.SetParam(ode.dParam.LoStop, Mathf.Min(jnt.loStop, jnt.hiStop) * Mathf.Deg2Rad);
                universal.SetParam(ode.dParam.HiStop, Mathf.Max(jnt.loStop, jnt.hiStop) * Mathf.Deg2Rad);
                universal.SetParam(ode.dParam.LoStop2, Mathf.Min(jnt.loStop2, jnt.hiStop2) * Mathf.Deg2Rad);
                universal.SetParam(ode.dParam.HiStop2, Mathf.Max(jnt.loStop2, jnt.hiStop2) * Mathf.Deg2Rad);
            }
            else if (jnt.jointType == OdeJoint.JointType.AMotor) {
                Phy.AMotor motor = (Phy.AMotor)joint;

                motor.SetMode(ode.dAMotorMode.Euler);
                motor.SetAxis(0, 1, new Vector3(0, 1, 0));
                motor.SetAxis(2, 2, new Vector3(0, 0, 1));
                //motor.SetAxis(2, 0, new Vector3(0, 0, 1));

                //motor.SetParam(ode.dParam.FMax , 0.200001f);
                //motor.SetParam(ode.dParam.FMax2, 0.200001f);
                motor.SetParam(ode.dParam.FMax3, 0.200001f);
                //motor.SetParam(ode.dParam.Vel , 0.0f);
                //motor.SetParam(ode.dParam.Vel2, 0.0f);
                motor.SetParam(ode.dParam.Vel3, 0.0f);
                motor.SetParam(ode.dParam.LoStop , Mathf.Min(jnt.loStop , jnt.hiStop ) * Mathf.Deg2Rad);
                motor.SetParam(ode.dParam.HiStop , Mathf.Max(jnt.loStop , jnt.hiStop ) * Mathf.Deg2Rad);
                motor.SetParam(ode.dParam.LoStop2, Mathf.Min(jnt.loStop2, jnt.hiStop2) * Mathf.Deg2Rad);
                motor.SetParam(ode.dParam.HiStop2, Mathf.Max(jnt.loStop2, jnt.hiStop2) * Mathf.Deg2Rad);
                motor.SetParam(ode.dParam.LoStop3, Mathf.Min(jnt.loStop3, jnt.hiStop3) * Mathf.Deg2Rad);
                motor.SetParam(ode.dParam.HiStop3, Mathf.Max(jnt.loStop3, jnt.hiStop3) * Mathf.Deg2Rad);
            }
            jnt.joint = joint;
        }

        List<OdeMuscle> ms = FindAllCommant<OdeMuscle>(null);
        for(int i = 0; i< ms.Count; i++) {
            OdeMuscle m = ms[i];
            if (m.body0 && m.body0.body != null && m.body1 && m.body1.body != null) {
                Vector3 relPos0 = Quaternion.Inverse(m.body0.transform.rotation) * (m.transform.position - m.body0.transform.position);
                Vector3 relPos1 = Quaternion.Inverse(m.body1.transform.rotation) * (m.tendon.transform.position - m.body1.transform.position);

                Phy.Muscle muscle = new Phy.Muscle();
                m.muscle = muscle;
                muscle.body0 = m.body0.body;
                muscle.body1 = m.body1.body;
                muscle.relPos0 = relPos0;
                muscle.relPos1 = relPos1;
                muscle.Reset();

                muscles.Add(muscle);
            }
        }

        OdeCollider[] colliders = GameObject.FindObjectsOfType<OdeCollider>();
        for(int i =0; i<colliders.Length; i++) {
            OdeCollider collider = colliders[i];
            Phy.Geom geom = collider.CreateGeom(space);
            OdeDynamic dyn = FindFirstDynamicFromSelfOrParent(collider.transform);
            if (dyn != null && dyn.body != null) {
                //ODE.Mass oldMass = dyn.body.GetMass();
                Phy.Mass mass = new Phy.Mass();
                collider.CalculateMass(mass, dyn.mass);
                //mass.Translate(collider.transform.position - dyn.transform.position);
                //oldMass.Add(mass);
                //dyn.body.SetMass(oldMass);
                if (!dyn.isKinematic) {
                    dyn.body.SetMass(mass);
                }

                geom.body = dyn.body;
                geom.offsetWorldPosition = collider.GetScaledOffsetPosition() + (collider.transform.position - dyn.transform.position);
                geom.offsetRotation = collider.GetOffsetRotation();
            } else {
                geom.position = collider.transform.TransformPoint(collider.offsetPosition);
                geom.rotation = collider.transform.rotation * collider.GetOffsetRotation();
            }
            geoms.Add(geom);
        }
    }

    public void FixedUpdate()
    {
        for(int i =0; i<muscles.Count; i++) {
            muscles[i].Update(Time.fixedDeltaTime);
        }

        space.Collide();
        world.Step(Time.fixedDeltaTime);
        
        for(int i= 0; i<bodies.Count; i++) {
            bodies[i].target.position = bodies[i].position;
            bodies[i].target.rotation = bodies[i].rotation;
        }
    }

    public void OnDisable()
    {
        space.Destroy();
        world.Destroy();
        ode.dCloseODE();
    }
}
