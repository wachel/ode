using UnityEngine;
using System.Collections;

public class PID
{
    public Vector3 pos;
    public Vector3 oldPos;
    public Vector3 speed;
    public Vector3 oldSpeed;
    public Vector3 acceleration;
    public void Reset(Vector3 curPos)
    {
        oldPos = pos = curPos;
        speed = oldSpeed = acceleration = Vector3.zero;
    }
    public void Copy(PID other)
    {
        this.pos = other.pos;
        this.oldPos = other.oldPos;
        this.speed = other.speed;
        this.oldSpeed = other.oldSpeed;
        this.acceleration = other.acceleration;
    }
    public void Update(Vector3 curPos, float deltaTime)
    {
        pos = curPos;
        speed = (pos - oldPos) / deltaTime;
        acceleration = (speed - oldSpeed) / deltaTime;
        oldPos = pos;
        oldSpeed = speed;
    }
}

public class RobotState
{
    public RobotState(int bodyNum,int muscleNum)
    {
        bodies = new Body[bodyNum];
        muscles = new Muscle[muscleNum];
    }
    public PID headPos;
    public float reward;

    public struct Body
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 linearVel;
        public Vector3 angularVel;
        public Vector3 force;
        public Vector3 torque;
    }
    public Body[] bodies;

    public struct Muscle
    {
        public float force;
        public float lastLength;
        public float activity;
    }
    public Muscle[] muscles;
}

public class Robot : MonoBehaviour {
    public OdeMuscle[] muscles;
    public OdeJoint[] joints;
    public OdeDynamic[] bodies;
    public float[] inputs;
    Vector3[] positions;
    Quaternion[] rotations;
    public PID pidHeadPos = new PID();
    public PID pidHeadDir = new PID();
    public OdeDynamic head;
    public OdeDynamic leftFoot;
    public OdeDynamic rightFoot;
    public Vector3 footPos;
    public Vector3 oldFootPos;
	void Awake () {
        inputs = new float[muscles.Length * 2 + 3 * 3 + 3 * 3];
        positions = new Vector3[bodies.Length];
        rotations = new Quaternion[bodies.Length];
	}

    public void DoStart()
    {
        for (int i = 0; i < bodies.Length; i++) {
            positions[i] = bodies[i].transform.position;
            rotations[i] = bodies[i].transform.rotation;
        }
        Reset();
    }

    public void UpdateInputs()
    {
        Vector3 lookDir = head.body.rotation * Vector3.back;
        Vector3 left = head.body.rotation * Vector3.right;
        Vector3 forward = Vector3.Cross(left, Vector3.down);
        float angle = Utility.GetAngle(new Vector2(forward.x, forward.z));
        Quaternion rot = Quaternion.AngleAxis(angle, Vector3.up);

        footPos = (leftFoot.body.position + rightFoot.body.position) / 2;

        Vector3 headPos = Quaternion.Inverse(rot) * (head.body.position - footPos);
        Vector3 headDir = Quaternion.Inverse(rot) * (head.body.rotation * Vector3.forward);

        pidHeadPos.Update(headPos, Time.fixedDeltaTime);
        pidHeadDir.Update(headDir, Time.fixedDeltaTime);

        int curNum = 0;
        for (int i = 0; i < muscles.Length; i++) {
            inputs[i * 2 + 0] = muscles[i].muscle.Length / muscles[i].muscle.slackLength;
            inputs[i * 2 + 1] = muscles[i].muscle.force / muscles[i].muscle.maxForce;
        }
        curNum += muscles.Length * 2;

        SetInputByPid(curNum, pidHeadPos.pos, pidHeadPos.speed * 0.1f, pidHeadPos.acceleration * 0.05f);
        curNum += 9;
        SetInputByPid(curNum, pidHeadDir.pos, pidHeadDir.speed * 0.01f, pidHeadDir.acceleration * 0.001f);
        curNum += 9;
    }

    void SetInputByPid(int curNum,Vector3 pos,Vector3 speed, Vector3 acceleration)
    {
        inputs[curNum + 0] = pos.x; inputs[curNum + 1] = pos.y; inputs[curNum + 2] = pos.z;
        inputs[curNum + 3] = speed.x; inputs[curNum + 4] = speed.y; inputs[curNum + 5] = speed.z;
        inputs[curNum + 6] = acceleration.x; inputs[curNum + 7] = acceleration.y; inputs[curNum + 8] = acceleration.z;

    }

    public void UpdateMuscle(float[] activity)
    {
        for (int i = 0; i < muscles.Length; i++) {
            muscles[i].activity = UnityEngine.Mathf.Clamp01(0.5f + activity[i] * 0.5f);
        }        
    }

    public bool IsAlive()
    {
        return pidHeadPos.pos.y > 0.5f;
	}

    public float GetReward()
    {
        float ahead = -(footPos.z - oldFootPos.z);
        oldFootPos = footPos;
        Vector3 hipPos = pidHeadPos.pos;
        float totalMuscleForce = 0;
        for (int i = 0; i < muscles.Length; i++) {
            totalMuscleForce += muscles[i].force;
        }
        return (hipPos.y + ahead) / (totalMuscleForce * 0.001f + 1);
    }

    public void Reset()
    {
        for (int i = 0; i < bodies.Length; i++) {
            if (bodies[i].body != null) {
                bodies[i].body.position = positions[i];
                bodies[i].body.rotation = rotations[i];
                bodies[i].body.linearVel = Vector3.zero;
                bodies[i].body.angularVel = Vector3.zero;
            }
        }
        for (int i = 0; i < muscles.Length; i++) {
            if (muscles[i].muscle != null) {
                muscles[i].muscle.Reset();
            }
        }   

        pidHeadPos.Reset(this.transform.InverseTransformPoint(bodies[0].transform.position));

        footPos = oldFootPos = (leftFoot.transform.position + rightFoot.transform.position) / 2;
    }

    public void GetState(ref RobotState state)
    {
        for (int i = 0; i < bodies.Length; i++) {
            Phy.Body src = bodies[i].body;
            state.bodies[i].position = src.position - transform.position;
            state.bodies[i].rotation = src.rotation;
            state.bodies[i].linearVel = src.linearVel;
            state.bodies[i].angularVel = src.angularVel;
            state.bodies[i].force = src.force;
            state.bodies[i].torque = src.torque;
        }
        for (int i = 0; i < muscles.Length; i++) {
            state.muscles[i].force = muscles[i].muscle.force;
            state.muscles[i].lastLength = muscles[i].muscle.lastLength;
            state.muscles[i].activity = muscles[i].muscle.activity;
        }
    }

    public void SetState(RobotState state)
    {
        for (int i = 0; i < bodies.Length; i++) {
            Phy.Body src = bodies[i].body;
            src.position = state.bodies[i].position + transform.position;
            src.rotation = state.bodies[i].rotation;
            src.linearVel = state.bodies[i].linearVel;
            src.angularVel = state.bodies[i].angularVel;
            src.force = state.bodies[i].force;
            src.torque = state.bodies[i].torque;
        }
        for (int i = 0; i < muscles.Length; i++) {
            muscles[i].muscle.force = state.muscles[i].force;
            muscles[i].muscle.lastLength = state.muscles[i].lastLength;
            muscles[i].muscle.activity = state.muscles[i].activity;
        }
    }
}
