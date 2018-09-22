using UnityEngine;
using System.Collections;

public class OdeJoint : MonoBehaviour
{
    public enum JointType
    {
        Hinge,
        Universal,
        Ball,
        AMotor,
    }
    public JointType jointType;
    public OdeDynamic body0;
    public OdeDynamic body1;

    public Phy.Joint joint;
    [Range(-180,180)]
    public float loStop = -180;
    [Range(-180, 180)]
    public float hiStop = 180;

    [Range(-180, 180)]
    public float loStop2 = -180;
    [Range(-180, 180)]
    public float hiStop2 = 180;

    [Range(-180, 180)]
    public float loStop3 = -180;
    [Range(-180, 180)]
    public float hiStop3 = 180;

    [HideInInspector]
    public Vector3 startTargetDir;

    public void DrawGizmos(bool bSelected)
    {
        Color oldColor = Gizmos.color;

        Vector3 pos = transform.position;
        float radius = 0.1f * transform.localScale.magnitude;

        if (jointType == JointType.Hinge) {
            Gizmos.color = new Color(0, 1, 0, bSelected ? 1 : 0.2f);
            Vector3 axis = transform.TransformDirection(Vector3.up);
            if (Application.isPlaying && joint != null) {
                pos = (joint as Phy.HingeJoint).anchor;
                axis = (joint as Phy.HingeJoint).axis;
            }
            Gizmos.DrawLine(pos + axis * (-0.2f), pos + axis * (0.2f));
            DrawCircle(axis, radius, pos, 12);

            if (body1) {
                Vector3 localTarget = body1.transform.position - pos;
                Vector3 targetDir = localTarget.normalized;
                if (Application.isPlaying) {
                    targetDir = transform.TransformDirection(startTargetDir);
                }
                DrawArc(axis, radius, pos, targetDir, loStop, hiStop, 12);
            }
        }
        else if (jointType == JointType.Ball) {
            Gizmos.color = new Color(0, 1, 0, bSelected ? 1 : 0.2f);
            if (Application.isPlaying && joint != null) {
                pos = (joint as Phy.BallJoint).anchor;
            }
            Gizmos.DrawWireSphere(pos, radius);
        }
        
        else if(jointType == JointType.Universal) {
            {
                Gizmos.color = new Color(0, 1, 0, bSelected ? 1 : 0.2f);
                Vector3 axis = transform.TransformDirection(Vector3.up);
                if (Application.isPlaying && joint != null) {
                    pos = (joint as Phy.UniversalJoint).anchor;
                    axis = (joint as Phy.UniversalJoint).axis1;
                }
                Gizmos.DrawLine(pos + axis * (-0.2f), pos + axis * (0.2f));
                DrawCircle(axis, radius, pos, 12);

                if (body1) {
                    Vector3 localTarget = body1.transform.position - pos;
                    Vector3 targetDir = localTarget.normalized;
                    if (Application.isPlaying) {
                        targetDir = transform.TransformDirection(startTargetDir);
                    }
                    DrawArc(axis, radius, pos, targetDir, loStop, hiStop, 12);
                }
            }
            {
                Gizmos.color = new Color(0, 0, 1, bSelected ? 1 : 0.2f);
                Vector3 axis = transform.TransformDirection(Vector3.forward);
                if (Application.isPlaying && joint != null) {
                    pos = (joint as Phy.UniversalJoint).anchor;
                    axis = (joint as Phy.UniversalJoint).axis2;
                }
                Gizmos.DrawLine(pos + axis * (-0.2f), pos + axis * (0.2f));
                DrawCircle(axis, radius, pos, 12);

                if (body1) {
                    Vector3 localTarget = body1.transform.position - pos;
                    Vector3 targetDir = localTarget.normalized;
                    if (Application.isPlaying) {
                        targetDir = transform.TransformDirection(startTargetDir);
                    }
                    DrawArc(axis, radius, pos, targetDir, loStop2, hiStop2, 12);
                }
            }
        }

        if (body0) {
            Gizmos.DrawLine(pos, body0.transform.position);
        }
        if (body1) {
            Gizmos.DrawLine(pos, body1.transform.position);
        }

        Gizmos.color = oldColor;
    }
    public void OnDrawGizmosSelected()
    {
        DrawGizmos(true);
    }

    public void OnDrawGizmos()
    {
        //DrawGizmos(false);
    }

    static void DrawCircle(Vector3 axis,float radius,Vector3 center,int segNum)
    {
        Vector3 startDir = Vector3.Cross(axis, Mathf.Abs(axis.y) < 0.9 ? Vector3.up : Vector3.left).normalized;
        for (int i = 0; i < segNum; i++) {
            float angle0 = i * 360.0f / segNum;
            float angle1 = (i + 1) * 360.0f / segNum;
            Quaternion rot0 = Quaternion.AngleAxis(angle0, axis);
            Quaternion rot1 = Quaternion.AngleAxis(angle1, axis);
            Gizmos.DrawLine(center + rot0 * startDir * radius, center + rot1 * startDir * radius);
        }
    }

    static void DrawArc(Vector3 axis, float radius, Vector3 center, Vector3 targetDir,float startAngle,float endAngle, int segNum)
    {
        Vector3 startDir = (targetDir - axis * Vector3.Dot(axis, targetDir)).normalized;
        for (int i = 0; i < segNum; i++) {
            float angleRange = endAngle - startAngle;
            float angle0 = startAngle + i * angleRange / segNum;
            float angle1 = startAngle + (i + 1) * angleRange / segNum;
            Quaternion rot0 = Quaternion.AngleAxis(-angle0, axis);
            Quaternion rot1 = Quaternion.AngleAxis(-angle1, axis);
            Gizmos.DrawLine(center + rot0 * startDir * radius * 0.8f, center + rot1 * startDir * radius * 0.8f);
        }
    }
}
