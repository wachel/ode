using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class OdeCollider : MonoBehaviour
{
    public enum ColliderType
    {
        Sphere,
        Capsule,
        Box,
    }

    public ColliderType colliderType;
    public Vector3 offsetPosition;
    public Vector3 offsetRotation;
    public float length = 1.0f;
    public float radius = 0.5f;
    public Vector3 size = Vector3.one;

    public Quaternion GetOffsetRotation()
    {
        return Quaternion.Euler(offsetRotation.x, offsetRotation.y, offsetRotation.z);
    }

    public Vector3 GetScaledOffsetPosition()
    {
        return Vector3.Scale(transform.localScale, offsetPosition);
    }

    public Phy.Geom CreateGeom(Phy.Space space)
    {
        Vector3 scale = transform.lossyScale;
        Phy.Geom geom = null;
        if(colliderType == ColliderType.Sphere) {
            geom = Phy.Geom.CreateSphere(space, radius * scale.x);
        }
        else if(colliderType == ColliderType.Capsule) {
            geom = Phy.Geom.CreateCapsule(space, radius * scale.x, length * scale.z);
        }
        else if(colliderType == ColliderType.Box) {
            geom = Phy.Geom.CreateBox(space, Vector3.Scale(size, scale));
        }
        return geom;
    }
     
    public void CalculateMass(Phy.Mass om,float totalMass)
    {
        Vector3 scale = transform.lossyScale;
        if (colliderType == ColliderType.Sphere) {
            om.SetSphereTotal(totalMass, radius * scale.x);
        }
        else if(colliderType == ColliderType.Capsule) {
            om.SetCapsultTotal(totalMass, 3, radius * scale.x, length * scale.z);
        }
        else if(colliderType == ColliderType.Box) {
            om.SetBoxTotal(totalMass, Vector3.Scale(size, scale));
        }
    }

    public void OnDrawGizmosSelected()
    {
        Color oldColor = Gizmos.color;
        Gizmos.color = Color.green;
        Matrix4x4 localMatrix = Matrix4x4.TRS(offsetPosition, GetOffsetRotation(), Vector3.one);
        Gizmos.matrix = transform.localToWorldMatrix * localMatrix;
        if (colliderType == ColliderType.Sphere) {
            Gizmos.DrawWireSphere(offsetPosition, radius);
        }else if(colliderType == ColliderType.Box){
            Gizmos.DrawWireCube(offsetPosition, size * 1.001f);
        } else if (colliderType == ColliderType.Capsule) {
            int segNum = 8;
            Vector3 p0 = -new Vector3(0, 0, length / 2);
            Vector3 p1 = new Vector3(0, 0, length / 2);
            Vector3 axisDir = p0 == p1 ? Vector3.up : (p1 - p0).normalized;
            Vector3 startDir = Vector3.Cross(axisDir, new Vector3(axisDir.y, axisDir.z, axisDir.x));
            for (int i = 0; i < segNum; i++) {
                float angle0 = i * 360.0f / segNum;
                float angle1 = (i + 1) * 360.0f / segNum;
                Quaternion rot0 = Quaternion.AngleAxis(angle0, axisDir);
                Quaternion rot1 = Quaternion.AngleAxis(angle1, axisDir);
                Gizmos.DrawLine(rot0 * startDir * radius + p0, rot0 * startDir * radius + p1);

                Gizmos.DrawLine(rot0 * startDir * radius + p0, rot1 * startDir * radius + p0);
                Gizmos.DrawLine(rot0 * startDir * radius + p1, rot1 * startDir * radius + p1);

                int stepNum = 4;
                for (int j = 0; j < stepNum; j++) {
                    float angle2 = (j * 90.0f / stepNum) * Mathf.Deg2Rad;
                    float angle3 = ((j + 1) * 90.0f / stepNum) * Mathf.Deg2Rad;
                    Vector3 center00 = p0 - axisDir * Mathf.Sin(angle2) * radius;
                    Vector3 center01 = p0 - axisDir * Mathf.Sin(angle3) * radius;
                    Gizmos.DrawLine(rot0 * startDir * Mathf.Cos(angle2) * radius + center00, rot0 * startDir * Mathf.Cos(angle3) * radius + center01);
                    Vector3 center10 = p1 + axisDir * Mathf.Sin(angle2) * radius;
                    Vector3 center11 = p1 + axisDir * Mathf.Sin(angle3) * radius;
                    Gizmos.DrawLine(rot0 * startDir * Mathf.Cos(angle2) * radius + center10, rot0 * startDir * Mathf.Cos(angle3) * radius + center11);
                }
            }
        }
        Gizmos.color = oldColor;
    }

    public string[] GetHidenProperty()
    {
        List<string> result = new List<string>();
        if(colliderType != ColliderType.Sphere && colliderType != ColliderType.Capsule) {
            result.Add("radius");
        }

        if(colliderType != ColliderType.Capsule) {
            result.Add("length");
        }

        if(colliderType != ColliderType.Box) {
            result.Add("size");
        }
        return result.ToArray();
    }
}
