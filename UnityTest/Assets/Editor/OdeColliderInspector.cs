using UnityEngine;
using System.Collections;
using UnityEditor;

[CustomEditor(typeof(OdeCollider))]
public class OdeColliderInspector : Editor
{
    OdeCollider collider;
    SerializedObject so;
    public void OnEnable()
    {
        collider = target as OdeCollider;
        so = new SerializedObject(collider);
    }
    public override void OnInspectorGUI()
    {
        DrawPropertiesExcluding(so, collider.GetHidenProperty());
        so.ApplyModifiedProperties();
    }
}
