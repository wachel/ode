using UnityEngine;
using System.Collections;

[RequireComponent(typeof(OdeCollider))]
public class OdeDynamic : MonoBehaviour
{
    [HideInInspector]
    public Phy.Body body;
    public float mass = 1;
    public bool isKinematic = false;
}
