using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestUtility : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        Vector3 left = transform.TransformDirection(Vector3.left);
        Vector3 dir = Vector3.Cross(left, Vector3.down);
        float angle = Utility.GetAngle(new Vector2(dir.x, dir.z));
        Debug.Log("angle = " + angle);
	}
}
