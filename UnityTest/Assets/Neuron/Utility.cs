using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utility {
    public static float GetAngle(Vector2 dir)
    {
        dir.Normalize();

        float angle = dir.y > 0 ? Mathf.Acos(dir.x) : Mathf.PI*2 - Mathf.Acos(dir.x);
        return 90 - angle * Mathf.Rad2Deg;
    }
}
