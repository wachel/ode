using UnityEngine;
using System.Collections;

public class OdeTendon : MonoBehaviour {
    void DrawGizmos(bool bSelected)
    {
        Color oldColor = Gizmos.color;
        Gizmos.color = new Color(0.9f, 0.75f, 0, bSelected ? 1 : 0.2f);

        float radius = 0.03f * transform.localScale.magnitude;

        Gizmos.DrawWireSphere(transform.position, radius);

        Gizmos.color = oldColor;
    }

    public void OnDrawGizmos()
    {
        //DrawGizmos(false);
    }

    public void OnDrawGizmosSelected()
    {
        DrawGizmos(true);
    }
}
