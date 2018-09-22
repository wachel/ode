using UnityEngine;
using System.Collections;

public class OdeMuscle : MonoBehaviour
{
    public OdeTendon tendon;
    public float maxForce = 10.0f;
    public float slackLength = 0.1f;
    [Range(0,1)]
    public float activity;


    public OdeDynamic body0;
    public OdeDynamic body1;
    public Phy.Muscle muscle;

    public float force;

    public void Update()
    {
        if (muscle != null) {
            muscle.maxForce = maxForce;
            muscle.slackLength = slackLength;
            muscle.activity = activity;
            muscle.slackLength = slackLength;

            force = muscle.force;
        }
    }

    void DrawGizmos(bool bSelected)
    {
        Color oldColor = Gizmos.color;
        Gizmos.color = new Color(0.5f, 0, 0, bSelected ? 1 : 0.2f);

        //Gizmos.matrix = transform.localToWorldMatrix;

        float radius = 0.05f * transform.localScale.magnitude;

        Gizmos.DrawWireSphere(transform.position, radius);

        //Gizmos.DrawWireCube(Vector3.zero, Vector3.one * 0.1f);

        if (tendon) {
            Gizmos.DrawLine(transform.position, tendon.transform.position);
            Gizmos.color = new Color(1.0f,0,0,bSelected ? 1 : 0.2f);
            Vector3 dir = (tendon.transform.position - transform.position).normalized;
            Gizmos.DrawLine(transform.position, transform.position + dir * slackLength);
            //Gizmos.matrix = tendon.transform.localToWorldMatrix;
            //Gizmos.DrawWireCube(Vector3.zero, Vector3.one * 0.08f);
        }

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

