using UnityEngine;
using System.Collections;
using System;
using System.Collections.Generic;

public class TestOde : MonoBehaviour
{
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactgroup;
    dGeomID ground;
    float radius = 0.2f, mass = 1.0f;
    public Transform target;

    class OdeObject
    {
        public dBodyID body;
        public List<dGeomID> geoms = new List<dGeomID>();
        public Transform target;
    }
    OdeObject ball;

    public void Awake()
    {
        ode.dInitODE2();
    }

    void nearCallback(IntPtr data, dGeomID o1, dGeomID o2)
    {
        dBodyID b1 = ode.dGeomGetBody(o1);
        dBodyID b2 = ode.dGeomGetBody(o2);

        const int MAX_CONTACTS = 5;
        ode.dContact[] contact = new ode.dContact[MAX_CONTACTS];

        for (int i = 0; i < MAX_CONTACTS; i++) {
            contact[i].surface.mode = (int)(ode.dContactType.Bounce | ode.dContactType.SoftCFM);
            contact[i].surface.mu = Mathf.Infinity;
            contact[i].surface.mu2 = 0;
            contact[i].surface.bounce = 0.01f;
            contact[i].surface.bounce_vel = 0.1f;
            contact[i].surface.soft_cfm = 0.01f;
        }

        int numc = ode.dCollide(o1, o2, MAX_CONTACTS, ref contact[0].geom, System.Runtime.InteropServices.Marshal.SizeOf(typeof(ode.dContact)));
        if (numc > 0) {
            for (int i = 0; i < numc; i++) {
                dJointID c = ode.dJointCreateContact(world, contactgroup, ref contact[i]);
                ode.dJointAttach(c, b1, b2);
            }
        }
    }

    public void Start()
    {
        // create world
        ode.dInitODE2(0);
        world = ode.dWorldCreate();
        ode.dWorldSetGravity(world, 0, -10, 0);

        space = ode.dHashSpaceCreate();
        contactgroup = ode.dJointGroupCreate(0);

        ground = ode.dCreatePlane(space, 0, 1, 0, 0);

        ode.dMass m1 = new ode.dMass();
        ball = new OdeObject();
        ball.body = ode.dBodyCreate(world);  //  Crate a rigid body
        ode.dMassSetZero(ref m1);     // Initialize mass parameters
        ode.dMassSetSphereTotal(ref m1, mass, radius); // Calculate a mass parameter
        ode.dBodySetMass(ball.body,ref  m1);
        ode.dBodySetPosition(ball.body, 0, 2, 0);
        ode.dBodySetQuaternion(ball.body, Quaternion.AngleAxis(45, Vector3.left));
        ode.dBodyGetRotation(ball.body);

        ball.geoms.Add(ode.dCreateSphere(space, radius));
        ball.target = target;
        ode.dGeomSetBody(ball.geoms[0], ball.body);
        ode.dBodyAddTorque(ball.body, 5, 0, 0);

        int a = 0;
    }

    public void LateUpdate()
    {
        ode.dSpaceCollide(space, (IntPtr)0, nearCallback);
        ode.dWorldStep(world, Time.fixedDeltaTime);  // Step a simulation world, time step is 0.05 [s]
        ode.dJointGroupEmpty(contactgroup);
        Vector3 pos = ode.dBodyGetPosition(ball.body);  // Get a position
        Quaternion rot = ode.dBodyGetQuaternion(ball.body);   // Get a rotation
        ball.target.position = pos;
        ball.target.rotation = rot;
    }

    public void OnDestroy()
    {
        ode.dSpaceDestroy(space);
        ode.dWorldDestroy(world);
        ode.dCloseODE();
    }
}
