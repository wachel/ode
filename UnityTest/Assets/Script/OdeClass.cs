using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Phy
{
    public abstract class Joint
    {
        public static Joint Create(World world, OdeJoint.JointType type)
        {
            if (type == OdeJoint.JointType.Hinge) {
                return new HingeJoint(world, ode.dJointCreateHinge(world.ptr, world.jointgroup));
            }
            else if (type == OdeJoint.JointType.Ball) {
                return new BallJoint(world, ode.dJointCreateBall(world.ptr, world.jointgroup));
            }
            else if (type == OdeJoint.JointType.AMotor) {
                return new AMotor(world, ode.dJointCreateAMotor(world.ptr, world.jointgroup));
            }
            else if (type == OdeJoint.JointType.Universal) {
                return new UniversalJoint(world, ode.dJointCreateUniversal(world.ptr, world.jointgroup));
            }
            return null;
        }


        public dJointID ptr;
        public World world;
        public int id;
        public Body body0;
        public Body body1;
        public OdeJoint.JointType type;
        public Joint(World world, dJointID ptr, OdeJoint.JointType type)
        {
            this.world = world; 
            this.type = type;
            this.ptr = ptr;
            id = InstanceMap.Add(this);
            ode.dJointSetData(ptr, (IntPtr)id);
        }
        ~Joint()
        {
            InstanceMap.Remove(id);
        }
        public void Destroy()
        {
            ode.dJointDestroy(ptr);
        }
        public void Attach(Body body0, Body body1)
        {
            this.body0 = body0; this.body1 = body1;
            ode.dJointAttach(ptr, body0.ptr, body1.ptr);
        }
        public abstract void SetParam(ode.dParam param, float value);
    }

    public class HingeJoint : Joint
    {
        public HingeJoint(World world, dJointID ptr) : base(world, ptr, OdeJoint.JointType.Hinge) { }
        public Vector3 anchor {
            get { Vector4 rlt; ode.dJointGetHingeAnchor(ptr, out rlt);return rlt; }
            set { ode.dJointSetHingeAnchor(ptr, value.x, value.y, value.z); }
        }
        public Vector3 axis {
            get { Vector4 rlt; ode.dJointGetHingeAxis(ptr, out rlt); return rlt; }
            set { ode.dJointSetHingeAxis(ptr, value.x, value.y, value.z); }
        }
        public override void SetParam(ode.dParam param, float value) { ode.dJointSetHingeParam(ptr, param, value); }
    }

    public class BallJoint : Joint
    { 
        public BallJoint(World world, dJointID ptr) : base(world, ptr, OdeJoint.JointType.Ball) { }
        public Vector3 anchor
        {

            get { Vector4 rlt; ode.dJointGetBallAnchor(ptr, out rlt); return rlt; }
            set { ode.dJointSetBallAnchor(ptr, value.x, value.y, value.z); }
        }
        public override void SetParam(ode.dParam param, float value) { ode.dJointSetBallParam(ptr, param, value); }
    }

    public class UniversalJoint:Joint
    {
        public UniversalJoint(World world, dJointID ptr) : base(world, ptr, OdeJoint.JointType.Universal) { }
        public Vector3 anchor
        {
            get { Vector4 rlt; ode.dJointGetUniversalAnchor(ptr, out rlt); return rlt; }
            set { ode.dJointSetUniversalAnchor(ptr, value.x, value.y, value.z); }
        }
        public Vector3 axis1
        {
            get { Vector4 rlt; ode.dJointGetUniversalAxis1(ptr, out rlt); return rlt; }
            set { ode.dJointSetUniversalAxis1(ptr, value.x, value.y, value.z); }
        }
        public Vector3 axis2
        {
            get { Vector4 rlt; ode.dJointGetUniversalAxis2(ptr, out rlt); return rlt; }
            set { ode.dJointSetUniversalAxis2(ptr, value.x, value.y, value.z); }
        }
        public float angle1 { get { return ode.dJointGetUniversalAngle1(ptr); } }
        public float angle2 { get { return ode.dJointGetUniversalAngle2(ptr); } }
        public override void SetParam(ode.dParam param, float value) { ode.dJointSetUniversalParam(ptr, param, value); }
    }

    public class AMotor : Joint
    {
        public AMotor(World world, dJointID ptr) : base(world, ptr, OdeJoint.JointType.AMotor) { }
        public override void SetParam(ode.dParam param, float value) { ode.dJointSetAMotorParam(ptr, param, value); }
        public void SetNumAxes(int num){ ode.dJointSetAMotorNumAxes(ptr,num); }
        public int GetNumAxes() { return ode.dJointGetAMotorNumAxes(ptr); }
        public void SetAxis(int anum, int rel, Vector3 dir) { ode.dJointSetAMotorAxis(ptr, anum, rel, dir.x, dir.y, dir.z); }
        public void SetMode(ode.dAMotorMode mode) { ode.dJointSetAMotorMode(ptr, mode); }
    }

    public class Geom
    {
        public static Geom CreateSphere(Space space, float radius)
        {
            return new Geom(space, ode.dCreateSphere(space.ptr, radius));
        }
        public static Geom CreateBox(Space space, Vector3 size)
        {
            return new Geom(space, ode.dCreateBox(space.ptr, size.x, size.y, size.z));
        }
        public static Geom CreateCapsule(Space space, float radius, float length)
        {
            return new Geom(space, ode.dCreateCapsule(space.ptr, radius, length));
        }
        public static Geom CreateCylinder(Space space, float radius, float length)
        {
            return new Geom(space, ode.dCreateCylinder(space.ptr, radius, length));
        }

        public dGeomID ptr;
        public Space space;
        public object userData0;
        public object userData1;
        private Body m_body;
        private int id;
        private Geom(Space space, dGeomID ptr)
        {
            this.space = space;
            this.ptr = ptr;
            this.id = InstanceMap.Add(this);
            ode.dGeomSetData(ptr, (IntPtr)id);
        }
        ~Geom()
        {
            InstanceMap.Remove(id);
        }
        public void Destroy()
        {
            ode.dGeomDestroy(ptr);
        }
        public Body body {
            get { return m_body; }
            set { ode.dGeomSetBody(ptr, value.ptr); m_body = value; }
        }
        private bool CheckBody()
        {
            if (body == null) {
                Debug.LogError("geom must has body");
                return false;
            }
            return true;
        }
        public Vector3 position {
            get { return ode.dGeomGetPosition(ptr); }
            set { ode.dGeomSetPosition(ptr, value.x, value.y, value.z); }
        }
        public Quaternion rotation {
            get {
                dQuaternion r;
                ode.dGeomGetQuaternion(ptr, out r);
                return r;
            }
            set { ode.dGeomSetQuaternion(ptr, value); }
        }
        public Vector3 offsetPosition {
            get { return ode.dGeomGetOffsetPosition(ptr); }
            set { if (CheckBody()) { ode.dGeomSetOffsetPosition(ptr, value.x, value.y, value.z); } }
        }
        public Quaternion offsetRotation {
            get { return ode.dGeomGetOffsetQuaternion(ptr); }
            set { if (CheckBody()) { ode.dGeomSetOffsetQuaternion(ptr, value); } }
        }
        public Vector3 offsetWorldPosition {
            set { if (CheckBody()) { ode.dGeomSetOffsetPosition(ptr, value.x, value.y, value.z); } }
        }
        public Quaternion offsetWorldRotation {
            set { if (CheckBody()) { ode.dGeomSetOffsetWorldQuaternion(ptr, value); } }
        }
    }

    public class Mass
    {
        public ode.dMass data;
        public Mass()
        {
            SetZero();
        }
        public void SetZero()
        {
            ode.dMassSetZero(ref data);
        }
        public void SetSphereTotal(float totalMass, float radius)
        {
            ode.dMassSetSphereTotal(ref data, totalMass, radius); // Calculate a mass parameter
        }
        public void SetBoxTotal(float totalMass, Vector3 size)
        {
            ode.dMassSetBoxTotal(ref data, totalMass, size.x, size.y, size.z);
        }
        public void SetCapsultTotal(float totalMass, int direction, float radius, float length)
        {
            ode.dMassSetCapsuleTotal(ref data, totalMass, direction, radius, length);
        }
        public void Add(Mass other)
        {
            ode.dMassAdd(ref data, ref other.data);
        }
        public void Translate(Vector3 pos)
        {
            ode.dMassTranslate(ref data, pos.x, pos.y, pos.z);
        }
    }

    public class Body
    {
        public dBodyID ptr;
        public World world;

        public Mass mass;

        public Transform target;
        public object userData0;
        public object userData1;

        public int id;
        public Body(World world)
        {
            this.world = world;
            world.AddBody(this);
            ptr = ode.dBodyCreate(world.ptr);
            id = InstanceMap.Add(this);
            ode.dBodySetData(ptr, (IntPtr)id);
        }
        ~Body()
        {
            InstanceMap.Remove(id);
        }
        public void Destroy()
        {
            ode.dBodyDestroy(ptr);
        }

        public Vector3 position {get { return ode.dBodyGetPosition(ptr); }set { ode.dBodySetPosition(ptr, value.x, value.y, value.z); }}
        public Quaternion rotation {get { return ode.dBodyGetQuaternion(ptr); }set { ode.dBodySetQuaternion(ptr, value); }}
        public Vector3 linearVel { get { return ode.dBodyGetLinearVel(ptr); } set { ode.dBodySetLinearVel(ptr, value.x, value.y, value.z); } }
        public Vector3 angularVel { get { return ode.dBodyGetAngularVel(ptr); } set { ode.dBodySetAngularVel(ptr, value.x, value.y, value.z); } }
        public Vector3 force { get { return ode.dBodyGetForce(ptr); } set { ode.dBodySetForce(ptr, value.x, value.y, value.z); } }
        public Vector3 torque { get { return ode.dBodyGetTorque(ptr); } set { ode.dBodySetTorque(ptr, value.x, value.y, value.z); } }


        public void SetMass(Mass mass)
        {
            this.mass = mass;
            if (mass.data.mass > 0) {
                ode.dBodySetMass(ptr, ref mass.data);
            } else {
                Debug.LogError("mass can not be zero");
            }
        }
        public Mass GetMass(){return mass;}

        public void SetKinematic() {ode.dBodySetKinematic(ptr);}
        public void SetDynamic(){ode.dBodySetDynamic(ptr);}
        public bool IsKinematic(){return ode.dBodyIsKinematic(ptr) != 0;}


        public void AddGeom(Geom geom){ode.dGeomSetBody(geom.ptr, ptr);}

        public void AddForce(Vector3 force) { ode.dBodyAddForce(ptr, force.x, force.y, force.z); }
        public void AddTorque(Vector3 torque) { ode.dBodyAddTorque(ptr, torque.x, torque.y, torque.z); }
        public void AddRelForce(Vector3 force) { ode.dBodyAddRelForce(ptr, force.x, force.y, force.z); }
        public void AddRelTorque(Vector3 torque) { ode.dBodyAddRelTorque(ptr, torque.x, torque.y, torque.z); }
        public void AddForceAtPos(Vector3 force, Vector3 pos) { ode.CheckVector3(force); ode.dBodyAddForceAtPos(ptr, force.x, force.y, force.z, pos.x, pos.y, pos.z); }
        public void AddForceAtRelPos(Vector3 force, Vector3 relPos) { ode.dBodyAddForceAtRelPos(ptr, force.x, force.y, force.z, relPos.x, relPos.y, relPos.z); }
        public void AddRelForceAtPos(Vector3 force, Vector3 pos) { ode.dBodyAddRelForceAtPos(ptr, force.x, force.y, force.z, pos.x, pos.y, pos.z); }
        public void AddRelForceAtRelPos(Vector3 force, Vector3 relPos) { ode.dBodyAddRelForceAtRelPos(ptr, force.x, force.y, force.z, relPos.x, relPos.y, relPos.z); }

    }

    public class Space
    {
        public dSpaceID ptr;
        public World world;
        public Space(World world)
        {
            ptr = ode.dHashSpaceCreate();
            this.world = world;
        }

        public void Destroy()
        {
            if (ptr.intPtr != IntPtr.Zero) {
                ode.dSpaceDestroy(ptr);
                ptr.intPtr = IntPtr.Zero;
            }
        }

        public void Collide()
        {
            //ode.dSpaceCollide(ptr, (IntPtr)0, nearCallback);
            ode.ExtSpaceCollide(ptr, OnNewContact);
        }

        void OnNewContact(ref ode.dContact contact,dBodyID body1,dBodyID body2)
        {
            dJointID c = ode.dJointCreateContact(world.ptr, world.contactgroup, ref contact);
            ode.dJointAttach(c, body1, body2);
        }

        //void nearCallback(IntPtr data, dGeomID o1, dGeomID o2)
        //{
        //    dBodyID b1 = ode.dGeomGetBody(o1);
        //    dBodyID b2 = ode.dGeomGetBody(o2);

        //    const int MAX_CONTACTS = 5;
        //    ode.dContact[] contact = new ode.dContact[MAX_CONTACTS];

        //    for (int i = 0; i < MAX_CONTACTS; i++) {
        //        contact[i].surface.mode = (int)(ode.dContactType.Bounce | ode.dContactType.SoftCFM);
        //        contact[i].surface.mu = 0.8f;// Mathf.Infinity;
        //        contact[i].surface.mu2 = 0;
        //        contact[i].surface.bounce = 0.01f;
        //        contact[i].surface.bounce_vel = 0.1f;
        //        contact[i].surface.soft_cfm = 0.01f;
        //    }

        //    int numc = ode.dCollide(o1, o2, MAX_CONTACTS, ref contact[0].geom, System.Runtime.InteropServices.Marshal.SizeOf(typeof(ode.dContact)));
        //    if (numc > 0) {
        //        for (int i = 0; i < numc; i++) {
        //            dJointID c = ode.dJointCreateContact(world.ptr, world.contactgroup, ref contact[i]);
        //            ode.dJointAttach(c, b1, b2);
        //        }
        //    }
        //}
    }

    public class World
    {
        public dWorldID ptr;
        public dJointGroupID contactgroup;
        public dJointGroupID jointgroup;
        public List<Body> bodies = new List<Body>();
        public World()
        {
            ptr = ode.dWorldCreate();
            contactgroup = ode.dJointGroupCreate(0);
            jointgroup = ode.dJointGroupCreate(0);
        }
        public void Destroy()
        {
            if (ptr.intPtr != IntPtr.Zero) {
                ode.dWorldDestroy(ptr);
                ptr.intPtr = IntPtr.Zero;
            }
        }
        public void SetGravity(Vector3 gravidy)
        {
            ode.dWorldSetGravity(ptr, gravidy.x, gravidy.y, gravidy.z);
        }
        public Vector3 gravity {
            set { ode.dWorldSetGravity(ptr, value.x, value.y, value.z); }
            get { return ode.dWorldGetGravity(ptr); }
        }
        public void Step(float deltaTime)
        {
            ode.dWorldStep(ptr, deltaTime);
            ode.dJointGroupEmpty(contactgroup);
        }
        public void AddBody(Body body)
        {
            bodies.Add(body);
        }
        public void RemoveBody(Body body)
        {
            bodies.Remove(body);
        }
    }

}