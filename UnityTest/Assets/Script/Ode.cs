using UnityEngine;
using System.Runtime.InteropServices;
using System;
using dReal = System.Single;
//using dWorldID = System.IntPtr;
//using dSpaceID = System.IntPtr;
//using dJointGroupID = System.IntPtr;
using dVector3 = UnityEngine.Vector4;
using dVector4 = UnityEngine.Vector4;
//using dQuaternion = UnityEngine.Quaternion;
//using dBodyID = System.IntPtr;
//using dJointID = System.IntPtr;
//using dGeomID = System.IntPtr;

[StructLayout(LayoutKind.Sequential)]
public struct dWorldID { public IntPtr intPtr; public bool IsZero() { return intPtr == IntPtr.Zero; } }
[StructLayout(LayoutKind.Sequential)]
public struct dSpaceID { public IntPtr intPtr; public bool IsZero() { return intPtr == IntPtr.Zero; } }
[StructLayout(LayoutKind.Sequential)]
public struct dJointGroupID { public IntPtr intPtr; public bool IsZero() { return intPtr == IntPtr.Zero; } }
[StructLayout(LayoutKind.Sequential)]
public struct dBodyID { public IntPtr intPtr; public bool IsZero() { return intPtr == IntPtr.Zero; } }
[StructLayout(LayoutKind.Sequential)]
public struct dJointID { public IntPtr intPtr; public bool IsZero() { return intPtr == IntPtr.Zero; } }
[StructLayout(LayoutKind.Sequential)]
public struct dGeomID { public IntPtr intPtr; public bool IsZero() { return intPtr == IntPtr.Zero; } }

[StructLayout(LayoutKind.Sequential)]
public struct dQuaternion
{
    float w, x, y, z;
    public static dQuaternion identity = new dQuaternion(1, 0, 0, 0);
    public dQuaternion(float x, float y, float z, float w)
    {
        this.w = w; this.x = x; this.y = y; this.z = z; 
    }
    public static implicit operator Quaternion(dQuaternion dq)
    {
        return new Quaternion(dq.x, dq.y, dq.z, dq.w);
    }
    public static implicit operator dQuaternion(Quaternion q)
    {
        return new dQuaternion(q.x, q.y, q.z, q.w);
    }
}


public class ode
{
    [DllImport("ode")]
    public static extern int dInitODE2(uint uiInitFlags = 0);
    [DllImport("ode")]
    public static extern void dCloseODE();

    public static void CheckVector3(Vector3 v)
    {
        if (v.x == Mathf.Infinity || v.y == Mathf.Infinity || v.z == Mathf.Infinity) {
            throw new Exception("Vector3 is Infinity");
        }
        if (float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z)) {
            throw new Exception("Vector3 is Nan");
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct dMatrix3
    {
        public dMatrix3(dReal[] array)
        {
            v0 = new dVector3(array[0 + 0], array[0 + 1], array[0 + 2]);
            v1 = new dVector3(array[3 + 0], array[3 + 1], array[3 + 2]);
            v2 = new dVector3(array[6 + 0], array[6 + 1], array[6 + 2]);
            v3 = new dVector3(array[9 + 0], array[9 + 1], array[9 + 2]);
        }
        public dVector3 v0;
        public dVector3 v1;
        public dVector3 v2;
        public dVector3 v3;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct dMass
    {
        public dReal mass;
        public dVector4 c;
        public dMatrix3 I;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct dJointFeedback
    {
        public dVector3 f1;        /* force applied to body 1 */
        public dVector3 t1;        /* torque applied to body 1 */
        public dVector3 f2;        /* force applied to body 2 */
        public dVector3 t2;        /* torque applied to body 2 */
    }


    public enum dContactType
    {
        Mu2 = 0x001,      /**< Use axis dependent friction */
        AxisDep = 0x001,      /**< Same as above */
        FDir1 = 0x002,      /**< Use FDir for the first friction value */
        Bounce = 0x004,      /**< Restore collision energy anti-parallel to the normal */
        SoftERP = 0x008,      /**< Don't use global erp for penetration reduction */
        SoftCFM = 0x010,      /**< Don't use global cfm for penetration constraint */
        Motion1 = 0x020,      /**< Use a non-zero target velocity for the constraint */
        Motion2 = 0x040,
        MotionN = 0x080,
        Slip1 = 0x100,      /**< Force-dependent slip. */
        Slip2 = 0x200,
        Rolling = 0x400,      /**< Rolling/Angular friction */

        Approx0 = 0x0000,
        Approx1_1 = 0x1000,
        Approx1_2 = 0x2000,
        Approx1_N = 0x4000,   /**< For rolling friction */
        Approx1 = 0x7000
    }

    public enum dParam
    {
        LoStop,
        HiStop,
        Vel,
        LoVel,
        HiVel,
        FMax,
        FudgeFactor,
        Bounce,
        CFM,
        StopERP,
        StopCFM,
        SuspensionERP,
        SuspensionCFM,
        ERP,

        Group1 = 0,
        LoStop1 = Group1,
        HiStop1,
        Vel1,
        LoVel1,
        HiVel1,
        FMax1,
        FudgeFactor1,
        Bounce1,
        CFM1,
        StopERP1,
        StopCFM1,
        SuspensionERP1,
        SuspensionCFM1,
        ERP1,

        Group2 = 0x100,
        LoStop2 = Group2,
        HiStop2,
        Vel2,
        LoVel2,
        HiVel2,
        FMax2,
        FudgeFactor2,
        Bounce2,
        CFM2,
        StopERP2,
        StopCFM2,
        SuspensionERP2,
        SuspensionCFM2,
        ERP2,

        Group3 = 0x200,
        LoStop3 = Group3,
        HiStop3,
        Vel3,
        LoVel3,
        HiVel3,
        FMax3,
        FudgeFactor3,
        Bounce3,
        CFM3,
        StopERP3,
        StopCFM3,
        SuspensionERP3,
        SuspensionCFM3,
        ERP3,
    }

    public enum dAMotorMode
    {
        User,
        Euler
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct dSurfaceParameters
    {
        /* must always be defined */
        public int mode;
        public dReal mu;

        /* only defined if the corresponding flag is set in mode */
        public dReal mu2;
        public dReal rho;                    /**< Rolling friction */
        public dReal rho2;
        public dReal rhoN;                   /**< Spinning friction */
        public dReal bounce;                 /**< Coefficient of restitution */
        public dReal bounce_vel;             /**< Bouncing threshold */
        public dReal soft_erp;
        public dReal soft_cfm;
        public dReal motion1, motion2, motionN;
        public dReal slip1, slip2;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct dContactGeom
    {
        public dVector3 pos;          /*< contact position*/
        public dVector3 normal;       /*< normal vector*/
        public dReal depth;           /*< penetration depth*/
        public dGeomID g1, g2;         /*< the colliding geoms*/
        public int side1, side2;       /*< (to be documented)*/
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct dContact
    {
        public dSurfaceParameters surface;
        public dContactGeom geom;
        public dVector3 fdir1;
    }


    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "ExtCopyFloat")]
    public static extern void ExtCopyFloatArrayToVector3(IntPtr inValue, out dVector3 outValue, int num);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "ExtCopyFloat")]
    public static extern void ExtCopyFloatArrayToQuaternion(IntPtr inValue, out dQuaternion outValue, int num);

    public delegate void FunOnNewContact(ref dContact contact,dBodyID body1,dBodyID body2);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void ExtSpaceCollide(dSpaceID space, FunOnNewContact callback);

    #region rotation.h

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dRSetIdentity(dMatrix3 R);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dRFromAxisAndAngle(dMatrix3 R, dReal ax, dReal ay, dReal az,
                 dReal angle);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dRFromEulerAngles(dMatrix3 R, dReal phi, dReal theta, dReal psi);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dRFrom2Axes(dMatrix3 R, dReal ax, dReal ay, dReal az,
              dReal bx, dReal by, dReal bz);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dRFromZAxis(dMatrix3 R, dReal ax, dReal ay, dReal az);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQSetIdentity(dQuaternion q);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQFromAxisAndAngle(dQuaternion q, dReal ax, dReal ay, dReal az,
                 dReal angle);

    /* Quaternion multiplication, analogous to the matrix multiplication routines. */
    /* qa = rotate by qc, then qb */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQMultiply0(dQuaternion qa, dQuaternion qb, dQuaternion qc);
    /* qa = rotate by qc, then by inverse of qb */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQMultiply1(dQuaternion qa, dQuaternion qb, dQuaternion qc);
    /* qa = rotate by inverse of qc, then by qb */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQMultiply2(dQuaternion qa, dQuaternion qb, dQuaternion qc);
    /* qa = rotate by inverse of qc, then by inverse of qb */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQMultiply3(dQuaternion qa, dQuaternion qb, dQuaternion qc);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dRfromQ(dMatrix3 R, dQuaternion q);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dQfromR(dQuaternion q, dMatrix3 R);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dDQfromW(dReal dq[4], dVector3 w, dQuaternion q);

    #endregion


    #region mass.h
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dMassCheck(ref dMass m);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetZero(ref dMass mass);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetParameters(ref dMass mass, dReal themass,
                 dReal cgx, dReal cgy, dReal cgz,
                 dReal I11, dReal I22, dReal I33,
                 dReal I12, dReal I13, dReal I23);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetSphere(ref dMass mass, dReal density, dReal radius);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetSphereTotal(ref dMass mass, dReal total_mass, dReal radius);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetCapsule(ref dMass mass, dReal density, int direction,
                  dReal radius, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetCapsuleTotal(ref dMass mass, dReal total_mass, int direction,
                dReal radius, dReal length);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetCylinder(ref dMass mass, dReal density, int direction,
                   dReal radius, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetCylinderTotal(ref dMass mass, dReal total_mass, int direction,
                    dReal radius, dReal length);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetBox(ref dMass mass, dReal density,
              dReal lx, dReal ly, dReal lz);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetBoxTotal(ref dMass mass, dReal total_mass,
                   dReal lx, dReal ly, dReal lz);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetTrimesh(ref dMass mass, dReal density, dGeomID g);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassSetTrimeshTotal(ref dMass m, dReal total_mass, dGeomID g);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassAdjust(ref dMass mass, dReal newmass);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassTranslate(ref dMass mass, dReal x, dReal y, dReal z);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassRotate(ref dMass mass, dMatrix3 R);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dMassAdd(ref dMass a, ref dMass b);


    /* Backwards compatible API */
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dMassSetCappedCylinder(ref dMass a, dReal b, int c, dReal d, dReal e);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dMassSetCappedCylinderTotal(ref dMass a, dReal b, int c, dReal d, dReal e);
    #endregion


    #region collision.h


    /**
     * @defgroup collide Collision Detection
     *
     * ODE has two main components: a dynamics simulation engine and a collision
     * detection engine. The collision engine is given information about the
     * shape of each body. At each time step it figures out which bodies touch
     * each other and passes the resulting contact point information to the user.
     * The user in turn creates contact joints between bodies.
     *
     * Using ODE's collision detection is optional - an alternative collision
     * detection system can be used as long as it can supply the right kinds of
     * contact information.
     */


    /* ************************************************************************ */
    /* general functions */

    /**
     * @brief Destroy a geom, removing it from any space.
     *
     * Destroy a geom, removing it from any space it is in first. This one
     * function destroys a geom of any type, but to create a geom you must call
     * a creation function for that type.
     *
     * When a space is destroyed, if its cleanup mode is 1 (the default) then all
     * the geoms in that space are automatically destroyed as well.
     *
     * @param geom the geom to be destroyed.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomDestroy(dGeomID geom);


    /**
     * @brief Set the user-defined data pointer stored in the geom.
     *
     * @param geom the geom to hold the data
     * @param data the data pointer to be stored
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetData(dGeomID geom, IntPtr data);


    /**
     * @brief Get the user-defined data pointer stored in the geom.
     *
     * @param geom the geom containing the data
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr dGeomGetData(dGeomID geom);


    /**
     * @brief Set the body associated with a placeable geom.
     *
     * Setting a body on a geom automatically combines the position vector and
     * rotation matrix of the body and geom, so that setting the position or
     * orientation of one will set the value for both objects. Setting a body
     * ID of zero gives the geom its own position and rotation, independent
     * from any body. If the geom was previously connected to a body then its
     * new independent position/rotation is set to the current position/rotation
     * of the body.
     *
     * Calling these functions on a non-placeable geom results in a runtime
     * error in the debug build of ODE.
     *
     * @param geom the geom to connect
     * @param body the body to attach to the geom
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetBody(dGeomID geom, dBodyID body);


    /**
     * @brief Get the body associated with a placeable geom.
     * @param geom the geom to query.
     * @sa dGeomSetBody
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dBodyID dGeomGetBody(dGeomID geom);


    /**
     * @brief Set the position vector of a placeable geom.
     *
     * If the geom is attached to a body, the body's position will also be changed.
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom the geom to set.
     * @param x the new X coordinate.
     * @param y the new Y coordinate.
     * @param z the new Z coordinate.
     * @sa dBodySetPosition
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetPosition(dGeomID geom, dReal x, dReal y, dReal z);


    /**
     * @brief Set the rotation matrix of a placeable geom.
     *
     * If the geom is attached to a body, the body's rotation will also be changed.
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom the geom to set.
     * @param R the new rotation matrix.
     * @sa dBodySetRotation
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetRotation(dGeomID geom, dMatrix3 R);


    /**
     * @brief Set the rotation of a placeable geom.
     *
     * If the geom is attached to a body, the body's rotation will also be changed.
     *
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom the geom to set.
     * @param Q the new rotation.
     * @sa dBodySetQuaternion
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetQuaternion(dGeomID geom, dQuaternion Q);


    /**
     * @brief Get the position vector of a placeable geom.
     *
     * If the geom is attached to a body, the body's position will be returned.
     *
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom the geom to query.
     * @returns A pointer to the geom's position vector.
     * @remarks The returned value is a pointer to the geom's internal
     *          data structure. It is valid until any changes are made
     *          to the geom.
     * @sa dBodyGetPosition
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "dGeomGetPosition")]
    static extern IntPtr _dGeomGetPosition(dGeomID geom);
    public static dVector3 dGeomGetPosition(dGeomID geom)
    {
        return ToVector3(_dGeomGetPosition(geom));
    }

    /**
     * @brief Copy the position of a geom into a vector.
     * @ingroup collide
     * @param geom  the geom to query
     * @param pos   a copy of the geom position
     * @sa dGeomGetPosition
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCopyPosition(dGeomID geom, dVector3 pos);


    /**
     * @brief Get the rotation matrix of a placeable geom.
     *
     * If the geom is attached to a body, the body's rotation will be returned.
     *
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom the geom to query.
     * @returns A pointer to the geom's rotation matrix.
     * @remarks The returned value is a pointer to the geom's internal
     *          data structure. It is valid until any changes are made
     *          to the geom.
     * @sa dBodyGetRotation
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "dGeomGetRotation")]
    static extern IntPtr _dGeomGetRotation(dGeomID geom);
    public static dMatrix3 dGeomGetRotation(dGeomID geom)
    {
        return ToMatrix3(_dGeomGetRotation(geom));
    }


    /**
     * @brief Get the rotation matrix of a placeable geom.
     *
     * If the geom is attached to a body, the body's rotation will be returned.
     *
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom   the geom to query.
     * @param R      a copy of the geom rotation
     * @sa dGeomGetRotation
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCopyRotation(dGeomID geom, dMatrix3 R);


    /**
     * @brief Get the rotation quaternion of a placeable geom.
     *
     * If the geom is attached to a body, the body's quaternion will be returned.
     *
     * Calling this function on a non-placeable geom results in a runtime error in
     * the debug build of ODE.
     *
     * @param geom the geom to query.
     * @param result a copy of the rotation quaternion.
     * @sa dBodyGetQuaternion
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomGetQuaternion(dGeomID geom, out dQuaternion result);


    /**
     * @brief Return the axis-aligned bounding box.
     *
     * Return in aabb an axis aligned bounding box that surrounds the given geom.
     * The aabb array has elements (minx, maxx, miny, maxy, minz, maxz). If the
     * geom is a space, a bounding box that surrounds all contained geoms is
     * returned.
     *
     * This function may return a pre-computed cached bounding box, if it can
     * determine that the geom has not moved since the last time the bounding
     * box was computed.
     *
     * @param geom the geom to query
     * @param aabb the returned bounding box
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomGetAABB(dGeomID geom, ref dReal[] aabb);


    /**
     * @brief Determing if a geom is a space.
     * @param geom the geom to query
     * @returns Non-zero if the geom is a space, zero otherwise.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomIsSpace(dGeomID geom);


    /**
     * @brief Query for the space containing a particular geom.
     * @param geom the geom to query
     * @returns The space that contains the geom, or NULL if the geom is
     *          not contained by a space.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dSpaceID dGeomGetSpace(dGeomID geom);


    /**
     * @brief Given a geom, this returns its class.
     *
     * The ODE classes are:
     *  @li dSphereClass
     *  @li dBoxClass
     *  @li dCylinderClass
     *  @li dPlaneClass
     *  @li dRayClass
     *  @li dConvexClass
     *  @li dGeomTransformClass
     *  @li dTriMeshClass
     *  @li dSimpleSpaceClass
     *  @li dHashSpaceClass
     *  @li dQuadTreeSpaceClass
     *  @li dFirstUserClass
     *  @li dLastUserClass
     *
     * User-defined class will return their own number.
     *
     * @param geom the geom to query
     * @returns The geom class ID.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomGetClass(dGeomID geom);


    /**
     * @brief Set the "category" bitfield for the given geom.
     *
     * The category bitfield is used by spaces to govern which geoms will
     * interact with each other. The bitfield is guaranteed to be at least
     * 32 bits wide. The default category values for newly created geoms
     * have all bits set.
     *
     * @param geom the geom to set
     * @param bits the new bitfield value
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetCategoryBits(dGeomID geom, uint bits);


    /**
     * @brief Set the "collide" bitfield for the given geom.
     *
     * The collide bitfield is used by spaces to govern which geoms will
     * interact with each other. The bitfield is guaranteed to be at least
     * 32 bits wide. The default category values for newly created geoms
     * have all bits set.
     *
     * @param geom the geom to set
     * @param bits the new bitfield value
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetCollideBits(dGeomID geom, uint bits);


    /**
     * @brief Get the "category" bitfield for the given geom.
     *
     * @param geom the geom to set
     * @param bits the new bitfield value
     * @sa dGeomSetCategoryBits
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern uint dGeomGetCategoryBits(dGeomID geom);


    /**
     * @brief Get the "collide" bitfield for the given geom.
     *
     * @param geom the geom to set
     * @param bits the new bitfield value
     * @sa dGeomSetCollideBits
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern uint dGeomGetCollideBits(dGeomID geom);


    /**
     * @brief Enable a geom.
     *
     * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
     * although they can still be members of a space. New geoms are created in
     * the enabled state.
     *
     * @param geom   the geom to enable
     * @sa dGeomDisable
     * @sa dGeomIsEnabled
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomEnable(dGeomID geom);


    /**
     * @brief Disable a geom.
     *
     * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
     * although they can still be members of a space. New geoms are created in
     * the enabled state.
     *
     * @param geom   the geom to disable
     * @sa dGeomDisable
     * @sa dGeomIsEnabled
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomDisable(dGeomID geom);


    /**
     * @brief Check to see if a geom is enabled.
     *
     * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
     * although they can still be members of a space. New geoms are created in
     * the enabled state.
     *
     * @param geom   the geom to query
     * @returns Non-zero if the geom is enabled, zero otherwise.
     * @sa dGeomDisable
     * @sa dGeomIsEnabled
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomIsEnabled(dGeomID geom);


    public enum GeomControlClass
    {
        dGeomCommonControlClass = 0,
        dGeomColliderControlClass = 1
    };

    public enum GeomControlCode
    {
        dGeomCommonAnyControlCode = 0,

        dGeomColliderSetMergeSphereContactsControlCode = 1,
        dGeomColliderGetMergeSphereContactsControlCode = 2
    };

    public enum GeomColliderMergeContactsValue
    {
        dGeomColliderMergeContactsValue__Default = 0, /* Used with Set... to restore default value*/
        dGeomColliderMergeContactsValue_None = 1,
        dGeomColliderMergeContactsValue_Normals = 2,
        dGeomColliderMergeContactsValue_Full = 3
    };

    /**
     * @brief Execute low level control operation for geometry.
     *
     * The variable the dataSize points to must be initialized before the call.
     * If the size does not match the one expected for the control class/code function
     * changes it to the size expected and returns failure. This implies the function 
     * can be called with NULL data and zero size to test if control class/code is supported
     * and obtain required data size for it.
     *
     * dGeomCommonAnyControlCode applies to any control class and returns success if 
     * at least one control code is available for the given class with given geom.
     *
     * Currently there are the folliwing control classes supported:
     *  @li dGeomColliderControlClass
     *
     * For dGeomColliderControlClass there are the following codes available:
     *  @li dGeomColliderSetMergeSphereContactsControlCode (arg of type int, dGeomColliderMergeContactsValue_*)
     *  @li dGeomColliderGetMergeSphereContactsControlCode (arg of type int, dGeomColliderMergeContactsValue_*)
     *
     * @param geom   the geom to control
     * @param controlClass   the control class
     * @param controlCode   the control code for the class
     * @param dataValue   the control argument pointer
     * @param dataSize   the control argument size provided or expected
     * @returns Boolean execution status
     * @ingroup collide
     */
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern int dGeomLowLevelControl(dGeomID geom, int controlClass, int controlCode, void* dataValue, int* dataSize);


    /**
     * @brief Get world position of a relative point on geom.
     *
     * Calling this function on a non-placeable geom results in the same point being
     * returned.
     *
     * @ingroup collide
     * @param result will contain the result.
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomGetRelPointPos
    (
      dGeomID geom, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief takes a point in global coordinates and returns
     * the point's position in geom-relative coordinates.
     *
     * Calling this function on a non-placeable geom results in the same point being
     * returned.
     *
     * @remarks
     * This is the inverse of dGeomGetRelPointPos()
     * @ingroup collide
     * @param result will contain the result.
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomGetPosRelPoint
    (
      dGeomID geom, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief Convert from geom-local to world coordinates.
     *
     * Calling this function on a non-placeable geom results in the same vector being
     * returned.
     *
     * @ingroup collide
     * @param result will contain the result.
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomVectorToWorld
    (
      dGeomID geom, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief Convert from world to geom-local coordinates.
     *
     * Calling this function on a non-placeable geom results in the same vector being
     * returned.
     *
     * @ingroup collide
     * @param result will contain the result.
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomVectorFromWorld
    (
      dGeomID geom, dReal px, dReal py, dReal pz,
      dVector3 result
    );


    /* ************************************************************************ */
    /* geom offset from body */

    /**
     * @brief Set the local offset position of a geom from its body.
     *
     * Sets the geom's positional offset in local coordinates.
     * After this call, the geom will be at a new position determined from the
     * body's position and the offset.
     * The geom must be attached to a body.
     * If the geom did not have an offset, it is automatically created.
     *
     * @param geom the geom to set.
     * @param x the new X coordinate.
     * @param y the new Y coordinate.
     * @param z the new Z coordinate.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetOffsetPosition(dGeomID geom, dReal x, dReal y, dReal z);


    /**
     * @brief Set the local offset rotation matrix of a geom from its body.
     *
     * Sets the geom's rotational offset in local coordinates.
     * After this call, the geom will be at a new position determined from the
     * body's position and the offset.
     * The geom must be attached to a body.
     * If the geom did not have an offset, it is automatically created.
     *
     * @param geom the geom to set.
     * @param R the new rotation matrix.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetOffsetRotation(dGeomID geom, dMatrix3 R);


    /**
     * @brief Set the local offset rotation of a geom from its body.
     *
     * Sets the geom's rotational offset in local coordinates.
     * After this call, the geom will be at a new position determined from the
     * body's position and the offset.
     * The geom must be attached to a body.
     * If the geom did not have an offset, it is automatically created.
     *
     * @param geom the geom to set.
     * @param Q the new rotation.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetOffsetQuaternion(dGeomID geom, dQuaternion Q);


    /**
     * @brief Set the offset position of a geom from its body.
     *
     * Sets the geom's positional offset to move it to the new world
     * coordinates.
     * After this call, the geom will be at the world position passed in,
     * and the offset will be the difference from the current body position.
     * The geom must be attached to a body.
     * If the geom did not have an offset, it is automatically created.
     *
     * @param geom the geom to set.
     * @param x the new X coordinate.
     * @param y the new Y coordinate.
     * @param z the new Z coordinate.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetOffsetWorldPosition(dGeomID geom, dReal x, dReal y, dReal z);


    /**
     * @brief Set the offset rotation of a geom from its body.
     *
     * Sets the geom's rotational offset to orient it to the new world
     * rotation matrix.
     * After this call, the geom will be at the world orientation passed in,
     * and the offset will be the difference from the current body orientation.
     * The geom must be attached to a body.
     * If the geom did not have an offset, it is automatically created.
     *
     * @param geom the geom to set.
     * @param R the new rotation matrix.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetOffsetWorldRotation(dGeomID geom, dMatrix3 R);


    /**
     * @brief Set the offset rotation of a geom from its body.
     *
     * Sets the geom's rotational offset to orient it to the new world
     * rotation matrix.
     * After this call, the geom will be at the world orientation passed in,
     * and the offset will be the difference from the current body orientation.
     * The geom must be attached to a body.
     * If the geom did not have an offset, it is automatically created.
     *
     * @param geom the geom to set.
     * @param Q the new rotation.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetOffsetWorldQuaternion(dGeomID geom, dQuaternion rot);


    /**
     * @brief Clear any offset from the geom.
     *
     * If the geom has an offset, it is eliminated and the geom is
     * repositioned at the body's position.  If the geom has no offset,
     * this function does nothing.
     * This is more efficient than calling dGeomSetOffsetPosition(zero)
     * and dGeomSetOffsetRotation(identiy), because this function actually
     * eliminates the offset, rather than leaving it as the identity transform.
     *
     * @param geom the geom to have its offset destroyed.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomClearOffset(dGeomID geom);


    /**
     * @brief Check to see whether the geom has an offset.
     *
     * This function will return non-zero if the offset has been created.
     * Note that there is a difference between a geom with no offset,
     * and a geom with an offset that is the identity transform.
     * In the latter case, although the observed behaviour is identical,
     * there is a unnecessary computation involved because the geom will
     * be applying the transform whenever it needs to recalculate its world
     * position.
     *
     * @param geom the geom to query.
     * @returns Non-zero if the geom has an offset, zero otherwise.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomIsOffset(dGeomID geom);


    /**
     * @brief Get the offset position vector of a geom.
     *
     * Returns the positional offset of the geom in local coordinates.
     * If the geom has no offset, this function returns the zero vector.
     *
     * @param geom the geom to query.
     * @returns A pointer to the geom's offset vector.
     * @remarks The returned value is a pointer to the geom's internal
     *          data structure. It is valid until any changes are made
     *          to the geom.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "dGeomGetOffsetPosition")]
    public static extern IntPtr _dGeomGetOffsetPosition(dGeomID geom);
    public static dVector3 dGeomGetOffsetPosition(dGeomID geom)
    {
        return ToVector3(_dGeomGetOffsetPosition(geom));
    }

    /**
     * @brief Copy the offset position vector of a geom.
     *
     * Returns the positional offset of the geom in local coordinates.
     * If the geom has no offset, this function returns the zero vector.
     *
     * @param geom   the geom to query.
     * @param pos    returns the offset position
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCopyOffsetPosition(dGeomID geom, dVector3 pos);


    /**
     * @brief Get the offset rotation matrix of a geom.
     *
     * Returns the rotational offset of the geom in local coordinates.
     * If the geom has no offset, this function returns the identity
     * matrix.
     *
     * @param geom the geom to query.
     * @returns A pointer to the geom's offset rotation matrix.
     * @remarks The returned value is a pointer to the geom's internal
     *          data structure. It is valid until any changes are made
     *          to the geom.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "dGeomGetOffsetRotation")]
    static extern IntPtr _dGeomGetOffsetRotation(dGeomID geom);
    public static dMatrix3 dGeomGetOffsetRotation(dGeomID geom)
    {
        return ToMatrix3(_dGeomGetOffsetPosition(geom));
    }


    /**
     * @brief Copy the offset rotation matrix of a geom.
     *
     * Returns the rotational offset of the geom in local coordinates.
     * If the geom has no offset, this function returns the identity
     * matrix.
     *
     * @param geom   the geom to query.
     * @param R      returns the rotation matrix.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCopyOffsetRotation(dGeomID geom, dMatrix3 R);


    /**
     * @brief Get the offset rotation quaternion of a geom.
     *
     * Returns the rotation offset of the geom as a quaternion.
     * If the geom has no offset, the identity quaternion is returned.
     *
     * @param geom the geom to query.
     * @param result a copy of the rotation quaternion.
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl, EntryPoint = "dGeomGetOffsetQuaternion")]
    static extern void _dGeomGetOffsetQuaternion(dGeomID geom, ref dQuaternion result);
    public static dQuaternion dGeomGetOffsetQuaternion(dGeomID geom)
    {
        dQuaternion rot = dQuaternion.identity;
        _dGeomGetOffsetQuaternion(geom, ref rot);
        return rot;
    }

    /* ************************************************************************ */
    /* collision detection */

    /*
     *	Just generate any contacts (disables any contact refining).
     */
    public const uint CONTACTS_UNIMPORTANT = 0x80000000;

    /**
     *
     * @brief Given two geoms o1 and o2 that potentially intersect,
     * generate contact information for them.
     *
     * Internally, this just calls the correct class-specific collision
     * functions for o1 and o2.
     *
     * @param o1 The first geom to test.
     * @param o2 The second geom to test.
     *
     * @param flags The flags specify how contacts should be generated if
     * the geoms touch. The lower 16 bits of flags is an integer that
     * specifies the maximum number of contact points to generate. You must
     * ask for at least one contact. 
     * Additionally, following bits may be set:
     * CONTACTS_UNIMPORTANT -- just generate any contacts (skip contact refining).
     * All other bits in flags must be set to zero. In the future the other bits 
     * may be used to select from different contact generation strategies.
     *
     * @param contact Points to an array of dContactGeom structures. The array
     * must be able to hold at least the maximum number of contacts. These
     * dContactGeom structures may be embedded within larger structures in the
     * array -- the skip parameter is the byte offset from one dContactGeom to
     * the next in the array. If skip is sizeof(dContactGeom) then contact
     * points to a normal (C-style) array. It is an error for skip to be smaller
     * than sizeof(dContactGeom).
     *
     * @returns If the geoms intersect, this function returns the number of contact
     * points generated (and updates the contact array), otherwise it returns 0
     * (and the contact array is not touched).
     *
     * @remarks If a space is passed as o1 or o2 then this function will collide
     * all objects contained in o1 with all objects contained in o2, and return
     * the resulting contact points. This method for colliding spaces with geoms
     * (or spaces with spaces) provides no user control over the individual
     * collisions. To get that control, use dSpaceCollide or dSpaceCollide2 instead.
     *
     * @remarks If o1 and o2 are the same geom then this function will do nothing
     * and return 0. Technically speaking an object intersects with itself, but it
     * is not useful to find contact points in this case.
     *
     * @remarks This function does not care if o1 and o2 are in the same space or not
     * (or indeed if they are in any space at all).
     *
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dCollide(dGeomID o1, dGeomID o2, int flags, ref dContactGeom contact,
              int skip);

    /**
     * @brief Determines which pairs of geoms in a space may potentially intersect,
     * and calls the callback function for each candidate pair.
     *
     * @param space The space to test.
     *
     * @param data Passed from dSpaceCollide directly to the callback
     * function. Its meaning is user defined. The o1 and o2 arguments are the
     * geoms that may be near each other.
     *
     * @param callback A callback function is of type @ref dNearCallback.
     *
     * @remarks Other spaces that are contained within the colliding space are
     * not treated specially, i.e. they are not recursed into. The callback
     * function may be passed these contained spaces as one or both geom
     * arguments.
     *
     * @remarks dSpaceCollide() is guaranteed to pass all intersecting geom
     * pairs to the callback function, but may also pass close but
     * non-intersecting pairs. The number of these calls depends on the
     * internal algorithms used by the space. Thus you should not expect
     * that dCollide will return contacts for every pair passed to the
     * callback.
     *
     * @sa dSpaceCollide2
     * @ingroup collide
     */
    public delegate void dNearCallback(IntPtr data, dGeomID o1, dGeomID o2);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dSpaceCollide(dSpaceID space, IntPtr data, dNearCallback callback);


    /**
     * @brief Determines which geoms from one space may potentially intersect with 
     * geoms from another space, and calls the callback function for each candidate 
     * pair. 
     *
     * @param space1 The first space to test.
     *
     * @param space2 The second space to test.
     *
     * @param data Passed from dSpaceCollide directly to the callback
     * function. Its meaning is user defined. The o1 and o2 arguments are the
     * geoms that may be near each other.
     *
     * @param callback A callback function is of type @ref dNearCallback.
     *
     * @remarks This function can also test a single non-space geom against a 
     * space. This function is useful when there is a collision hierarchy, i.e. 
     * when there are spaces that contain other spaces.
     *
     * @remarks Other spaces that are contained within the colliding space are
     * not treated specially, i.e. they are not recursed into. The callback
     * function may be passed these contained spaces as one or both geom
     * arguments.
     *
     * @remarks Sublevel value of space affects how the spaces are iterated.
     * Both spaces are recursed only if their sublevels match. Otherwise, only
     * the space with greater sublevel is recursed and the one with lesser sublevel
     * is used as a geom itself.
     *
     * @remarks dSpaceCollide2() is guaranteed to pass all intersecting geom
     * pairs to the callback function, but may also pass close but
     * non-intersecting pairs. The number of these calls depends on the
     * internal algorithms used by the space. Thus you should not expect
     * that dCollide will return contacts for every pair passed to the
     * callback.
     *
     * @sa dSpaceCollide
     * @sa dSpaceSetSublevel
     * @ingroup collide
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dSpaceCollide2(dGeomID space1, dGeomID space2, IntPtr data, dNearCallback callback);


    /* ************************************************************************ */
    /* standard classes */

    /* the maximum number of user classes that are supported */
    public enum UserClass
    {
        dMaxUserClasses = 4
    };

    /* class numbers - each geometry object needs a unique number */
    public enum GeomClass
    {
        dSphereClass = 0,
        dBoxClass,
        dCapsuleClass,
        dCylinderClass,
        dPlaneClass,
        dRayClass,
        dConvexClass,
        dGeomTransformClass,
        dTriMeshClass,
        dHeightfieldClass,

        dFirstSpaceClass,
        dSimpleSpaceClass = dFirstSpaceClass,
        dHashSpaceClass,
        dSweepAndPruneSpaceClass, /* SAP */
        dQuadTreeSpaceClass,
        dLastSpaceClass = dQuadTreeSpaceClass,

        dFirstUserClass,
        dLastUserClass = dFirstUserClass + UserClass.dMaxUserClasses - 1,
        dGeomNumClasses
    };


    /**
     * @defgroup collide_sphere Sphere Class
     * @ingroup collide
     */

    /**
     * @brief Create a sphere geom of the given radius, and return its ID. 
     *
     * @param space   a space to contain the new geom. May be null.
     * @param radius  the radius of the sphere.
     *
     * @returns A new sphere geom.
     *
     * @remarks The point of reference for a sphere is its center.
     *
     * @sa dGeomDestroy
     * @sa dGeomSphereSetRadius
     * @ingroup collide_sphere
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateSphere(dSpaceID space, dReal radius);


    /**
     * @brief Set the radius of a sphere geom.
     *
     * @param sphere  the sphere to set.
     * @param radius  the new radius.
     *
     * @sa dGeomSphereGetRadius
     * @ingroup collide_sphere
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSphereSetRadius(dGeomID sphere, dReal radius);


    /**
     * @brief Retrieves the radius of a sphere geom.
     *
     * @param sphere  the sphere to query.
     *
     * @sa dGeomSphereSetRadius
     * @ingroup collide_sphere
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dReal dGeomSphereGetRadius(dGeomID sphere);


    /**
     * @brief Calculate the depth of the a given point within a sphere.
     *
     * @param sphere  the sphere to query.
     * @param x       the X coordinate of the point.
     * @param y       the Y coordinate of the point.
     * @param z       the Z coordinate of the point.
     *
     * @returns The depth of the point. Points inside the sphere will have a 
     * positive depth, points outside it will have a negative depth, and points
     * on the surface will have a depth of zero.
     *
     * @ingroup collide_sphere
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dReal dGeomSpherePointDepth(dGeomID sphere, dReal x, dReal y, dReal z);


    /*--> Convex Functions*/
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateConvex(dSpaceID space, dReal[] _planes,
                   uint _planecount,
                   dReal[] _points,
                   uint _pointcount,
                   uint[] _polygons);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomSetConvex(dGeomID g,
                 dReal[] _planes,
                 uint _count,
                 dReal[] _points,
                 uint _pointcount,
                 uint[] _polygons);
    /*<-- Convex Functions*/

    /**
     * @defgroup collide_box Box Class
     * @ingroup collide
     */

    /**
     * @brief Create a box geom with the provided side lengths.
     *
     * @param space   a space to contain the new geom. May be null.
     * @param lx      the length of the box along the X axis
     * @param ly      the length of the box along the Y axis
     * @param lz      the length of the box along the Z axis
     *
     * @returns A new box geom.
     *
     * @remarks The point of reference for a box is its center.
     *
     * @sa dGeomDestroy
     * @sa dGeomBoxSetLengths
     * @ingroup collide_box
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateBox(dSpaceID space, dReal lx, dReal ly, dReal lz);


    /**
     * @brief Set the side lengths of the given box.
     *
     * @param box  the box to set
     * @param lx      the length of the box along the X axis
     * @param ly      the length of the box along the Y axis
     * @param lz      the length of the box along the Z axis
     *
     * @sa dGeomBoxGetLengths
     * @ingroup collide_box
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomBoxSetLengths(dGeomID box, dReal lx, dReal ly, dReal lz);


    /**
     * @brief Get the side lengths of a box.
     *
     * @param box     the box to query
     * @param result  the returned side lengths
     *
     * @sa dGeomBoxSetLengths
     * @ingroup collide_box
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomBoxGetLengths(dGeomID box, dVector3 result);


    /**
     * @brief Return the depth of a point in a box.
     * 
     * @param box  the box to query
     * @param x    the X coordinate of the point to test.
     * @param y    the Y coordinate of the point to test.
     * @param z    the Z coordinate of the point to test.
     *
     * @returns The depth of the point. Points inside the box will have a 
     * positive depth, points outside it will have a negative depth, and points
     * on the surface will have a depth of zero.
     */
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dReal dGeomBoxPointDepth(dGeomID box, dReal x, dReal y, dReal z);


    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreatePlane(dSpaceID space, dReal a, dReal b, dReal c, dReal d);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomPlaneSetParams(dGeomID plane, dReal a, dReal b, dReal c, dReal d);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomPlaneGetParams(dGeomID plane, dVector4 result);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dReal dGeomPlanePointDepth(dGeomID plane, dReal x, dReal y, dReal z);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateCapsule(dSpaceID space, dReal radius, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCapsuleSetParams(dGeomID ccylinder, dReal radius, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCapsuleGetParams(dGeomID ccylinder, ref dReal radius, ref dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dReal dGeomCapsulePointDepth(dGeomID ccylinder, dReal x, dReal y, dReal z);

    /* For now we want to have a backwards compatible C-API, note: C++ API is not.*/
    //#define dCreateCCylinder dCreateCapsule
    //#define dGeomCCylinderSetParams dGeomCapsuleSetParams
    //#define dGeomCCylinderGetParams dGeomCapsuleGetParams
    //#define dGeomCCylinderPointDepth dGeomCapsulePointDepth
    //#define dCCylinderClass dCapsuleClass

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateCylinder(dSpaceID space, dReal radius, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCylinderSetParams(dGeomID cylinder, dReal radius, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomCylinderGetParams(dGeomID cylinder, ref dReal radius, ref dReal length);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateRay(dSpaceID space, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomRaySetLength(dGeomID ray, dReal length);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dReal dGeomRayGetLength(dGeomID ray);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomRaySet(dGeomID ray, dReal px, dReal py, dReal pz,
              dReal dx, dReal dy, dReal dz);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomRayGet(dGeomID ray, dVector3 start, dVector3 dir);

    /*
     * Set/get ray flags that influence ray collision detection.
     * These flags are currently only noticed by the trimesh collider, because
     * they can make a major differences there.
     */
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomRaySetParams(dGeomID g, int FirstContact, int BackfaceCull);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomRayGetParams(dGeomID g, int* FirstContact, int* BackfaceCull);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomRaySetFirstContact(dGeomID g, int firstContact);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomRayGetFirstContact(dGeomID g);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomRaySetBackfaceCull(dGeomID g, int backfaceCull);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomRayGetBackfaceCull(dGeomID g);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dGeomRaySetClosestHit(dGeomID g, int closestHit);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dGeomRayGetClosestHit(dGeomID g);

    /*# include "collision_trimesh.h"*/

    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern dGeomID dCreateGeomTransform(dSpaceID space);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomTransformSetGeom(dGeomID g, dGeomID obj);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern dGeomID dGeomTransformGetGeom(dGeomID g);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomTransformSetCleanup(dGeomID g, int mode);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern int dGeomTransformGetCleanup(dGeomID g);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomTransformSetInfo(dGeomID g, int mode);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern_DEPRECATED [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern int dGeomTransformGetInfo(dGeomID g);


    /* ************************************************************************ */
    /* heightfield functions */


    /* Data storage for heightfield data.*/
    //struct dxHeightfieldData;
    //typedef struct dxHeightfieldData* dHeightfieldDataID;


    /**
     * @brief Callback prototype
     *
     * Used by the callback heightfield data type to sample a height for a
     * given cell position.
     *
     * @param p_user_data User data specified when creating the dHeightfieldDataID
     * @param x The index of a sample in the local x axis. It is a value
     * in the range zero to ( nWidthSamples - 1 ).
     * @param x The index of a sample in the local z axis. It is a value
     * in the range zero to ( nDepthSamples - 1 ).
     *
     * @return The sample height which is then scaled and offset using the
     * values specified when the heightfield data was created.
     *
     * @ingroup collide
     */
    //typedef dReal dHeightfieldGetHeight(void* p_user_data, int x, int z);



    /**
     * @brief Creates a heightfield geom.
     *
     * Uses the information in the given dHeightfieldDataID to construct
     * a geom representing a heightfield in a collision space.
     *
     * @param space The space to add the geom to.
     * @param data The dHeightfieldDataID created by dGeomHeightfieldDataCreate and
     * setup by dGeomHeightfieldDataBuildCallback, dGeomHeightfieldDataBuildByte,
     * dGeomHeightfieldDataBuildShort or dGeomHeightfieldDataBuildFloat.
     * @param bPlaceable If non-zero this geom can be transformed in the world using the
     * usual functions such as dGeomSetPosition and dGeomSetRotation. If the geom is
     * not set as placeable, then it uses a fixed orientation where the global y axis
     * represents the dynamic 'height' of the heightfield.
     *
     * @return A geom id to reference this geom in other calls.
     *
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern dGeomID dCreateHeightfield(dSpaceID space,
    //                        dHeightfieldDataID data, int bPlaceable);


    /**
     * @brief Creates a new empty dHeightfieldDataID.
     *
     * Allocates a new dHeightfieldDataID and returns it. You must call
     * dGeomHeightfieldDataDestroy to destroy it after the geom has been removed.
     * The dHeightfieldDataID value is used when specifying a data format type.
     *
     * @return A dHeightfieldDataID for use with dGeomHeightfieldDataBuildCallback,
     * dGeomHeightfieldDataBuildByte, dGeomHeightfieldDataBuildShort or
     * dGeomHeightfieldDataBuildFloat.
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern dHeightfieldDataID dGeomHeightfieldDataCreate(void);


    /**
     * @brief Destroys a dHeightfieldDataID.
     *
     * Deallocates a given dHeightfieldDataID and all managed resources.
     *
     * @param d A dHeightfieldDataID created by dGeomHeightfieldDataCreate
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataDestroy(dHeightfieldDataID d);



    /**
     * @brief Configures a dHeightfieldDataID to use a callback to
     * retrieve height data.
     *
     * Before a dHeightfieldDataID can be used by a geom it must be
     * configured to specify the format of the height data.
     * This call specifies that the heightfield data is computed by
     * the user and it should use the given callback when determining
     * the height of a given element of it's shape.
     *
     * @param d A new dHeightfieldDataID created by dGeomHeightfieldDataCreate
     *
     * @param width Specifies the total 'width' of the heightfield along
     * the geom's local x axis.
     * @param depth Specifies the total 'depth' of the heightfield along
     * the geom's local z axis.
     *
     * @param widthSamples Specifies the number of vertices to sample
     * along the width of the heightfield. Each vertex has a corresponding
     * height value which forms the overall shape.
     * Naturally this value must be at least two or more.
     * @param depthSamples Specifies the number of vertices to sample
     * along the depth of the heightfield.
     *
     * @param scale A uniform scale applied to all raw height data.
     * @param offset An offset applied to the scaled height data.
     *
     * @param thickness A value subtracted from the lowest height
     * value which in effect adds an additional cuboid to the base of the
     * heightfield. This is used to prevent geoms from looping under the
     * desired terrain and not registering as a collision. Note that the
     * thickness is not affected by the scale or offset parameters.
     *
     * @param bWrap If non-zero the heightfield will infinitely tile in both
     * directions along the local x and z axes. If zero the heightfield is
     * bounded from zero to width in the local x axis, and zero to depth in
     * the local z axis.
     *
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataBuildCallback(dHeightfieldDataID d,
    //                    void* pUserData, dHeightfieldGetHeight* pCallback,
    //                    dReal width, dReal depth, int widthSamples, int depthSamples,
    //                    dReal scale, dReal offset, dReal thickness, int bWrap);

    /**
     * @brief Configures a dHeightfieldDataID to use height data in byte format.
     *
     * Before a dHeightfieldDataID can be used by a geom it must be
     * configured to specify the format of the height data.
     * This call specifies that the heightfield data is stored as a rectangular
     * array of bytes (8 bit unsigned) representing the height at each sample point.
     *
     * @param d A new dHeightfieldDataID created by dGeomHeightfieldDataCreate
     *
     * @param pHeightData A pointer to the height data.
     * @param bCopyHeightData When non-zero the height data is copied to an
     * internal store. When zero the height data is accessed by reference and
     * so must persist throughout the lifetime of the heightfield.
     *
     * @param width Specifies the total 'width' of the heightfield along
     * the geom's local x axis.
     * @param depth Specifies the total 'depth' of the heightfield along
     * the geom's local z axis.
     *
     * @param widthSamples Specifies the number of vertices to sample
     * along the width of the heightfield. Each vertex has a corresponding
     * height value which forms the overall shape.
     * Naturally this value must be at least two or more.
     * @param depthSamples Specifies the number of vertices to sample
     * along the depth of the heightfield.
     *
     * @param scale A uniform scale applied to all raw height data.
     * @param offset An offset applied to the scaled height data.
     *
     * @param thickness A value subtracted from the lowest height
     * value which in effect adds an additional cuboid to the base of the
     * heightfield. This is used to prevent geoms from looping under the
     * desired terrain and not registering as a collision. Note that the
     * thickness is not affected by the scale or offset parameters.
     *
     * @param bWrap If non-zero the heightfield will infinitely tile in both
     * directions along the local x and z axes. If zero the heightfield is
     * bounded from zero to width in the local x axis, and zero to depth in
     * the local z axis.
     *
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataBuildByte(dHeightfieldDataID d,
    //                const unsigned char* pHeightData, int bCopyHeightData,
    //                dReal width, dReal depth, int widthSamples, int depthSamples,
    //                dReal scale, dReal offset, dReal thickness,	int bWrap );

    /**
     * @brief Configures a dHeightfieldDataID to use height data in short format.
     *
     * Before a dHeightfieldDataID can be used by a geom it must be
     * configured to specify the format of the height data.
     * This call specifies that the heightfield data is stored as a rectangular
     * array of shorts (16 bit signed) representing the height at each sample point.
     *
     * @param d A new dHeightfieldDataID created by dGeomHeightfieldDataCreate
     *
     * @param pHeightData A pointer to the height data.
     * @param bCopyHeightData When non-zero the height data is copied to an
     * internal store. When zero the height data is accessed by reference and
     * so must persist throughout the lifetime of the heightfield.
     *
     * @param width Specifies the total 'width' of the heightfield along
     * the geom's local x axis.
     * @param depth Specifies the total 'depth' of the heightfield along
     * the geom's local z axis.
     *
     * @param widthSamples Specifies the number of vertices to sample
     * along the width of the heightfield. Each vertex has a corresponding
     * height value which forms the overall shape.
     * Naturally this value must be at least two or more.
     * @param depthSamples Specifies the number of vertices to sample
     * along the depth of the heightfield.
     *
     * @param scale A uniform scale applied to all raw height data.
     * @param offset An offset applied to the scaled height data.
     *
     * @param thickness A value subtracted from the lowest height
     * value which in effect adds an additional cuboid to the base of the
     * heightfield. This is used to prevent geoms from looping under the
     * desired terrain and not registering as a collision. Note that the
     * thickness is not affected by the scale or offset parameters.
     *
     * @param bWrap If non-zero the heightfield will infinitely tile in both
     * directions along the local x and z axes. If zero the heightfield is
     * bounded from zero to width in the local x axis, and zero to depth in
     * the local z axis.
     *
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataBuildShort(dHeightfieldDataID d,
    //
    //                const short* pHeightData, int bCopyHeightData,
    //                dReal width, dReal depth, int widthSamples, int depthSamples,
    //                dReal scale, dReal offset, dReal thickness, int bWrap );
    //
    /**
     * @brief Configures a dHeightfieldDataID to use height data in 
     * single precision floating point format.
     *
     * Before a dHeightfieldDataID can be used by a geom it must be
     * configured to specify the format of the height data.
     * This call specifies that the heightfield data is stored as a rectangular
     * array of single precision floats representing the height at each
     * sample point.
     *
     * @param d A new dHeightfieldDataID created by dGeomHeightfieldDataCreate
     *
     * @param pHeightData A pointer to the height data.
     * @param bCopyHeightData When non-zero the height data is copied to an
     * internal store. When zero the height data is accessed by reference and
     * so must persist throughout the lifetime of the heightfield.
     *
     * @param width Specifies the total 'width' of the heightfield along
     * the geom's local x axis.
     * @param depth Specifies the total 'depth' of the heightfield along
     * the geom's local z axis.
     *
     * @param widthSamples Specifies the number of vertices to sample
     * along the width of the heightfield. Each vertex has a corresponding
     * height value which forms the overall shape.
     * Naturally this value must be at least two or more.
     * @param depthSamples Specifies the number of vertices to sample
     * along the depth of the heightfield.
     *
     * @param scale A uniform scale applied to all raw height data.
     * @param offset An offset applied to the scaled height data.
     *
     * @param thickness A value subtracted from the lowest height
     * value which in effect adds an additional cuboid to the base of the
     * heightfield. This is used to prevent geoms from looping under the
     * desired terrain and not registering as a collision. Note that the
     * thickness is not affected by the scale or offset parameters.
     *
     * @param bWrap If non-zero the heightfield will infinitely tile in both
     * directions along the local x and z axes. If zero the heightfield is
     * bounded from zero to width in the local x axis, and zero to depth in
     * the local z axis.
     *
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataBuildSingle(dHeightfieldDataID d,
    //
    //                const float* pHeightData, int bCopyHeightData,
    //                dReal width, dReal depth, int widthSamples, int depthSamples,
    //                dReal scale, dReal offset, dReal thickness, int bWrap );
    //
    /**
     * @brief Configures a dHeightfieldDataID to use height data in 
     * double precision floating point format.
     *
     * Before a dHeightfieldDataID can be used by a geom it must be
     * configured to specify the format of the height data.
     * This call specifies that the heightfield data is stored as a rectangular
     * array of double precision floats representing the height at each
     * sample point.
     *
     * @param d A new dHeightfieldDataID created by dGeomHeightfieldDataCreate
     *
     * @param pHeightData A pointer to the height data.
     * @param bCopyHeightData When non-zero the height data is copied to an
     * internal store. When zero the height data is accessed by reference and
     * so must persist throughout the lifetime of the heightfield.
     *
     * @param width Specifies the total 'width' of the heightfield along
     * the geom's local x axis.
     * @param depth Specifies the total 'depth' of the heightfield along
     * the geom's local z axis.
     *
     * @param widthSamples Specifies the number of vertices to sample
     * along the width of the heightfield. Each vertex has a corresponding
     * height value which forms the overall shape.
     * Naturally this value must be at least two or more.
     * @param depthSamples Specifies the number of vertices to sample
     * along the depth of the heightfield.
     *
     * @param scale A uniform scale applied to all raw height data.
     * @param offset An offset applied to the scaled height data.
     *
     * @param thickness A value subtracted from the lowest height
     * value which in effect adds an additional cuboid to the base of the
     * heightfield. This is used to prevent geoms from looping under the
     * desired terrain and not registering as a collision. Note that the
     * thickness is not affected by the scale or offset parameters.
     *
     * @param bWrap If non-zero the heightfield will infinitely tile in both
     * directions along the local x and z axes. If zero the heightfield is
     * bounded from zero to width in the local x axis, and zero to depth in
     * the local z axis.
     *
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataBuildDouble(dHeightfieldDataID d,
    //
    //                const double* pHeightData, int bCopyHeightData,
    //                dReal width, dReal depth, int widthSamples, int depthSamples,
    //                dReal scale, dReal offset, dReal thickness, int bWrap );

    /**
     * @brief Manually set the minimum and maximum height bounds.
     *
     * This call allows you to set explicit min / max values after initial
     * creation typically for callback heightfields which default to +/- infinity,
     * or those whose data has changed. This must be set prior to binding with a
     * geom, as the the AABB is not recomputed after it's first generation.
     *
     * @remarks The minimum and maximum values are used to compute the AABB
     * for the heightfield which is used for early rejection of collisions.
     * A close fit will yield a more efficient collision check.
     *
     * @param d A dHeightfieldDataID created by dGeomHeightfieldDataCreate
     * @param min_height The new minimum height value. Scale, offset and thickness is then applied.
     * @param max_height The new maximum height value. Scale and offset is then applied.
     * @ingroup collide
     */
    //   [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldDataSetBounds(dHeightfieldDataID d,
    //                   dReal minHeight, dReal maxHeight);


    /**
     * @brief Assigns a dHeightfieldDataID to a heightfield geom.
     *
     * Associates the given dHeightfieldDataID with a heightfield geom.
     * This is done without affecting the GEOM_PLACEABLE flag.
     *
     * @param g A geom created by dCreateHeightfield
     * @param d A dHeightfieldDataID created by dGeomHeightfieldDataCreate
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dGeomHeightfieldSetHeightfieldData(dGeomID g, dHeightfieldDataID d);


    /**
     * @brief Gets the dHeightfieldDataID bound to a heightfield geom.
     *
     * Returns the dHeightfieldDataID associated with a heightfield geom.
     *
     * @param g A geom created by dCreateHeightfield
     * @return The dHeightfieldDataID which may be NULL if none was assigned.
     * @ingroup collide
     */
    //    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern dHeightfieldDataID dGeomHeightfieldGetHeightfieldData(dGeomID g);



    /* ************************************************************************ */
    /* utility functions */

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dClosestLineSegmentPoints(dVector3 a1, dVector3 a2,
                dVector3 b1, dVector3 b2,
                dVector3 cp1, dVector3 cp2);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern int dBoxTouchesBox(dVector3 _p1, dMatrix3 R1,
            dVector3 side1, dVector3 _p2,
            dMatrix3 R2, dVector3 side2);

    /* The meaning of flags parameter is the same as in dCollide()*/
    // [DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern int dBoxBox(dVector3 p1, dMatrix3 R1,
    //      dVector3 side1, dVector3 p2,
    //      dMatrix3 R2, dVector3 side2,
    //      dVector3 normal, dReal* depth, int* return_code,
    //
    //      int flags, dContactGeom *contact, int skip);

    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern void dInfiniteAABB(dGeomID geom, dReal[] aabb);


    /* ************************************************************************ */
    /* custom classes */

    //typedef void dGetAABBFn(dGeomID, dReal aabb[6]);
    //typedef int dColliderFn(dGeomID o1, dGeomID o2,
    //             int flags, dContactGeom* contact, int skip);
    //typedef dColliderFn * dGetColliderFnFn(int num);
    //typedef void dGeomDtorFn(dGeomID o);
    //typedef int dAABBTestFn(dGeomID o1, dGeomID o2, dReal aabb[6]);
    ////
    //public struct dGeomClass
    //{
    //    int bytes;
    //    dGetColliderFnFn* collider;
    //    dGetAABBFn* aabb;
    //    dAABBTestFn* aabb_test;
    //    dGeomDtorFn* dtor;
    //}
    //
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern int dCreateGeomClass(const dGeomClass* classptr);
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void* dGeomGetClassData(dGeomID geom);
    [DllImport("ode", CallingConvention = CallingConvention.Cdecl)]
    public static extern dGeomID dCreateGeom(int classnum);

    /**
     * @brief Sets a custom collider function for two geom classes. 
     *
     * @param i The first geom class handled by this collider
     * @param j The second geom class handled by this collider
     * @param fn The collider function to use to determine collisions.
     * @ingroup collide
     */
    //[DllImport("ode", CallingConvention = CallingConvention.Cdecl)] public static extern void dSetColliderOverride(int i, int j, dColliderFn* fn);


    #endregion

    #region collision_space.h

    /**
     * @brief User callback for geom-geom collision testing.
     *
     * @param data The user data object, as passed to dSpaceCollide.
     * @param o1   The first geom being tested.
     * @param o2   The second geom being test.
     *
     * @remarks The callback function can call dCollide on o1 and o2 to generate
     * contact points between each pair. Then these contact points may be added
     * to the simulation as contact joints. The user's callback function can of
     * course chose not to call dCollide for any pair, e.g. if the user decides
     * that those pairs should not interact.
     *
     * @ingroup collide
     */
    //typedef void dNearCallback(void* data, dGeomID o1, dGeomID o2);


    [DllImport("ode")]
    public static extern dSpaceID dSimpleSpaceCreate(dSpaceID space);
    [DllImport("ode")]
    public static extern dSpaceID dHashSpaceCreate(dSpaceID space);
    public static dSpaceID dHashSpaceCreate() { return dHashSpaceCreate(new dSpaceID()); }
    [DllImport("ode")]
    public static extern dSpaceID dQuadTreeSpaceCreate(dSpaceID space, dVector3 Center, dVector3 Extents, int Depth);


    /* SAP */
    /* Order XZY or ZXY usually works best, if your Y is up. */
    public const int dSAP_AXES_XYZ = ((0) | (1 << 2) | (2 << 4));
    public const int dSAP_AXES_XZY = ((0) | (2 << 2) | (1 << 4));
    public const int dSAP_AXES_YXZ = ((1) | (0 << 2) | (2 << 4));
    public const int dSAP_AXES_YZX = ((1) | (2 << 2) | (0 << 4));
    public const int dSAP_AXES_ZXY = ((2) | (0 << 2) | (1 << 4));
    public const int dSAP_AXES_ZYX = ((2) | (1 << 2) | (0 << 4));

    [DllImport("ode")]
    public static extern dSpaceID dSweepAndPruneSpaceCreate(dSpaceID space, int axisorder);



    [DllImport("ode")]
    public static extern void dSpaceDestroy(dSpaceID space);

    [DllImport("ode")]
    public static extern void dHashSpaceSetLevels(dSpaceID space, int minlevel, int maxlevel);
    [DllImport("ode")]
    public static extern void dHashSpaceGetLevels(dSpaceID space, ref int minlevel, ref int maxlevel);

    [DllImport("ode")]
    public static extern void dSpaceSetCleanup(dSpaceID space, int mode);
    [DllImport("ode")]
    public static extern int dSpaceGetCleanup(dSpaceID space);

    /**
    * @brief Sets sublevel value for a space.
    *
    * Sublevel affects how the space is handled in dSpaceCollide2 when it is collided
    * with another space. If sublevels of both spaces match, the function iterates 
    * geometries of both spaces and collides them with each other. If sublevel of one
    * space is greater than the sublevel of another one, only the geometries of the 
    * space with greater sublevel are iterated, another space is passed into 
    * collision callback as a geometry itself. By default all the spaces are assigned
    * zero sublevel.
    *
    * @note
    * The space sublevel @e IS @e NOT automatically updated when one space is inserted
    * into another or removed from one. It is a client's responsibility to update sublevel
    * value if necessary.
    *
    * @param space the space to modify
    * @param sublevel the sublevel value to be assigned
    * @ingroup collide
    * @see dSpaceGetSublevel
    * @see dSpaceCollide2
*/
    [DllImport("ode")]
    public static extern void dSpaceSetSublevel(dSpaceID space, int sublevel);

    /**
    * @brief Gets sublevel value of a space.
    *
    * Sublevel affects how the space is handled in dSpaceCollide2 when it is collided
    * with another space. See @c dSpaceSetSublevel for more details.
    *
    * @param space the space to query
    * @returns the sublevel value of the space
    * @ingroup collide
    * @see dSpaceSetSublevel
    * @see dSpaceCollide2
*/
    [DllImport("ode")]
    public static extern int dSpaceGetSublevel(dSpaceID space);


    /**
    * @brief Sets manual cleanup flag for a space.
    *
    * Manual cleanup flag marks a space as eligible for manual thread data cleanup.
    * This function should be called for every space object right after creation in 
    * case if ODE has been initialized with @c dInitFlagManualThreadCleanup flag.
    * 
    * Failure to set manual cleanup flag for a space may lead to some resources 
    * remaining leaked until the program exit.
    *
    * @param space the space to modify
    * @param mode 1 for manual cleanup mode and 0 for default cleanup mode
    * @ingroup collide
    * @see dSpaceGetManualCleanup
    * @see dInitODE2
*/
    [DllImport("ode")]
    public static extern void dSpaceSetManualCleanup(dSpaceID space, int mode);

    /**
    * @brief Get manual cleanup flag of a space.
    *
    * Manual cleanup flag marks a space space as eligible for manual thread data cleanup.
    * See @c dSpaceSetManualCleanup for more details.
    * 
    * @param space the space to query
    * @returns 1 for manual cleanup mode and 0 for default cleanup mode of the space
    * @ingroup collide
    * @see dSpaceSetManualCleanup
    * @see dInitODE2
*/
    [DllImport("ode")]
    public static extern int dSpaceGetManualCleanup(dSpaceID space);

    [DllImport("ode")]
    public static extern void dSpaceAdd(dSpaceID space, dGeomID geom);
    [DllImport("ode")]
    public static extern void dSpaceRemove(dSpaceID space, dGeomID geom);
    [DllImport("ode")]
    public static extern int dSpaceQuery(dSpaceID space, dGeomID geom);
    [DllImport("ode")]
    public static extern void dSpaceClean(dSpaceID space);
    [DllImport("ode")]
    public static extern int dSpaceGetNumGeoms(dSpaceID space);
    [DllImport("ode")]
    public static extern dGeomID dSpaceGetGeom(dSpaceID space, int i);

    /**
     * @brief Given a space, this returns its class.
     *
     * The ODE classes are:
     *  @li dSimpleSpaceClass
     *  @li dHashSpaceClass
     *  @li dSweepAndPruneSpaceClass
     *  @li dQuadTreeSpaceClass
     *  @li dFirstUserClass
     *  @li dLastUserClass
     *
     * The class id not defined by the user should be between
     * dFirstSpaceClass and dLastSpaceClass.
     *
     * User-defined class will return their own number.
     *
     * @param space the space to query
     * @returns The space class ID.
     * @ingroup collide
     */
    [DllImport("ode")]
    public static extern int dSpaceGetClass(dSpaceID space);

    #endregion
    #region objects.h
    public const float dWORLDSTEP_RESERVEFACTOR_DEFAULT = 1.2f;
    public const uint dWORLDSTEP_RESERVESIZE_DEFAULT = 65536;

    public enum dJointType
    {
        dJointTypeNone = 0,     /* or "unknown" */
        dJointTypeBall,
        dJointTypeHinge,
        dJointTypeSlider,
        dJointTypeContact,
        dJointTypeUniversal,
        dJointTypeHinge2,
        dJointTypeFixed,
        dJointTypeNull,
        dJointTypeAMotor,
        dJointTypeLMotor,
        dJointTypePlane2D,
        dJointTypePR,
        dJointTypePU,
        dJointTypePiston,
        dJointTypeDBall,
        dJointTypeDHinge,
        dJointTypeTransmission,
    }

    /**
     * @defgroup world World
     *
     * The world object is a container for rigid bodies and joints. Objects in
     * different worlds can not interact, for example rigid bodies from two
     * different worlds can not collide.
     *
     * All the objects in a world exist at the same point in time, thus one
     * reason to use separate worlds is to simulate systems at different rates.
     * Most applications will only need one world.
     */

    /**
     * @brief Create a new, empty world and return its ID number.
     * @return an identifier
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern dWorldID dWorldCreate();


    /**
     * @brief Destroy a world and everything in it.
     *
     * This includes all bodies, and all joints that are not part of a joint
     * group. Joints that are part of a joint group will be deactivated, and
     * can be destroyed by calling, for example, dJointGroupEmpty().
     * @ingroup world
     * @param world the identifier for the world the be destroyed.
     */
    [DllImport("ode")]
    public static extern void dWorldDestroy(dWorldID world);


    /**
     * @brief Set the user-data pointer
     * @param world the world to set the data on
     * @param data
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern void dWorldSetData(dWorldID world, IntPtr data);


    /**
     * @brief Get the user-data pointer
     * @param world the world to set the data on
     * @param data
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern IntPtr dWorldGetData(dWorldID world);


    /**
     * @brief Set the world's global gravity vector.
     *
     * The units are m/s^2, so Earth's gravity vector would be (0,0,-9.81),
     * assuming that +z is up. The default is no gravity, i.e. (0,0,0).
     *
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern void dWorldSetGravity(dWorldID world, dReal x, dReal y, dReal z);


    /**
     * @brief Get the gravity vector for a given world.
     * @ingroup world
     */
    [DllImport("ode", EntryPoint = "dWorldGetGravity")]
    static extern void _dWorldGetGravity(dWorldID world, out dVector3 gravity);
    public static Vector3 dWorldGetGravity(dWorldID world)
    {
        dVector3 gravity;
        _dWorldGetGravity(world, out gravity);
        return gravity;
    }


    /**
     * @brief Set the global ERP value, that controls how much error
     * correction is performed in each time step.
     * @ingroup world
     * @param dWorldID the identifier of the world.
     * @param erp Typical values are in the range 0.1--0.8. The default is 0.2.
     */
    [DllImport("ode")]
    public static extern void dWorldSetERP(dWorldID world, dReal erp);

    /**
     * @brief Get the error reduction parameter.
     * @ingroup world
     * @return ERP value
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetERP(dWorldID world);


    /**
     * @brief Set the global CFM (constraint force mixing) value.
     * @ingroup world
     * @param cfm Typical values are in the range @m{10^{-9}} -- 1.
     * The default is 10^-5 if single precision is being used, or 10^-10
     * if double precision is being used.
     */
    [DllImport("ode")]
    public static extern void dWorldSetCFM(dWorldID world, dReal cfm);

    /**
     * @brief Get the constraint force mixing value.
     * @ingroup world
     * @return CFM value
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetCFM(dWorldID world);


    /**
     * @brief Set maximum threads to be used for island stepping
     *
     * The actual number of threads that is going to be used will be the minimum
     * of this limit and number of threads in the threading pool. By default 
     * there is no limit (@c dWORLDSTEP_THREADCOUNT_UNLIMITED).
     *
     * @warning
     * WARNING! Running island stepping in multiple threads requires allocating 
     * individual stepping memory buffer for each of those threads. The size of buffers
     * allocated is the size needed to handle the largest island in the world.
     *
     * Note: Setting a limit for island stepping does not affect threading at lower
     * levels in stepper functions. The sub-calls scheduled from them can be executed
     * in as many threads as there are available in the pool.
     *
     * @param w The world affected
     * @param count Thread count limit value for island stepping
     * @ingroup world
     * @see dWorldGetStepIslandsProcessingMaxThreadCount
     */
    [DllImport("ode")]
    public static extern void dWorldSetStepIslandsProcessingMaxThreadCount(dWorldID w, uint count);
    /**
     * @brief Get maximum threads that are allowed to be used for island stepping.
     *
     * Please read commentaries to @c dWorldSetStepIslandsProcessingMaxThreadCount for 
     * important information regarding the value returned.
     *
     * @param w The world queried
     * @returns Current thread count limit value for island stepping
     * @ingroup world
     * @see dWorldSetStepIslandsProcessingMaxThreadCount
     */
    [DllImport("ode")]
    public static extern uint dWorldGetStepIslandsProcessingMaxThreadCount(dWorldID w);

    /**
     * @brief Set the world to use shared working memory along with another world.
     *
     * The worlds allocate working memory internally for simulation stepping. This
     * memory is cached among the calls to @c dWordStep and @c dWorldQuickStep. 
     * Similarly, several worlds can be set up to share this memory caches thus 
     * reducing overall memory usage by cost of making worlds inappropriate for 
     * simultaneous simulation in multiple threads.
     *
     * If null value is passed for @a from_world parameter the world is detached from 
     * sharing and returns to defaults for working memory, reservation policy and 
     * memory manager as if just created. This can also be used to enable use of shared 
     * memory for a world that has already had working memory allocated privately.
     * Normally using shared memory after a world has its private working memory allocated
     * is prohibited.
     *
     * Allocation policy used can only increase world's internal reserved memory size
     * and never decreases it. @c dWorldCleanupWorkingMemory can be used to release 
     * working memory for a world in case if number of objects/joint decreases 
     * significantly in it.
     *
     * With sharing working memory worlds also automatically share memory reservation 
     * policy and memory manager. Thus, these parameters need to be customized for
     * initial world to be used as sharing source only.
     *
     * If worlds share working memory they must also use compatible threading implementations
     * (i.e. it is illegal for one world to perform stepping with self-threaded implementation
     * when the other world is assigned a multi-threaded implementation). 
     * For more information read section about threading approaches in ODE.
     *
     * Failure result status means a memory allocation failure.
     *
     * @param w The world to use the shared memory with.
     * @param from_world Null or the world the shared memory is to be used from.
     * @returns 1 for success and 0 for failure.
     *
     * @ingroup world
     * @see dWorldCleanupWorkingMemory
     * @see dWorldSetStepMemoryReservationPolicy
     * @see dWorldSetStepMemoryManager
     */
    [DllImport("ode")]
    public static extern int dWorldUseSharedWorkingMemory(dWorldID w, dWorldID from_world/*=NULL*/);

    /**
     * @brief Release internal working memory allocated for world
     *
     * The worlds allocate working memory internally for simulation stepping. This 
     * function can be used to free world's internal memory cache in case if number of
     * objects/joints in the world decreases significantly. By default, internal 
     * allocation policy is used to only increase cache size as necessary and never 
     * decrease it.
     *
     * If a world shares its working memory with other worlds the cache deletion 
     * affects all the linked worlds. However the shared status itself remains intact.
     *
     * The function call does affect neither memory reservation policy nor memory manager.
     *
     * @param w The world to release working memory for.
     *
     * @ingroup world
     * @see dWorldUseSharedWorkingMemory
     * @see dWorldSetStepMemoryReservationPolicy
     * @see dWorldSetStepMemoryManager
     */
    [DllImport("ode")]
    public static extern void dWorldCleanupWorkingMemory(dWorldID w);




    /**
     * @struct dWorldStepReserveInfo
     * @brief Memory reservation policy descriptor structure for world stepping functions.
     *
     * @c struct_size should be assigned the size of the structure.
     *
     * @c reserve_factor is a quotient that is multiplied by required memory size
     *  to allocate extra reserve whenever reallocation is needed.
     *
     * @c reserve_minimum is a minimum size that is checked against whenever reallocation 
     * is needed to allocate expected working memory minimum at once without extra 
     * reallocations as number of bodies/joints grows.
     *
     * @ingroup world
     * @see dWorldSetStepMemoryReservationPolicy
     */
    public struct dWorldStepReserveInfo
    {
        uint struct_size;
        float reserve_factor; /* Use float as precision does not matter here*/
        uint reserve_minimum;
    }

    /**
     * @brief Set memory reservation policy for world to be used with simulation stepping functions
     *
     * The function allows to customize reservation policy to be used for internal
     * memory which is allocated to aid simulation for a world. By default, values
     * of @c dWORLDSTEP_RESERVEFACTOR_DEFAULT and @c dWORLDSTEP_RESERVESIZE_DEFAULT
     * are used.
     *
     * Passing @a policyinfo argument as NULL results in reservation policy being
     * reset to defaults as if the world has been just created. The content of 
     * @a policyinfo structure is copied internally and does not need to remain valid
     * after the call returns.
     *
     * If the world uses working memory sharing, changing memory reservation policy
     * affects all the worlds linked together.
     *
     * Failure result status means a memory allocation failure.
     *
     * @param w The world to change memory reservation policy for.
     * @param policyinfo Null or a pointer to policy descriptor structure.
     * @returns 1 for success and 0 for failure.
     *
     * @ingroup world
     * @see dWorldUseSharedWorkingMemory
     */
    [DllImport("ode")]
    public static extern int dWorldSetStepMemoryReservationPolicy(dWorldID w, ref dWorldStepReserveInfo policyinfo/*=NULL*/);

    /**
    * @struct dWorldStepMemoryFunctionsInfo
    * @brief World stepping memory manager descriptor structure
    *
    * This structure is intended to define the functions of memory manager to be used
    * with world stepping functions.
    *
    * @c struct_size should be assigned the size of the structure
    *
    * @c alloc_block is a function to allocate memory block of given size.
    *
    * @c shrink_block is a function to shrink existing memory block to a smaller size.
    * It must preserve the contents of block head while shrinking. The new block size
    * is guaranteed to be always less than the existing one.
    *
    * @c free_block is a function to delete existing memory block.
    *
    * @ingroup init
    * @see dWorldSetStepMemoryManager
*/
    //public struct dWorldStepMemoryFunctionsInfo
    //    {
    //  int struct_size;
    //IntPtr(* alloc_block)(size_t block_size);
    //  IntPtr(* shrink_block)(IntPtr block_pointer, size_t block_current_size, size_t block_smaller_size);
    //  void (* free_block)(IntPtr block_pointer, size_t block_current_size);
    //
    //} ;

    /**
    * @brief Set memory manager for world to be used with simulation stepping functions
    *
    * The function allows to customize memory manager to be used for internal
    * memory allocation during simulation for a world. By default, @c dAlloc/@c dRealloc/@c dFree
    * based memory manager is used.
    *
    * Passing @a memfuncs argument as NULL results in memory manager being
    * reset to default one as if the world has been just created. The content of 
    * @a memfuncs structure is copied internally and does not need to remain valid
    * after the call returns.
    *
    * If the world uses working memory sharing, changing memory manager
    * affects all the worlds linked together. 
    *
    * Failure result status means a memory allocation failure.
    *
    * @param w The world to change memory reservation policy for.
    * @param memfuncs Null or a pointer to memory manager descriptor structure.
    * @returns 1 for success and 0 for failure.
    *
    * @ingroup world
    * @see dWorldUseSharedWorkingMemory
*/
    //[DllImport("ode")] public static extern int dWorldSetStepMemoryManager(dWorldID w, const dWorldStepMemoryFunctionsInfo* memfuncs);

    /**
     * @brief Assign threading implementation to be used for [quick]stepping the world.
     *
     * @warning It is not recommended to assign the same threading implementation to
     * different worlds if they are going to be called in parallel. In particular this
     * makes resources preallocation for threaded calls to lose its sense. 
     * Built-in threading implementation is likely to crash if misused this way.
     * 
     * @param w The world to change threading implementation for.
     * @param functions_info Pointer to threading functions structure
     * @param threading_impl ID of threading implementation object
     * 
     * @ingroup world
     */
    //[DllImport("ode")] public static extern void dWorldSetStepThreadingImplementation(dWorldID w, const dThreadingFunctionsInfo* functions_info, dThreadingImplementationID threading_impl);

    /**
     * @brief Step the world.
     *
     * This uses a "big matrix" method that takes time on the order of m^3
     * and memory on the order of m^2, where m is the total number of constraint
     * rows. For large systems this will use a lot of memory and can be very slow,
     * but this is currently the most accurate method.
     *
     * Failure result status means that the memory allocation has failed for operation.
     * In such a case all the objects remain in unchanged state and simulation can be
     * retried as soon as more memory is available.
     *
     * @param w The world to be stepped
     * @param stepsize The number of seconds that the simulation has to advance.
     * @returns 1 for success and 0 for failure
     *
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern int dWorldStep(dWorldID w, dReal stepsize);

    /**
     * @brief Quick-step the world.
     *
     * This uses an iterative method that takes time on the order of m*N
     * and memory on the order of m, where m is the total number of constraint
     * rows N is the number of iterations.
     * For large systems this is a lot faster than dWorldStep(),
     * but it is less accurate.
     *
     * QuickStep is great for stacks of objects especially when the
     * auto-disable feature is used as well.
     * However, it has poor accuracy for near-singular systems.
     * Near-singular systems can occur when using high-friction contacts, motors,
     * or certain articulated structures. For example, a robot with multiple legs
     * sitting on the ground may be near-singular.
     *
     * There are ways to help overcome QuickStep's inaccuracy problems:
     *
     * \li Increase CFM.
     * \li Reduce the number of contacts in your system (e.g. use the minimum
     *     number of contacts for the feet of a robot or creature).
     * \li Don't use excessive friction in the contacts.
     * \li Use contact slip if appropriate
     * \li Avoid kinematic loops (however, kinematic loops are inevitable in
     *     legged creatures).
     * \li Don't use excessive motor strength.
     * \liUse force-based motors instead of velocity-based motors.
     *
     * Increasing the number of QuickStep iterations may help a little bit, but
     * it is not going to help much if your system is really near singular.
     *
     * Failure result status means that the memory allocation has failed for operation.
     * In such a case all the objects remain in unchanged state and simulation can be
     * retried as soon as more memory is available.
     *
     * @param w The world to be stepped
     * @param stepsize The number of seconds that the simulation has to advance.
     * @returns 1 for success and 0 for failure
     *
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern int dWorldQuickStep(dWorldID w, dReal stepsize);


    /**
    * @brief Converts an impulse to a force.
    * @ingroup world
    * @remarks
    * If you want to apply a linear or angular impulse to a rigid body,
    * instead of a force or a torque, then you can use this function to convert
    * the desired impulse into a force/torque vector before calling the
    * BodyAdd... function.
    * The current algorithm simply scales the impulse by 1/stepsize,
    * where stepsize is the step size for the next step that will be taken.
    * This function is given a dWorldID because, in the future, the force
    * computation may depend on integrator parameters that are set as
    * properties of the world.
*/
    [DllImport("ode")]
    public static extern void dWorldImpulseToForce
    (
     dWorldID world, dReal stepsize,
     dReal ix, dReal iy, dReal iz, dVector3 force
     );


    /**
     * @brief Set the number of iterations that the QuickStep method performs per
     *        step.
     * @ingroup world
     * @remarks
     * More iterations will give a more accurate solution, but will take
     * longer to compute.
     * @param num The default is 20 iterations.
     */
    [DllImport("ode")]
    public static extern void dWorldSetQuickStepNumIterations(dWorldID world, int num);


    /**
     * @brief Get the number of iterations that the QuickStep method performs per
     *        step.
     * @ingroup world
     * @return nr of iterations
     */
    [DllImport("ode")]
    public static extern int dWorldGetQuickStepNumIterations(dWorldID world);

    /**
     * @brief Set the SOR over-relaxation parameter
     * @ingroup world
     * @param over_relaxation value to use by SOR
     */
    [DllImport("ode")]
    public static extern void dWorldSetQuickStepW(dWorldID world, dReal over_relaxation);

    /**
     * @brief Get the SOR over-relaxation parameter
     * @ingroup world
     * @returns the over-relaxation setting
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetQuickStepW(dWorldID world);

    /* World contact parameter functions */

    /**
     * @brief Set the maximum correcting velocity that contacts are allowed
     * to generate.
     * @ingroup world
     * @param vel The default value is infinity (i.e. no limit).
     * @remarks
     * Reducing this value can help prevent "popping" of deeply embedded objects.
     */
    [DllImport("ode")]
    public static extern void dWorldSetContactMaxCorrectingVel(dWorldID world, dReal vel);

    /**
     * @brief Get the maximum correcting velocity that contacts are allowed
     * to generated.
     * @ingroup world
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetContactMaxCorrectingVel(dWorldID world);

    /**
     * @brief Set the depth of the surface layer around all geometry objects.
     * @ingroup world
     * @remarks
     * Contacts are allowed to sink into the surface layer up to the given
     * depth before coming to rest.
     * @param depth The default value is zero.
     * @remarks
     * Increasing this to some small value (e.g. 0.001) can help prevent
     * jittering problems due to contacts being repeatedly made and broken.
     */
    [DllImport("ode")]
    public static extern void dWorldSetContactSurfaceLayer(dWorldID world, dReal depth);

    /**
     * @brief Get the depth of the surface layer around all geometry objects.
     * @ingroup world
     * @returns the depth
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetContactSurfaceLayer(dWorldID world);


    /**
     * @defgroup disable Automatic Enabling and Disabling
     * @ingroup world bodies
     *
     * Every body can be enabled or disabled. Enabled bodies participate in the
     * simulation, while disabled bodies are turned off and do not get updated
     * during a simulation step. New bodies are always created in the enabled state.
     *
     * A disabled body that is connected through a joint to an enabled body will be
     * automatically re-enabled at the next simulation step.
     *
     * Disabled bodies do not consume CPU time, therefore to speed up the simulation
     * bodies should be disabled when they come to rest. This can be done automatically
     * with the auto-disable feature.
     *
     * If a body has its auto-disable flag turned on, it will automatically disable
     * itself when
     *   @li It has been idle for a given number of simulation steps.
     *   @li It has also been idle for a given amount of simulation time.
     *
     * A body is considered to be idle when the magnitudes of both its
     * linear average velocity and angular average velocity are below given thresholds.
     * The sample size for the average defaults to one and can be disabled by setting
     * to zero with 
     *
     * Thus, every body has six auto-disable parameters: an enabled flag, a idle step
     * count, an idle time, linear/angular average velocity thresholds, and the
     * average samples count.
     *
     * Newly created bodies get these parameters from world.
     */

    /**
     * @brief Get auto disable linear average threshold for newly created bodies.
     * @ingroup disable
     * @return the threshold
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetAutoDisableLinearThreshold(dWorldID world);

    /**
     * @brief Set auto disable linear average threshold for newly created bodies.
     * @param linear_average_threshold default is 0.01
     * @ingroup disable
     */
    [DllImport("ode")]
    public static extern void dWorldSetAutoDisableLinearThreshold(dWorldID world, dReal linear_average_threshold);

    /**
     * @brief Get auto disable angular average threshold for newly created bodies.
     * @ingroup disable
     * @return the threshold
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetAutoDisableAngularThreshold(dWorldID world);

    /**
     * @brief Set auto disable angular average threshold for newly created bodies.
     * @param linear_average_threshold default is 0.01
     * @ingroup disable
     */
    [DllImport("ode")]
    public static extern void dWorldSetAutoDisableAngularThreshold(dWorldID world, dReal angular_average_threshold);

    /**
     * @brief Get auto disable sample count for newly created bodies.
     * @ingroup disable
     * @return number of samples used
     */
    [DllImport("ode")]
    public static extern int dWorldGetAutoDisableAverageSamplesCount(dWorldID world);

    /**
     * @brief Set auto disable average sample count for newly created bodies.
     * @ingroup disable
     * @param average_samples_count Default is 1, meaning only instantaneous velocity is used.
     * Set to zero to disable sampling and thus prevent any body from auto-disabling.
     */
    [DllImport("ode")]
    public static extern void dWorldSetAutoDisableAverageSamplesCount(dWorldID world, uint average_samples_count);

    /**
     * @brief Get auto disable steps for newly created bodies.
     * @ingroup disable
     * @return nr of steps
     */
    [DllImport("ode")]
    public static extern int dWorldGetAutoDisableSteps(dWorldID world);

    /**
     * @brief Set auto disable steps for newly created bodies.
     * @ingroup disable
     * @param steps default is 10
     */
    [DllImport("ode")]
    public static extern void dWorldSetAutoDisableSteps(dWorldID world, int steps);

    /**
     * @brief Get auto disable time for newly created bodies.
     * @ingroup disable
     * @return nr of seconds
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetAutoDisableTime(dWorldID world);

    /**
     * @brief Set auto disable time for newly created bodies.
     * @ingroup disable
     * @param time default is 0 seconds
     */
    [DllImport("ode")]
    public static extern void dWorldSetAutoDisableTime(dWorldID world, dReal time);

    /**
     * @brief Get auto disable flag for newly created bodies.
     * @ingroup disable
     * @return 0 or 1
     */
    [DllImport("ode")]
    public static extern int dWorldGetAutoDisableFlag(dWorldID world);

    /**
     * @brief Set auto disable flag for newly created bodies.
     * @ingroup disable
     * @param do_auto_disable default is false.
     */
    [DllImport("ode")]
    public static extern void dWorldSetAutoDisableFlag(dWorldID world, int do_auto_disable);


    /**
     * @defgroup damping Damping
     * @ingroup bodies world
     *
     * Damping serves two purposes: reduce simulation instability, and to allow
     * the bodies to come to rest (and possibly auto-disabling them).
     *
     * Bodies are constructed using the world's current damping parameters. Setting
     * the scales to 0 disables the damping.
     *
     * Here is how it is done: after every time step linear and angular
     * velocities are tested against the corresponding thresholds. If they
     * are above, they are multiplied by (1 - scale). So a negative scale value
     * will actually increase the speed, and values greater than one will
     * make the object oscillate every step; both can make the simulation unstable.
     *
     * To disable damping just set the damping scale to zero.
     *
     * You can also limit the maximum angular velocity. In contrast to the damping
     * functions, the angular velocity is affected before the body is moved.
     * This means that it will introduce errors in joints that are forcing the body
     * to rotate too fast. Some bodies have naturally high angular velocities
     * (like cars' wheels), so you may want to give them a very high (like the default,
     * dInfinity) limit.
     *
     * @note The velocities are damped after the stepper function has moved the
     * object. Otherwise the damping could introduce errors in joints. First the
     * joint constraints are processed by the stepper (moving the body), then
     * the damping is applied.
     *
     * @note The damping happens right after the moved callback is called; this way 
     * it still possible use the exact velocities the body has acquired during the
     * step. You can even use the callback to create your own customized damping.
     */

    /**
     * @brief Get the world's linear damping threshold.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetLinearDampingThreshold(dWorldID w);

    /**
     * @brief Set the world's linear damping threshold.
     * @param threshold The damping won't be applied if the linear speed is
     *        below this threshold. Default is 0.01.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern void dWorldSetLinearDampingThreshold(dWorldID w, dReal threshold);

    /**
     * @brief Get the world's angular damping threshold.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetAngularDampingThreshold(dWorldID w);

    /**
     * @brief Set the world's angular damping threshold.
     * @param threshold The damping won't be applied if the angular speed is
     *        below this threshold. Default is 0.01.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern void dWorldSetAngularDampingThreshold(dWorldID w, dReal threshold);

    /**
     * @brief Get the world's linear damping scale.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetLinearDamping(dWorldID w);

    /**
     * @brief Set the world's linear damping scale.
     * @param scale The linear damping scale that is to be applied to bodies.
     * Default is 0 (no damping). Should be in the interval [0, 1].
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern void dWorldSetLinearDamping(dWorldID w, dReal scale);

    /**
     * @brief Get the world's angular damping scale.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetAngularDamping(dWorldID w);

    /**
     * @brief Set the world's angular damping scale.
     * @param scale The angular damping scale that is to be applied to bodies.
     * Default is 0 (no damping). Should be in the interval [0, 1].
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern void dWorldSetAngularDamping(dWorldID w, dReal scale);

    /**
     * @brief Convenience function to set body linear and angular scales.
     * @param linear_scale The linear damping scale that is to be applied to bodies.
     * @param angular_scale The angular damping scale that is to be applied to bodies.
     * @ingroup damping
     */
    [DllImport("ode")]
    public static extern void dWorldSetDamping(dWorldID w,
                                    dReal linear_scale,
                                    dReal angular_scale);

    /**
     * @brief Get the default maximum angular speed.
     * @ingroup damping
     * @sa dBodyGetMaxAngularSpeed()
     */
    [DllImport("ode")]
    public static extern dReal dWorldGetMaxAngularSpeed(dWorldID w);


    /**
     * @brief Set the default maximum angular speed for new bodies.
     * @ingroup damping
     * @sa dBodySetMaxAngularSpeed()
     */
    [DllImport("ode")]
    public static extern void dWorldSetMaxAngularSpeed(dWorldID w, dReal max_speed);



    /**
     * @defgroup bodies Rigid Bodies
     *
     * A rigid body has various properties from the point of view of the
     * simulation. Some properties change over time:
     *
     *  @li Position vector (x,y,z) of the body's point of reference.
     *      Currently the point of reference must correspond to the body's center of mass.
     *  @li Linear velocity of the point of reference, a vector (vx,vy,vz).
     *  @li Orientation of a body, represented by a quaternion (qs,qx,qy,qz) or
     *      a 3x3 rotation matrix.
     *  @li Angular velocity vector (wx,wy,wz) which describes how the orientation
     *      changes over time.
     *
     * Other body properties are usually constant over time:
     *
     *  @li Mass of the body.
     *  @li Position of the center of mass with respect to the point of reference.
     *      In the current implementation the center of mass and the point of
     *      reference must coincide.
     *  @li Inertia matrix. This is a 3x3 matrix that describes how the body's mass
     *      is distributed around the center of mass. Conceptually each body has an
     *      x-y-z coordinate frame embedded in it that moves and rotates with the body.
     *
     * The origin of this coordinate frame is the body's point of reference. Some values
     * in ODE (vectors, matrices etc) are relative to the body coordinate frame, and others
     * are relative to the global coordinate frame.
     *
     * Note that the shape of a rigid body is not a dynamical property (except insofar as
     * it influences the various mass properties). It is only collision detection that cares
     * about the detailed shape of the body.
     */


    /**
     * @brief Get auto disable linear average threshold.
     * @ingroup bodies disable
     * @return the threshold
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetAutoDisableLinearThreshold(dBodyID body);

    /**
     * @brief Set auto disable linear average threshold.
     * @ingroup bodies disable
     * @return the threshold
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableLinearThreshold(dBodyID body, dReal linear_average_threshold);

    /**
     * @brief Get auto disable angular average threshold.
     * @ingroup bodies disable
     * @return the threshold
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetAutoDisableAngularThreshold(dBodyID body);

    /**
     * @brief Set auto disable angular average threshold.
     * @ingroup bodies disable
     * @return the threshold
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableAngularThreshold(dBodyID body, dReal angular_average_threshold);

    /**
     * @brief Get auto disable average size (samples count).
     * @ingroup bodies disable
     * @return the nr of steps/size.
     */
    [DllImport("ode")]
    public static extern int dBodyGetAutoDisableAverageSamplesCount(dBodyID body);

    /**
     * @brief Set auto disable average buffer size (average steps).
     * @ingroup bodies disable
     * @param average_samples_count the nr of samples to review.
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableAverageSamplesCount(dBodyID body, uint average_samples_count);


    /**
     * @brief Get auto steps a body must be thought of as idle to disable
     * @ingroup bodies disable
     * @return the nr of steps
     */
    [DllImport("ode")]
    public static extern int dBodyGetAutoDisableSteps(dBodyID body);

    /**
     * @brief Set auto disable steps.
     * @ingroup bodies disable
     * @param steps the nr of steps.
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableSteps(dBodyID body, int steps);

    /**
     * @brief Get auto disable time.
     * @ingroup bodies disable
     * @return nr of seconds
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetAutoDisableTime(dBodyID body);

    /**
     * @brief Set auto disable time.
     * @ingroup bodies disable
     * @param time nr of seconds.
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableTime(dBodyID body, dReal time);

    /**
     * @brief Get auto disable flag.
     * @ingroup bodies disable
     * @return 0 or 1
     */
    [DllImport("ode")]
    public static extern int dBodyGetAutoDisableFlag(dBodyID body);

    /**
     * @brief Set auto disable flag.
     * @ingroup bodies disable
     * @param do_auto_disable 0 or 1
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableFlag(dBodyID body, int do_auto_disable);

    /**
     * @brief Set auto disable defaults.
     * @remarks
     * Set the values for the body to those set as default for the world.
     * @ingroup bodies disable
     */
    [DllImport("ode")]
    public static extern void dBodySetAutoDisableDefaults(dBodyID body);


    /**
     * @brief Retrieves the world attached to te given body.
     * @remarks
     * 
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern dWorldID dBodyGetWorld(dBodyID body);

    /**
     * @brief Create a body in given world.
     * @remarks
     * Default mass parameters are at position (0,0,0).
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern dBodyID dBodyCreate(dWorldID world);

    /**
     * @brief Destroy a body.
     * @remarks
     * All joints that are attached to this body will be put into limbo:
     * i.e. unattached and not affecting the simulation, but they will NOT be
     * deleted.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyDestroy(dBodyID body);

    /**
     * @brief Set the body's user-data pointer.
     * @ingroup bodies
     * @param data arbitraty pointer
     */
    [DllImport("ode")]
    public static extern void dBodySetData(dBodyID body, IntPtr data);

    /**
     * @brief Get the body's user-data pointer.
     * @ingroup bodies
     * @return a pointer to the user's data.
     */
    [DllImport("ode")]
    public static extern IntPtr dBodyGetData(dBodyID body);

    /**
     * @brief Set position of a body.
     * @remarks
     * After setting, the outcome of the simulation is undefined
     * if the new configuration is inconsistent with the joints/constraints
     * that are present.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetPosition(dBodyID body, dReal x, dReal y, dReal z);

    /**
     * @brief Set the orientation of a body.
     * @ingroup bodies
     * @remarks
     * After setting, the outcome of the simulation is undefined
     * if the new configuration is inconsistent with the joints/constraints
     * that are present.
     */
    [DllImport("ode")]
    public static extern void dBodySetRotation(dBodyID body, dMatrix3 R);

    /**
     * @brief Set the orientation of a body.
     * @ingroup bodies
     * @remarks
     * After setting, the outcome of the simulation is undefined
     * if the new configuration is inconsistent with the joints/constraints
     * that are present.
     */
    [DllImport("ode")]
    public static extern void dBodySetQuaternion(dBodyID body, dQuaternion q);

    /**
     * @brief Set the linear velocity of a body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetLinearVel(dBodyID body, dReal x, dReal y, dReal z);

    /**
     * @brief Set the angular velocity of a body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetAngularVel(dBodyID body, dReal x, dReal y, dReal z);


    private static dVector3 ToVector3(IntPtr p)
    {
        //dReal[] array = new dReal[3];
        //Marshal.Copy(p, array, 0, 3);
        //return new dVector3(array[0], array[1], array[2]);
        dVector3 vec;
        ExtCopyFloatArrayToVector3(p, out vec, 3);
        return vec;
    }
    private static dQuaternion ToQuaternion(IntPtr p)
    {
        //dReal[] array = new dReal[4];
        //Marshal.Copy(p, array, 0, 4);
        //return new dQuaternion(array[1], array[2], array[3], array[0]);
        dQuaternion rot;
        ExtCopyFloatArrayToQuaternion(p, out rot, 4);
        return rot;
    }
    private static dMatrix3 ToMatrix3(IntPtr p)
    {
        dReal[] array = new dReal[12];
        Marshal.Copy(p, array, 0, 12);
        return new dMatrix3(array);
    }
    /**
     * @brief Get the position of a body.
     * @ingroup bodies
     * @remarks
     * When getting, the returned values are pointers to internal data structures,
     * so the vectors are valid until any changes are made to the rigid body
     * system structure.
     * @sa dBodyCopyPosition
     */
    [DllImport("ode", EntryPoint = "dBodyGetPosition")]
    static extern IntPtr _dBodyGetPosition(dBodyID body);
    public static dVector3 dBodyGetPosition(dBodyID body)
    {
        return ToVector3(_dBodyGetPosition(body));
    }

    /**
     * @brief Copy the position of a body into a vector.
     * @ingroup bodies
     * @param body  the body to query
     * @param pos   a copy of the body position
     * @sa dBodyGetPosition
     */
    [DllImport("ode")]
    public static extern void dBodyCopyPosition(dBodyID body, dVector3 pos);


    /**
     * @brief Get the rotation of a body.
     * @ingroup bodies
     * @return pointer to a 4x3 rotation matrix.
     */
    [DllImport("ode", EntryPoint = "dBodyGetRotation")]
    static extern IntPtr _dBodyGetRotation(dBodyID body);
    public static dMatrix3 dBodyGetRotation(dBodyID body)
    {
        return ToMatrix3(_dBodyGetRotation(body));
    }

    /**
     * @brief Copy the rotation of a body.
     * @ingroup bodies
     * @param body   the body to query
     * @param R      a copy of the rotation matrix
     * @sa dBodyGetRotation
     */
    [DllImport("ode")]
    public static extern void dBodyCopyRotation(dBodyID body, dMatrix3 R);


    /**
     * @brief Get the rotation of a body.
     * @ingroup bodies
     * @return pointer to 4 scalars that represent the quaternion.
     */
    [DllImport("ode", EntryPoint = "dBodyGetQuaternion")]
    static extern IntPtr _dBodyGetQuaternion(dBodyID body);
    public static dQuaternion dBodyGetQuaternion(dBodyID body)
    {
        return ToQuaternion(_dBodyGetQuaternion(body));
    }

    /**
     * @brief Copy the orientation of a body into a quaternion.
     * @ingroup bodies
     * @param body  the body to query
     * @param quat  a copy of the orientation quaternion
     * @sa dBodyGetQuaternion
     */
    [DllImport("ode")]
    public static extern void dBodyCopyQuaternion(dBodyID body, dQuaternion quat);


    /**
     * @brief Get the linear velocity of a body.
     * @ingroup bodies
     */
    [DllImport("ode", EntryPoint = "dBodyGetLinearVel")]
    static extern IntPtr _dBodyGetLinearVel(dBodyID body);
    public static dVector3 dBodyGetLinearVel(dBodyID body)
    {
        return ToVector3(_dBodyGetLinearVel(body));
    }

    /**
        * @brief Get the angular velocity of a body.
        * @ingroup bodies
        */
    [DllImport("ode", EntryPoint = "dBodyGetAngularVel")]
    static extern IntPtr _dBodyGetAngularVel(dBodyID body);
    public static dVector3 dBodyGetAngularVel(dBodyID body)
    {
        return ToVector3(_dBodyGetAngularVel(body));
    }
    /**
        * @brief Set the mass of a body.
        * @ingroup bodies
        */
    [DllImport("ode")]
    public static extern void dBodySetMass(dBodyID body, ref dMass mass);

    ///**
    // * @brief Get the mass of a body.
    // * @ingroup bodies
    // */
    //[DllImport("ode")] public static extern void dBodyGetMass(dBodyID body, ref dMass mass);

    /**
     * @brief Add force at centre of mass of body in absolute coordinates.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddForce(dBodyID body, dReal fx, dReal fy, dReal fz);

    /**
     * @brief Add torque at centre of mass of body in absolute coordinates.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddTorque(dBodyID body, dReal fx, dReal fy, dReal fz);

    /**
     * @brief Add force at centre of mass of body in coordinates relative to body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddRelForce(dBodyID body, dReal fx, dReal fy, dReal fz);

    /**
     * @brief Add torque at centre of mass of body in coordinates relative to body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddRelTorque(dBodyID body, dReal fx, dReal fy, dReal fz);

    /**
     * @brief Add force at specified point in body in global coordinates.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddForceAtPos(dBodyID body, dReal fx, dReal fy, dReal fz,
                                dReal px, dReal py, dReal pz);
    /**
     * @brief Add force at specified point in body in local coordinates.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddForceAtRelPos(dBodyID body, dReal fx, dReal fy, dReal fz,
                                dReal px, dReal py, dReal pz);
    /**
     * @brief Add force at specified point in body in global coordinates.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddRelForceAtPos(dBodyID body, dReal fx, dReal fy, dReal fz,
                                dReal px, dReal py, dReal pz);
    /**
     * @brief Add force at specified point in body in local coordinates.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyAddRelForceAtRelPos(dBodyID body, dReal fx, dReal fy, dReal fz,
                                dReal px, dReal py, dReal pz);

    /**
     * @brief Return the current accumulated force vector.
     * @return points to an array of 3 reals.
     * @remarks
     * The returned values are pointers to internal data structures, so
     * the vectors are only valid until any changes are made to the rigid
     * body system.
     * @ingroup bodies
     */
    [DllImport("ode", EntryPoint = "dBodyGetForce")]
    static extern IntPtr _dBodyGetForce(dBodyID body);
    public static dVector3 dBodyGetForce(dBodyID body)
    {
        return ToVector3(_dBodyGetForce(body));
    }

    /**
     * @brief Return the current accumulated torque vector.
     * @return points to an array of 3 reals.
     * @remarks
     * The returned values are pointers to internal data structures, so
     * the vectors are only valid until any changes are made to the rigid
     * body system.
     * @ingroup bodies
     */
    [DllImport("ode", EntryPoint = "dBodyGetTorque")]
    static extern IntPtr _dBodyGetTorque(dBodyID body);
    public static dVector3 dBodyGetTorque(dBodyID body)
    {
        return ToVector3(_dBodyGetTorque(body));
    }

    /**
     * @brief Set the body force accumulation vector.
     * @remarks
     * This is mostly useful to zero the force and torque for deactivated bodies
     * before they are reactivated, in the case where the force-adding functions
     * were called on them while they were deactivated.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetForce(dBodyID b, dReal x, dReal y, dReal z);

    /**
     * @brief Set the body torque accumulation vector.
     * @remarks
     * This is mostly useful to zero the force and torque for deactivated bodies
     * before they are reactivated, in the case where the force-adding functions
     * were called on them while they were deactivated.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetTorque(dBodyID b, dReal x, dReal y, dReal z);

    /**
     * @brief Get world position of a relative point on body.
     * @ingroup bodies
     * @param result will contain the result.
     */
    [DllImport("ode")]
    public static extern void dBodyGetRelPointPos
    (
      dBodyID body, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief Get velocity vector in global coords of a relative point on body.
     * @ingroup bodies
     * @param result will contain the result.
     */
    [DllImport("ode")]
    public static extern void dBodyGetRelPointVel
    (
      dBodyID body, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief Get velocity vector in global coords of a globally
     * specified point on a body.
     * @ingroup bodies
     * @param result will contain the result.
     */
    [DllImport("ode")]
    public static extern void dBodyGetPointVel
    (
      dBodyID body, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief takes a point in global coordinates and returns
     * the point's position in body-relative coordinates.
     * @remarks
     * This is the inverse of dBodyGetRelPointPos()
     * @ingroup bodies
     * @param result will contain the result.
     */
    [DllImport("ode")]
    public static extern void dBodyGetPosRelPoint
    (
      dBodyID body, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief Convert from local to world coordinates.
     * @ingroup bodies
     * @param result will contain the result.
     */
    [DllImport("ode")]
    public static extern void dBodyVectorToWorld
    (
      dBodyID body, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief Convert from world to local coordinates.
     * @ingroup bodies
     * @param result will contain the result.
     */
    [DllImport("ode")]
    public static extern void dBodyVectorFromWorld
    (
      dBodyID body, dReal px, dReal py, dReal pz,
      dVector3 result
    );

    /**
     * @brief controls the way a body's orientation is updated at each timestep.
     * @ingroup bodies
     * @param mode can be 0 or 1:
     * \li 0: An ``infinitesimal'' orientation update is used.
     * This is fast to compute, but it can occasionally cause inaccuracies
     * for bodies that are rotating at high speed, especially when those
     * bodies are joined to other bodies.
     * This is the default for every new body that is created.
     * \li 1: A ``finite'' orientation update is used.
     * This is more costly to compute, but will be more accurate for high
     * speed rotations.
     * @remarks
     * Note however that high speed rotations can result in many types of
     * error in a simulation, and the finite mode will only fix one of those
     * sources of error.
     */
    [DllImport("ode")]
    public static extern void dBodySetFiniteRotationMode(dBodyID body, int mode);

    /**
     * @brief sets the finite rotation axis for a body.
     * @ingroup bodies
     * @remarks
     * This is axis only has meaning when the finite rotation mode is set
     * If this axis is zero (0,0,0), full finite rotations are performed on
     * the body.
     * If this axis is nonzero, the body is rotated by performing a partial finite
     * rotation along the axis direction followed by an infinitesimal rotation
     * along an orthogonal direction.
     * @remarks
     * This can be useful to alleviate certain sources of error caused by quickly
     * spinning bodies. For example, if a car wheel is rotating at high speed
     * you can call this function with the wheel's hinge axis as the argument to
     * try and improve its behavior.
     */
    [DllImport("ode")]
    public static extern void dBodySetFiniteRotationAxis(dBodyID body, dReal x, dReal y, dReal z);

    /**
     * @brief Get the way a body's orientation is updated each timestep.
     * @ingroup bodies
     * @return the mode 0 (infitesimal) or 1 (finite).
     */
    [DllImport("ode")]
    public static extern int dBodyGetFiniteRotationMode(dBodyID body);

    /**
     * @brief Get the finite rotation axis.
     * @param result will contain the axis.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyGetFiniteRotationAxis(dBodyID body, dVector3 result);

    /**
     * @brief Get the number of joints that are attached to this body.
     * @ingroup bodies
     * @return nr of joints
     */
    [DllImport("ode")]
    public static extern int dBodyGetNumJoints(dBodyID b);

    /**
     * @brief Return a joint attached to this body, given by index.
     * @ingroup bodies
     * @param index valid range is  0 to n-1 where n is the value returned by
     * dBodyGetNumJoints().
     */
    [DllImport("ode")]
    public static extern dJointID dBodyGetJoint(dBodyID body, int index);




    /**
     * @brief Set rigid body to dynamic state (default).
     * @param dBodyID identification of body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetDynamic(dBodyID body);

    /**
     * @brief Set rigid body to kinematic state.
     * When in kinematic state the body isn't simulated as a dynamic
     * body (it's "unstoppable", doesn't respond to forces),
     * but can still affect dynamic bodies (e.g. in joints).
     * Kinematic bodies can be controlled by position and velocity.
     * @note A kinematic body has infinite mass. If you set its mass
     * to something else, it loses the kinematic state and behaves
     * as a normal dynamic body.
     * @param dBodyID identification of body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetKinematic(dBodyID body);

    /**
     * @brief Check wether a body is in kinematic state.
     * @ingroup bodies
     * @return 1 if a body is kinematic or 0 if it is dynamic.
     */
    [DllImport("ode")]
    public static extern int dBodyIsKinematic(dBodyID body);

    /**
     * @brief Manually enable a body.
     * @param dBodyID identification of body.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodyEnable(dBodyID body);

    /**
     * @brief Manually disable a body.
     * @ingroup bodies
     * @remarks
     * A disabled body that is connected through a joint to an enabled body will
     * be automatically re-enabled at the next simulation step.
     */
    [DllImport("ode")]
    public static extern void dBodyDisable(dBodyID body);

    /**
     * @brief Check wether a body is enabled.
     * @ingroup bodies
     * @return 1 if a body is currently enabled or 0 if it is disabled.
     */
    [DllImport("ode")]
    public static extern int dBodyIsEnabled(dBodyID body);

    /**
     * @brief Set whether the body is influenced by the world's gravity or not.
     * @ingroup bodies
     * @param mode when nonzero gravity affects this body.
     * @remarks
     * Newly created bodies are always influenced by the world's gravity.
     */
    [DllImport("ode")]
    public static extern void dBodySetGravityMode(dBodyID b, int mode);

    /**
     * @brief Get whether the body is influenced by the world's gravity or not.
     * @ingroup bodies
     * @return nonzero means gravity affects this body.
     */
    [DllImport("ode")]
    public static extern int dBodyGetGravityMode(dBodyID b);

    /**
     * @brief Set the 'moved' callback of a body.
     *
     * Whenever a body has its position or rotation changed during the
     * timestep, the callback will be called (with body as the argument).
     * Use it to know which body may need an update in an external
     * structure (like a 3D engine).
     *
     * @param b the body that needs to be watched.
     * @param callback the callback to be invoked when the body moves. Set to zero
     * to disable.
     * @ingroup bodies
     */
    public delegate void BodySetMovedCallback(dBodyID body);
    [DllImport("ode")]
    public static extern void dBodySetMovedCallback(dBodyID b, BodySetMovedCallback fun);


    /**
     * @brief Return the first geom associated with the body.
     * 
     * You can traverse through the geoms by repeatedly calling
     * dBodyGetNextGeom().
     *
     * @return the first geom attached to this body, or 0.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern dGeomID dBodyGetFirstGeom(dBodyID b);


    /**
     * @brief returns the next geom associated with the same body.
     * @param g a geom attached to some body.
     * @return the next geom attached to the same body, or 0.
     * @sa dBodyGetFirstGeom
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern dGeomID dBodyGetNextGeom(dGeomID g);


    /**
     * @brief Resets the damping settings to the current world's settings.
     * @ingroup bodies damping
     */
    [DllImport("ode")]
    public static extern void dBodySetDampingDefaults(dBodyID b);

    /**
     * @brief Get the body's linear damping scale.
     * @ingroup bodies damping
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetLinearDamping(dBodyID b);

    /**
     * @brief Set the body's linear damping scale.
     * @param scale The linear damping scale. Should be in the interval [0, 1].
     * @ingroup bodies damping
     * @remarks From now on the body will not use the world's linear damping
     * scale until dBodySetDampingDefaults() is called.
     * @sa dBodySetDampingDefaults()
     */
    [DllImport("ode")]
    public static extern void dBodySetLinearDamping(dBodyID b, dReal scale);

    /**
     * @brief Get the body's angular damping scale.
     * @ingroup bodies damping
     * @remarks If the body's angular damping scale was not set, this function
     * returns the world's angular damping scale.
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetAngularDamping(dBodyID b);

    /**
     * @brief Set the body's angular damping scale.
     * @param scale The angular damping scale. Should be in the interval [0, 1].
     * @ingroup bodies damping
     * @remarks From now on the body will not use the world's angular damping
     * scale until dBodyResetAngularDamping() is called.
     * @sa dBodyResetAngularDamping()
     */
    [DllImport("ode")]
    public static extern void dBodySetAngularDamping(dBodyID b, dReal scale);

    /**
     * @brief Convenience function to set linear and angular scales at once.
     * @param linear_scale The linear damping scale. Should be in the interval [0, 1].
     * @param angular_scale The angular damping scale. Should be in the interval [0, 1].
     * @ingroup bodies damping
     * @sa dBodySetLinearDamping() dBodySetAngularDamping()
     */
    [DllImport("ode")]
    public static extern void dBodySetDamping(dBodyID b, dReal linear_scale, dReal angular_scale);

    /**
     * @brief Get the body's linear damping threshold.
     * @ingroup bodies damping
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetLinearDampingThreshold(dBodyID b);

    /**
     * @brief Set the body's linear damping threshold.
     * @param threshold The linear threshold to be used. Damping
     *      is only applied if the linear speed is above this limit.
     * @ingroup bodies damping
     */
    [DllImport("ode")]
    public static extern void dBodySetLinearDampingThreshold(dBodyID b, dReal threshold);

    /**
     * @brief Get the body's angular damping threshold.
     * @ingroup bodies damping
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetAngularDampingThreshold(dBodyID b);

    /**
     * @brief Set the body's angular damping threshold.
     * @param threshold The angular threshold to be used. Damping is
     *      only used if the angular speed is above this limit.
     * @ingroup bodies damping
     */
    [DllImport("ode")]
    public static extern void dBodySetAngularDampingThreshold(dBodyID b, dReal threshold);

    /**
     * @brief Get the body's maximum angular speed.
     * @ingroup damping bodies
     * @sa dWorldGetMaxAngularSpeed()
     */
    [DllImport("ode")]
    public static extern dReal dBodyGetMaxAngularSpeed(dBodyID b);

    /**
     * @brief Set the body's maximum angular speed.
     * @ingroup damping bodies
     * @sa dWorldSetMaxAngularSpeed() dBodyResetMaxAngularSpeed()
     * The default value is dInfinity, but it's a good idea to limit
     * it at less than 500 if the body has the gyroscopic term
     * enabled.
     */
    [DllImport("ode")]
    public static extern void dBodySetMaxAngularSpeed(dBodyID b, dReal max_speed);



    /**
     * @brief Get the body's gyroscopic state.
     *
     * @return nonzero if gyroscopic term computation is enabled (default),
     * zero otherwise.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern int dBodyGetGyroscopicMode(dBodyID b);


    /**
     * @brief Enable/disable the body's gyroscopic term.
     *
     * Disabling the gyroscopic term of a body usually improves
     * stability. It also helps turning spining objects, like cars'
     * wheels.
     *
     * @param enabled   nonzero (default) to enable gyroscopic term, 0
     * to disable.
     * @ingroup bodies
     */
    [DllImport("ode")]
    public static extern void dBodySetGyroscopicMode(dBodyID b, int enabled);




    /**
     * @defgroup joints Joints
     *
     * In real life a joint is something like a hinge, that is used to connect two
     * objects.
     * In ODE a joint is very similar: It is a relationship that is enforced between
     * two bodies so that they can only have certain positions and orientations
     * relative to each other.
     * This relationship is called a constraint -- the words joint and
     * constraint are often used interchangeably.
     *
     * A joint has a set of parameters that can be set. These include:
     *
     *
     * \li  dParamLoStop Low stop angle or position. Setting this to
     *	-dInfinity (the default value) turns off the low stop.
     *	For rotational joints, this stop must be greater than -pi to be
     *	effective.
     * \li  dParamHiStop High stop angle or position. Setting this to
     *	dInfinity (the default value) turns off the high stop.
     *	For rotational joints, this stop must be less than pi to be
     *	effective.
     *	If the high stop is less than the low stop then both stops will
     *	be ineffective.
     * \li  dParamVel Desired motor velocity (this will be an angular or
     *	linear velocity).
     * \li  dParamFMax The maximum force or torque that the motor will use to
     *	achieve the desired velocity.
     *	This must always be greater than or equal to zero.
     *	Setting this to zero (the default value) turns off the motor.
     * \li  dParamFudgeFactor The current joint stop/motor implementation has
     *	a small problem:
     *	when the joint is at one stop and the motor is set to move it away
     *	from the stop, too much force may be applied for one time step,
     *	causing a ``jumping'' motion.
     *	This fudge factor is used to scale this excess force.
     *	It should have a value between zero and one (the default value).
     *	If the jumping motion is too visible in a joint, the value can be
     *	reduced.
     *	Making this value too small can prevent the motor from being able to
     *	move the joint away from a stop.
     * \li  dParamBounce The bouncyness of the stops.
     *	This is a restitution parameter in the range 0..1.
     *	0 means the stops are not bouncy at all, 1 means maximum bouncyness.
     * \li  dParamCFM The constraint force mixing (CFM) value used when not
     *	at a stop.
     * \li  dParamStopERP The error reduction parameter (ERP) used by the
     *	stops.
     * \li  dParamStopCFM The constraint force mixing (CFM) value used by the
     *	stops. Together with the ERP value this can be used to get spongy or
     *	soft stops.
     *	Note that this is intended for unpowered joints, it does not really
     *	work as expected when a powered joint reaches its limit.
     * \li  dParamSuspensionERP Suspension error reduction parameter (ERP).
     *	Currently this is only implemented on the hinge-2 joint.
     * \li  dParamSuspensionCFM Suspension constraint force mixing (CFM) value.
     *	Currently this is only implemented on the hinge-2 joint.
     *
     * If a particular parameter is not implemented by a given joint, setting it
     * will have no effect.
     * These parameter names can be optionally followed by a digit (2 or 3)
     * to indicate the second or third set of parameters, e.g. for the second axis
     * in a hinge-2 joint, or the third axis in an AMotor joint.
     */


    /**
     * @brief Create a new joint of the ball type.
     * @ingroup joints
     * @remarks
     * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
     * because it does not connect to any bodies.
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateBall(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the hinge type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateHinge(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the slider type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateSlider(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the contact type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateContact(dWorldID world, dJointGroupID jointGroup, ref dContact contace);

    /**
     * @brief Create a new joint of the hinge2 type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateHinge2(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the universal type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateUniversal(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the PR (Prismatic and Rotoide) type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreatePR(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the PU (Prismatic and Universal) type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreatePU(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the Piston type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     *                      If it is nonzero the joint is allocated in the given
     *                      joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreatePiston(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the fixed type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateFixed(dWorldID world, dJointGroupID jointGroup);

    [DllImport("ode")]
    public static extern dJointID dJointCreateNull(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the A-motor type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateAMotor(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the L-motor type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateLMotor(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the plane-2d type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreatePlane2D(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the double ball type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateDBall(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the double hinge type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateDHinge(dWorldID world, dJointGroupID jointGroup);

    /**
     * @brief Create a new joint of the Transmission type.
     * @ingroup joints
     * @param dJointGroupID set to 0 to allocate the joint normally.
     * If it is nonzero the joint is allocated in the given joint group.
     */
    [DllImport("ode")]
    public static extern dJointID dJointCreateTransmission(dWorldID world, dJointGroupID jointGroup);


    /**
     * @brief Destroy a joint.
     * @ingroup joints
     *
     * disconnects it from its attached bodies and removing it from the world.
     * However, if the joint is a member of a group then this function has no
     * effect - to destroy that joint the group must be emptied or destroyed.
     */
    [DllImport("ode")]
    public static extern void dJointDestroy(dJointID joint);


    /**
     * @brief Create a joint group
     * @ingroup joints
     * @param max_size deprecated. Set to 0.
     */
    [DllImport("ode")]
    public static extern dJointGroupID dJointGroupCreate(int max_size);

    /**
     * @brief Destroy a joint group.
     * @ingroup joints
     *
     * All joints in the joint group will be destroyed.
     */
    [DllImport("ode")]
    public static extern void dJointGroupDestroy(dJointGroupID jointGroup);

    /**
     * @brief Empty a joint group.
     * @ingroup joints
     *
     * All joints in the joint group will be destroyed,
     * but the joint group itself will not be destroyed.
     */
    [DllImport("ode")]
    public static extern void dJointGroupEmpty(dJointGroupID jointGroup);

    /**
     * @brief Return the number of bodies attached to the joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dJointGetNumBodies(dJointID joint);

    /**
     * @brief Attach the joint to some new bodies.
     * @ingroup joints
     *
     * If the joint is already attached, it will be detached from the old bodies
     * first.
     * To attach this joint to only one body, set body1 or body2 to zero - a zero
     * body refers to the static environment.
     * Setting both bodies to zero puts the joint into "limbo", i.e. it will
     * have no effect on the simulation.
     * @remarks
     * Some joints, like hinge-2 need to be attached to two bodies to work.
     */
    [DllImport("ode")]
    public static extern void dJointAttach(dJointID joint, dBodyID body1, dBodyID body2);

    /**
     * @brief Manually enable a joint.
     * @param dJointID identification of joint.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointEnable(dJointID joint);

    /**
     * @brief Manually disable a joint.
     * @ingroup joints
     * @remarks
     * A disabled joint will not affect the simulation, but will maintain the anchors and
     * axes so it can be enabled later.
     */
    [DllImport("ode")]
    public static extern void dJointDisable(dJointID joint);

    /**
     * @brief Check wether a joint is enabled.
     * @ingroup joints
     * @return 1 if a joint is currently enabled or 0 if it is disabled.
     */
    [DllImport("ode")]
    public static extern int dJointIsEnabled(dJointID joint);

    /**
     * @brief Set the user-data pointer
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetData(dJointID joint, IntPtr data);

    /**
     * @brief Get the user-data pointer
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern IntPtr dJointGetData(dJointID joint);

    /**
     * @brief Get the type of the joint
     * @ingroup joints
     * @return the type, being one of these:
     * \li dJointTypeBall
     * \li dJointTypeHinge
     * \li dJointTypeSlider
     * \li dJointTypeContact
     * \li dJointTypeUniversal
     * \li dJointTypeHinge2
     * \li dJointTypeFixed
     * \li dJointTypeNull
     * \li dJointTypeAMotor
     * \li dJointTypeLMotor
     * \li dJointTypePlane2D
     * \li dJointTypePR
     * \li dJointTypePU
     * \li dJointTypePiston
     */
    [DllImport("ode")]
    public static extern dJointType dJointGetType(dJointID joint);

    /**
     * @brief Return the bodies that this joint connects.
     * @ingroup joints
     * @param index return the first (0) or second (1) body.
     * @remarks
     * If one of these returned body IDs is zero, the joint connects the other body
     * to the static environment.
     * If both body IDs are zero, the joint is in ``limbo'' and has no effect on
     * the simulation.
     */
    [DllImport("ode")]
    public static extern dBodyID dJointGetBody(dJointID joint, int index);

    /**
     * @brief Sets the datastructure that is to receive the feedback.
     *
     * The feedback can be used by the user, so that it is known how
     * much force an individual joint exerts.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetFeedback(dJointID joint, ref dJointFeedback feedback);

    /**
     * @brief Gets the datastructure that is to receive the feedback.
     * @ingroup joints
     */
    [DllImport("ode", EntryPoint = "dJointGetFeedback")]
    static extern IntPtr _dJointGetFeedback(dJointID joint);
    public static void dJointGetFeedback(ref dJointFeedback feedback, dJointID joint)
    {
        Marshal.PtrToStructure(_dJointGetFeedback(joint), feedback);
    }

    /**
     * @brief Set the joint anchor point.
     * @ingroup joints
     *
     * The joint will try to keep this point on each body
     * together. The input is specified in world coordinates.
     */
    [DllImport("ode")]
    public static extern void dJointSetBallAnchor(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief Set the joint anchor point.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetBallAnchor2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief Param setting for Ball joints
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetBallParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Set hinge anchor parameter.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetHingeAnchor(dJointID joint, dReal x, dReal y, dReal z);

    [DllImport("ode")]
    public static extern void dJointSetHingeAnchorDelta(dJointID joint, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);

    /**
     * @brief Set hinge axis.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetHingeAxis(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief Set the Hinge axis as if the 2 bodies were already at angle appart.
     * @ingroup joints
     *
     * This function initialize the Axis and the relative orientation of each body
     * as if body1 was rotated around the axis by the angle value. \br
     * Ex:
     * <PRE>
     * dJointSetHingeAxis(jId, 1, 0, 0);
     * // If you request the position you will have: dJointGetHingeAngle(jId) == 0
     * dJointSetHingeAxisDelta(jId, 1, 0, 0, 0.23);
     * // If you request the position you will have: dJointGetHingeAngle(jId) == 0.23
     * </PRE>

     * @param j The Hinge joint ID for which the axis will be set
     * @param x The X component of the axis in world frame
     * @param y The Y component of the axis in world frame
     * @param z The Z component of the axis in world frame
     * @param angle The angle for the offset of the relative orientation.
     *              As if body1 was rotated by angle when the Axis was set (see below).
     *              The rotation is around the new Hinge axis.
     *
     * @note Usually the function dJointSetHingeAxis set the current position of body1
     *       and body2 as the zero angle position. This function set the current position
     *       as the if the 2 bodies where \b angle appart.
     * @warning Calling dJointSetHingeAnchor or dJointSetHingeAxis will reset the "zero"
     *          angle position.
     */
    [DllImport("ode")]
    public static extern void dJointSetHingeAxisOffset(dJointID j, dReal x, dReal y, dReal z, dReal angle);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetHingeParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies the torque about the hinge axis.
     *
     * That is, it applies a torque with specified magnitude in the direction
     * of the hinge axis, to body 1, and with the same magnitude but in opposite
     * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddHingeTorque(dJointID joint, dReal torque);

    /**
     * @brief set the joint axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetSliderAxis(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetSliderAxisDelta(dJointID joint, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetSliderParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies the given force in the slider's direction.
     *
     * That is, it applies a force with specified magnitude, in the direction of
     * slider's axis, to body1, and with the same magnitude but opposite
     * direction to body2.  This function is just a wrapper for dBodyAddForce().
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddSliderForce(dJointID joint, dReal force);

    /**
     * @brief set anchor
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetHinge2Anchor(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set both axes (optionally)
     *
     * This can change both axes at once avoiding transitions via invalid states
     * while changing axes one by one and having the first changed axis coincide 
     * with the other axis existing direction.
     *
     * At least one of the axes must be not NULL. If NULL is passed, the corresponding 
     * axis retains its existing value.
     * 
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetHinge2Axes(dJointID j, ref dVector3 axis1, ref dVector3 axis2);

    /**
     * @brief set axis
     *
     * Deprecated. Use @fn dJointSetHinge2Axes instead.
     * 
     * @ingroup joints
     * @see dJointSetHinge2Axes
     */
    //[DllImport("ode")] public static extern_DEPRECATED [DllImport("ode")] public static extern void dJointSetHinge2Axis1(dJointID j, dReal x, dReal y, dReal z);

    /**
     * @brief set axis
     *
     * Deprecated. Use @fn dJointSetHinge2Axes instead.
     * 
     * @ingroup joints
     * @see dJointSetHinge2Axes
     */
    //[DllImport("ode")] public static extern_DEPRECATED [DllImport("ode")] public static extern void dJointSetHinge2Axis2(dJointID j, dReal x, dReal y, dReal z);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetHinge2Param(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies torque1 about the hinge2's axis 1, torque2 about the
     * hinge2's axis 2.
     * @remarks  This function is just a wrapper for dBodyAddTorque().
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddHinge2Torques(dJointID joint, dReal torque1, dReal torque2);

    /**
     * @brief set anchor
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetUniversalAnchor(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetUniversalAxis1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief Set the Universal axis1 as if the 2 bodies were already at 
     *        offset1 and offset2 appart with respect to axis1 and axis2.
     * @ingroup joints
     *
     * This function initialize the axis1 and the relative orientation of 
     * each body as if body1 was rotated around the new axis1 by the offset1 
     * value and as if body2 was rotated around the axis2 by offset2. \br
     * Ex:
    * <PRE>
     * dJointSetHuniversalAxis1(jId, 1, 0, 0);
     * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0
     * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0
     * dJointSetHuniversalAxis1Offset(jId, 1, 0, 0, 0.2, 0.17);
     * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2
     * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17
     * </PRE>
     *
     * @param j The Hinge joint ID for which the axis will be set
     * @param x The X component of the axis in world frame
     * @param y The Y component of the axis in world frame
     * @param z The Z component of the axis in world frame
     * @param angle The angle for the offset of the relative orientation.
     *              As if body1 was rotated by angle when the Axis was set (see below).
     *              The rotation is around the new Hinge axis.
     *
     * @note Usually the function dJointSetHingeAxis set the current position of body1
     *       and body2 as the zero angle position. This function set the current position
     *       as the if the 2 bodies where \b offsets appart.
     *
     * @note Any previous offsets are erased.
     *
     * @warning Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
     *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
     *          will reset the "zero" angle position.
     */
    [DllImport("ode")]
    public static extern void dJointSetUniversalAxis1Offset(dJointID joint, dReal x, dReal y, dReal z,
                                                dReal offset1, dReal offset2);

    /**
     * @brief set axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetUniversalAxis2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief Set the Universal axis2 as if the 2 bodies were already at 
     *        offset1 and offset2 appart with respect to axis1 and axis2.
     * @ingroup joints
     *
     * This function initialize the axis2 and the relative orientation of 
     * each body as if body1 was rotated around the axis1 by the offset1 
     * value and as if body2 was rotated around the new axis2 by offset2. \br
     * Ex:
     * <PRE>
     * dJointSetHuniversalAxis2(jId, 0, 1, 0);
     * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0
     * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0
     * dJointSetHuniversalAxis2Offset(jId, 0, 1, 0, 0.2, 0.17);
     * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2
     * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17
     * </PRE>

     * @param j The Hinge joint ID for which the axis will be set
     * @param x The X component of the axis in world frame
     * @param y The Y component of the axis in world frame
     * @param z The Z component of the axis in world frame
     * @param angle The angle for the offset of the relative orientation.
     *              As if body1 was rotated by angle when the Axis was set (see below).
     *              The rotation is around the new Hinge axis.
     *
     * @note Usually the function dJointSetHingeAxis set the current position of body1
     *       and body2 as the zero angle position. This function set the current position
     *       as the if the 2 bodies where \b offsets appart.
     *
     * @note Any previous offsets are erased.
     *
     * @warning Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
     *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
     *          will reset the "zero" angle position.
     */


    [DllImport("ode")]
    public static extern void dJointSetUniversalAxis2Offset(dJointID joint, dReal x, dReal y, dReal z,
                                                dReal offset1, dReal offset2);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetUniversalParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies torque1 about the universal's axis 1, torque2 about the
     * universal's axis 2.
     * @remarks This function is just a wrapper for dBodyAddTorque().
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddUniversalTorques(dJointID joint, dReal torque1, dReal torque2);


    /**
     * @brief set anchor
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPRAnchor(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set the axis for the prismatic articulation
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPRAxis1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set the axis for the rotoide articulation
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPRAxis2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set joint parameter
     * @ingroup joints
     *
     * @note parameterX where X equal 2 refer to parameter for the rotoide articulation
     */
    [DllImport("ode")]
    public static extern void dJointSetPRParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies the torque about the rotoide axis of the PR joint
     *
     * That is, it applies a torque with specified magnitude in the direction 
     * of the rotoide axis, to body 1, and with the same magnitude but in opposite
     * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddPRTorque(dJointID j, dReal torque);


    /**
    * @brief set anchor
    * @ingroup joints
*/
    [DllImport("ode")]
    public static extern void dJointSetPUAnchor(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set anchor
     * @ingroup joints
     */
    //[DllImport("ode")] public static extern_DEPRECATED [DllImport("ode")] public static extern void dJointSetPUAnchorDelta(dJointID joint, dReal x, dReal y, dReal z,
    //                                                        dReal dx, dReal dy, dReal dz);

    /**
     * @brief Set the PU anchor as if the 2 bodies were already at [dx, dy, dz] appart.
     * @ingroup joints
     *
     * This function initialize the anchor and the relative position of each body
     * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
     * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
     * axis is set).
     * Ex:
     * <PRE>
     * dReal offset = 3;
     * dVector3 axis;
     * dJointGetPUAxis(jId, axis);
     * dJointSetPUAnchor(jId, 0, 0, 0);
     * // If you request the position you will have: dJointGetPUPosition(jId) == 0
     * dJointSetPUAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);
     * // If you request the position you will have: dJointGetPUPosition(jId) == offset
     * </PRE>
     * @param j The PU joint for which the anchor point will be set
     * @param x The X position of the anchor point in world frame
     * @param y The Y position of the anchor point in world frame
     * @param z The Z position of the anchor point in world frame
     * @param dx A delta to be substracted to the X position as if the anchor was set
     *           when body1 was at current_position[X] - dx
     * @param dx A delta to be substracted to the Y position as if the anchor was set
     *           when body1 was at current_position[Y] - dy
     * @param dx A delta to be substracted to the Z position as if the anchor was set
     *           when body1 was at current_position[Z] - dz
     */
    [DllImport("ode")]
    public static extern void dJointSetPUAnchorOffset(dJointID joint, dReal x, dReal y, dReal z,
                                         dReal dx, dReal dy, dReal dz);

    /**
     * @brief set the axis for the first axis or the universal articulation
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPUAxis1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set the axis for the second axis or the universal articulation
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPUAxis2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set the axis for the prismatic articulation
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPUAxis3(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set the axis for the prismatic articulation
     * @ingroup joints
     * @note This function was added for convenience it is the same as
     *       dJointSetPUAxis3
     */
    [DllImport("ode")]
    public static extern void dJointSetPUAxisP(dJointID id, dReal x, dReal y, dReal z);



    /**
     * @brief set joint parameter
     * @ingroup joints
     *
     * @note parameterX where X equal 2 refer to parameter for second axis of the
     *       universal articulation
     * @note parameterX where X equal 3 refer to parameter for prismatic
     *       articulation
     */
    [DllImport("ode")]
    public static extern void dJointSetPUParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies the torque about the rotoide axis of the PU joint
     *
     * That is, it applies a torque with specified magnitude in the direction
     * of the rotoide axis, to body 1, and with the same magnitude but in opposite
     * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddPUTorque(dJointID j, dReal torque);




    /**
     * @brief set the joint anchor
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPistonAnchor(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief Set the Piston anchor as if the 2 bodies were already at [dx,dy, dz] appart.
     * @ingroup joints
     *
     * This function initialize the anchor and the relative position of each body
     * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
     * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
     * axis is set).
     * Ex:
     * <PRE>
     * dReal offset = 3;
     * dVector3 axis;
     * dJointGetPistonAxis(jId, axis);
     * dJointSetPistonAnchor(jId, 0, 0, 0);
     * // If you request the position you will have: dJointGetPistonPosition(jId) == 0
     * dJointSetPistonAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);
     * // If you request the position you will have: dJointGetPistonPosition(jId) == offset
     * </PRE>
     * @param j The Piston joint for which the anchor point will be set
     * @param x The X position of the anchor point in world frame
     * @param y The Y position of the anchor point in world frame
     * @param z The Z position of the anchor point in world frame
     * @param dx A delta to be substracted to the X position as if the anchor was set
     *           when body1 was at current_position[X] - dx
     * @param dx A delta to be substracted to the Y position as if the anchor was set
     *           when body1 was at current_position[Y] - dy
     * @param dx A delta to be substracted to the Z position as if the anchor was set
     *           when body1 was at current_position[Z] - dz
     */
    [DllImport("ode")]
    public static extern void dJointSetPistonAnchorOffset(dJointID j, dReal x, dReal y, dReal z,
                                             dReal dx, dReal dy, dReal dz);

    /**
     * @brief set the joint axis
    * @ingroup joints
*/
    [DllImport("ode")]
    public static extern void dJointSetPistonAxis(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * This function set prismatic axis of the joint and also set the position
     * of the joint.
     *
     * @ingroup joints
     * @param j The joint affected by this function
     * @param x The x component of the axis
     * @param y The y component of the axis
     * @param z The z component of the axis
     * @param dx The Initial position of the prismatic join in the x direction
     * @param dy The Initial position of the prismatic join in the y direction
     * @param dz The Initial position of the prismatic join in the z direction
     */
    //[DllImport("ode")] public static extern_DEPRECATED [DllImport("ode")] public static extern void dJointSetPistonAxisDelta(dJointID j, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPistonParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Applies the given force in the slider's direction.
     *
     * That is, it applies a force with specified magnitude, in the direction of
     * prismatic's axis, to body1, and with the same magnitude but opposite
     * direction to body2.  This function is just a wrapper for dBodyAddForce().
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddPistonForce(dJointID joint, dReal force);


    /**
     * @brief Call this on the fixed joint after it has been attached to
     * remember the current desired relative offset and desired relative
     * rotation between the bodies.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetFixed(dJointID joint);

    /*
     * @brief Sets joint parameter
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetFixedParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief set the nr of axes
     * @param num 0..3
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetAMotorNumAxes(dJointID joint, int num);

    /**
     * @brief set axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetAMotorAxis(dJointID joint, int anum, int rel,
                  dReal x, dReal y, dReal z);

    /**
     * @brief Tell the AMotor what the current angle is along axis anum.
     *
     * This function should only be called in dAMotorUser mode, because in this
     * mode the AMotor has no other way of knowing the joint angles.
     * The angle information is needed if stops have been set along the axis,
     * but it is not needed for axis motors.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetAMotorAngle(dJointID joint, int anum, dReal angle);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetAMotorParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief set mode
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetAMotorMode(dJointID joint, dAMotorMode mode);

    /**
     * @brief Applies torque0 about the AMotor's axis 0, torque1 about the
     * AMotor's axis 1, and torque2 about the AMotor's axis 2.
     * @remarks
     * If the motor has fewer than three axes, the higher torques are ignored.
     * This function is just a wrapper for dBodyAddTorque().
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointAddAMotorTorques(dJointID joint, dReal torque1, dReal torque2, dReal torque3);

    /**
     * @brief Set the number of axes that will be controlled by the LMotor.
     * @param num can range from 0 (which effectively deactivates the joint) to 3.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetLMotorNumAxes(dJointID joint, int num);

    /**
     * @brief Set the AMotor axes.
     * @param anum selects the axis to change (0,1 or 2).
     * @param rel Each axis can have one of three ``relative orientation'' modes
     * \li 0: The axis is anchored to the global frame.
     * \li 1: The axis is anchored to the first body.
     * \li 2: The axis is anchored to the second body.
     * @remarks The axis vector is always specified in global coordinates
     * regardless of the setting of rel.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetLMotorAxis(dJointID joint, int anum, int rel, dReal x, dReal y, dReal z);

    /**
     * @brief set joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetLMotorParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPlane2DXParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @ingroup joints
     */

    [DllImport("ode")]
    public static extern void dJointSetPlane2DYParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetPlane2DAngleParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     *
     * This returns the point on body 1. If the joint is perfectly satisfied,
     * this will be the same as the point on body 2.
     */
    [DllImport("ode")]
    public static extern void dJointGetBallAnchor(dJointID joint,out dVector3 result);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     *
     * This returns the point on body 2. You can think of a ball and socket
     * joint as trying to keep the result of dJointGetBallAnchor() and
     * dJointGetBallAnchor2() the same.  If the joint is perfectly satisfied,
     * this function will return the same value as dJointGetBallAnchor() to
     * within roundoff errors. dJointGetBallAnchor2() can be used, along with
     * dJointGetBallAnchor(), to see how far the joint has come apart.
     */
    [DllImport("ode")]
    public static extern void dJointGetBallAnchor2(dJointID joint, out dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetBallParam(dJointID joint, dParam parameter);

    /**
     * @brief Get the hinge anchor point, in world coordinates.
     *
     * This returns the point on body 1. If the joint is perfectly satisfied,
     * this will be the same as the point on body 2.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHingeAnchor(dJointID joint, out dVector3 result);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     * @return The point on body 2. If the joint is perfectly satisfied,
     * this will return the same value as dJointGetHingeAnchor().
     * If not, this value will be slightly different.
     * This can be used, for example, to see how far the joint has come apart.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHingeAnchor2(dJointID joint, out dVector3 result);

    /**
     * @brief get axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHingeAxis(dJointID joint, out dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHingeParam(dJointID joint, dParam parameter);

    /**
     * @brief Get the hinge angle.
     *
     * The angle is measured between the two bodies, or between the body and
     * the static environment.
     * The angle will be between -pi..pi.
     * Give the relative rotation with respect to the Hinge axis of Body 1 with
     * respect to Body 2.
     * When the hinge anchor or axis is set, the current position of the attached
     * bodies is examined and that position will be the zero angle.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHingeAngle(dJointID joint);

    /**
     * @brief Get the hinge angle time derivative.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHingeAngleRate(dJointID joint);

    /**
     * @brief Get the slider linear position (i.e. the slider's extension)
     *
     * When the axis is set, the current position of the attached bodies is
     * examined and that position will be the zero position.

     * The position is the distance, with respect to the zero position,
     * along the slider axis of body 1 with respect to
     * body 2. (A NULL body is replaced by the world).
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetSliderPosition(dJointID joint);

    /**
     * @brief Get the slider linear position's time derivative.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetSliderPositionRate(dJointID joint);

    /**
     * @brief Get the slider axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetSliderAxis(dJointID joint, dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetSliderParam(dJointID joint, dParam parameter);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     * @return the point on body 1.  If the joint is perfectly satisfied,
     * this will be the same as the point on body 2.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHinge2Anchor(dJointID joint, dVector3 result);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     * This returns the point on body 2. If the joint is perfectly satisfied,
     * this will return the same value as dJointGetHinge2Anchor.
     * If not, this value will be slightly different.
     * This can be used, for example, to see how far the joint has come apart.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHinge2Anchor2(dJointID joint, dVector3 result);

    /**
     * @brief Get joint axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHinge2Axis1(dJointID joint, dVector3 result);

    /**
     * @brief Get joint axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetHinge2Axis2(dJointID joint, dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHinge2Param(dJointID joint, dParam parameter);

    /**
     * @brief Get angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHinge2Angle1(dJointID joint);

    /**
     * @brief Get angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHinge2Angle2(dJointID joint);

    /**
     * @brief Get time derivative of angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHinge2Angle1Rate(dJointID joint);

    /**
     * @brief Get time derivative of angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetHinge2Angle2Rate(dJointID joint);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     * @return the point on body 1. If the joint is perfectly satisfied,
     * this will be the same as the point on body 2.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetUniversalAnchor(dJointID joint, out dVector3 result);

    /**
     * @brief Get the joint anchor point, in world coordinates.
     * @return This returns the point on body 2.
     * @remarks
     * You can think of the ball and socket part of a universal joint as
     * trying to keep the result of dJointGetBallAnchor() and
     * dJointGetBallAnchor2() the same. If the joint is
     * perfectly satisfied, this function will return the same value
     * as dJointGetUniversalAnchor() to within roundoff errors.
     * dJointGetUniversalAnchor2() can be used, along with
     * dJointGetUniversalAnchor(), to see how far the joint has come apart.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetUniversalAnchor2(dJointID joint, out dVector3 result);

    /**
     * @brief Get axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetUniversalAxis1(dJointID joint, out dVector3 result);

    /**
     * @brief Get axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetUniversalAxis2(dJointID joint, out dVector3 result);


    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetUniversalParam(dJointID joint, out dParam parameter);

    /**
     * @brief Get both angles at the same time.
     * @ingroup joints
     *
     * @param joint   The universal joint for which we want to calculate the angles
     * @param angle1  The angle between the body1 and the axis 1
     * @param angle2  The angle between the body2 and the axis 2
     *
     * @note This function combine getUniversalAngle1 and getUniversalAngle2 together
     *       and try to avoid redundant calculation
     */
    [DllImport("ode")]
    public static extern void dJointGetUniversalAngles(dJointID joint, ref dReal angle1, ref dReal angle2);

    /**
     * @brief Get angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetUniversalAngle1(dJointID joint);

    /**
     * @brief Get angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetUniversalAngle2(dJointID joint);

    /**
     * @brief Get time derivative of angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetUniversalAngle1Rate(dJointID joint);

    /**
     * @brief Get time derivative of angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetUniversalAngle2Rate(dJointID joint);



    /**
     * @brief Get the joint anchor point, in world coordinates.
     * @return the point on body 1. If the joint is perfectly satisfied, 
     * this will be the same as the point on body 2.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPRAnchor(dJointID joint, dVector3 result);

    /**
     * @brief Get the PR linear position (i.e. the prismatic's extension)
     *
     * When the axis is set, the current position of the attached bodies is
     * examined and that position will be the zero position.
     *
     * The position is the "oriented" length between the
     * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPRPosition(dJointID joint);

    /**
     * @brief Get the PR linear position's time derivative
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPRPositionRate(dJointID joint);


    /**
     * @brief Get the PR angular position (i.e. the  twist between the 2 bodies)
     *
     * When the axis is set, the current position of the attached bodies is
     * examined and that position will be the zero position.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPRAngle(dJointID joint);

    /**
     * @brief Get the PR angular position's time derivative
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPRAngleRate(dJointID joint);


    /**
     * @brief Get the prismatic axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPRAxis1(dJointID joint, dVector3 result);

    /**
     * @brief Get the Rotoide axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPRAxis2(dJointID joint, dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPRParam(dJointID joint, dParam parameter);



    /**
     * @brief Get the joint anchor point, in world coordinates.
     * @return the point on body 1. If the joint is perfectly satisfied,
     * this will be the same as the point on body 2.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPUAnchor(dJointID joint, dVector3 result);

    /**
     * @brief Get the PU linear position (i.e. the prismatic's extension)
     *
     * When the axis is set, the current position of the attached bodies is
     * examined and that position will be the zero position.
     *
     * The position is the "oriented" length between the
     * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUPosition(dJointID joint);

    /**
     * @brief Get the PR linear position's time derivative
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUPositionRate(dJointID joint);

    /**
     * @brief Get the first axis of the universal component of the joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPUAxis1(dJointID joint, dVector3 result);

    /**
     * @brief Get the second axis of the Universal component of the joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPUAxis2(dJointID joint, dVector3 result);

    /**
     * @brief Get the prismatic axis
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPUAxis3(dJointID joint, dVector3 result);

    /**
     * @brief Get the prismatic axis
     * @ingroup joints
     *
     * @note This function was added for convenience it is the same as
     *       dJointGetPUAxis3
     */
    [DllImport("ode")]
    public static extern void dJointGetPUAxisP(dJointID id, dVector3 result);




    /**
     * @brief Get both angles at the same time.
     * @ingroup joints
     *
     * @param joint   The Prismatic universal joint for which we want to calculate the angles
     * @param angle1  The angle between the body1 and the axis 1
     * @param angle2  The angle between the body2 and the axis 2
     *
     * @note This function combine dJointGetPUAngle1 and dJointGetPUAngle2 together
     *       and try to avoid redundant calculation
     */
    [DllImport("ode")]
    public static extern void dJointGetPUAngles(dJointID joint, ref dReal angle1, ref dReal angle2);

    /**
     * @brief Get angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUAngle1(dJointID joint);

    /**
     * @brief * @brief Get time derivative of angle1
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUAngle1Rate(dJointID joint);


    /**
     * @brief Get angle
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUAngle2(dJointID joint);

    /**
     * @brief * @brief Get time derivative of angle2
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUAngle2Rate(dJointID joint);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPUParam(dJointID joint, dParam parameter);





    /**
     * @brief Get the Piston linear position (i.e. the piston's extension)
     *
     * When the axis is set, the current position of the attached bodies is
     * examined and that position will be the zero position.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPistonPosition(dJointID joint);

    /**
     * @brief Get the piston linear position's time derivative.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPistonPositionRate(dJointID joint);

    /**
     * @brief Get the Piston angular position (i.e. the  twist between the 2 bodies)
     *
     * When the axis is set, the current position of the attached bodies is
     * examined and that position will be the zero position.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPistonAngle(dJointID joint);

    /**
     * @brief Get the piston angular position's time derivative.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPistonAngleRate(dJointID joint);


    /**
     * @brief Get the joint anchor
     *
     * This returns the point on body 1. If the joint is perfectly satisfied,
     * this will be the same as the point on body 2 in direction perpendicular
     * to the prismatic axis.
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPistonAnchor(dJointID joint, dVector3 result);

    /**
     * @brief Get the joint anchor w.r.t. body 2
     *
     * This returns the point on body 2. You can think of a Piston
     * joint as trying to keep the result of dJointGetPistonAnchor() and
     * dJointGetPistonAnchor2() the same in the direction perpendicular to the
     * pirsmatic axis. If the joint is perfectly satisfied,
     * this function will return the same value as dJointGetPistonAnchor() to
     * within roundoff errors. dJointGetPistonAnchor2() can be used, along with
     * dJointGetPistonAnchor(), to see how far the joint has come apart.
     *
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPistonAnchor2(dJointID joint, dVector3 result);

    /**
     * @brief Get the prismatic axis (This is also the rotoide axis.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetPistonAxis(dJointID joint, dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetPistonParam(dJointID joint, dParam parameter);


    /**
     * @brief Get the number of angular axes that will be controlled by the
     * AMotor.
     * @param num can range from 0 (which effectively deactivates the
     * joint) to 3.
     * This is automatically set to 3 in dAMotorEuler mode.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dJointGetAMotorNumAxes(dJointID joint);

    /**
     * @brief Get the AMotor axes.
     * @param anum selects the axis to change (0,1 or 2).
     * @param rel Each axis can have one of three ``relative orientation'' modes.
     * \li 0: The axis is anchored to the global frame.
     * \li 1: The axis is anchored to the first body.
     * \li 2: The axis is anchored to the second body.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetAMotorAxis(dJointID joint, int anum, dVector3 result);

    /**
     * @brief Get axis
     * @remarks
     * The axis vector is always specified in global coordinates regardless
     * of the setting of rel.
     * There are two GetAMotorAxis functions, one to return the axis and one to
     * return the relative mode.
     *
     * For dAMotorEuler mode:
     * \li	Only axes 0 and 2 need to be set. Axis 1 will be determined
        automatically at each time step.
     * \li	Axes 0 and 2 must be perpendicular to each other.
     * \li	Axis 0 must be anchored to the first body, axis 2 must be anchored
        to the second body.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dJointGetAMotorAxisRel(dJointID joint, int anum);

    /**
     * @brief Get the current angle for axis.
     * @remarks
     * In dAMotorUser mode this is simply the value that was set with
     * dJointSetAMotorAngle().
     * In dAMotorEuler mode this is the corresponding euler angle.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetAMotorAngle(dJointID joint, int anum);

    /**
     * @brief Get the current angle rate for axis anum.
     * @remarks
     * In dAMotorUser mode this is always zero, as not enough information is
     * available.
     * In dAMotorEuler mode this is the corresponding euler angle rate.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetAMotorAngleRate(dJointID joint, int anum);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetAMotorParam(dJointID joint, dParam parameter);

    /**
     * @brief Get the angular motor mode.
     * @param mode must be one of the following constants:
     * \li dAMotorUser The AMotor axes and joint angle settings are entirely
     * controlled by the user.  This is the default mode.
     * \li dAMotorEuler Euler angles are automatically computed.
     * The axis a1 is also automatically computed.
     * The AMotor axes must be set correctly when in this mode,
     * as described below.
     * When this mode is initially set the current relative orientations
     * of the bodies will correspond to all euler angles at zero.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dJointGetAMotorMode(dJointID joint);

    /**
     * @brief Get nr of axes.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dJointGetLMotorNumAxes(dJointID joint);

    /**
     * @brief Get axis.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetLMotorAxis(dJointID joint, int anum, dVector3 result);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetLMotorParam(dJointID joint, dParam parameter);

    /**
     * @brief get joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetFixedParam(dJointID joint, dParam parameter);


    /**
     * @brief get the contact point of the first wheel of the Transmission joint.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionContactPoint1(dJointID joint, dVector3 result);

    /**
     * @brief get contact point of the second wheel of the Transmission joint.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionContactPoint2(dJointID joint, dVector3 result);

    /**
     * @brief set the first axis for the Transmission joint
     * @remarks This is the axis around which the first body is allowed to
     * revolve and is attached to it.  It is given in global coordinates
     * and can only be set explicitly in intersecting-axes mode.  For the
     * parallel-axes and chain modes which share one common axis of
     * revolution for both gears dJointSetTransmissionAxis should be used.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionAxis1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get first axis for the Transmission joint
     * @remarks In parallel-axes and chain mode the common axis with
     * respect to the first body is returned.  If the joint constraint is
     * satisfied it should be the same as the axis return with
     * dJointGetTransmissionAxis2 or dJointGetTransmissionAxis.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionAxis1(dJointID joint, dVector3 result);

    /**
     * @brief set second axis for the Transmission joint
     * @remarks This is the axis around which the second body is allowed
     * to revolve and is attached to it.  It is given in global
     * coordinates and can only be set explicitly in intersecting-axes
     * mode.  For the parallel-axes and chain modes which share one common
     * axis of revolution for both gears dJointSetTransmissionAxis should
     * be used.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionAxis2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get second axis for the Transmission joint
     * @remarks In parallel-axes and chain mode the common axis with
     * respect to the second body is returned.  If the joint constraint is
     * satisfied it should be the same as the axis return with
     * dJointGetTransmissionAxis1 or dJointGetTransmissionAxis.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionAxis2(dJointID joint, dVector3 result);

    /**
     * @brief set the first anchor for the Transmission joint
     * @remarks This is the point of attachment of the wheel on the
     * first body.  It is given in global coordinates.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionAnchor1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get the first anchor of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionAnchor1(dJointID joint, dVector3 result);

    /**
     * @brief set the second anchor for the Transmission joint
     * @remarks This is the point of attachment of the wheel on the
     * second body.  It is given in global coordinates.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionAnchor2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get the second anchor for the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionAnchor2(dJointID joint, dVector3 result);

    /**
     * @brief set a Transmission joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief get a Transmission joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionParam(dJointID joint, dParam parameter);

    /**
     * @brief set the Transmission joint mode
     * @remarks The mode can be one of dTransmissionParallelAxes,
     * dTransmissionIntersectingAxes and dTransmissionChainDrive simulating a
     * set of parallel-axes gears, intersecting-axes beveled gears or
     * chain and sprockets respectively.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionMode(dJointID j, int mode);

    /**
     * @brief get the Transmission joint mode
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dJointGetTransmissionMode(dJointID j);

    /**
     * @brief set the Transmission ratio
     * @remarks This is the ratio of the angular speed of the first gear
     * to that of the second gear.  It can only be set explicitly in
     * parallel-axes mode.  In intersecting-axes mode the ratio is defined
     * implicitly by the initial configuration of the wheels and in chain
     * mode it is defined implicitly be the wheel radii.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionRatio(dJointID j, dReal ratio);

    /**
     * @brief get the Transmission joint ratio
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionRatio(dJointID j);

    /**
     * @brief set the common axis for both wheels of the Transmission joint
     * @remarks This sets the common axis of revolution for both wheels
     * and should only be used in parallel-axes or chain mode.  For
     * intersecting-axes mode where each wheel axis needs to be specified
     * individually dJointSetTransmissionAxis1 and
     * dJointSetTransmissionAxis2 should be used.  The axis is given in
     * global coordinates
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionAxis(dJointID j, dReal x, dReal y, dReal z);

    /**
     * @brief get the common axis for both wheels of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetTransmissionAxis(dJointID j, dVector3 result);

    /**
     * @brief get the phase, that is the traversed angle for the first
     * wheel of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionAngle1(dJointID j);

    /**
     * @brief get the phase, that is the traversed angle for the second
     * wheel of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionAngle2(dJointID j);

    /**
     * @brief get the radius of the first wheel of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionRadius1(dJointID j);

    /**
     * @brief get the radius of the second wheel of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionRadius2(dJointID j);

    /**
     * @brief set the radius of the first wheel of the Transmission joint
     * @remarks The wheel radii can only be set explicitly in chain mode.
     * In the other modes they're defined implicitly by the initial
     * configuration and ratio of the wheels.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionRadius1(dJointID j, dReal radius);

    /**
     * @brief set the radius of the second wheel of the Transmission joint
     * @remarks The wheel radii can only be set explicitly in chain mode.
     * In the other modes they're defined implicitly by the initial
     * configuration and ratio of the wheels.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionRadius2(dJointID j, dReal radius);

    /**
     * @brief get the backlash of the Transmission joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetTransmissionBacklash(dJointID j);

    /**
     * @brief set the backlash of the Transmission joint
     * @remarks Backlash is the clearance in the mesh of the wheels of the
     * transmission and is defined as the maximum distance that the
     * geometric contact point can travel without any actual contact or
     * transfer of power between the wheels.  This can be converted in
     * degrees of revolution for each wheel by dividing by the wheel's
     * radius.  To further illustrate this consider the situation where a
     * wheel of radius r_1 is driving another wheel of radius r_2 and
     * there is an amount of backlash equal to b in their mesh.  If the
     * driving wheel were to instantaneously stop there would be no
     * contact and hence the driven wheel would continue to turn for
     * another b / r_2 radians until all the backlash in the mesh was take
     * up and contact restored with the relationship of driving and driven
     * wheel reversed.  The backlash is therefore given in untis of
     * length.
      * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetTransmissionBacklash(dJointID j, dReal backlash);

    /**
     * @brief set anchor1 for double ball joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDBallAnchor1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set anchor2 for double ball joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDBallAnchor2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get anchor1 from double ball joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetDBallAnchor1(dJointID joint, dVector3 result);

    /**
     * @brief get anchor2 from double ball joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetDBallAnchor2(dJointID joint, dVector3 result);

    /**
     * @brief get the target distance from double ball joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetDBallDistance(dJointID joint);

    /**
     * @brief set the target distance for the double ball joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDBallDistance(dJointID joint, dReal dist);

    /**
     * @brief set double ball joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDBallParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief get double ball joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetDBallParam(dJointID joint, dParam parameter);

    /**
     * @brief set axis for double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDHingeAxis(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get axis for double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetDHingeAxis(dJointID joint, dVector3 result);

    /**
     * @brief set anchor1 for double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDHingeAnchor1(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief set anchor2 for double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDHingeAnchor2(dJointID joint, dReal x, dReal y, dReal z);

    /**
     * @brief get anchor1 from double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetDHingeAnchor1(dJointID joint, dVector3 result);

    /**
     * @brief get anchor2 from double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointGetDHingeAnchor2(dJointID joint, dVector3 result);

    /**
     * @brief get the set distance from double hinge joint
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetDHingeDistance(dJointID joint);

    /**
     * @brief set double hinge joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern void dJointSetDHingeParam(dJointID joint, dParam parameter, dReal value);

    /**
     * @brief get double hinge joint parameter
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dReal dJointGetDHingeParam(dJointID joint, dParam parameter);




    /**
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern dJointID dConnectingJoint(dBodyID body1, dBodyID body2);

    /**
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dConnectingJointList(dBodyID body1, dBodyID body2, ref dJointID joint);

    /**
     * @brief Utility function
     * @return 1 if the two bodies are connected together by
     * a joint, otherwise return 0.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dAreConnected(dBodyID body1, dBodyID body2);

    /**
     * @brief Utility function
     * @return 1 if the two bodies are connected together by
     * a joint that does not have type @arg{joint_type}, otherwise return 0.
     * @param body1 A body to check.
     * @param body2 A body to check.
     * @param joint_type is a dJointTypeXXX constant.
     * This is useful for deciding whether to add contact joints between two bodies:
     * if they are already connected by non-contact joints then it may not be
     * appropriate to add contacts, however it is okay to add more contact between-
     * bodies that already have contacts.
     * @ingroup joints
     */
    [DllImport("ode")]
    public static extern int dAreConnectedExcluding(dBodyID body1, dBodyID body2, int joint_type);
    #endregion
}
