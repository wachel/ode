#include <ode/common.h>
#include <ode/collision_space.h>
#include <ode/contact.h>
#include <ode/ode.h>

#ifdef __cplusplus
extern "C" {
#endif

	ODE_API void ExtCopyFloat(float* inValue, float* outValue , int num)
	{
		for (int i = 0; i < num; i++){
			outValue[i] = inValue[i];
		}
	}

	typedef void (*FunOnNewContact)(dContact* contact,dBodyID b1,dBodyID b2);
	static void defaultNearCallback(void* callback, dGeomID o1, dGeomID o2)
	{
		FunOnNewContact fun = (FunOnNewContact)callback;
		dBodyID b1 = dGeomGetBody(o1);
		dBodyID b2 = dGeomGetBody(o2);

		if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
			return;
		const int MAX_CONTACTS = 8;
		dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
		for (int i = 0; i < MAX_CONTACTS; i++) {
			contact[i].surface.mode = dContactBounce | dContactSoftCFM;
			contact[i].surface.mu = 1.5f;
			contact[i].surface.mu2 = 0;
			contact[i].surface.bounce = 0.1;
			contact[i].surface.bounce_vel = 0.1;
			contact[i].surface.soft_cfm = 0.01;
		}

		if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
			for (int i = 0; i < numc; i++) {
				(fun)(contact + i,b1,b2);
				//dJointID c = dJointCreateContact(world, contactgroup, contact + i);
				//dJointAttach(c, b1, b2);
			}
		}
	}

	ODE_API void ExtSpaceCollide(dSpaceID space, FunOnNewContact callback)
	{
		dSpaceCollide(space, callback, &defaultNearCallback);
	}

#ifdef __cplusplus
}
#endif