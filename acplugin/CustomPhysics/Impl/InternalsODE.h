#pragma once

// ODE 0.13.1

struct dBase {
    void *operator new (size_t size); // { return 0; }
    void *operator new (size_t, void *p); // { return 0; }
    void operator delete (void *ptr, size_t size); // { }
    void *operator new[] (size_t size); // { return 0; }
    void operator delete[] (void *ptr, size_t size); // { }
};

struct dObject : public dBase {
    dxWorld *world;		// world this object is in
    dObject *next;		// next object of this type in list
    dObject **tome;		// pointer to previous object's next ptr
    int tag;			// used by dynamics algorithms
    void *userdata;		// user settable data

    virtual ~dObject();
}; 

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxAutoDisable {
    dReal idle_time;		// time the body needs to be idle to auto-disable it
    int idle_steps;		// steps the body needs to be idle to auto-disable it
    unsigned int average_samples;     // size of the average_lvel and average_avel buffers
    dReal linear_average_threshold;   // linear (squared) average velocity threshold
    dReal angular_average_threshold;  // angular (squared) average velocity threshold
}; 

struct dxDampingParameters {
    dReal linear_scale;  // multiply the linear velocity by (1 - scale)
    dReal angular_scale; // multiply the angular velocity by (1 - scale)
    dReal linear_threshold;   // linear (squared) average speed threshold
    dReal angular_threshold;  // angular (squared) average speed threshold
}; 

struct dxQuickStepParameters {
    int num_iterations;		// number of SOR iterations to perform
    dReal w;			// the SOR over-relaxation parameter
};

struct dxContactParameters {
    dReal max_vel;		// maximum correcting velocity
    dReal min_depth;		// thickness of 'surface layer'
}; 

struct dxPosR {
    dVector3 pos;
    dMatrix3 R;
}; 

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxBody : public dObject {
    dxJointNode *firstjoint;	// list of attached joints
    unsigned flags;			// some dxBodyFlagXXX flags
    dGeomID geom;			// first collision geom associated with body
    dMass mass;			// mass parameters about POR
    dMatrix3 invI;		// inverse of mass.I
    dReal invMass;		// 1 / mass.mass
    dxPosR posr;			// position and orientation of point of reference
    dQuaternion q;		// orientation quaternion
    dVector3 lvel,avel;		// linear and angular velocity of POR
    dVector3 facc,tacc;		// force and torque accumulators
    dVector3 finite_rot_axis;	// finite rotation axis, unit length or 0=none

    // auto-disable information
    dxAutoDisable adis;		// auto-disable parameters
    dReal adis_timeleft;		// time left to be idle
    int adis_stepsleft;		// steps left to be idle
    dVector3* average_lvel_buffer;      // buffer for the linear average velocity calculation
    dVector3* average_avel_buffer;      // buffer for the angular average velocity calculation
    unsigned int average_counter;      // counter/index to fill the average-buffers
    int average_ready;            // indicates ( with = 1 ), if the Body's buffers are ready for average-calculations

    void (*moved_callback)(dxBody*); // let the user know the body moved
    dxDampingParameters dampingp; // damping parameters, depends on flags
    dReal max_angular_speed;      // limit the angular velocity to this magnitude
};

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxIThreadingDefaultImplProvider {
    virtual const dxThreadingFunctionsInfo *RetrieveThreadingDefaultImpl(dThreadingImplementationID &out_default_impl) = 0;
};

struct dxThreadingBase { 
    dxIThreadingDefaultImplProvider   *m_default_impl_provider;
    const dxThreadingFunctionsInfo    *m_functions_info;
    dThreadingImplementationID        m_threading_impl;
};

struct dxWorld : public dBase, public dxThreadingBase, private dxIThreadingDefaultImplProvider {
    dxBody *firstbody;		// body linked list
    dxJoint *firstjoint;		// joint linked list
    int nb,nj;			// number of bodies and joints in lists
    dVector3 gravity;		// gravity vector (m/s/s)
    dReal global_erp;		// global error reduction parameter
    dReal global_cfm;		// global constraint force mixing parameter
    dxAutoDisable adis;		// auto-disable parameters
    int body_flags;               // flags for new bodies
    unsigned islands_max_threads; // maximum threads to allocate for island processing
    class dxStepWorkingMemory *wmem; // Working memory object for dWorldStep/dWorldQuickStep

    dxQuickStepParameters qs;
    dxContactParameters contactp;
    dxDampingParameters dampingp; // damping parameters
    dReal max_angular_speed;      // limit the angular velocity to this magnitude

    void* userdata;

    virtual ~dxWorld();
}; 

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxGeom : public dBase {
    int type;		// geom type number, set by subclass constructor
    int gflags;		// flags used by geom and space
    void *data;		// user-defined data pointer
    dBodyID body;		// dynamics body associated with this object (if any)
    dxGeom *body_next;	// next geom in body's linked list of associated geoms
    dxPosR *final_posr;	// final position of the geom in world coordinates
    dxPosR *offset_posr;	// offset from body in local coordinates

    // information used by spaces
    dxGeom *next;		// next geom in linked list of geoms
    dxGeom **tome;	// linked list backpointer
    dxGeom *next_ex;	// next geom in extra linked list of geoms (for higher level structures)
    dxGeom **tome_ex;	// extra linked list backpointer (for higher level structures)
    dxSpace *parent_space;// the space this geom is contained in, 0 if none
    dReal aabb[6];	// cached AABB for this space
    unsigned long category_bits,collide_bits; 

	virtual ~dxGeom();
};

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxSpace : public dxGeom {
    int count;			// number of geoms in this space
    dxGeom *first;		// first geom in list
    int cleanup;			// cleanup mode, 1=destroy geoms on exit
    int sublevel;         // space sublevel (used in dSpaceCollide2). NOT TRACKED AUTOMATICALLY!!!
    unsigned tls_kind;	// space TLS kind to be used for global caches retrieval

    // cached state for getGeom()
    int current_index;		// only valid if current_geom != 0
    dxGeom *current_geom;		// if 0 then there is no information

    // locking stuff. the space is locked when it is currently traversing its
    // internal data structures, e.g. in collide() and collide2(). operations
    // that modify the contents of the space are not permitted when the space
    // is locked.
    int lock_count; 
};

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxSphere : public dxGeom {
    dReal radius;		// sphere radius
};

struct dxBox : public dxGeom {
    dVector3 side;	// side lengths (x,y,z)
};

struct dxCapsule : public dxGeom {
    dReal radius,lz;	// radius, length along z axis
};

struct dxCylinder : public dxGeom {
    dReal radius,lz;        // radius, length along z axis
};

struct dxPlane : public dxGeom {
    dReal p[4];
};

struct dxRay : public dxGeom {
    dReal length;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxConvex : public dxGeom 
{  
    dReal *planes; /*!< An array of planes in the form:
                   normal X, normal Y, normal Z,Distance
                   */
    dReal *points; /*!< An array of points X,Y,Z */  
    unsigned int *polygons; /*! An array of indices to the points of each polygon, it should be the number of vertices followed by that amount of indices to "points" in counter clockwise order*/
    unsigned int planecount; /*!< Amount of planes in planes */
    unsigned int pointcount;/*!< Amount of points in points */
    unsigned int edgecount;/*!< Amount of edges in convex */
    dReal saabb[6];/*!< Static AABB */ 
	struct edge
    {
        unsigned int first;
        unsigned int second;
    };
    edge* edges;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

struct dxTriMesh : public dxGeom{
    // Callbacks
    dTriCallback* Callback;
    dTriArrayCallback* ArrayCallback;
    dTriRayCallback* RayCallback;
    dTriTriMergeCallback* TriMergeCallback;

    // Data types
    dxTriMeshData* Data;

    bool doSphereTC;
    bool doBoxTC;
    bool doCapsuleTC; 

	//...
};

///////////////////////////////////////////////////////////////////////////////////////////////////
