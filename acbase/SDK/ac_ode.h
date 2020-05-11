// ### AUTO-GENERATED ###

#define ODE_CALL(name) ((proto_##name)(_drva(RVA_##name)))
#define ODE_V3(v) v.x, v.y, v.z

#define RVA_dGeomDestroy 0x003444B0
#define RVA_dGeomSetData 0x003447F0
#define RVA_dGeomGetData 0x00344520
#define RVA_dGeomSetBody 0x00344620
#define RVA_dGeomGetBody 0x003444D0
#define RVA_dGeomSetRotation 0x003448C0
#define RVA_dGeomGetPosition 0x00344530
#define RVA_dGeomGetRotation 0x003445E0
#define RVA_dGeomGetQuaternion 0x00344560
#define RVA_dGeomIsSpace 0x00344610
#define RVA_dGeomGetClass 0x00344500
#define RVA_dGeomSetCategoryBits 0x003404B0
#define RVA_dGeomSetCollideBits 0x003447E0
#define RVA_dGeomGetCategoryBits 0x003444F0
#define RVA_dGeomGetCollideBits 0x00344510
#define RVA_dGeomSetOffsetPosition 0x00344800
#define RVA_dGeomSetOffsetRotation 0x00344870
#define RVA_dCollide 0x00344120
#define RVA_dSpaceCollide 0x00343090
#define RVA_dSpaceCollide2 0x003430A0
#define RVA_dGeomSphereGetRadius 0x00345E40
#define RVA_dGeomSpherePointDepth 0x00379CE0
#define RVA_dCreateBox 0x0034A320
#define RVA_dGeomBoxGetLengths 0x0034A380
#define RVA_dGeomBoxPointDepth 0x0034A3A0
#define RVA_dGeomPlaneGetParams 0x00395E30
#define RVA_dGeomCapsuleGetParams 0x00395B60
#define RVA_dGeomCapsulePointDepth 0x0037A8C0
#define RVA_dGeomCylinderGetParams 0x00395B60
#define RVA_dCreateRay 0x00345D20
#define RVA_dGeomRaySetLength 0x00345F60
#define RVA_dGeomRayGetLength 0x00345E40
#define RVA_dGeomRaySet 0x00345E50
#define RVA_dGeomRayGet 0x00345DA0
#define RVA_dGeomRaySetFirstContact 0x00345F40
#define RVA_dGeomRayGetFirstContact 0x00345E30
#define RVA_dGeomRaySetBackfaceCull 0x00345F20
#define RVA_dGeomRayGetBackfaceCull 0x00345E10
#define RVA_dGeomRayGetClosestHit 0x00345E20
#define RVA_dClosestLineSegmentPoints 0x0038F3E0
#define RVA_dBoxBox 0x00348260
#define RVA_dSimpleSpaceCreate 0x00342FF0
#define RVA_dSpaceDestroy 0x00343270
#define RVA_dSpaceAdd 0x00343080
#define RVA_dSpaceRemove 0x00391530
#define RVA_dGeomTriMeshDataCreate 0x0034B1B0
#define RVA_dGeomTriMeshDataBuildSingle 0x0034B170
#define RVA_dCreateTriMesh 0x0034B0F0
#define RVA_dGetConfiguration 0x0033FE10
#define RVA_dDebug 0x0034B2D0
#define RVA_dMessage 0x0030B780
#define RVA_dMassCheck 0x00346A20
#define RVA_dMassSetZero 0x00346D30
#define RVA_dMassSetParameters 0x00346C60
#define RVA_dMassSetBoxTotal 0x00346BC0
#define RVA_dDot 0x00390530
#define RVA_dFactorCholesky 0x0034B7F0
#define RVA_dSolveCholesky 0x0034CA70
#define RVA_dInvertPDMatrix 0x0034BAC0
#define RVA_dIsPositiveDefinite 0x0034BCE0
#define RVA_dFactorLDLT 0x003988D0
#define RVA_dSolveL1 0x00390610
#define RVA_dSolveL1T 0x00390FD0
#define RVA_dVectorScale 0x0034CE60
#define RVA_dSolveLDLT 0x0034CDF0
#define RVA_dLDLTAddTL 0x0034BDA0
#define RVA_dLDLTRemove 0x0034C420
#define RVA_dRemoveRowCol 0x0034C910
#define RVA_dAlloc 0x0034B3B0
#define RVA_dRealloc 0x0034B3F0
#define RVA_dFree 0x0034B3D0
#define RVA_dWorldCreate 0x003401F0
#define RVA_dWorldDestroy 0x00340220
#define RVA_dWorldSetGravity 0x003404A0
#define RVA_dWorldSetERP 0x00340490
#define RVA_dWorldSetCFM 0x00340420
#define RVA_dWorldStep 0x003404C0
#define RVA_dWorldSetQuickStepNumIterations 0x003404B0
#define RVA_dWorldSetContactMaxCorrectingVel 0x00340430
#define RVA_dWorldSetContactSurfaceLayer 0x00340440
#define RVA_dWorldSetDamping 0x00340450
#define RVA_dBodySetAutoDisableAverageSamplesCount 0x0033F910
#define RVA_dBodySetAutoDisableFlag 0x0033FA00
#define RVA_dBodyCreate 0x0033EF50
#define RVA_dBodyDestroy 0x0033F340
#define RVA_dBodySetData 0x0033FA30
#define RVA_dBodyGetData 0x0033F4E0
#define RVA_dBodySetPosition 0x0033FBB0
#define RVA_dBodySetRotation 0x0033FC00
#define RVA_dBodySetLinearVel 0x0033FB10
#define RVA_dBodySetAngularVel 0x0033F8F0
#define RVA_dBodyGetPosition 0x0033F6A0
#define RVA_dBodyGetRotation 0x0033F8B0
#define RVA_dBodyGetQuaternion 0x0033F6B0
#define RVA_dBodyGetAngularVel 0x0033F4D0
#define RVA_dBodySetMass 0x0033FB30
#define RVA_dBodyGetMass 0x0033F4F0
#define RVA_dBodyAddForce 0x0033E860
#define RVA_dBodyAddTorque 0x0033EF10
#define RVA_dBodyAddRelTorque 0x0033EE50
#define RVA_dBodyAddForceAtPos 0x0033E8A0
#define RVA_dBodyAddForceAtRelPos 0x0033E990
#define RVA_dBodyAddRelForceAtPos 0x0033EB00
#define RVA_dBodyAddRelForceAtRelPos 0x0033EC70
#define RVA_dBodySetForce 0x0033FAD0
#define RVA_dBodySetTorque 0x0033FC90
#define RVA_dBodyGetRelPointPos 0x0033F6C0
#define RVA_dBodyGetRelPointVel 0x0033F780
#define RVA_dBodyGetPointVel 0x0033F520
#define RVA_dBodyGetPosRelPoint 0x0033F5E0
#define RVA_dBodyVectorToWorld 0x0033FD60
#define RVA_dBodyVectorFromWorld 0x0033FCB0
#define RVA_dBodySetFiniteRotationMode 0x0033FA90
#define RVA_dBodySetFiniteRotationAxis 0x0033FA40
#define RVA_dBodyEnable 0x0033F4B0
#define RVA_dBodyDisable 0x0033F4A0
#define RVA_dBodyIsEnabled 0x0033F8C0
#define RVA_dBodySetLinearDamping 0x0033FAF0
#define RVA_dBodySetAngularDamping 0x0033F8D0
#define RVA_dJointCreateBall 0x0033FED0
#define RVA_dJointCreateSlider 0x0033FF80
#define RVA_dJointCreateContact 0x0033FEE0
#define RVA_dJointCreateFixed 0x0033FF70
#define RVA_dJointCreateDBall 0x0033FF60
#define RVA_dJointGroupCreate 0x0033FFC0
#define RVA_dJointGroupDestroy 0x00340000
#define RVA_dJointGroupEmpty 0x00340040
#define RVA_dJointAttach 0x0033FE20
#define RVA_dJointGetBody 0x0033FF90
#define RVA_dJointSetBallAnchor 0x00340DE0
#define RVA_dJointSetBallParam 0x00340E10
#define RVA_dJointSetSliderAxis 0x003419C0
#define RVA_dJointSetSliderParam 0x00341A00
#define RVA_dJointSetFixed 0x00341E30
#define RVA_dJointSetFixedParam 0x00340E10
#define RVA_dJointGetSliderPosition 0x003417C0
#define RVA_dJointSetDBallAnchor1 0x003426A0
#define RVA_dJointSetDBallAnchor2 0x00342750
#define RVA_dJointGetDBallDistance 0x00342690
#define RVA_dJointSetDBallParam 0x00340E10
#define RVA_dInitODE2 0x00340BC0
#define RVA_dAllocateODEDataForThread 0x00340B90
#define RVA_dCloseODE 0x00340BB0
#define RVA_dSafeNormalize3 0x0034B410
#define RVA_dSafeNormalize4 0x0034B500
#define RVA_dNormalize4 0x0034B570
#define RVA_dPlaneSpace 0x0034B6E0
#define RVA_dOrthogonalizeR 0x0034B580
#define RVA_dRSetIdentity 0x003465F0
#define RVA_dQMultiply0 0x00346040
#define RVA_dQMultiply1 0x00346130
#define RVA_dQMultiply2 0x00346220
#define RVA_dQMultiply3 0x00346310
#define RVA_dRfromQ 0x00346620
#define RVA_dQfromR 0x00346400
#define RVA_dDQfromW 0x00345F70
#define RVA_dThreadingImplementationGetFunctions 0x00391AF0
#define RVA_dThreadingFreeImplementation 0x00391AE0

typedef ODE_API void (*proto_dGeomDestroy) (dGeomID geom);
typedef ODE_API void (*proto_dGeomSetData) (dGeomID geom, void* data);
typedef ODE_API void* (*proto_dGeomGetData) (dGeomID geom);
typedef ODE_API void (*proto_dGeomSetBody) (dGeomID geom, dBodyID body);
typedef ODE_API dBodyID (*proto_dGeomGetBody) (dGeomID geom);
typedef ODE_API void (*proto_dGeomSetRotation) (dGeomID geom, const dMatrix3 R);
typedef ODE_API const dReal * (*proto_dGeomGetPosition) (dGeomID geom);
typedef ODE_API const dReal * (*proto_dGeomGetRotation) (dGeomID geom);
typedef ODE_API void (*proto_dGeomGetQuaternion) (dGeomID geom, dQuaternion result);
typedef ODE_API int (*proto_dGeomIsSpace) (dGeomID geom);
typedef ODE_API int (*proto_dGeomGetClass) (dGeomID geom);
typedef ODE_API void (*proto_dGeomSetCategoryBits) (dGeomID geom, unsigned long bits);
typedef ODE_API void (*proto_dGeomSetCollideBits) (dGeomID geom, unsigned long bits);
typedef ODE_API unsigned long (*proto_dGeomGetCategoryBits) (dGeomID);
typedef ODE_API unsigned long (*proto_dGeomGetCollideBits) (dGeomID);
typedef ODE_API void (*proto_dGeomSetOffsetPosition) (dGeomID geom, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dGeomSetOffsetRotation) (dGeomID geom, const dMatrix3 R);
typedef ODE_API int (*proto_dCollide) (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact,int skip);
typedef ODE_API void (*proto_dSpaceCollide) (dSpaceID space, void* data, dNearCallback *callback);
typedef ODE_API void (*proto_dSpaceCollide2) (dGeomID space1, dGeomID space2, void* data, dNearCallback *callback);
typedef ODE_API dReal (*proto_dGeomSphereGetRadius) (dGeomID sphere);
typedef ODE_API dReal (*proto_dGeomSpherePointDepth) (dGeomID sphere, dReal x, dReal y, dReal z);
typedef ODE_API dGeomID (*proto_dCreateBox) (dSpaceID space, dReal lx, dReal ly, dReal lz);
typedef ODE_API void (*proto_dGeomBoxGetLengths) (dGeomID box, dVector3 result);
typedef ODE_API dReal (*proto_dGeomBoxPointDepth) (dGeomID box, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dGeomPlaneGetParams) (dGeomID plane, dVector4 result);
typedef ODE_API void (*proto_dGeomCapsuleGetParams) (dGeomID ccylinder, dReal *radius, dReal *length);
typedef ODE_API dReal (*proto_dGeomCapsulePointDepth) (dGeomID ccylinder, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dGeomCylinderGetParams) (dGeomID cylinder, dReal *radius, dReal *length);
typedef ODE_API dGeomID (*proto_dCreateRay) (dSpaceID space, dReal length);
typedef ODE_API void (*proto_dGeomRaySetLength) (dGeomID ray, dReal length);
typedef ODE_API dReal (*proto_dGeomRayGetLength) (dGeomID ray);
typedef ODE_API void (*proto_dGeomRaySet) (dGeomID ray, dReal px, dReal py, dReal pz,dReal dx, dReal dy, dReal dz);
typedef ODE_API void (*proto_dGeomRayGet) (dGeomID ray, dVector3 start, dVector3 dir);
typedef ODE_API void (*proto_dGeomRaySetFirstContact) (dGeomID g, int firstContact);
typedef ODE_API int (*proto_dGeomRayGetFirstContact) (dGeomID g);
typedef ODE_API void (*proto_dGeomRaySetBackfaceCull) (dGeomID g, int backfaceCull);
typedef ODE_API int (*proto_dGeomRayGetBackfaceCull) (dGeomID g);
typedef ODE_API int (*proto_dGeomRayGetClosestHit) (dGeomID g);
typedef ODE_API void (*proto_dClosestLineSegmentPoints) (const dVector3 a1, const dVector3 a2,const dVector3 b1, const dVector3 b2,dVector3 cp1, dVector3 cp2);
typedef ODE_API int (*proto_dBoxBox) (const dVector3 p1, const dMatrix3 R1,const dVector3 side1, const dVector3 p2,const dMatrix3 R2, const dVector3 side2,dVector3 normal, dReal* depth, int *return_code,int flags, dContactGeom *contact, int skip);
typedef ODE_API dSpaceID (*proto_dSimpleSpaceCreate) (dSpaceID space);
typedef ODE_API void (*proto_dSpaceDestroy) (dSpaceID);
typedef ODE_API void (*proto_dSpaceAdd) (dSpaceID, dGeomID);
typedef ODE_API void (*proto_dSpaceRemove) (dSpaceID, dGeomID);
typedef ODE_API dTriMeshDataID (*proto_dGeomTriMeshDataCreate)(void);
typedef ODE_API void (*proto_dGeomTriMeshDataBuildSingle)(dTriMeshDataID g,const void* Vertices, int VertexStride, int VertexCount,const void* Indices, int IndexCount, int TriStride);
typedef ODE_API dGeomID (*proto_dCreateTriMesh)(dSpaceID space, dTriMeshDataID Data, dTriCallback* Callback, dTriArrayCallback* ArrayCallback, dTriRayCallback* RayCallback);
typedef ODE_API const char* (*proto_dGetConfiguration) (void);
typedef ODE_API void (*proto_dDebug) (int num, const char *msg, ...);
typedef ODE_API void (*proto_dMessage) (int num, const char *msg, ...);
typedef ODE_API int (*proto_dMassCheck)(const dMass *m);
typedef ODE_API void (*proto_dMassSetZero) (dMass *);
typedef ODE_API void (*proto_dMassSetParameters) (dMass *, dReal themass,dReal cgx, dReal cgy, dReal cgz,dReal I11, dReal I22, dReal I33,dReal I12, dReal I13, dReal I23);
typedef ODE_API void (*proto_dMassSetBoxTotal) (dMass *, dReal total_mass,dReal lx, dReal ly, dReal lz);
typedef ODE_API dReal (*proto_dDot) (const dReal *a, const dReal *b, int n);
typedef ODE_API int (*proto_dFactorCholesky) (dReal *A, int n);
typedef ODE_API void (*proto_dSolveCholesky) (const dReal *L, dReal *b, int n);
typedef ODE_API int (*proto_dInvertPDMatrix) (const dReal *A, dReal *Ainv, int n);
typedef ODE_API int (*proto_dIsPositiveDefinite) (const dReal *A, int n);
typedef ODE_API void (*proto_dFactorLDLT) (dReal *A, dReal* d, int n, int nskip);
typedef ODE_API void (*proto_dSolveL1) (const dReal *L, dReal *b, int n, int nskip);
typedef ODE_API void (*proto_dSolveL1T) (const dReal *L, dReal *b, int n, int nskip);
typedef ODE_API void (*proto_dVectorScale) (dReal *a, const dReal* d, int n);
typedef ODE_API void (*proto_dSolveLDLT) (const dReal *L, const dReal* d, dReal *b, int n, int nskip);
typedef ODE_API void (*proto_dLDLTAddTL) (dReal *L, dReal* d, const dReal *a, int n, int nskip);
typedef ODE_API void (*proto_dLDLTRemove) (dReal **A, const int *p, dReal *L, dReal* d,int n1, int n2, int r, int nskip);
typedef ODE_API void (*proto_dRemoveRowCol) (dReal *A, int n, int nskip, int r);
typedef ODE_API void * (*proto_dAlloc) (size_t size);
typedef ODE_API void * (*proto_dRealloc) (void *ptr, size_t oldsize, size_t newsize);
typedef ODE_API void (*proto_dFree) (void *ptr, size_t size);
typedef ODE_API dWorldID (*proto_dWorldCreate)(void);
typedef ODE_API void (*proto_dWorldDestroy) (dWorldID world);
typedef ODE_API void (*proto_dWorldSetGravity) (dWorldID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dWorldSetERP) (dWorldID, dReal erp);
typedef ODE_API void (*proto_dWorldSetCFM) (dWorldID, dReal cfm);
typedef ODE_API int (*proto_dWorldStep) (dWorldID w, dReal stepsize);
typedef ODE_API void (*proto_dWorldSetQuickStepNumIterations) (dWorldID, int num);
typedef ODE_API void (*proto_dWorldSetContactMaxCorrectingVel) (dWorldID, dReal vel);
typedef ODE_API void (*proto_dWorldSetContactSurfaceLayer) (dWorldID, dReal depth);
typedef ODE_API void (*proto_dWorldSetDamping)(dWorldID w,dReal linear_scale,dReal angular_scale);
typedef ODE_API void (*proto_dBodySetAutoDisableAverageSamplesCount) (dBodyID, unsigned int average_samples_count);
typedef ODE_API void (*proto_dBodySetAutoDisableFlag) (dBodyID, int do_auto_disable);
typedef ODE_API dBodyID (*proto_dBodyCreate) (dWorldID);
typedef ODE_API void (*proto_dBodyDestroy) (dBodyID);
typedef ODE_API void  (*proto_dBodySetData) (dBodyID, void* data);
typedef ODE_API void* (*proto_dBodyGetData) (dBodyID);
typedef ODE_API void (*proto_dBodySetPosition)   (dBodyID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dBodySetRotation)   (dBodyID, const dMatrix3 R);
typedef ODE_API void (*proto_dBodySetLinearVel)  (dBodyID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dBodySetAngularVel) (dBodyID, dReal x, dReal y, dReal z);
typedef ODE_API const dReal * (*proto_dBodyGetPosition) (dBodyID);
typedef ODE_API const dReal * (*proto_dBodyGetRotation) (dBodyID);
typedef ODE_API const dReal * (*proto_dBodyGetQuaternion) (dBodyID);
typedef ODE_API const dReal * (*proto_dBodyGetAngularVel) (dBodyID);
typedef ODE_API void (*proto_dBodySetMass) (dBodyID, const dMass *mass);
typedef ODE_API void (*proto_dBodyGetMass) (dBodyID, dMass *mass);
typedef ODE_API void (*proto_dBodyAddForce)            (dBodyID, dReal fx, dReal fy, dReal fz);
typedef ODE_API void (*proto_dBodyAddTorque)           (dBodyID, dReal fx, dReal fy, dReal fz);
typedef ODE_API void (*proto_dBodyAddRelTorque)        (dBodyID, dReal fx, dReal fy, dReal fz);
typedef ODE_API void (*proto_dBodyAddForceAtPos)       (dBodyID, dReal fx, dReal fy, dReal fz,dReal px, dReal py, dReal pz);
typedef ODE_API void (*proto_dBodyAddForceAtRelPos)    (dBodyID, dReal fx, dReal fy, dReal fz,dReal px, dReal py, dReal pz);
typedef ODE_API void (*proto_dBodyAddRelForceAtPos)    (dBodyID, dReal fx, dReal fy, dReal fz,dReal px, dReal py, dReal pz);
typedef ODE_API void (*proto_dBodyAddRelForceAtRelPos) (dBodyID, dReal fx, dReal fy, dReal fz,dReal px, dReal py, dReal pz);
typedef ODE_API void (*proto_dBodySetForce)  (dBodyID b, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dBodySetTorque) (dBodyID b, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dBodyGetRelPointPos)(dBodyID, dReal px, dReal py, dReal pz,dVector3 result);
typedef ODE_API void (*proto_dBodyGetRelPointVel)(dBodyID, dReal px, dReal py, dReal pz,dVector3 result);
typedef ODE_API void (*proto_dBodyGetPointVel)(dBodyID, dReal px, dReal py, dReal pz,dVector3 result);
typedef ODE_API void (*proto_dBodyGetPosRelPoint)(dBodyID, dReal px, dReal py, dReal pz,dVector3 result);
typedef ODE_API void (*proto_dBodyVectorToWorld)(dBodyID, dReal px, dReal py, dReal pz,dVector3 result);
typedef ODE_API void (*proto_dBodyVectorFromWorld)(dBodyID, dReal px, dReal py, dReal pz,dVector3 result);
typedef ODE_API void (*proto_dBodySetFiniteRotationMode) (dBodyID, int mode);
typedef ODE_API void (*proto_dBodySetFiniteRotationAxis) (dBodyID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dBodyEnable) (dBodyID);
typedef ODE_API void (*proto_dBodyDisable) (dBodyID);
typedef ODE_API int (*proto_dBodyIsEnabled) (dBodyID);
typedef ODE_API void (*proto_dBodySetLinearDamping)(dBodyID b, dReal scale);
typedef ODE_API void (*proto_dBodySetAngularDamping)(dBodyID b, dReal scale);
typedef ODE_API dJointID (*proto_dJointCreateBall) (dWorldID, dJointGroupID);
typedef ODE_API dJointID (*proto_dJointCreateSlider) (dWorldID, dJointGroupID);
typedef ODE_API dJointID (*proto_dJointCreateContact) (dWorldID, dJointGroupID, const dContact *);
typedef ODE_API dJointID (*proto_dJointCreateFixed) (dWorldID, dJointGroupID);
typedef ODE_API dJointID (*proto_dJointCreateDBall) (dWorldID, dJointGroupID);
typedef ODE_API dJointGroupID (*proto_dJointGroupCreate) (int max_size);
typedef ODE_API void (*proto_dJointGroupDestroy) (dJointGroupID);
typedef ODE_API void (*proto_dJointGroupEmpty) (dJointGroupID);
typedef ODE_API void (*proto_dJointAttach) (dJointID, dBodyID body1, dBodyID body2);
typedef ODE_API dBodyID (*proto_dJointGetBody) (dJointID, int index);
typedef ODE_API void (*proto_dJointSetBallAnchor) (dJointID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dJointSetBallParam) (dJointID, int parameter, dReal value);
typedef ODE_API void (*proto_dJointSetSliderAxis) (dJointID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dJointSetSliderParam) (dJointID, int parameter, dReal value);
typedef ODE_API void (*proto_dJointSetFixed) (dJointID);
typedef ODE_API void (*proto_dJointSetFixedParam) (dJointID, int parameter, dReal value);
typedef ODE_API dReal (*proto_dJointGetSliderPosition) (dJointID);
typedef ODE_API void (*proto_dJointSetDBallAnchor1)(dJointID, dReal x, dReal y, dReal z);
typedef ODE_API void (*proto_dJointSetDBallAnchor2)(dJointID, dReal x, dReal y, dReal z);
typedef ODE_API dReal (*proto_dJointGetDBallDistance)(dJointID);
typedef ODE_API void (*proto_dJointSetDBallParam)(dJointID, int parameter, dReal value);
typedef ODE_API int (*proto_dInitODE2)(unsigned int uiInitFlags/*=0*/);
typedef ODE_API int (*proto_dAllocateODEDataForThread)(unsigned int uiAllocateFlags);
typedef ODE_API void (*proto_dCloseODE)(void);
typedef ODE_API int  (*proto_dSafeNormalize3) (dVector3 a);
typedef ODE_API int  (*proto_dSafeNormalize4) (dVector4 a);
typedef ODE_API void (*proto_dNormalize4) (dVector4 a);
typedef ODE_API void (*proto_dPlaneSpace) (const dVector3 n, dVector3 p, dVector3 q);
typedef ODE_API void (*proto_dOrthogonalizeR)(dMatrix3 m);
typedef ODE_API void (*proto_dRSetIdentity) (dMatrix3 R);
typedef ODE_API void (*proto_dQMultiply0) (dQuaternion qa, const dQuaternion qb, const dQuaternion qc);
typedef ODE_API void (*proto_dQMultiply1) (dQuaternion qa, const dQuaternion qb, const dQuaternion qc);
typedef ODE_API void (*proto_dQMultiply2) (dQuaternion qa, const dQuaternion qb, const dQuaternion qc);
typedef ODE_API void (*proto_dQMultiply3) (dQuaternion qa, const dQuaternion qb, const dQuaternion qc);
typedef ODE_API void (*proto_dRfromQ) (dMatrix3 R, const dQuaternion q);
typedef ODE_API void (*proto_dQfromR) (dQuaternion q, const dMatrix3 R);
typedef ODE_API void (*proto_dDQfromW) (dReal dq[4], const dVector3 w, const dQuaternion q);
typedef ODE_API const dThreadingFunctionsInfo* (*proto_dThreadingImplementationGetFunctions)(dThreadingImplementationID impl);
typedef ODE_API void (*proto_dThreadingFreeImplementation)(dThreadingImplementationID impl);
