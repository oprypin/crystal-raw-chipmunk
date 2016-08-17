@[Link("chipmunk")]
lib CP
  fun message = cpMessage(condition : UInt8*, file : UInt8*, line : Int32, is_error : Int32, is_hard_error : Int32, message : UInt8*, ...)
  
  alias Float = Float64
  
  alias HashValue = LibC::SizeT
  
  alias CollisionID = UInt32
  
  #alias Bool = UInt8
  
  alias DataPointer = Void*
  
  alias CollisionType = LibC::SizeT
  
  alias Group = LibC::SizeT
  
  alias Bitmask = UInt32
  
  alias Timestamp = UInt32
  
  struct Vect
    x : Float
    y : Float
  end
  
  struct Transform
    a : Float
    b : Float
    c : Float
    d : Float
    tx : Float
    ty : Float
  end
  
  struct Mat2x2
    a : Float
    b : Float
    c : Float
    d : Float
  end
  
  type Array = Void*
  
  type HashSet = Void*
  
  type Body = Void*
  
  type Shape = Void*
  
  type CircleShape = Void*
  
  type SegmentShape = Void*
  
  type PolyShape = Void*
  
  type Constraint = Void*
  
  type PinJoint = Void*
  
  type SlideJoint = Void*
  
  type PivotJoint = Void*
  
  type GrooveJoint = Void*
  
  type DampedSpring = Void*
  
  type DampedRotarySpring = Void*
  
  type RotaryLimitJoint = Void*
  
  type RatchetJoint = Void*
  
  type GearJoint = Void*
  
  type SimpleMotorJoint = Void*
  
  type Arbiter = Void*
  
  type Space = Void*
  
  struct BB
    l : Float
    b : Float
    r : Float
    t : Float
  end
  
  alias SpatialIndexBBFunc = Void* -> BB
  
  alias SpatialIndexIteratorFunc = (Void*, Void*) ->
  
  alias SpatialIndexQueryFunc = (Void*, Void*, CollisionID, Void*) -> CollisionID
  
  alias SpatialIndexSegmentQueryFunc = (Void*, Void*, Void*) -> Float
  
  struct SpatialIndex
    klass : SpatialIndexClass*
    bbfunc : SpatialIndexBBFunc
    static_index : SpatialIndex*
    dynamic_index : SpatialIndex*
  end
  
  type SpaceHash = Void*
  
  fun space_hash_alloc = cpSpaceHashAlloc() : SpaceHash
  
  fun space_hash_init = cpSpaceHashInit(hash : SpaceHash, celldim : Float, numcells : Int32, bbfunc : SpatialIndexBBFunc, static_index : SpatialIndex*) : SpatialIndex*
  
  fun space_hash_new = cpSpaceHashNew(celldim : Float, cells : Int32, bbfunc : SpatialIndexBBFunc, static_index : SpatialIndex*) : SpatialIndex*
  
  fun space_hash_resize = cpSpaceHashResize(hash : SpaceHash, celldim : Float, numcells : Int32)
  
  type BBTree = Void*
  
  fun bb_tree_alloc = cpBBTreeAlloc() : BBTree
  
  fun bb_tree_init = cpBBTreeInit(tree : BBTree, bbfunc : SpatialIndexBBFunc, static_index : SpatialIndex*) : SpatialIndex*
  
  fun bb_tree_new = cpBBTreeNew(bbfunc : SpatialIndexBBFunc, static_index : SpatialIndex*) : SpatialIndex*
  
  fun bb_tree_optimize = cpBBTreeOptimize(index : SpatialIndex*)
  
  alias BBTreeVelocityFunc = Void* -> Vect
  
  fun bb_tree_set_velocity_func = cpBBTreeSetVelocityFunc(index : SpatialIndex*, func : BBTreeVelocityFunc)
  
  type Sweep1D = Void*
  
  fun sweep1d_alloc = cpSweep1DAlloc() : Sweep1D
  
  fun sweep1d_init = cpSweep1DInit(sweep : Sweep1D, bbfunc : SpatialIndexBBFunc, static_index : SpatialIndex*) : SpatialIndex*
  
  fun sweep1d_new = cpSweep1DNew(bbfunc : SpatialIndexBBFunc, static_index : SpatialIndex*) : SpatialIndex*
  
  alias SpatialIndexDestroyImpl = SpatialIndex* ->
  
  alias SpatialIndexCountImpl = SpatialIndex* -> Int32
  
  alias SpatialIndexEachImpl = (SpatialIndex*, SpatialIndexIteratorFunc, Void*) ->
  
  alias SpatialIndexContainsImpl = (SpatialIndex*, Void*, HashValue) -> Bool
  
  alias SpatialIndexInsertImpl = (SpatialIndex*, Void*, HashValue) ->
  
  alias SpatialIndexRemoveImpl = (SpatialIndex*, Void*, HashValue) ->
  
  alias SpatialIndexReindexImpl = SpatialIndex* ->
  
  alias SpatialIndexReindexObjectImpl = (SpatialIndex*, Void*, HashValue) ->
  
  alias SpatialIndexReindexQueryImpl = (SpatialIndex*, SpatialIndexQueryFunc, Void*) ->
  
  alias SpatialIndexQueryImpl = (SpatialIndex*, Void*, BB, SpatialIndexQueryFunc, Void*) ->
  
  alias SpatialIndexSegmentQueryImpl = (SpatialIndex*, Void*, Vect, Vect, Float, SpatialIndexSegmentQueryFunc, Void*) ->
  
  struct SpatialIndexClass
    destroy : SpatialIndexDestroyImpl
    count : SpatialIndexCountImpl
    each : SpatialIndexEachImpl
    contains : SpatialIndexContainsImpl
    insert : SpatialIndexInsertImpl
    remove : SpatialIndexRemoveImpl
    reindex : SpatialIndexReindexImpl
    reindex_object : SpatialIndexReindexObjectImpl
    reindex_query : SpatialIndexReindexQueryImpl
    query : SpatialIndexQueryImpl
    segment_query : SpatialIndexSegmentQueryImpl
  end
  
  fun spatial_index_free = cpSpatialIndexFree(index : SpatialIndex*)
  
  fun spatial_index_collide_static = cpSpatialIndexCollideStatic(dynamic_index : SpatialIndex*, static_index : SpatialIndex*, func : SpatialIndexQueryFunc, data : Void*)
  
  fun arbiter_get_restitution = cpArbiterGetRestitution(arb : Arbiter) : Float
  
  fun arbiter_set_restitution = cpArbiterSetRestitution(arb : Arbiter, restitution : Float)
  
  fun arbiter_get_friction = cpArbiterGetFriction(arb : Arbiter) : Float
  
  fun arbiter_set_friction = cpArbiterSetFriction(arb : Arbiter, friction : Float)
  
  fun arbiter_get_surface_velocity = cpArbiterGetSurfaceVelocity(arb : Arbiter) : Vect
  
  fun arbiter_set_surface_velocity = cpArbiterSetSurfaceVelocity(arb : Arbiter, vr : Vect)
  
  fun arbiter_get_user_data = cpArbiterGetUserData(arb : Arbiter) : DataPointer
  
  fun arbiter_set_user_data = cpArbiterSetUserData(arb : Arbiter, user_data : DataPointer)
  
  fun arbiter_total_impulse = cpArbiterTotalImpulse(arb : Arbiter) : Vect
  
  fun arbiter_total_ke = cpArbiterTotalKE(arb : Arbiter) : Float
  
  fun arbiter_ignore = cpArbiterIgnore(arb : Arbiter) : Bool
  
  fun arbiter_get_shapes = cpArbiterGetShapes(arb : Arbiter, a : Shape, b : Shape)
  
  fun arbiter_get_bodies = cpArbiterGetBodies(arb : Arbiter, a : Body, b : Body)
  
  struct Anonymous1
    point_a : Vect
    point_b : Vect
    distance : Float
  end
  
  struct ContactPointSet
    count : Int32
    normal : Vect
    points : Anonymous1[2]
  end
  
  fun arbiter_get_contact_point_set = cpArbiterGetContactPointSet(arb : Arbiter) : ContactPointSet
  
  fun arbiter_set_contact_point_set = cpArbiterSetContactPointSet(arb : Arbiter, set : ContactPointSet*)
  
  fun arbiter_is_first_contact = cpArbiterIsFirstContact(arb : Arbiter) : Bool
  
  fun arbiter_is_removal = cpArbiterIsRemoval(arb : Arbiter) : Bool
  
  fun arbiter_get_count = cpArbiterGetCount(arb : Arbiter) : Int32
  
  fun arbiter_get_normal = cpArbiterGetNormal(arb : Arbiter) : Vect
  
  fun arbiter_get_point_a = cpArbiterGetPointA(arb : Arbiter, i : Int32) : Vect
  
  fun arbiter_get_point_b = cpArbiterGetPointB(arb : Arbiter, i : Int32) : Vect
  
  fun arbiter_get_depth = cpArbiterGetDepth(arb : Arbiter, i : Int32) : Float
  
  fun arbiter_call_wildcard_begin_a = cpArbiterCallWildcardBeginA(arb : Arbiter, space : Space) : Bool
  
  fun arbiter_call_wildcard_begin_b = cpArbiterCallWildcardBeginB(arb : Arbiter, space : Space) : Bool
  
  fun arbiter_call_wildcard_pre_solve_a = cpArbiterCallWildcardPreSolveA(arb : Arbiter, space : Space) : Bool
  
  fun arbiter_call_wildcard_pre_solve_b = cpArbiterCallWildcardPreSolveB(arb : Arbiter, space : Space) : Bool
  
  fun arbiter_call_wildcard_post_solve_a = cpArbiterCallWildcardPostSolveA(arb : Arbiter, space : Space)
  
  fun arbiter_call_wildcard_post_solve_b = cpArbiterCallWildcardPostSolveB(arb : Arbiter, space : Space)
  
  fun arbiter_call_wildcard_separate_a = cpArbiterCallWildcardSeparateA(arb : Arbiter, space : Space)
  
  fun arbiter_call_wildcard_separate_b = cpArbiterCallWildcardSeparateB(arb : Arbiter, space : Space)
  
  enum BodyType
    BODY_TYPE_DYNAMIC
    BODY_TYPE_KINEMATIC
    BODY_TYPE_STATIC
  end
  
  alias BodyVelocityFunc = (Body, Vect, Float, Float) ->
  
  alias BodyPositionFunc = (Body, Float) ->
  
  fun body_alloc = cpBodyAlloc() : Body
  
  fun body_init = cpBodyInit(body : Body, mass : Float, moment : Float) : Body
  
  fun body_new = cpBodyNew(mass : Float, moment : Float) : Body
  
  fun body_new_kinematic = cpBodyNewKinematic() : Body
  
  fun body_new_static = cpBodyNewStatic() : Body
  
  fun body_destroy = cpBodyDestroy(body : Body)
  
  fun body_free = cpBodyFree(body : Body)
  
  fun body_activate = cpBodyActivate(body : Body)
  
  fun body_activate_static = cpBodyActivateStatic(body : Body, filter : Shape)
  
  fun body_sleep = cpBodySleep(body : Body)
  
  fun body_sleep_with_group = cpBodySleepWithGroup(body : Body, group : Body)
  
  fun body_is_sleeping = cpBodyIsSleeping(body : Body) : Bool
  
  fun body_get_type = cpBodyGetType(body : Body) : BodyType
  
  fun body_set_type = cpBodySetType(body : Body, type : BodyType)
  
  fun body_get_space = cpBodyGetSpace(body : Body) : Space
  
  fun body_get_mass = cpBodyGetMass(body : Body) : Float
  
  fun body_set_mass = cpBodySetMass(body : Body, m : Float)
  
  fun body_get_moment = cpBodyGetMoment(body : Body) : Float
  
  fun body_set_moment = cpBodySetMoment(body : Body, i : Float)
  
  fun body_get_position = cpBodyGetPosition(body : Body) : Vect
  
  fun body_set_position = cpBodySetPosition(body : Body, pos : Vect)
  
  fun body_get_center_of_gravity = cpBodyGetCenterOfGravity(body : Body) : Vect
  
  fun body_set_center_of_gravity = cpBodySetCenterOfGravity(body : Body, cog : Vect)
  
  fun body_get_velocity = cpBodyGetVelocity(body : Body) : Vect
  
  fun body_set_velocity = cpBodySetVelocity(body : Body, velocity : Vect)
  
  fun body_get_force = cpBodyGetForce(body : Body) : Vect
  
  fun body_set_force = cpBodySetForce(body : Body, force : Vect)
  
  fun body_get_angle = cpBodyGetAngle(body : Body) : Float
  
  fun body_set_angle = cpBodySetAngle(body : Body, a : Float)
  
  fun body_get_angular_velocity = cpBodyGetAngularVelocity(body : Body) : Float
  
  fun body_set_angular_velocity = cpBodySetAngularVelocity(body : Body, angular_velocity : Float)
  
  fun body_get_torque = cpBodyGetTorque(body : Body) : Float
  
  fun body_set_torque = cpBodySetTorque(body : Body, torque : Float)
  
  fun body_get_rotation = cpBodyGetRotation(body : Body) : Vect
  
  fun body_get_user_data = cpBodyGetUserData(body : Body) : DataPointer
  
  fun body_set_user_data = cpBodySetUserData(body : Body, user_data : DataPointer)
  
  fun body_set_velocity_update_func = cpBodySetVelocityUpdateFunc(body : Body, velocity_func : BodyVelocityFunc)
  
  fun body_set_position_update_func = cpBodySetPositionUpdateFunc(body : Body, position_func : BodyPositionFunc)
  
  fun body_update_velocity = cpBodyUpdateVelocity(body : Body, gravity : Vect, damping : Float, dt : Float)
  
  fun body_update_position = cpBodyUpdatePosition(body : Body, dt : Float)
  
  fun body_local_to_world = cpBodyLocalToWorld(body : Body, point : Vect) : Vect
  
  fun body_world_to_local = cpBodyWorldToLocal(body : Body, point : Vect) : Vect
  
  fun body_apply_force_at_world_point = cpBodyApplyForceAtWorldPoint(body : Body, force : Vect, point : Vect)
  
  fun body_apply_force_at_local_point = cpBodyApplyForceAtLocalPoint(body : Body, force : Vect, point : Vect)

  fun body_apply_impulse_at_world_point = cpBodyApplyImpulseAtWorldPoint(body : Body, impulse : Vect, point : Vect)
  
  fun body_apply_impulse_at_local_point = cpBodyApplyImpulseAtLocalPoint(body : Body, impulse : Vect, point : Vect)
  
  fun body_get_velocity_at_world_point = cpBodyGetVelocityAtWorldPoint(body : Body, point : Vect) : Vect
  
  fun body_get_velocity_at_local_point = cpBodyGetVelocityAtLocalPoint(body : Body, point : Vect) : Vect
  
  fun body_kinetic_energy = cpBodyKineticEnergy(body : Body) : Float
  
  alias BodyShapeIteratorFunc = (Body, Shape, Void*) ->
  
  fun body_each_shape = cpBodyEachShape(body : Body, func : BodyShapeIteratorFunc, data : Void*)
  
  alias BodyConstraintIteratorFunc = (Body, Constraint, Void*) ->
  
  fun body_each_constraint = cpBodyEachConstraint(body : Body, func : BodyConstraintIteratorFunc, data : Void*)
  
  alias BodyArbiterIteratorFunc = (Body, Arbiter, Void*) ->
  
  fun body_each_arbiter = cpBodyEachArbiter(body : Body, func : BodyArbiterIteratorFunc, data : Void*)
  
  struct PointQueryInfo
    shape : Shape
    point : Vect
    distance : Float
    gradient : Vect
  end
  
  struct SegmentQueryInfo
    shape : Shape
    point : Vect
    normal : Vect
    alpha : Float
  end
  
  struct ShapeFilter
    group : Group
    categories : Bitmask
    mask : Bitmask
  end
  
  fun shape_destroy = cpShapeDestroy(shape : Shape)
  
  fun shape_free = cpShapeFree(shape : Shape)
  
  fun shape_cache_bb = cpShapeCacheBB(shape : Shape) : BB
  
  fun shape_update = cpShapeUpdate(shape : Shape, transform : Transform) : BB
  
  fun shape_point_query = cpShapePointQuery(shape : Shape, p : Vect, out : PointQueryInfo*) : Float
  
  fun shape_segment_query = cpShapeSegmentQuery(shape : Shape, a : Vect, b : Vect, radius : Float, info : SegmentQueryInfo*) : Bool
  
  fun shapes_collide = cpShapesCollide(a : Shape, b : Shape) : ContactPointSet
  
  fun shape_get_space = cpShapeGetSpace(shape : Shape) : Space
  
  fun shape_get_body = cpShapeGetBody(shape : Shape) : Body
  
  fun shape_set_body = cpShapeSetBody(shape : Shape, body : Body)
  
  fun shape_get_mass = cpShapeGetMass(shape : Shape) : Float
  
  fun shape_set_mass = cpShapeSetMass(shape : Shape, mass : Float)
  
  fun shape_get_density = cpShapeGetDensity(shape : Shape) : Float
  
  fun shape_set_density = cpShapeSetDensity(shape : Shape, density : Float)
  
  fun shape_get_moment = cpShapeGetMoment(shape : Shape) : Float
  
  fun shape_get_area = cpShapeGetArea(shape : Shape) : Float
  
  fun shape_get_center_of_gravity = cpShapeGetCenterOfGravity(shape : Shape) : Vect
  
  fun shape_get_bb = cpShapeGetBB(shape : Shape) : BB
  
  fun shape_get_sensor = cpShapeGetSensor(shape : Shape) : Bool
  
  fun shape_set_sensor = cpShapeSetSensor(shape : Shape, sensor : Bool)
  
  fun shape_get_elasticity = cpShapeGetElasticity(shape : Shape) : Float
  
  fun shape_set_elasticity = cpShapeSetElasticity(shape : Shape, elasticity : Float)
  
  fun shape_get_friction = cpShapeGetFriction(shape : Shape) : Float
  
  fun shape_set_friction = cpShapeSetFriction(shape : Shape, friction : Float)
  
  fun shape_get_surface_velocity = cpShapeGetSurfaceVelocity(shape : Shape) : Vect
  
  fun shape_set_surface_velocity = cpShapeSetSurfaceVelocity(shape : Shape, surface_velocity : Vect)
  
  fun shape_get_user_data = cpShapeGetUserData(shape : Shape) : DataPointer
  
  fun shape_set_user_data = cpShapeSetUserData(shape : Shape, user_data : DataPointer)
  
  fun shape_get_collision_type = cpShapeGetCollisionType(shape : Shape) : CollisionType
  
  fun shape_set_collision_type = cpShapeSetCollisionType(shape : Shape, collision_type : CollisionType)
  
  fun shape_get_filter = cpShapeGetFilter(shape : Shape) : ShapeFilter
  
  fun shape_set_filter = cpShapeSetFilter(shape : Shape, filter : ShapeFilter)
  
  fun circle_shape_alloc = cpCircleShapeAlloc() : CircleShape
  
  fun circle_shape_init = cpCircleShapeInit(circle : CircleShape, body : Body, radius : Float, offset : Vect) : CircleShape
  
  fun circle_shape_new = cpCircleShapeNew(body : Body, radius : Float, offset : Vect) : Shape
  
  fun circle_shape_get_offset = cpCircleShapeGetOffset(shape : Shape) : Vect
  
  fun circle_shape_get_radius = cpCircleShapeGetRadius(shape : Shape) : Float
  
  fun segment_shape_alloc = cpSegmentShapeAlloc() : SegmentShape
  
  fun segment_shape_init = cpSegmentShapeInit(seg : SegmentShape, body : Body, a : Vect, b : Vect, radius : Float) : SegmentShape
  
  fun segment_shape_new = cpSegmentShapeNew(body : Body, a : Vect, b : Vect, radius : Float) : Shape
  
  fun segment_shape_set_neighbors = cpSegmentShapeSetNeighbors(shape : Shape, prev : Vect, next_ : Vect)
  
  fun segment_shape_get_a = cpSegmentShapeGetA(shape : Shape) : Vect
  
  fun segment_shape_get_b = cpSegmentShapeGetB(shape : Shape) : Vect
  
  fun segment_shape_get_normal = cpSegmentShapeGetNormal(shape : Shape) : Vect
  
  fun segment_shape_get_radius = cpSegmentShapeGetRadius(shape : Shape) : Float
  
  fun poly_shape_alloc = cpPolyShapeAlloc() : PolyShape
  
  fun poly_shape_init = cpPolyShapeInit(poly : PolyShape, body : Body, count : Int32, verts : Vect*, transform : Transform, radius : Float) : PolyShape
  
  fun poly_shape_init_raw = cpPolyShapeInitRaw(poly : PolyShape, body : Body, count : Int32, verts : Vect*, radius : Float) : PolyShape
  
  fun poly_shape_new = cpPolyShapeNew(body : Body, count : Int32, verts : Vect*, transform : Transform, radius : Float) : Shape
  
  fun poly_shape_new_raw = cpPolyShapeNewRaw(body : Body, count : Int32, verts : Vect*, radius : Float) : Shape
  
  fun box_shape_init = cpBoxShapeInit(poly : PolyShape, body : Body, width : Float, height : Float, radius : Float) : PolyShape
  
  fun box_shape_init2 = cpBoxShapeInit2(poly : PolyShape, body : Body, box : BB, radius : Float) : PolyShape
  
  fun box_shape_new = cpBoxShapeNew(body : Body, width : Float, height : Float, radius : Float) : Shape
  
  fun box_shape_new2 = cpBoxShapeNew2(body : Body, box : BB, radius : Float) : Shape
  
  fun poly_shape_get_count = cpPolyShapeGetCount(shape : Shape) : Int32
  
  fun poly_shape_get_vert = cpPolyShapeGetVert(shape : Shape, index : Int32) : Vect
  
  fun poly_shape_get_radius = cpPolyShapeGetRadius(shape : Shape) : Float
  
  alias ConstraintPreSolveFunc = (Constraint, Space) ->
  
  alias ConstraintPostSolveFunc = (Constraint, Space) ->
  
  fun constraint_destroy = cpConstraintDestroy(constraint : Constraint)
  
  fun constraint_free = cpConstraintFree(constraint : Constraint)
  
  fun constraint_get_space = cpConstraintGetSpace(constraint : Constraint) : Space
  
  fun constraint_get_body_a = cpConstraintGetBodyA(constraint : Constraint) : Body
  
  fun constraint_get_body_b = cpConstraintGetBodyB(constraint : Constraint) : Body
  
  fun constraint_get_max_force = cpConstraintGetMaxForce(constraint : Constraint) : Float
  
  fun constraint_set_max_force = cpConstraintSetMaxForce(constraint : Constraint, max_force : Float)
  
  fun constraint_get_error_bias = cpConstraintGetErrorBias(constraint : Constraint) : Float
  
  fun constraint_set_error_bias = cpConstraintSetErrorBias(constraint : Constraint, error_bias : Float)
  
  fun constraint_get_max_bias = cpConstraintGetMaxBias(constraint : Constraint) : Float
  
  fun constraint_set_max_bias = cpConstraintSetMaxBias(constraint : Constraint, max_bias : Float)
  
  fun constraint_get_collide_bodies = cpConstraintGetCollideBodies(constraint : Constraint) : Bool
  
  fun constraint_set_collide_bodies = cpConstraintSetCollideBodies(constraint : Constraint, collide_bodies : Bool)
  
  fun constraint_get_pre_solve_func = cpConstraintGetPreSolveFunc(constraint : Constraint) : ConstraintPreSolveFunc
  
  fun constraint_set_pre_solve_func = cpConstraintSetPreSolveFunc(constraint : Constraint, pre_solve_func : ConstraintPreSolveFunc)
  
  fun constraint_get_post_solve_func = cpConstraintGetPostSolveFunc(constraint : Constraint) : ConstraintPostSolveFunc
  
  fun constraint_set_post_solve_func = cpConstraintSetPostSolveFunc(constraint : Constraint, post_solve_func : ConstraintPostSolveFunc)
  
  fun constraint_get_user_data = cpConstraintGetUserData(constraint : Constraint) : DataPointer
  
  fun constraint_set_user_data = cpConstraintSetUserData(constraint : Constraint, user_data : DataPointer)
  
  fun constraint_get_impulse = cpConstraintGetImpulse(constraint : Constraint) : Float
  
  fun constraint_is_pin_joint = cpConstraintIsPinJoint(constraint : Constraint) : Bool
  
  fun pin_joint_alloc = cpPinJointAlloc() : PinJoint
  
  fun pin_joint_init = cpPinJointInit(joint : PinJoint, a : Body, b : Body, anchor_a : Vect, anchor_b : Vect) : PinJoint
  
  fun pin_joint_new = cpPinJointNew(a : Body, b : Body, anchor_a : Vect, anchor_b : Vect) : Constraint
  
  fun pin_joint_get_anchor_a = cpPinJointGetAnchorA(constraint : Constraint) : Vect
  
  fun pin_joint_set_anchor_a = cpPinJointSetAnchorA(constraint : Constraint, anchor_a : Vect)
  
  fun pin_joint_get_anchor_b = cpPinJointGetAnchorB(constraint : Constraint) : Vect
  
  fun pin_joint_set_anchor_b = cpPinJointSetAnchorB(constraint : Constraint, anchor_b : Vect)
  
  fun pin_joint_get_dist = cpPinJointGetDist(constraint : Constraint) : Float
  
  fun pin_joint_set_dist = cpPinJointSetDist(constraint : Constraint, dist : Float)
  
  fun constraint_is_slide_joint = cpConstraintIsSlideJoint(constraint : Constraint) : Bool
  
  fun slide_joint_alloc = cpSlideJointAlloc() : SlideJoint
  
  fun slide_joint_init = cpSlideJointInit(joint : SlideJoint, a : Body, b : Body, anchor_a : Vect, anchor_b : Vect, min : Float, max : Float) : SlideJoint
  
  fun slide_joint_new = cpSlideJointNew(a : Body, b : Body, anchor_a : Vect, anchor_b : Vect, min : Float, max : Float) : Constraint
  
  fun slide_joint_get_anchor_a = cpSlideJointGetAnchorA(constraint : Constraint) : Vect
  
  fun slide_joint_set_anchor_a = cpSlideJointSetAnchorA(constraint : Constraint, anchor_a : Vect)
  
  fun slide_joint_get_anchor_b = cpSlideJointGetAnchorB(constraint : Constraint) : Vect
  
  fun slide_joint_set_anchor_b = cpSlideJointSetAnchorB(constraint : Constraint, anchor_b : Vect)
  
  fun slide_joint_get_min = cpSlideJointGetMin(constraint : Constraint) : Float
  
  fun slide_joint_set_min = cpSlideJointSetMin(constraint : Constraint, min : Float)
  
  fun slide_joint_get_max = cpSlideJointGetMax(constraint : Constraint) : Float
  
  fun slide_joint_set_max = cpSlideJointSetMax(constraint : Constraint, max : Float)
  
  fun constraint_is_pivot_joint = cpConstraintIsPivotJoint(constraint : Constraint) : Bool
  
  fun pivot_joint_alloc = cpPivotJointAlloc() : PivotJoint
  
  fun pivot_joint_init = cpPivotJointInit(joint : PivotJoint, a : Body, b : Body, anchor_a : Vect, anchor_b : Vect) : PivotJoint
  
  fun pivot_joint_new = cpPivotJointNew(a : Body, b : Body, pivot : Vect) : Constraint
  
  fun pivot_joint_new2 = cpPivotJointNew2(a : Body, b : Body, anchor_a : Vect, anchor_b : Vect) : Constraint
  
  fun pivot_joint_get_anchor_a = cpPivotJointGetAnchorA(constraint : Constraint) : Vect
  
  fun pivot_joint_set_anchor_a = cpPivotJointSetAnchorA(constraint : Constraint, anchor_a : Vect)
  
  fun pivot_joint_get_anchor_b = cpPivotJointGetAnchorB(constraint : Constraint) : Vect
  
  fun pivot_joint_set_anchor_b = cpPivotJointSetAnchorB(constraint : Constraint, anchor_b : Vect)
  
  fun constraint_is_groove_joint = cpConstraintIsGrooveJoint(constraint : Constraint) : Bool
  
  fun groove_joint_alloc = cpGrooveJointAlloc() : GrooveJoint
  
  fun groove_joint_init = cpGrooveJointInit(joint : GrooveJoint, a : Body, b : Body, groove_a : Vect, groove_b : Vect, anchor_b : Vect) : GrooveJoint
  
  fun groove_joint_new = cpGrooveJointNew(a : Body, b : Body, groove_a : Vect, groove_b : Vect, anchor_b : Vect) : Constraint
  
  fun groove_joint_get_groove_a = cpGrooveJointGetGrooveA(constraint : Constraint) : Vect
  
  fun groove_joint_set_groove_a = cpGrooveJointSetGrooveA(constraint : Constraint, groove_a : Vect)
  
  fun groove_joint_get_groove_b = cpGrooveJointGetGrooveB(constraint : Constraint) : Vect
  
  fun groove_joint_set_groove_b = cpGrooveJointSetGrooveB(constraint : Constraint, groove_b : Vect)
  
  fun groove_joint_get_anchor_b = cpGrooveJointGetAnchorB(constraint : Constraint) : Vect
  
  fun groove_joint_set_anchor_b = cpGrooveJointSetAnchorB(constraint : Constraint, anchor_b : Vect)
  
  fun constraint_is_damped_spring = cpConstraintIsDampedSpring(constraint : Constraint) : Bool
  
  alias DampedSpringForceFunc = (Constraint, Float) -> Float
  
  fun damped_spring_alloc = cpDampedSpringAlloc() : DampedSpring
  
  fun damped_spring_init = cpDampedSpringInit(joint : DampedSpring, a : Body, b : Body, anchor_a : Vect, anchor_b : Vect, rest_length : Float, stiffness : Float, damping : Float) : DampedSpring
  
  fun damped_spring_new = cpDampedSpringNew(a : Body, b : Body, anchor_a : Vect, anchor_b : Vect, rest_length : Float, stiffness : Float, damping : Float) : Constraint
  
  fun damped_spring_get_anchor_a = cpDampedSpringGetAnchorA(constraint : Constraint) : Vect
  
  fun damped_spring_set_anchor_a = cpDampedSpringSetAnchorA(constraint : Constraint, anchor_a : Vect)
  
  fun damped_spring_get_anchor_b = cpDampedSpringGetAnchorB(constraint : Constraint) : Vect
  
  fun damped_spring_set_anchor_b = cpDampedSpringSetAnchorB(constraint : Constraint, anchor_b : Vect)
  
  fun damped_spring_get_rest_length = cpDampedSpringGetRestLength(constraint : Constraint) : Float
  
  fun damped_spring_set_rest_length = cpDampedSpringSetRestLength(constraint : Constraint, rest_length : Float)
  
  fun damped_spring_get_stiffness = cpDampedSpringGetStiffness(constraint : Constraint) : Float
  
  fun damped_spring_set_stiffness = cpDampedSpringSetStiffness(constraint : Constraint, stiffness : Float)
  
  fun damped_spring_get_damping = cpDampedSpringGetDamping(constraint : Constraint) : Float
  
  fun damped_spring_set_damping = cpDampedSpringSetDamping(constraint : Constraint, damping : Float)
  
  fun damped_spring_get_spring_force_func = cpDampedSpringGetSpringForceFunc(constraint : Constraint) : DampedSpringForceFunc
  
  fun damped_spring_set_spring_force_func = cpDampedSpringSetSpringForceFunc(constraint : Constraint, spring_force_func : DampedSpringForceFunc)
  
  fun constraint_is_damped_rotary_spring = cpConstraintIsDampedRotarySpring(constraint : Constraint) : Bool
  
  alias DampedRotarySpringTorqueFunc = (Constraint, Float) -> Float
  
  fun damped_rotary_spring_alloc = cpDampedRotarySpringAlloc() : DampedRotarySpring
  
  fun damped_rotary_spring_init = cpDampedRotarySpringInit(joint : DampedRotarySpring, a : Body, b : Body, rest_angle : Float, stiffness : Float, damping : Float) : DampedRotarySpring
  
  fun damped_rotary_spring_new = cpDampedRotarySpringNew(a : Body, b : Body, rest_angle : Float, stiffness : Float, damping : Float) : Constraint
  
  fun damped_rotary_spring_get_rest_angle = cpDampedRotarySpringGetRestAngle(constraint : Constraint) : Float
  
  fun damped_rotary_spring_set_rest_angle = cpDampedRotarySpringSetRestAngle(constraint : Constraint, rest_angle : Float)
  
  fun damped_rotary_spring_get_stiffness = cpDampedRotarySpringGetStiffness(constraint : Constraint) : Float
  
  fun damped_rotary_spring_set_stiffness = cpDampedRotarySpringSetStiffness(constraint : Constraint, stiffness : Float)
  
  fun damped_rotary_spring_get_damping = cpDampedRotarySpringGetDamping(constraint : Constraint) : Float
  
  fun damped_rotary_spring_set_damping = cpDampedRotarySpringSetDamping(constraint : Constraint, damping : Float)
  
  fun damped_rotary_spring_get_spring_torque_func = cpDampedRotarySpringGetSpringTorqueFunc(constraint : Constraint) : DampedRotarySpringTorqueFunc
  
  fun damped_rotary_spring_set_spring_torque_func = cpDampedRotarySpringSetSpringTorqueFunc(constraint : Constraint, spring_torque_func : DampedRotarySpringTorqueFunc)
  
  fun constraint_is_rotary_limit_joint = cpConstraintIsRotaryLimitJoint(constraint : Constraint) : Bool
  
  fun rotary_limit_joint_alloc = cpRotaryLimitJointAlloc() : RotaryLimitJoint
  
  fun rotary_limit_joint_init = cpRotaryLimitJointInit(joint : RotaryLimitJoint, a : Body, b : Body, min : Float, max : Float) : RotaryLimitJoint
  
  fun rotary_limit_joint_new = cpRotaryLimitJointNew(a : Body, b : Body, min : Float, max : Float) : Constraint
  
  fun rotary_limit_joint_get_min = cpRotaryLimitJointGetMin(constraint : Constraint) : Float
  
  fun rotary_limit_joint_set_min = cpRotaryLimitJointSetMin(constraint : Constraint, min : Float)
  
  fun rotary_limit_joint_get_max = cpRotaryLimitJointGetMax(constraint : Constraint) : Float
  
  fun rotary_limit_joint_set_max = cpRotaryLimitJointSetMax(constraint : Constraint, max : Float)
  
  fun constraint_is_ratchet_joint = cpConstraintIsRatchetJoint(constraint : Constraint) : Bool
  
  fun ratchet_joint_alloc = cpRatchetJointAlloc() : RatchetJoint
  
  fun ratchet_joint_init = cpRatchetJointInit(joint : RatchetJoint, a : Body, b : Body, phase : Float, ratchet : Float) : RatchetJoint
  
  fun ratchet_joint_new = cpRatchetJointNew(a : Body, b : Body, phase : Float, ratchet : Float) : Constraint
  
  fun ratchet_joint_get_angle = cpRatchetJointGetAngle(constraint : Constraint) : Float
  
  fun ratchet_joint_set_angle = cpRatchetJointSetAngle(constraint : Constraint, angle : Float)
  
  fun ratchet_joint_get_phase = cpRatchetJointGetPhase(constraint : Constraint) : Float
  
  fun ratchet_joint_set_phase = cpRatchetJointSetPhase(constraint : Constraint, phase : Float)
  
  fun ratchet_joint_get_ratchet = cpRatchetJointGetRatchet(constraint : Constraint) : Float
  
  fun ratchet_joint_set_ratchet = cpRatchetJointSetRatchet(constraint : Constraint, ratchet : Float)
  
  fun constraint_is_gear_joint = cpConstraintIsGearJoint(constraint : Constraint) : Bool
  
  fun gear_joint_alloc = cpGearJointAlloc() : GearJoint
  
  fun gear_joint_init = cpGearJointInit(joint : GearJoint, a : Body, b : Body, phase : Float, ratio : Float) : GearJoint
  
  fun gear_joint_new = cpGearJointNew(a : Body, b : Body, phase : Float, ratio : Float) : Constraint
  
  fun gear_joint_get_phase = cpGearJointGetPhase(constraint : Constraint) : Float
  
  fun gear_joint_set_phase = cpGearJointSetPhase(constraint : Constraint, phase : Float)
  
  fun gear_joint_get_ratio = cpGearJointGetRatio(constraint : Constraint) : Float
  
  fun gear_joint_set_ratio = cpGearJointSetRatio(constraint : Constraint, ratio : Float)
  
  type SimpleMotor = Void*
  
  fun constraint_is_simple_motor = cpConstraintIsSimpleMotor(constraint : Constraint) : Bool
  
  fun simple_motor_alloc = cpSimpleMotorAlloc() : SimpleMotor
  
  fun simple_motor_init = cpSimpleMotorInit(joint : SimpleMotor, a : Body, b : Body, rate : Float) : SimpleMotor
  
  fun simple_motor_new = cpSimpleMotorNew(a : Body, b : Body, rate : Float) : Constraint
  
  fun simple_motor_get_rate = cpSimpleMotorGetRate(constraint : Constraint) : Float
  
  fun simple_motor_set_rate = cpSimpleMotorSetRate(constraint : Constraint, rate : Float)
  
  alias CollisionBeginFunc = (Arbiter, Space, DataPointer) -> Bool
  
  alias CollisionPreSolveFunc = (Arbiter, Space, DataPointer) -> Bool
  
  alias CollisionPostSolveFunc = (Arbiter, Space, DataPointer) ->
  
  alias CollisionSeparateFunc = (Arbiter, Space, DataPointer) ->
  
  struct CollisionHandler
    type_a : CollisionType
    type_b : CollisionType
    begin_func : CollisionBeginFunc
    pre_solve_func : CollisionPreSolveFunc
    post_solve_func : CollisionPostSolveFunc
    separate_func : CollisionSeparateFunc
    user_data : DataPointer
  end
  
  fun space_alloc = cpSpaceAlloc() : Space
  
  fun space_init = cpSpaceInit(space : Space) : Space
  
  fun space_new = cpSpaceNew() : Space
  
  fun space_destroy = cpSpaceDestroy(space : Space)
  
  fun space_free = cpSpaceFree(space : Space)
  
  fun space_get_iterations = cpSpaceGetIterations(space : Space) : Int32
  
  fun space_set_iterations = cpSpaceSetIterations(space : Space, iterations : Int32)
  
  fun space_get_gravity = cpSpaceGetGravity(space : Space) : Vect
  
  fun space_set_gravity = cpSpaceSetGravity(space : Space, gravity : Vect)
  
  fun space_get_damping = cpSpaceGetDamping(space : Space) : Float
  
  fun space_set_damping = cpSpaceSetDamping(space : Space, damping : Float)
  
  fun space_get_idle_speed_threshold = cpSpaceGetIdleSpeedThreshold(space : Space) : Float
  
  fun space_set_idle_speed_threshold = cpSpaceSetIdleSpeedThreshold(space : Space, idle_speed_threshold : Float)
  
  fun space_get_sleep_time_threshold = cpSpaceGetSleepTimeThreshold(space : Space) : Float
  
  fun space_set_sleep_time_threshold = cpSpaceSetSleepTimeThreshold(space : Space, sleep_time_threshold : Float)
  
  fun space_get_collision_slop = cpSpaceGetCollisionSlop(space : Space) : Float
  
  fun space_set_collision_slop = cpSpaceSetCollisionSlop(space : Space, collision_slop : Float)
  
  fun space_get_collision_bias = cpSpaceGetCollisionBias(space : Space) : Float
  
  fun space_set_collision_bias = cpSpaceSetCollisionBias(space : Space, collision_bias : Float)
  
  fun space_get_collision_persistence = cpSpaceGetCollisionPersistence(space : Space) : Timestamp
  
  fun space_set_collision_persistence = cpSpaceSetCollisionPersistence(space : Space, collision_persistence : Timestamp)
  
  fun space_get_user_data = cpSpaceGetUserData(space : Space) : DataPointer
  
  fun space_set_user_data = cpSpaceSetUserData(space : Space, user_data : DataPointer)
  
  fun space_get_static_body = cpSpaceGetStaticBody(space : Space) : Body
  
  fun space_get_current_time_step = cpSpaceGetCurrentTimeStep(space : Space) : Float
  
  fun space_is_locked = cpSpaceIsLocked(space : Space) : Bool
  
  fun space_add_default_collision_handler = cpSpaceAddDefaultCollisionHandler(space : Space) : CollisionHandler*
  
  fun space_add_collision_handler = cpSpaceAddCollisionHandler(space : Space, a : CollisionType, b : CollisionType) : CollisionHandler*
  
  fun space_add_wildcard_handler = cpSpaceAddWildcardHandler(space : Space, type : CollisionType) : CollisionHandler*
  
  fun space_add_shape = cpSpaceAddShape(space : Space, shape : Shape) : Shape
  
  fun space_add_body = cpSpaceAddBody(space : Space, body : Body) : Body
  
  fun space_add_constraint = cpSpaceAddConstraint(space : Space, constraint : Constraint) : Constraint
  
  fun space_remove_shape = cpSpaceRemoveShape(space : Space, shape : Shape)
  
  fun space_remove_body = cpSpaceRemoveBody(space : Space, body : Body)
  
  fun space_remove_constraint = cpSpaceRemoveConstraint(space : Space, constraint : Constraint)
  
  fun space_contains_shape = cpSpaceContainsShape(space : Space, shape : Shape) : Bool
  
  fun space_contains_body = cpSpaceContainsBody(space : Space, body : Body) : Bool
  
  fun space_contains_constraint = cpSpaceContainsConstraint(space : Space, constraint : Constraint) : Bool
  
  alias PostStepFunc = (Space, Void*, Void*) ->
  
  fun space_add_post_step_callback = cpSpaceAddPostStepCallback(space : Space, func : PostStepFunc, key : Void*, data : Void*) : Bool
  
  alias SpacePointQueryFunc = (Shape, Vect, Float, Vect, Void*) ->
  
  fun space_point_query = cpSpacePointQuery(space : Space, point : Vect, max_distance : Float, filter : ShapeFilter, func : SpacePointQueryFunc, data : Void*)
  
  fun space_point_query_nearest = cpSpacePointQueryNearest(space : Space, point : Vect, max_distance : Float, filter : ShapeFilter, out : PointQueryInfo*) : Shape
  
  alias SpaceSegmentQueryFunc = (Shape, Vect, Vect, Float, Void*) ->
  
  fun space_segment_query = cpSpaceSegmentQuery(space : Space, start : Vect, end_ : Vect, radius : Float, filter : ShapeFilter, func : SpaceSegmentQueryFunc, data : Void*)
  
  fun space_segment_query_first = cpSpaceSegmentQueryFirst(space : Space, start : Vect, end_ : Vect, radius : Float, filter : ShapeFilter, out : SegmentQueryInfo*) : Shape
  
  alias SpaceBBQueryFunc = (Shape, Void*) ->
  
  fun space_bb_query = cpSpaceBBQuery(space : Space, bb : BB, filter : ShapeFilter, func : SpaceBBQueryFunc, data : Void*)
  
  alias SpaceShapeQueryFunc = (Shape, ContactPointSet*, Void*) ->
  
  fun space_shape_query = cpSpaceShapeQuery(space : Space, shape : Shape, func : SpaceShapeQueryFunc, data : Void*) : Bool
  
  alias SpaceBodyIteratorFunc = (Body, Void*) ->
  
  fun space_each_body = cpSpaceEachBody(space : Space, func : SpaceBodyIteratorFunc, data : Void*)
  
  alias SpaceShapeIteratorFunc = (Shape, Void*) ->
  
  fun space_each_shape = cpSpaceEachShape(space : Space, func : SpaceShapeIteratorFunc, data : Void*)
  
  alias SpaceConstraintIteratorFunc = (Constraint, Void*) ->
  
  fun space_each_constraint = cpSpaceEachConstraint(space : Space, func : SpaceConstraintIteratorFunc, data : Void*)
  
  fun space_reindex_static = cpSpaceReindexStatic(space : Space)
  
  fun space_reindex_shape = cpSpaceReindexShape(space : Space, shape : Shape)
  
  fun space_reindex_shapes_for_body = cpSpaceReindexShapesForBody(space : Space, body : Body)
  
  fun space_use_spatial_hash = cpSpaceUseSpatialHash(space : Space, dim : Float, count : Int32)
  
  fun space_step = cpSpaceStep(space : Space, dt : Float)
  
  struct SpaceDebugColor
    r : Float32
    g : Float32
    b : Float32
    a : Float32
  end
  
  alias SpaceDebugDrawCircleImpl = (Vect, Float, Float, SpaceDebugColor, SpaceDebugColor, DataPointer) ->
  
  alias SpaceDebugDrawSegmentImpl = (Vect, Vect, SpaceDebugColor, DataPointer) ->
  
  alias SpaceDebugDrawFatSegmentImpl = (Vect, Vect, Float, SpaceDebugColor, SpaceDebugColor, DataPointer) ->
  
  alias SpaceDebugDrawPolygonImpl = (Int32, Vect*, Float, SpaceDebugColor, SpaceDebugColor, DataPointer) ->
  
  alias SpaceDebugDrawDotImpl = (Float, Vect, SpaceDebugColor, DataPointer) ->
  
  alias SpaceDebugDrawColorForShapeImpl = (Shape, DataPointer) -> SpaceDebugColor
  
  enum SpaceDebugDrawFlags
    SPACE_DEBUG_DRAW_SHAPES = 1 << 0
    SPACE_DEBUG_DRAW_CONSTRAINTS = 1 << 1
    SPACE_DEBUG_DRAW_COLLISION_POINTS = 1 << 2
  end
  
  struct SpaceDebugDrawOptions
    draw_circle : SpaceDebugDrawCircleImpl
    draw_segment : SpaceDebugDrawSegmentImpl
    draw_fat_segment : SpaceDebugDrawFatSegmentImpl
    draw_polygon : SpaceDebugDrawPolygonImpl
    draw_dot : SpaceDebugDrawDotImpl
    flags : SpaceDebugDrawFlags
    shape_outline_color : SpaceDebugColor
    color_for_shape : SpaceDebugDrawColorForShapeImpl
    constraint_color : SpaceDebugColor
    collision_point_color : SpaceDebugColor
    data : DataPointer
  end
  
  fun space_debug_draw = cpSpaceDebugDraw(space : Space, options : SpaceDebugDrawOptions*)
  
  VERSION_MAJOR = 7
  
  VERSION_MINOR = 0
  
  VERSION_RELEASE = 0
  
  fun moment_for_circle = cpMomentForCircle(m : Float, r1 : Float, r2 : Float, offset : Vect) : Float
  
  fun area_for_circle = cpAreaForCircle(r1 : Float, r2 : Float) : Float
  
  fun moment_for_segment = cpMomentForSegment(m : Float, a : Vect, b : Vect, radius : Float) : Float
  
  fun area_for_segment = cpAreaForSegment(a : Vect, b : Vect, radius : Float) : Float
  
  fun moment_for_poly = cpMomentForPoly(m : Float, count : Int32, verts : Vect*, offset : Vect, radius : Float) : Float
  
  fun area_for_poly = cpAreaForPoly(count : Int32, verts : Vect*, radius : Float) : Float
  
  fun centroid_for_poly = cpCentroidForPoly(count : Int32, verts : Vect*) : Vect
  
  fun moment_for_box = cpMomentForBox(m : Float, width : Float, height : Float) : Float
  
  fun moment_for_box2 = cpMomentForBox2(m : Float, box : BB) : Float
  
  fun convex_hull = cpConvexHull(count : Int32, verts : Vect*, result : Vect*, first : Int32*, tol : Float) : Int32
end

module Chipmunk
  extend self

  def fmax(a : Number, b : Number) : Number
    a > b ? a : b
  end

  def fmin(a : Number, b : Number) : Number
    a < b ? a : b
  end

  def fabs(f : Number) : Number
    f < 0 ? -f : f
  end

  def fclamp(f : Number, min : Number, max : Number) : Number
    fmin(fmax(f, min), max)
  end

  def fclamp01(f : Number) : Number
    fmax(0.0, fmin(f, 1.0))
  end

  def flerp(f1 : Number, f2 : Number, t : Number) : Number
    (f1 * (1.0 - t)) + (f2 * t)
  end

  def flerpconst(f1 : Number, f2 : Number, d : Number) : Number
    f1 + fclamp(f2 - f1, -d, d)
  end

  def v(x : Number, y : Number) : CP::Vect
    CP::Vect.new(x: x.to_f, y: y.to_f)
  end

  def veql(v1 : CP::Vect, v2 : CP::Vect) : Bool
    (v1.x == v2.x) && (v1.y == v2.y)
  end

  def vadd(v1 : CP::Vect, v2 : CP::Vect) : CP::Vect
    v(v1.x + v2.x, v1.y + v2.y)
  end

  def vsub(v1 : CP::Vect, v2 : CP::Vect) : CP::Vect
    v(v1.x - v2.x, v1.y - v2.y)
  end

  def vneg(v : CP::Vect) : CP::Vect
    v(-v.x, -v.y)
  end

  def vmult(v : CP::Vect, s : Number) : CP::Vect
    v(v.x * s, v.y * s)
  end

  def vdot(v1 : CP::Vect, v2 : CP::Vect) : CP::Float
    (v1.x * v2.x) + (v1.y * v2.y)
  end

  def vcross(v1 : CP::Vect, v2 : CP::Vect) : CP::Float
    (v1.x * v2.y) - (v1.y * v2.x)
  end

  def vperp(v : CP::Vect) : CP::Vect
    v(-v.y, v.x)
  end

  def vrperp(v : CP::Vect) : CP::Vect
    v(v.y, -v.x)
  end

  def vproject(v1 : CP::Vect, v2 : CP::Vect) : CP::Vect
    vmult(v2, vdot(v1, v2) / vdot(v2, v2))
  end

  def vforangle(a : Number) : CP::Vect
    v(Math.cos(a), Math.sin(a))
  end

  def vtoangle(v : CP::Vect) : Float64
    Math.atan2(v.y, v.x)
  end

  def vrotate(v1 : CP::Vect, v2 : CP::Vect) : CP::Vect
    v((v1.x * v2.x) - (v1.y * v2.y), (v1.x * v2.y) + (v1.y * v2.x))
  end

  def vunrotate(v1 : CP::Vect, v2 : CP::Vect) : CP::Vect
    v((v1.x * v2.x) + (v1.y * v2.y), (v1.y * v2.x) - (v1.x * v2.y))
  end

  def vlengthsq(v : CP::Vect) : CP::Float
    vdot(v, v)
  end

  def vlength(v : CP::Vect) : Float64
    Math.sqrt(vdot(v, v))
  end

  def vlerp(v1 : CP::Vect, v2 : CP::Vect, t : Number) : CP::Vect
    vadd(vmult(v1, 1.0 - t), vmult(v2, t))
  end

  def vnormalize(v : CP::Vect) : CP::Vect
    vmult(v, 1.0 / (vlength(v) + Float64::MIN))
  end

  def vslerp(v1 : CP::Vect, v2 : CP::Vect, t : Number) : CP::Vect
    dot = vdot(vnormalize(v1), vnormalize(v2))
    omega = Math.acos(fclamp(dot, -1.0, 1.0))
    if omega < 1e-3
      vlerp(v1, v2, t)
    else
      denom = 1.0 / Math.sin(omega)
      vadd(vmult(v1, Math.sin((1.0 - t) * omega) * denom), vmult(v2, Math.sin(t * omega) * denom))
    end
  end

  def vslerpconst(v1 : CP::Vect, v2 : CP::Vect, a : Number) : CP::Vect
    dot = vdot(vnormalize(v1), vnormalize(v2))
    omega = Math.acos(fclamp(dot, -1.0, 1.0))
    vslerp(v1, v2, fmin(a, omega) / omega)
  end

  def vclamp(v : CP::Vect, len : Number) : CP::Vect
    vdot(v, v) > (len * len) ? vmult(vnormalize(v), len) : v
  end

  def vlerpconst(v1 : CP::Vect, v2 : CP::Vect, d : Number) : CP::Vect
    vadd(v1, vclamp(vsub(v2, v1), d))
  end

  def vdist(v1 : CP::Vect, v2 : CP::Vect) : Float64
    vlength(vsub(v1, v2))
  end

  def vdistsq(v1 : CP::Vect, v2 : CP::Vect) : Float64
    vlengthsq(vsub(v1, v2))
  end

  def vnear(v1 : CP::Vect, v2 : CP::Vect, dist : Number) : Bool
    vdistsq(v1, v2) < (dist * dist)
  end

  def mat2x2_new(a : Number, b : Number, c : Number, d : Number) : CP::Mat2x2
    CP::Mat2x2.new(a: a.to_f, b: b.to_f, c: c.to_f, d: d.to_f)
  end

  def mat2x2_transform(m : CP::Mat2x2, v : CP::Vect) : CP::Vect
    v((v.x * m.a) + (v.y * m.b), (v.x * m.c) + (v.y * m.d))
  end

  def bb_new(l : Number, b : Number, r : Number, t : Number) : CP::BB
    CP::BB.new(l: l.to_f, b: b.to_f, r: r.to_f, t: t.to_f)
  end

  def bb_new_for_extents(c : CP::Vect, hw : Number, hh : Number) : CP::BB
    bb_new(c.x - hw, c.y - hh, c.x + hw, c.y + hh)
  end

  def bb_new_for_circle(p : CP::Vect, r : Number) : CP::BB
    bb_new_for_extents(p, r, r)
  end

  def bb_intersects(a : CP::BB, b : CP::BB) : Bool
    (((a.l <= b.r) && (b.l <= a.r)) && (a.b <= b.t)) && (b.b <= a.t)
  end

  def bb_contains_bb(bb : CP::BB, other : CP::BB) : Bool
    (((bb.l <= other.l) && (bb.r >= other.r)) && (bb.b <= other.b)) && (bb.t >= other.t)
  end

  def bb_contains_vect(bb : CP::BB, v : CP::Vect) : Bool
    (((bb.l <= v.x) && (bb.r >= v.x)) && (bb.b <= v.y)) && (bb.t >= v.y)
  end

  def bb_merge(a : CP::BB, b : CP::BB) : CP::BB
    bb_new(fmin(a.l, b.l), fmin(a.b, b.b), fmax(a.r, b.r), fmax(a.t, b.t))
  end

  def bb_expand(bb : CP::BB, v : CP::Vect) : CP::BB
    bb_new(fmin(bb.l, v.x), fmin(bb.b, v.y), fmax(bb.r, v.x), fmax(bb.t, v.y))
  end

  def bb_center(bb : CP::BB) : CP::Vect
    vlerp(v(bb.l, bb.b), v(bb.r, bb.t), 0.5)
  end

  def bb_area(bb : CP::BB) : Float64
    (bb.r - bb.l) * (bb.t - bb.b)
  end

  def bb_merged_area(a : CP::BB, b : CP::BB) : Float64
    (fmax(a.r, b.r) - fmin(a.l, b.l)) * (fmax(a.t, b.t) - fmin(a.b, b.b))
  end

  def bb_segment_query(bb : CP::BB, a : CP::Vect, b : CP::Vect) : Float64
    idx = 1.0 / (b.x - a.x)
    tx1 = bb.l == a.x ? -1e1000 : (bb.l - a.x) * idx
    tx2 = bb.r == a.x ? 1e1000 : (bb.r - a.x) * idx
    txmin = fmin(tx1, tx2)
    txmax = fmax(tx1, tx2)
    idy = 1.0 / (b.y - a.y)
    ty1 = bb.b == a.y ? -1e1000 : (bb.b - a.y) * idy
    ty2 = bb.t == a.y ? 1e1000 : (bb.t - a.y) * idy
    tymin = fmin(ty1, ty2)
    tymax = fmax(ty1, ty2)
    if (tymin <= txmax) && (txmin <= tymax)
      min = fmax(txmin, tymin)
      max = fmin(txmax, tymax)
      if (0.0 <= max) && (min <= 1.0)
        return fmax(min, 0.0)
      end
    end
    1e1000
  end

  def bb_intersects_segment(bb : CP::BB, a : CP::Vect, b : CP::Vect) : Bool
    bb_segment_query(bb, a, b) != 1e1000
  end

  def bb_clamp_vect(bb : CP::BB, v : CP::Vect) : CP::Vect
    v(fclamp(v.x, bb.l, bb.r), fclamp(v.y, bb.b, bb.t))
  end

  def bb_wrap_vect(bb : CP::BB, v : CP::Vect) : CP::Vect
    dx = fabs(bb.r - bb.l)
    modx = (v.x - bb.l).fdiv(dx)
    x = modx > 0.0 ? modx : modx + dx
    dy = fabs(bb.t - bb.b)
    mody = (v.y - bb.b).fdiv(dy)
    y = mody > 0.0 ? mody : mody + dy
    v(x + bb.l, y + bb.b)
  end

  def bb_offset(bb : CP::BB, v : CP::Vect) : CP::BB
    bb_new(bb.l + v.x, bb.b + v.y, bb.r + v.x, bb.t + v.y)
  end
  
  def transform_new(a : Number, b : Number, c : Number, d : Number, tx : Number, ty : Number) : CP::Transform
    CP::Transform.new(a: a.to_f, b: b.to_f, c: c.to_f, d: d.to_f, tx: tx.to_f, ty: ty.to_f)
  end

  def transform_new_transpose(a : Number, c : Number, tx : Number, b : Number, d : Number, ty : Number) : CP::Transform
    CP::Transform.new(a: a.to_f, b: b.to_f, c: c.to_f, d: d.to_f, tx: tx.to_f, ty: ty.to_f)
  end

  def transform_inverse(t : CP::Transform) : CP::Transform
    inv_det = 1.0 / ((t.a * t.d) - (t.c * t.b))
    transform_new_transpose(t.d * inv_det, (-t.c) * inv_det, ((t.c * t.ty) - (t.tx * t.d)) * inv_det, (-t.b) * inv_det, t.a * inv_det, ((t.tx * t.b) - (t.a * t.ty)) * inv_det)
  end

  def transform_mult(t1 : CP::Transform, t2 : CP::Transform) : CP::Transform
    transform_new_transpose((t1.a * t2.a) + (t1.c * t2.b), (t1.a * t2.c) + (t1.c * t2.d), ((t1.a * t2.tx) + (t1.c * t2.ty)) + t1.tx, (t1.b * t2.a) + (t1.d * t2.b), (t1.b * t2.c) + (t1.d * t2.d), ((t1.b * t2.tx) + (t1.d * t2.ty)) + t1.ty)
  end

  def transform_point(t : CP::Transform, p : CP::Vect) : CP::Vect
    v(((t.a * p.x) + (t.c * p.y)) + t.tx, ((t.b * p.x) + (t.d * p.y)) + t.ty)
  end

  def transform_vect(t : CP::Transform, v : CP::Vect) : CP::Vect
    v((t.a * v.x) + (t.c * v.y), (t.b * v.x) + (t.d * v.y))
  end

  def transform_bb(t : CP::Transform, bb : CP::BB) : CP::BB
    center = bb_center(bb)
    hw = (bb.r - bb.l) * 0.5
    hh = (bb.t - bb.b) * 0.5
    a = t.a * hw
    b = t.c * hh
    d = t.b * hw
    e = t.d * hh
    hw_max = fmax(fabs(a + b), fabs(a - b))
    hh_max = fmax(fabs(d + e), fabs(d - e))
    bb_new_for_extents(transform_point(t, center), hw_max, hh_max)
  end

  def transform_translate(translate : CP::Vect) : CP::Transform
    transform_new_transpose(1.0, 0.0, translate.x, 0.0, 1.0, translate.y)
  end

  def transform_scale(scale_x : Number, scale_y : Number) : CP::Transform
    transform_new_transpose(scale_x, 0.0, 0.0, 0.0, scale_y, 0.0)
  end

  def transform_rotate(radians : Number) : CP::Transform
    rot = vforangle(radians)
    transform_new_transpose(rot.x, -rot.y, 0.0, rot.y, rot.x, 0.0)
  end

  def transform_rigid(translate : CP::Vect, radians : Number) : CP::Transform
    rot = vforangle(radians)
    transform_new_transpose(rot.x, -rot.y, translate.x, rot.y, rot.x, translate.y)
  end

  def transform_rigid_inverse(t : CP::Transform) : CP::Transform
    transform_new_transpose(t.d, -t.c, (t.c * t.ty) - (t.tx * t.d), -t.b, t.a, (t.tx * t.b) - (t.a * t.ty))
  end

  def transform_wrap(outer : CP::Transform, inner : CP::Transform) : CP::Transform
    transform_mult(transform_inverse(outer), transform_mult(inner, outer))
  end

  def transform_wrap_inverse(outer : CP::Transform, inner : CP::Transform) : CP::Transform
    transform_mult(outer, transform_mult(inner, transform_inverse(outer)))
  end

  def transform_ortho(bb : CP::BB) : CP::Transform
    transform_new_transpose(2.0 / (bb.r - bb.l), 0.0, (-(bb.r + bb.l)) / (bb.r - bb.l), 0.0, 2.0 / (bb.t - bb.b), (-(bb.t + bb.b)) / (bb.t - bb.b))
  end

  def transform_bone_scale(v0 : CP::Vect, v1 : CP::Vect) : CP::Transform
    d = vsub(v1, v0)
    transform_new_transpose(d.x, -d.y, v0.x, d.y, d.x, v0.y)
  end

  def transform_axial_scale(axis : CP::Vect, pivot : CP::Vect, scale : Number) : CP::Transform
    a = (axis.x * axis.y) * (scale - 1.0)
    b = vdot(axis, pivot) * (1.0 - scale)
    transform_new_transpose(((scale * axis.x) * axis.x) + (axis.y * axis.y), a, axis.x * b, a, (axis.x * axis.x) + ((scale * axis.y) * axis.y), axis.y * b)
  end

  def spatial_index_destroy(index : SpatialIndex*)
    if (klass = index.value.klass)
      klass.value.destroy.call(index)
    end
  end

  def spatial_index_count(index : SpatialIndex*) : Int32
    index.value.klass.value.count.call(index)
  end

  def spatial_index_each(index : SpatialIndex*, func : SpatialIndexIteratorFunc, data : Void*)
    index.value.klass.value.each.call(index, func, data)
  end

  def spatial_index_contains(index : SpatialIndex*, obj : Void*, hashid : HashValue) : Bool
    index.value.klass.value.contains.call(index, obj, hashid)
  end

  def spatial_index_insert(index : SpatialIndex*, obj : Void*, hashid : HashValue)
    index.value.klass.value.insert.call(index, obj, hashid)
  end

  def spatial_index_remove(index : SpatialIndex*, obj : Void*, hashid : HashValue)
    index.value.klass.value.remove.call(index, obj, hashid)
  end

  def spatial_index_reindex(index : SpatialIndex*)
    index.value.klass.value.reindex.call(index)
  end

  def spatial_index_reindex_object(index : SpatialIndex*, obj : Void*, hashid : HashValue)
    index.value.klass.value.reindex_object.call(index, obj, hashid)
  end

  def spatial_index_query(index : SpatialIndex*, obj : Void*, bb : CP::BB, func : SpatialIndexQueryFunc, data : Void*)
    index.value.klass.value.query.call(index, obj, bb, func, data)
  end

  def spatial_index_segment_query(index : SpatialIndex*, obj : Void*, a : CP::Vect, b : CP::Vect, t_exit : Float, func : SpatialIndexSegmentQueryFunc, data : Void*)
    index.value.klass.value.segment_query.call(index, obj, a, b, t_exit, func, data)
  end

  def spatial_index_reindex_query(index : SpatialIndex*, func : SpatialIndexQueryFunc, data : Void*)
    index.value.klass.value.reindex_query.call(index, func, data)
  end

  def shape_filter_new(group : CP::Group, categories : CP::Bitmask, mask : CP::Bitmask) : CP::ShapeFilter
    CP::ShapeFilter.new(group: group, categories: categories, mask: mask)
  end

  def closest_point_on_segment(p : CP::Vect, a : CP::Vect, b : CP::Vect) : CP::Vect
    delta = vsub(a, b)
    t = fclamp01(vdot(delta, vsub(p, b)) / vlengthsq(delta))
    vadd(b, vmult(delta, t))
  end
end
