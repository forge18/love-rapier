// C-ABI shim over Rapier2D, bound for LuaJIT FFI (the `rapier` Lua module wraps this).
//
// Everything crosses the boundary as opaque pointers + primitives + #[repr(C)] structs — never
// Rust types (no String/Vec/enums/generics), because LuaJIT's FFI binds the C ABI, not Rust's.
// Rapier handles are (index, generation); we pack them into a u64 for the boundary.
//
// Surface (bound in full — the build cost is fixed regardless of how much is exposed):
//   * world: step, gravity, solver iterations
//   * bodies: all types (dynamic/fixed/kinematic-vel/kinematic-pos), linear + angular velocity,
//     impulse/force/torque (incl. at-a-point), damping, gravity scale, mass, CCD, sleeping, enable,
//     dominance, runtime type change
//   * colliders: ball/cuboid/capsule + triangle/segment/polyline/convex-hull/heightfield, attached
//     or static, sensors, friction/restitution/density/mass, parent offsets, collision + solver
//     groups, enable
//   * events: collision + sensor, drained each step
//   * queries: raycast (+ normal), point, shape-cast, point projection, shape overlap
//   * joints: fixed/revolute/prismatic/rope/spring
//   * a kinematic character controller (ball or any analytic shape, with a grounded report)

use rapier2d::control::{CharacterLength, KinematicCharacterController};
use rapier2d::pipeline::{
  DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline, DebugRenderStyle,
};
use rapier2d::prelude::*;
use std::sync::Mutex;

// ---- event collection -------------------------------------------------------------------------

/// One collision/sensor event, flattened for the C boundary.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ContactRecord {
  pub a: u64,       // packed collider handle 1
  pub b: u64,       // packed collider handle 2
  pub started: i32, // 1 = began touching, 0 = stopped
  pub sensor: i32,  // 1 = at least one collider is a sensor (intersection), 0 = solid contact
}

/// One contact-force event (fires when total contact force exceeds a collider's threshold).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ContactForceRecord {
  pub a: u64,         // packed collider handle 1
  pub b: u64,         // packed collider handle 2
  pub magnitude: f32, // total contact-force magnitude this step
}

/// Accumulates collision + contact-force events during a step; drained by the Lua adapter each
/// frame. Interior mutability (Mutex) because `EventHandler` methods take `&self` while the step
/// borrows the rest of the world mutably (disjoint field borrows).
#[derive(Default)]
struct CollisionCollector {
  events: Mutex<Vec<ContactRecord>>,
  force_events: Mutex<Vec<ContactForceRecord>>,
}

impl EventHandler for CollisionCollector {
  fn handle_collision_event(
    &self,
    _bodies: &RigidBodySet,
    _colliders: &ColliderSet,
    event: CollisionEvent,
    _contact_pair: Option<&ContactPair>,
  ) {
    let rec = ContactRecord {
      a: pack_col(event.collider1()),
      b: pack_col(event.collider2()),
      started: event.started() as i32,
      sensor: event.sensor() as i32,
    };
    self.events.lock().unwrap().push(rec);
  }

  fn handle_contact_force_event(
    &self,
    _dt: Real,
    _bodies: &RigidBodySet,
    _colliders: &ColliderSet,
    contact_pair: &ContactPair,
    total_force_magnitude: Real,
  ) {
    let rec = ContactForceRecord {
      a: pack_col(contact_pair.collider1),
      b: pack_col(contact_pair.collider2),
      magnitude: total_force_magnitude,
    };
    self.force_events.lock().unwrap().push(rec);
  }
}

// ---- physics hooks (optional C callbacks for pair filtering) ----------------------------------

/// Optional C callbacks invoked during the step to filter collision/intersection pairs. A callback
/// receives the two packed collider handles and returns non-zero to keep the pair, 0 to discard it.
/// Only consulted for colliders that opted in via `shim_collider_set_active_hooks`. NOTE: these run
/// per candidate pair every step and call back into Lua — keep them cheap and allocation-free.
#[derive(Default)]
struct ShimHooks {
  contact_filter: Option<extern "C" fn(u64, u64) -> i32>,
  intersection_filter: Option<extern "C" fn(u64, u64) -> i32>,
}

impl PhysicsHooks for ShimHooks {
  fn filter_contact_pair(&self, ctx: &PairFilterContext) -> Option<SolverFlags> {
    match self.contact_filter {
      Some(cb) if cb(pack_col(ctx.collider1), pack_col(ctx.collider2)) == 0 => None,
      _ => Some(SolverFlags::default()),
    }
  }

  fn filter_intersection_pair(&self, ctx: &PairFilterContext) -> bool {
    match self.intersection_filter {
      Some(cb) => cb(pack_col(ctx.collider1), pack_col(ctx.collider2)) != 0,
      None => true,
    }
  }
}

// ---- world ------------------------------------------------------------------------------------

/// Opaque physics world handle passed across the FFI boundary as `*mut PhysicsWorld`.
pub struct PhysicsWorld {
  pipeline: PhysicsPipeline,
  gravity: Vector,
  integration_parameters: IntegrationParameters,
  islands: IslandManager,
  broad_phase: DefaultBroadPhase,
  narrow_phase: NarrowPhase,
  bodies: RigidBodySet,
  colliders: ColliderSet,
  impulse_joints: ImpulseJointSet,
  multibody_joints: MultibodyJointSet,
  ccd_solver: CCDSolver,
  events: CollisionCollector,
  hooks: ShimHooks,
  last_dt: Real,
  overlap_buf: Vec<u64>, // scratch for the most recent overlap query result
}

// Handles pack as (index << 32 | generation).
fn pack_body(h: RigidBodyHandle) -> u64 {
  let (i, g) = h.into_raw_parts();
  ((i as u64) << 32) | (g as u64)
}
fn unpack_body(v: u64) -> RigidBodyHandle {
  RigidBodyHandle::from_raw_parts((v >> 32) as u32, (v & 0xffff_ffff) as u32)
}
fn pack_col(h: ColliderHandle) -> u64 {
  let (i, g) = h.into_raw_parts();
  ((i as u64) << 32) | (g as u64)
}
fn unpack_col(v: u64) -> ColliderHandle {
  ColliderHandle::from_raw_parts((v >> 32) as u32, (v & 0xffff_ffff) as u32)
}

// Sentinel for "no handle" returns and "exclude nothing" args. A real handle packs (index, gen) as
// two u32s; the first body/collider is (0, 0) → packs to 0, so 0 is a *valid* handle and can't be
// the null sentinel. u64::MAX (index & gen both 0xffffffff) can never be a live handle.
const NULL_HANDLE: u64 = u64::MAX;

/// Create a new physics world (top-down: zero gravity). Free with `shim_world_free`.
#[no_mangle]
pub extern "C" fn shim_world_new() -> *mut PhysicsWorld {
  let world = PhysicsWorld {
    pipeline: PhysicsPipeline::new(),
    gravity: Vector::new(0.0, 0.0),
    integration_parameters: IntegrationParameters::default(),
    islands: IslandManager::new(),
    broad_phase: DefaultBroadPhase::new(),
    narrow_phase: NarrowPhase::new(),
    bodies: RigidBodySet::new(),
    colliders: ColliderSet::new(),
    impulse_joints: ImpulseJointSet::new(),
    multibody_joints: MultibodyJointSet::new(),
    ccd_solver: CCDSolver::new(),
    events: CollisionCollector::default(),
    hooks: ShimHooks::default(),
    last_dt: 1.0 / 60.0,
    overlap_buf: Vec::new(),
  };
  Box::into_raw(Box::new(world))
}

/// Free a world created by `shim_world_new`.
#[no_mangle]
pub extern "C" fn shim_world_free(world: *mut PhysicsWorld) {
  if !world.is_null() {
    unsafe { drop(Box::from_raw(world)) };
  }
}

/// Advance the simulation by `dt` seconds (caller drives the fixed timestep).
#[no_mangle]
pub extern "C" fn shim_world_step(world: *mut PhysicsWorld, dt: f32) {
  let w = unsafe { &mut *world };
  w.integration_parameters.dt = dt;
  w.last_dt = dt;
  let gravity = w.gravity;
  w.pipeline.step(
    gravity,
    &w.integration_parameters,
    &mut w.islands,
    &mut w.broad_phase,
    &mut w.narrow_phase,
    &mut w.bodies,
    &mut w.colliders,
    &mut w.impulse_joints,
    &mut w.multibody_joints,
    &mut w.ccd_solver,
    &w.hooks,
    &w.events,
  );
}

// ---- events -----------------------------------------------------------------------------------

/// Number of collision/sensor events queued since the last `shim_events_clear`.
#[no_mangle]
pub extern "C" fn shim_events_count(world: *mut PhysicsWorld) -> u32 {
  let w = unsafe { &*world };
  w.events.events.lock().unwrap().len() as u32
}

/// Copy event `i` into `out`. Returns 1 on success, 0 if out of range.
#[no_mangle]
pub extern "C" fn shim_events_get(world: *mut PhysicsWorld, i: u32, out: *mut ContactRecord) -> i32 {
  let w = unsafe { &*world };
  let ev = w.events.events.lock().unwrap();
  match ev.get(i as usize) {
    Some(r) => {
      unsafe { *out = *r };
      1
    }
    None => 0,
  }
}

/// Drop all queued events (call once per frame after draining).
#[no_mangle]
pub extern "C" fn shim_events_clear(world: *mut PhysicsWorld) {
  let w = unsafe { &*world };
  w.events.events.lock().unwrap().clear();
}

// ---- bodies -----------------------------------------------------------------------------------

// Body kind codes (0 = dynamic, the default arm). See `shim_body_create`.
const BODY_FIXED: i32 = 1;
const BODY_KINEMATIC_VEL: i32 = 2;

/// Create a rigid body of `kind` (0=dynamic, 1=fixed/static, 2=kinematic-velocity) at (x, y).
#[no_mangle]
pub extern "C" fn shim_body_create(world: *mut PhysicsWorld, kind: i32, x: f32, y: f32) -> u64 {
  let w = unsafe { &mut *world };
  let builder = match kind {
    BODY_FIXED => RigidBodyBuilder::fixed(),
    BODY_KINEMATIC_VEL => RigidBodyBuilder::kinematic_velocity_based(),
    _ => RigidBodyBuilder::dynamic(),
  };
  let rb = builder.translation(Vector::new(x, y)).build();
  pack_body(w.bodies.insert(rb))
}

#[no_mangle]
pub extern "C" fn shim_body_remove(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  w.bodies.remove(
    unpack_body(handle),
    &mut w.islands,
    &mut w.colliders,
    &mut w.impulse_joints,
    &mut w.multibody_joints,
    true,
  );
}

#[no_mangle]
pub extern "C" fn shim_body_position(
  world: *mut PhysicsWorld,
  handle: u64,
  out_x: *mut f32,
  out_y: *mut f32,
) {
  let w = unsafe { &*world };
  if let Some(rb) = w.bodies.get(unpack_body(handle)) {
    let t = rb.translation();
    unsafe {
      *out_x = t.x;
      *out_y = t.y;
    }
  }
}

/// Batched: write (x,y) for each of `count` body handles into `out` (a flat `float[2*count]`), in one
/// FFI call instead of `count` calls to `shim_body_position`. Missing bodies write (0,0). The caller
/// keeps persistent `handles`/`out` buffers across frames to avoid per-call allocation.
#[no_mangle]
pub extern "C" fn shim_bodies_read_transforms(
  world: *mut PhysicsWorld,
  handles: *const u64,
  count: u32,
  out: *mut f32,
) {
  let w = unsafe { &*world };
  let handles = unsafe { std::slice::from_raw_parts(handles, count as usize) };
  let out = unsafe { std::slice::from_raw_parts_mut(out, count as usize * 2) };
  for i in 0..count as usize {
    let (x, y) = match w.bodies.get(unpack_body(handles[i])) {
      Some(rb) => {
        let t = rb.translation();
        (t.x, t.y)
      }
      None => (0.0, 0.0),
    };
    out[i * 2] = x;
    out[i * 2 + 1] = y;
  }
}

/// Body rotation angle (radians), for rendering orientation.
#[no_mangle]
pub extern "C" fn shim_body_rotation(world: *mut PhysicsWorld, handle: u64) -> f32 {
  let w = unsafe { &*world };
  w.bodies
    .get(unpack_body(handle))
    .map(|rb| rb.rotation().angle())
    .unwrap_or(0.0)
}

/// Teleport a body (no velocity change).
#[no_mangle]
pub extern "C" fn shim_body_set_translation(world: *mut PhysicsWorld, handle: u64, x: f32, y: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_translation(Vector::new(x, y), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_linvel(world: *mut PhysicsWorld, handle: u64, vx: f32, vy: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_linvel(Vector::new(vx, vy), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_linvel(
  world: *mut PhysicsWorld,
  handle: u64,
  out_x: *mut f32,
  out_y: *mut f32,
) {
  let w = unsafe { &*world };
  if let Some(rb) = w.bodies.get(unpack_body(handle)) {
    let v = rb.linvel();
    unsafe {
      *out_x = v.x;
      *out_y = v.y;
    }
  }
}

#[no_mangle]
pub extern "C" fn shim_body_lock_rotations(world: *mut PhysicsWorld, handle: u64, locked: bool) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.lock_rotations(locked, true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_linear_damping(world: *mut PhysicsWorld, handle: u64, damping: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_linear_damping(damping);
  }
}

/// Apply an instantaneous impulse — knockback, explosion shove.
#[no_mangle]
pub extern "C" fn shim_body_apply_impulse(world: *mut PhysicsWorld, handle: u64, x: f32, y: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.apply_impulse(Vector::new(x, y), true);
  }
}

/// Add a force (applied over the next step) — sustained pushes/wind.
#[no_mangle]
pub extern "C" fn shim_body_add_force(world: *mut PhysicsWorld, handle: u64, x: f32, y: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.add_force(Vector::new(x, y), true);
  }
}

/// Enable continuous collision detection so fast bodies don't tunnel thin walls.
#[no_mangle]
pub extern "C" fn shim_body_enable_ccd(world: *mut PhysicsWorld, handle: u64, enabled: bool) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.enable_ccd(enabled);
  }
}

// ---- colliders --------------------------------------------------------------------------------

// Shape codes (0 = ball, the default arm). See `shape_builder`.
const SHAPE_CUBOID: i32 = 1;
const SHAPE_CAPSULE: i32 = 2;

fn shape_builder(shape: i32, a: f32, b: f32) -> ColliderBuilder {
  match shape {
    SHAPE_CUBOID => ColliderBuilder::cuboid(a, b),     // a=half-width, b=half-height
    SHAPE_CAPSULE => ColliderBuilder::capsule_y(a, b), // a=half-height, b=radius
    _ => ColliderBuilder::ball(a),                     // a=radius
  }
}

/// Attach a collider (shape: 0=ball,1=cuboid,2=capsule) to a body. Collision events enabled.
#[no_mangle]
pub extern "C" fn shim_collider_attach(
  world: *mut PhysicsWorld,
  body: u64,
  shape: i32,
  a: f32,
  b: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let col = shape_builder(shape, a, b)
    .active_events(ActiveEvents::COLLISION_EVENTS)
    .build();
  let parent = unpack_body(body);
  pack_col(w.colliders.insert_with_parent(col, parent, &mut w.bodies))
}

/// Create a free-standing static collider at (x, y) — map walls/obstacles, no body.
#[no_mangle]
pub extern "C" fn shim_collider_static(
  world: *mut PhysicsWorld,
  shape: i32,
  x: f32,
  y: f32,
  a: f32,
  b: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let col = shape_builder(shape, a, b)
    .translation(Vector::new(x, y))
    .active_events(ActiveEvents::COLLISION_EVENTS)
    .build();
  pack_col(w.colliders.insert(col))
}

#[no_mangle]
pub extern "C" fn shim_collider_remove(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  w.colliders.remove(
    unpack_col(handle),
    &mut w.islands,
    &mut w.bodies,
    true,
  );
}

/// Make a collider a sensor (intersections reported, no physical response) or solid again.
#[no_mangle]
pub extern "C" fn shim_collider_set_sensor(world: *mut PhysicsWorld, handle: u64, sensor: bool) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_sensor(sensor);
  }
}

/// Collision filtering: `memberships` = the groups this collider belongs to; `filter` = the groups
/// it may interact with. Two colliders collide iff each is in the other's filter (bitwise).
#[no_mangle]
pub extern "C" fn shim_collider_set_groups(
  world: *mut PhysicsWorld,
  handle: u64,
  memberships: u32,
  filter: u32,
) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_collision_groups(InteractionGroups::new(
      Group::from_bits_truncate(memberships),
      Group::from_bits_truncate(filter),
      InteractionTestMode::And,
    ));
  }
}

// ---- queries ----------------------------------------------------------------------------------

/// Raycast from (ox,oy) along (dx,dy) up to `max_toi`. On hit writes the collider handle + distance
/// and returns 1; else returns 0.
#[no_mangle]
pub extern "C" fn shim_query_raycast(
  world: *mut PhysicsWorld,
  ox: f32,
  oy: f32,
  dx: f32,
  dy: f32,
  max_toi: f32,
  out_collider: *mut u64,
  out_toi: *mut f32,
) -> i32 {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  let ray = Ray::new(Vector::new(ox, oy), Vector::new(dx, dy));
  match qp.cast_ray(&ray, max_toi, true) {
    Some((handle, toi)) => {
      unsafe {
        *out_collider = pack_col(handle);
        *out_toi = toi;
      }
      1
    }
    None => 0,
  }
}

/// First collider containing point (px,py), or 0 if none. Writes its handle to `out_collider`.
#[no_mangle]
pub extern "C" fn shim_query_point(
  world: *mut PhysicsWorld,
  px: f32,
  py: f32,
  out_collider: *mut u64,
) -> i32 {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  // Collapse to a Copy u64 so the iterator borrow on `qp` ends before `qp` is dropped.
  let hit = qp.intersect_point(Vector::new(px, py)).next().map(|(h, _)| pack_col(h));
  match hit {
    Some(handle) => {
      unsafe { *out_collider = handle };
      1
    }
    None => 0,
  }
}

/// Overlap test: all colliders intersecting a circle at (x,y). Fills the world's scratch buffer
/// (read back with `shim_overlap_count` / `shim_overlap_get`) and returns the count. For AoE.
#[no_mangle]
pub extern "C" fn shim_query_overlap_circle(
  world: *mut PhysicsWorld,
  x: f32,
  y: f32,
  radius: f32,
) -> u32 {
  let w = unsafe { &mut *world };
  let hits: Vec<u64> = {
    let qp = w.broad_phase.as_query_pipeline(
      w.narrow_phase.query_dispatcher(),
      &w.bodies,
      &w.colliders,
      QueryFilter::default(),
    );
    let ball = Ball::new(radius);
    qp.intersect_shape(Pose::from_translation(Vector::new(x, y)), &ball)
      .map(|(h, _)| pack_col(h))
      .collect()
  };
  w.overlap_buf = hits;
  w.overlap_buf.len() as u32
}

#[no_mangle]
pub extern "C" fn shim_overlap_count(world: *mut PhysicsWorld) -> u32 {
  let w = unsafe { &*world };
  w.overlap_buf.len() as u32
}

#[no_mangle]
pub extern "C" fn shim_overlap_get(world: *mut PhysicsWorld, i: u32) -> u64 {
  let w = unsafe { &*world };
  w.overlap_buf.get(i as usize).copied().unwrap_or(0)
}

// ---- kinematic character controller -----------------------------------------------------------

/// Compute a collision-corrected movement for a ball-shaped character at (x,y) wanting to move by
/// (dx,dy). Writes the allowed translation to out_x/out_y (the caller then teleports the body).
#[no_mangle]
pub extern "C" fn shim_kcc_move_ball(
  world: *mut PhysicsWorld,
  x: f32,
  y: f32,
  radius: f32,
  dx: f32,
  dy: f32,
  out_x: *mut f32,
  out_y: *mut f32,
) {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  let controller = KinematicCharacterController::default();
  let shape = Ball::new(radius);
  let pose = Pose::from_translation(Vector::new(x, y));
  let mov = controller.move_shape(w.last_dt, &qp, &shape, &pose, Vector::new(dx, dy), |_| {});
  unsafe {
    *out_x = mov.translation.x;
    *out_y = mov.translation.y;
  }
}

// ===============================================================================================
// Extended surface — the rest of the Rapier API bound for completeness. The build cost is fixed
// regardless of how much is exposed, so we bind it all rather than re-opening the crate later.
// ===============================================================================================

fn pack_joint(h: ImpulseJointHandle) -> u64 {
  let (i, g) = h.into_raw_parts();
  ((i as u64) << 32) | (g as u64)
}
fn unpack_joint(v: u64) -> ImpulseJointHandle {
  ImpulseJointHandle::from_raw_parts((v >> 32) as u32, (v & 0xffff_ffff) as u32)
}

const BODY_KINEMATIC_POS: i32 = 3;
fn body_type_of(kind: i32) -> RigidBodyType {
  match kind {
    BODY_FIXED => RigidBodyType::Fixed,
    BODY_KINEMATIC_VEL => RigidBodyType::KinematicVelocityBased,
    BODY_KINEMATIC_POS => RigidBodyType::KinematicPositionBased,
    _ => RigidBodyType::Dynamic,
  }
}

/// A `SharedShape` for the analytic shape codes (ball/cuboid/capsule), for shape queries + the KCC.
fn shape_of(shape: i32, a: f32, b: f32) -> SharedShape {
  match shape {
    SHAPE_CUBOID => SharedShape::cuboid(a, b),
    SHAPE_CAPSULE => SharedShape::capsule_y(a, b),
    _ => SharedShape::ball(a),
  }
}

// ---- world config -----------------------------------------------------------------------------

/// Set world gravity (default is zero — top-down). Side-scrollers use e.g. (0, 9.81).
#[no_mangle]
pub extern "C" fn shim_world_set_gravity(world: *mut PhysicsWorld, gx: f32, gy: f32) {
  let w = unsafe { &mut *world };
  w.gravity = Vector::new(gx, gy);
}

/// Number of constraint solver iterations per step (higher = stiffer/more accurate, slower).
#[no_mangle]
pub extern "C" fn shim_world_set_solver_iterations(world: *mut PhysicsWorld, iters: u32) {
  let w = unsafe { &mut *world };
  w.integration_parameters.num_solver_iterations = (iters.max(1)) as usize;
}

// ---- bodies: angular + extended control + properties ------------------------------------------

#[no_mangle]
pub extern "C" fn shim_body_angvel(world: *mut PhysicsWorld, handle: u64) -> f32 {
  let w = unsafe { &*world };
  w.bodies.get(unpack_body(handle)).map(|rb| rb.angvel()).unwrap_or(0.0)
}

#[no_mangle]
pub extern "C" fn shim_body_set_angvel(world: *mut PhysicsWorld, handle: u64, av: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_angvel(av, true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_rotation(world: *mut PhysicsWorld, handle: u64, angle: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_rotation(Rotation::new(angle), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_apply_torque_impulse(world: *mut PhysicsWorld, handle: u64, torque: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.apply_torque_impulse(torque, true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_add_torque(world: *mut PhysicsWorld, handle: u64, torque: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.add_torque(torque, true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_apply_impulse_at_point(
  world: *mut PhysicsWorld,
  handle: u64,
  ix: f32,
  iy: f32,
  px: f32,
  py: f32,
) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.apply_impulse_at_point(Vector::new(ix, iy), Vector::new(px, py), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_add_force_at_point(
  world: *mut PhysicsWorld,
  handle: u64,
  fx: f32,
  fy: f32,
  px: f32,
  py: f32,
) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.add_force_at_point(Vector::new(fx, fy), Vector::new(px, py), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_angular_damping(world: *mut PhysicsWorld, handle: u64, damping: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_angular_damping(damping);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_gravity_scale(world: *mut PhysicsWorld, handle: u64, scale: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_gravity_scale(scale, true);
  }
}

/// Change body type at runtime (0=dynamic, 1=fixed, 2=kinematic-velocity, 3=kinematic-position).
#[no_mangle]
pub extern "C" fn shim_body_set_type(world: *mut PhysicsWorld, handle: u64, kind: i32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_body_type(body_type_of(kind), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_additional_mass(world: *mut PhysicsWorld, handle: u64, mass: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_additional_mass(mass, true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_mass(world: *mut PhysicsWorld, handle: u64) -> f32 {
  let w = unsafe { &*world };
  w.bodies.get(unpack_body(handle)).map(|rb| rb.mass()).unwrap_or(0.0)
}

#[no_mangle]
pub extern "C" fn shim_body_set_enabled(world: *mut PhysicsWorld, handle: u64, enabled: bool) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_enabled(enabled);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_wake_up(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.wake_up(true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_sleep(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.sleep();
  }
}

#[no_mangle]
pub extern "C" fn shim_body_is_sleeping(world: *mut PhysicsWorld, handle: u64) -> i32 {
  let w = unsafe { &*world };
  w.bodies.get(unpack_body(handle)).map(|rb| rb.is_sleeping() as i32).unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn shim_body_set_dominance(world: *mut PhysicsWorld, handle: u64, group: i32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_dominance_group(group.clamp(-127, 127) as i8);
  }
}

/// For kinematic-position bodies: the target translation reached at the end of the next step.
#[no_mangle]
pub extern "C" fn shim_body_set_next_kinematic_translation(
  world: *mut PhysicsWorld,
  handle: u64,
  x: f32,
  y: f32,
) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_next_kinematic_translation(Vector::new(x, y));
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_next_kinematic_rotation(world: *mut PhysicsWorld, handle: u64, angle: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_next_kinematic_rotation(Rotation::new(angle));
  }
}

// ---- colliders: materials + config + offsets --------------------------------------------------

#[no_mangle]
pub extern "C" fn shim_collider_set_friction(world: *mut PhysicsWorld, handle: u64, friction: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_friction(friction);
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_set_restitution(world: *mut PhysicsWorld, handle: u64, restitution: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_restitution(restitution);
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_set_density(world: *mut PhysicsWorld, handle: u64, density: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_density(density);
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_set_mass(world: *mut PhysicsWorld, handle: u64, mass: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_mass(mass);
  }
}

/// Offset of an attached collider relative to its parent body's frame.
#[no_mangle]
pub extern "C" fn shim_collider_set_translation_wrt_parent(
  world: *mut PhysicsWorld,
  handle: u64,
  x: f32,
  y: f32,
) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_translation_wrt_parent(Vector::new(x, y));
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_set_rotation_wrt_parent(world: *mut PhysicsWorld, handle: u64, angle: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_rotation_wrt_parent(angle);
  }
}

/// Solver groups (which colliders generate contact *forces*), separate from collision groups
/// (which generate contact *events/geometry*). Same bitmask semantics as `set_groups`.
#[no_mangle]
pub extern "C" fn shim_collider_set_solver_groups(
  world: *mut PhysicsWorld,
  handle: u64,
  memberships: u32,
  filter: u32,
) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_solver_groups(InteractionGroups::new(
      Group::from_bits_truncate(memberships),
      Group::from_bits_truncate(filter),
      InteractionTestMode::And,
    ));
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_set_enabled(world: *mut PhysicsWorld, handle: u64, enabled: bool) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_enabled(enabled);
  }
}

/// Absolute world position of a collider (for attached colliders this reflects the parent body).
#[no_mangle]
pub extern "C" fn shim_collider_position(
  world: *mut PhysicsWorld,
  handle: u64,
  out_x: *mut f32,
  out_y: *mut f32,
) {
  let w = unsafe { &*world };
  if let Some(c) = w.colliders.get(unpack_col(handle)) {
    let t = c.translation();
    unsafe {
      *out_x = t.x;
      *out_y = t.y;
    }
  }
}

/// Teleport a free-standing (static) collider.
#[no_mangle]
pub extern "C" fn shim_collider_set_translation(world: *mut PhysicsWorld, handle: u64, x: f32, y: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_translation(Vector::new(x, y));
  }
}

// ---- colliders: extended static shapes (map geometry) -----------------------------------------

fn read_points(ptr: *const f32, count: u32) -> Vec<Vector> {
  let s = unsafe { std::slice::from_raw_parts(ptr, count as usize * 2) };
  (0..count as usize).map(|i| Vector::new(s[i * 2], s[i * 2 + 1])).collect()
}

fn insert_static(w: &mut PhysicsWorld, b: ColliderBuilder) -> u64 {
  pack_col(w.colliders.insert(b.active_events(ActiveEvents::COLLISION_EVENTS).build()))
}

#[no_mangle]
pub extern "C" fn shim_collider_static_triangle(
  world: *mut PhysicsWorld,
  ax: f32,
  ay: f32,
  bx: f32,
  by: f32,
  cx: f32,
  cy: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  insert_static(w, ColliderBuilder::triangle(Vector::new(ax, ay), Vector::new(bx, by), Vector::new(cx, cy)))
}

#[no_mangle]
pub extern "C" fn shim_collider_static_segment(
  world: *mut PhysicsWorld,
  ax: f32,
  ay: f32,
  bx: f32,
  by: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  insert_static(w, ColliderBuilder::segment(Vector::new(ax, ay), Vector::new(bx, by)))
}

/// A polyline from `count` points (flat `float[2*count]`) — open chains of map walls.
#[no_mangle]
pub extern "C" fn shim_collider_static_polyline(
  world: *mut PhysicsWorld,
  points: *const f32,
  count: u32,
) -> u64 {
  let w = unsafe { &mut *world };
  insert_static(w, ColliderBuilder::polyline(read_points(points, count), None))
}

/// Convex hull of `count` points. Returns 0 if the hull is degenerate (collinear/too few points).
#[no_mangle]
pub extern "C" fn shim_collider_static_convex_hull(
  world: *mut PhysicsWorld,
  points: *const f32,
  count: u32,
) -> u64 {
  let w = unsafe { &mut *world };
  match ColliderBuilder::convex_hull(&read_points(points, count)) {
    Some(b) => insert_static(w, b),
    None => NULL_HANDLE,
  }
}

/// A heightfield from `count` height samples spaced evenly along x by `scale_x`, scaled in y by
/// `scale_y` — efficient terrain/floor geometry.
#[no_mangle]
pub extern "C" fn shim_collider_static_heightfield(
  world: *mut PhysicsWorld,
  heights: *const f32,
  count: u32,
  scale_x: f32,
  scale_y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let hs = unsafe { std::slice::from_raw_parts(heights, count as usize) }.to_vec();
  insert_static(w, ColliderBuilder::heightfield(hs, Vector::new(scale_x, scale_y)))
}

// ---- joints -----------------------------------------------------------------------------------

fn insert_joint(
  w: &mut PhysicsWorld,
  b1: u64,
  b2: u64,
  data: impl Into<GenericJoint>,
) -> u64 {
  pack_joint(w.impulse_joints.insert(unpack_body(b1), unpack_body(b2), data, true))
}

/// Rigidly weld two bodies (no relative motion) at the given local anchors.
#[no_mangle]
pub extern "C" fn shim_joint_fixed(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = FixedJointBuilder::new()
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_joint(w, b1, b2, j)
}

/// Pin two bodies at a point they rotate freely about (hinge).
#[no_mangle]
pub extern "C" fn shim_joint_revolute(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = RevoluteJointBuilder::new()
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_joint(w, b1, b2, j)
}

/// Constrain two bodies to slide along `axis` (no rotation) — pistons, elevators.
#[no_mangle]
pub extern "C" fn shim_joint_prismatic(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
  axis_x: f32,
  axis_y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = PrismaticJointBuilder::new(Vector::new(axis_x, axis_y))
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_joint(w, b1, b2, j)
}

/// Limit the distance between two anchors to `max_dist` (a slack rope).
#[no_mangle]
pub extern "C" fn shim_joint_rope(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
  max_dist: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = RopeJointBuilder::new(max_dist)
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_joint(w, b1, b2, j)
}

/// A damped spring pulling two anchors toward `rest_length`.
#[no_mangle]
pub extern "C" fn shim_joint_spring(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
  rest_length: f32,
  stiffness: f32,
  damping: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = SpringJointBuilder::new(rest_length, stiffness, damping)
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_joint(w, b1, b2, j)
}

#[no_mangle]
pub extern "C" fn shim_joint_remove(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  w.impulse_joints.remove(unpack_joint(handle), true);
}

// ---- extended queries -------------------------------------------------------------------------

/// Raycast that also returns the surface normal at the hit. Returns 1 on hit (writes collider,
/// distance, normal), else 0.
#[no_mangle]
pub extern "C" fn shim_query_raycast_normal(
  world: *mut PhysicsWorld,
  ox: f32,
  oy: f32,
  dx: f32,
  dy: f32,
  max_toi: f32,
  out_collider: *mut u64,
  out_toi: *mut f32,
  out_nx: *mut f32,
  out_ny: *mut f32,
) -> i32 {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  let ray = Ray::new(Vector::new(ox, oy), Vector::new(dx, dy));
  match qp.cast_ray_and_get_normal(&ray, max_toi, true) {
    Some((handle, inter)) => {
      unsafe {
        *out_collider = pack_col(handle);
        *out_toi = inter.time_of_impact;
        *out_nx = inter.normal.x;
        *out_ny = inter.normal.y;
      }
      1
    }
    None => 0,
  }
}

/// Sweep a shape (ball/cuboid/capsule) from (ox,oy)+angle along (dx,dy), up to `max_toi`. Returns 1
/// on the first hit (writes collider + time-of-impact), else 0. For projectiles / movement sweeps.
#[no_mangle]
pub extern "C" fn shim_query_shapecast(
  world: *mut PhysicsWorld,
  shape: i32,
  a: f32,
  b: f32,
  ox: f32,
  oy: f32,
  angle: f32,
  dx: f32,
  dy: f32,
  max_toi: f32,
  out_collider: *mut u64,
  out_toi: *mut f32,
) -> i32 {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  let s = shape_of(shape, a, b);
  let pose = Pose::new(Vector::new(ox, oy), angle);
  let opts = rapier2d::parry::query::ShapeCastOptions {
    max_time_of_impact: max_toi,
    ..Default::default()
  };
  match qp.cast_shape(&pose, Vector::new(dx, dy), &*s, opts) {
    Some((handle, hit)) => {
      unsafe {
        *out_collider = pack_col(handle);
        *out_toi = hit.time_of_impact;
      }
      1
    }
    None => 0,
  }
}

/// Project a point onto the nearest collider within `max_dist`. Returns 1 on success (writes the
/// collider, the closest surface point, and whether the query point was inside it), else 0.
#[no_mangle]
pub extern "C" fn shim_query_project_point(
  world: *mut PhysicsWorld,
  px: f32,
  py: f32,
  max_dist: f32,
  out_collider: *mut u64,
  out_x: *mut f32,
  out_y: *mut f32,
  out_inside: *mut i32,
) -> i32 {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  match qp.project_point(Vector::new(px, py), max_dist, true) {
    Some((handle, proj)) => {
      unsafe {
        *out_collider = pack_col(handle);
        *out_x = proj.point.x;
        *out_y = proj.point.y;
        *out_inside = proj.is_inside as i32;
      }
      1
    }
    None => 0,
  }
}

/// Overlap test with an arbitrary shape (ball/cuboid/capsule) posed at (x,y)+angle. Fills the
/// world's scratch buffer (read with `shim_overlap_count`/`shim_overlap_get`) and returns the count.
/// Generalizes `shim_query_overlap_circle` to any analytic shape.
#[no_mangle]
pub extern "C" fn shim_query_overlap_shape(
  world: *mut PhysicsWorld,
  shape: i32,
  a: f32,
  b: f32,
  x: f32,
  y: f32,
  angle: f32,
) -> u32 {
  let w = unsafe { &mut *world };
  let hits: Vec<u64> = {
    let qp = w.broad_phase.as_query_pipeline(
      w.narrow_phase.query_dispatcher(),
      &w.bodies,
      &w.colliders,
      QueryFilter::default(),
    );
    let s = shape_of(shape, a, b);
    qp.intersect_shape(Pose::new(Vector::new(x, y), angle), &*s)
      .map(|(h, _)| pack_col(h))
      .collect()
  };
  w.overlap_buf = hits;
  w.overlap_buf.len() as u32
}

// ---- generalized kinematic character controller -----------------------------------------------

/// Like `shim_kcc_move_ball` but for any analytic shape, with a skin `offset` and a grounded report.
/// Writes the collision-corrected translation to out_x/out_y and 1/0 (grounded) to `out_grounded`.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn shim_kcc_move(
  world: *mut PhysicsWorld,
  shape: i32,
  a: f32,
  b: f32,
  x: f32,
  y: f32,
  angle: f32,
  dx: f32,
  dy: f32,
  offset: f32,
  out_x: *mut f32,
  out_y: *mut f32,
  out_grounded: *mut i32,
) {
  let w = unsafe { &*world };
  let qp = w.broad_phase.as_query_pipeline(
    w.narrow_phase.query_dispatcher(),
    &w.bodies,
    &w.colliders,
    QueryFilter::default(),
  );
  let mut controller = KinematicCharacterController::default();
  if offset > 0.0 {
    controller.offset = CharacterLength::Absolute(offset);
  }
  let s = shape_of(shape, a, b);
  let pose = Pose::new(Vector::new(x, y), angle);
  let mov = controller.move_shape(w.last_dt, &qp, &*s, &pose, Vector::new(dx, dy), |_| {});
  unsafe {
    *out_x = mov.translation.x;
    *out_y = mov.translation.y;
    *out_grounded = mov.grounded as i32;
  }
}

// ===============================================================================================
// Completion pass — the remaining Rapier surface: joint motors/limits, multibody joints, body
// locks/forces/full-pose/mass-properties/reads, collider shape-swap/events/reads, narrow-phase
// contact & intersection queries, filtered raycast, and the rest of the world tuning params.
// ===============================================================================================

// ---- joint motors + limits (impulse joints) ---------------------------------------------------

// Joint axis codes: 0 = linear-x, 1 = linear-y, 2 = angular.
fn joint_axis(a: i32) -> JointAxis {
  match a {
    0 => JointAxis::LinX,
    1 => JointAxis::LinY,
    _ => JointAxis::AngX,
  }
}

fn with_impulse_joint<F: FnOnce(&mut GenericJoint)>(w: &mut PhysicsWorld, h: u64, f: F) {
  if let Some(j) = w.impulse_joints.get_mut(unpack_joint(h), true) {
    f(&mut j.data);
  }
}

/// Drive a joint axis toward a target position/velocity with the given spring stiffness + damping
/// (e.g. a revolute motor uses axis 2). Stiffness 0 + a target velocity = a pure velocity motor.
#[no_mangle]
pub extern "C" fn shim_joint_set_motor(
  world: *mut PhysicsWorld,
  joint: u64,
  axis: i32,
  target_pos: f32,
  target_vel: f32,
  stiffness: f32,
  damping: f32,
) {
  let w = unsafe { &mut *world };
  with_impulse_joint(w, joint, |g| {
    g.set_motor(joint_axis(axis), target_pos, target_vel, stiffness, damping);
  });
}

/// Limit a joint axis to [min, max] (radians for the angular axis, world units for linear).
#[no_mangle]
pub extern "C" fn shim_joint_set_limits(world: *mut PhysicsWorld, joint: u64, axis: i32, min: f32, max: f32) {
  let w = unsafe { &mut *world };
  with_impulse_joint(w, joint, |g| {
    g.set_limits(joint_axis(axis), [min, max]);
  });
}

#[no_mangle]
pub extern "C" fn shim_joint_set_motor_max_force(world: *mut PhysicsWorld, joint: u64, axis: i32, max_force: f32) {
  let w = unsafe { &mut *world };
  with_impulse_joint(w, joint, |g| {
    g.set_motor_max_force(joint_axis(axis), max_force);
  });
}

/// Whether the two jointed bodies still generate contacts with each other.
#[no_mangle]
pub extern "C" fn shim_joint_set_contacts_enabled(world: *mut PhysicsWorld, joint: u64, enabled: bool) {
  let w = unsafe { &mut *world };
  with_impulse_joint(w, joint, |g| {
    g.set_contacts_enabled(enabled);
  });
}

// ---- multibody joints (reduced-coordinates / articulated, no positional drift) -----------------

fn pack_mb_joint(h: MultibodyJointHandle) -> u64 {
  let (i, g) = h.0.into_raw_parts();
  ((i as u64) << 32) | (g as u64)
}
fn unpack_mb_joint(v: u64) -> MultibodyJointHandle {
  MultibodyJointHandle(rapier2d::data::Index::from_raw_parts((v >> 32) as u32, (v & 0xffff_ffff) as u32))
}

fn insert_mb(w: &mut PhysicsWorld, b1: u64, b2: u64, data: impl Into<GenericJoint>) -> u64 {
  match w.multibody_joints.insert(unpack_body(b1), unpack_body(b2), data, true) {
    Some(h) => pack_mb_joint(h),
    None => NULL_HANDLE,
  }
}

#[no_mangle]
pub extern "C" fn shim_multibody_joint_fixed(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = FixedJointBuilder::new()
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_mb(w, b1, b2, j)
}

#[no_mangle]
pub extern "C" fn shim_multibody_joint_revolute(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = RevoluteJointBuilder::new()
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_mb(w, b1, b2, j)
}

#[no_mangle]
pub extern "C" fn shim_multibody_joint_prismatic(
  world: *mut PhysicsWorld,
  b1: u64,
  b2: u64,
  a1x: f32,
  a1y: f32,
  a2x: f32,
  a2y: f32,
  axis_x: f32,
  axis_y: f32,
) -> u64 {
  let w = unsafe { &mut *world };
  let j = PrismaticJointBuilder::new(Vector::new(axis_x, axis_y))
    .local_anchor1(Vector::new(a1x, a1y))
    .local_anchor2(Vector::new(a2x, a2y));
  insert_mb(w, b1, b2, j)
}

#[no_mangle]
pub extern "C" fn shim_multibody_joint_remove(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  w.multibody_joints.remove(unpack_mb_joint(handle), true);
}

// ---- bodies: locks, forces, full pose, mass properties, reads ---------------------------------

#[no_mangle]
pub extern "C" fn shim_body_lock_translations(world: *mut PhysicsWorld, handle: u64, locked: bool) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.lock_translations(locked, true);
  }
}

/// Allow/forbid translation per axis (e.g. a 2.5D platformer locking y).
#[no_mangle]
pub extern "C" fn shim_body_set_enabled_translations(world: *mut PhysicsWorld, handle: u64, allow_x: bool, allow_y: bool) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_enabled_translations(allow_x, allow_y, true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_reset_forces(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.reset_forces(true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_reset_torques(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.reset_torques(true);
  }
}

/// Set full pose (translation + rotation) in one call.
#[no_mangle]
pub extern "C" fn shim_body_set_position(world: *mut PhysicsWorld, handle: u64, x: f32, y: f32, angle: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_position(Pose::new(Vector::new(x, y), angle), true);
  }
}

/// Body type code: 0=dynamic, 1=fixed, 2=kinematic-velocity, 3=kinematic-position.
#[no_mangle]
pub extern "C" fn shim_body_type(world: *mut PhysicsWorld, handle: u64) -> i32 {
  let w = unsafe { &*world };
  match w.bodies.get(unpack_body(handle)).map(|rb| rb.body_type()) {
    Some(RigidBodyType::Fixed) => BODY_FIXED,
    Some(RigidBodyType::KinematicVelocityBased) => BODY_KINEMATIC_VEL,
    Some(RigidBodyType::KinematicPositionBased) => BODY_KINEMATIC_POS,
    _ => 0,
  }
}

#[no_mangle]
pub extern "C" fn shim_body_is_enabled(world: *mut PhysicsWorld, handle: u64) -> i32 {
  let w = unsafe { &*world };
  w.bodies.get(unpack_body(handle)).map(|rb| rb.is_enabled() as i32).unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn shim_body_center_of_mass(world: *mut PhysicsWorld, handle: u64, out_x: *mut f32, out_y: *mut f32) {
  let w = unsafe { &*world };
  if let Some(rb) = w.bodies.get(unpack_body(handle)) {
    let c = rb.center_of_mass();
    unsafe {
      *out_x = c.x;
      *out_y = c.y;
    }
  }
}

/// Override mass properties directly (center of mass + scalar 2D inertia), added to collider-derived.
#[no_mangle]
pub extern "C" fn shim_body_set_additional_mass_properties(
  world: *mut PhysicsWorld,
  handle: u64,
  mass: f32,
  com_x: f32,
  com_y: f32,
  inertia: f32,
) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_additional_mass_properties(MassProperties::new(Vector::new(com_x, com_y), mass, inertia), true);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_recompute_mass(world: *mut PhysicsWorld, handle: u64) {
  let w = unsafe { &mut *world };
  let colliders = &w.colliders;
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.recompute_mass_properties_from_colliders(colliders);
  }
}

#[no_mangle]
pub extern "C" fn shim_body_set_soft_ccd_prediction(world: *mut PhysicsWorld, handle: u64, distance: f32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_soft_ccd_prediction(distance);
  }
}

// ---- colliders: shape swap, events, reads -----------------------------------------------------

/// Toggle which events a collider emits: collision events and/or contact-force events.
#[no_mangle]
pub extern "C" fn shim_collider_set_active_events(world: *mut PhysicsWorld, handle: u64, collision: bool, contact_force: bool) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    let mut ev = ActiveEvents::empty();
    if collision {
      ev |= ActiveEvents::COLLISION_EVENTS;
    }
    if contact_force {
      ev |= ActiveEvents::CONTACT_FORCE_EVENTS;
    }
    c.set_active_events(ev);
  }
}

/// Raw `ActiveCollisionTypes` bitmask — which body-type pairs generate contacts (default excludes
/// fixed/fixed and kinematic/kinematic; set bits to enable those).
#[no_mangle]
pub extern "C" fn shim_collider_set_active_collision_types(world: *mut PhysicsWorld, handle: u64, bits: u32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_active_collision_types(ActiveCollisionTypes::from_bits_truncate(bits as u16));
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_set_contact_force_threshold(world: *mut PhysicsWorld, handle: u64, threshold: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_contact_force_event_threshold(threshold);
  }
}

/// Swap a collider's shape in place (ball/cuboid/capsule).
#[no_mangle]
pub extern "C" fn shim_collider_set_shape(world: *mut PhysicsWorld, handle: u64, shape: i32, a: f32, b: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_shape(shape_of(shape, a, b));
  }
}

/// Absolute rotation of a free-standing (static) collider.
#[no_mangle]
pub extern "C" fn shim_collider_set_rotation(world: *mut PhysicsWorld, handle: u64, angle: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_rotation(Rotation::new(angle));
  }
}

#[no_mangle]
pub extern "C" fn shim_collider_density(world: *mut PhysicsWorld, handle: u64) -> f32 {
  let w = unsafe { &*world };
  w.colliders.get(unpack_col(handle)).map(|c| c.density()).unwrap_or(0.0)
}

#[no_mangle]
pub extern "C" fn shim_collider_mass(world: *mut PhysicsWorld, handle: u64) -> f32 {
  let w = unsafe { &*world };
  w.colliders.get(unpack_col(handle)).map(|c| c.mass()).unwrap_or(0.0)
}

#[no_mangle]
pub extern "C" fn shim_collider_volume(world: *mut PhysicsWorld, handle: u64) -> f32 {
  let w = unsafe { &*world };
  w.colliders.get(unpack_col(handle)).map(|c| c.volume()).unwrap_or(0.0)
}

/// The body a collider is attached to, or 0 if it is free-standing (static).
#[no_mangle]
pub extern "C" fn shim_collider_parent(world: *mut PhysicsWorld, handle: u64) -> u64 {
  let w = unsafe { &*world };
  w.colliders.get(unpack_col(handle)).and_then(|c| c.parent()).map(pack_body).unwrap_or(NULL_HANDLE)
}

#[no_mangle]
pub extern "C" fn shim_collider_is_sensor(world: *mut PhysicsWorld, handle: u64) -> i32 {
  let w = unsafe { &*world };
  w.colliders.get(unpack_col(handle)).map(|c| c.is_sensor() as i32).unwrap_or(0)
}

// ---- narrow-phase contact / intersection queries ----------------------------------------------

/// Colliders currently in solid contact with `collider` (as of the last step). Fills the scratch
/// buffer (read with `shim_overlap_count`/`shim_overlap_get`) and returns the count.
#[no_mangle]
pub extern "C" fn shim_contacts_with(world: *mut PhysicsWorld, collider: u64) -> u32 {
  let w = unsafe { &mut *world };
  let h = unpack_col(collider);
  let hits: Vec<u64> = w
    .narrow_phase
    .contact_pairs_with(h)
    .filter(|p| p.has_any_active_contact())
    .map(|p| pack_col(if p.collider1 == h { p.collider2 } else { p.collider1 }))
    .collect();
  w.overlap_buf = hits;
  w.overlap_buf.len() as u32
}

/// Colliders currently intersecting `collider` via sensor/intersection pairs (as of the last step).
/// Fills the scratch buffer and returns the count.
#[no_mangle]
pub extern "C" fn shim_intersections_with(world: *mut PhysicsWorld, collider: u64) -> u32 {
  let w = unsafe { &mut *world };
  let h = unpack_col(collider);
  let hits: Vec<u64> = w
    .narrow_phase
    .intersection_pairs_with(h)
    .filter(|&(_, _, intersecting)| intersecting)
    .map(|(c1, c2, _)| pack_col(if c1 == h { c2 } else { c1 }))
    .collect();
  w.overlap_buf = hits;
  w.overlap_buf.len() as u32
}

// ---- filtered raycast -------------------------------------------------------------------------

/// Raycast honoring collision groups and optionally excluding one collider (0 = exclude none).
/// Returns 1 on hit (writes collider + distance), else 0.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn shim_query_raycast_filtered(
  world: *mut PhysicsWorld,
  ox: f32,
  oy: f32,
  dx: f32,
  dy: f32,
  max_toi: f32,
  memberships: u32,
  filter: u32,
  exclude: u64,
  out_collider: *mut u64,
  out_toi: *mut f32,
) -> i32 {
  let w = unsafe { &*world };
  let mut qf = QueryFilter::default().groups(InteractionGroups::new(
    Group::from_bits_truncate(memberships),
    Group::from_bits_truncate(filter),
    InteractionTestMode::And,
  ));
  if exclude != NULL_HANDLE {
    qf = qf.exclude_collider(unpack_col(exclude));
  }
  let qp = w.broad_phase.as_query_pipeline(w.narrow_phase.query_dispatcher(), &w.bodies, &w.colliders, qf);
  let ray = Ray::new(Vector::new(ox, oy), Vector::new(dx, dy));
  match qp.cast_ray(&ray, max_toi, true) {
    Some((handle, toi)) => {
      unsafe {
        *out_collider = pack_col(handle);
        *out_toi = toi;
      }
      1
    }
    None => 0,
  }
}

// ---- remaining world tuning -------------------------------------------------------------------

/// The simulation's length unit (≈ the size of a typical dynamic object, in world units). Tunes
/// internal tolerances; set once after creating the world if your units aren't ~1 = 1 meter.
#[no_mangle]
pub extern "C" fn shim_world_set_length_unit(world: *mut PhysicsWorld, length_unit: f32) {
  let w = unsafe { &mut *world };
  w.integration_parameters.length_unit = length_unit;
}

#[no_mangle]
pub extern "C" fn shim_world_set_max_ccd_substeps(world: *mut PhysicsWorld, substeps: u32) {
  let w = unsafe { &mut *world };
  w.integration_parameters.max_ccd_substeps = substeps as usize;
}

// ===============================================================================================
// Audit pass — close the last real gaps: contact-force events (were dropped), contact-manifold
// geometry, and the remaining body/collider setters. (Deliberately NOT bound: physics hooks — they
// need Rust callbacks, incompatible with this callback-free FFI; gyroscopic forces — 3D-only;
// serialization / debug-render buffers. Several Rapier convenience setters like set_vels /
// set_next_kinematic_position / set_position_wrt_parent are covered by the finer setters above.)
// ===============================================================================================

// ---- contact-force events ---------------------------------------------------------------------

/// Number of contact-force events queued since the last `shim_force_events_clear`.
#[no_mangle]
pub extern "C" fn shim_force_events_count(world: *mut PhysicsWorld) -> u32 {
  let w = unsafe { &*world };
  w.events.force_events.lock().unwrap().len() as u32
}

#[no_mangle]
pub extern "C" fn shim_force_events_get(world: *mut PhysicsWorld, i: u32, out: *mut ContactForceRecord) -> i32 {
  let w = unsafe { &*world };
  let ev = w.events.force_events.lock().unwrap();
  match ev.get(i as usize) {
    Some(r) => {
      unsafe { *out = *r };
      1
    }
    None => 0,
  }
}

#[no_mangle]
pub extern "C" fn shim_force_events_clear(world: *mut PhysicsWorld) {
  let w = unsafe { &*world };
  w.events.force_events.lock().unwrap().clear();
}

// ---- contact-manifold geometry ----------------------------------------------------------------

/// Geometry of the deepest current contact between two colliders (as of the last step): world-space
/// normal, world-space contact point, and penetration depth (positive = overlapping). Returns 1 if
/// the pair is in contact, else 0. For hit sparks, decals, surface-aligned effects.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn shim_contact_pair_info(
  world: *mut PhysicsWorld,
  c1: u64,
  c2: u64,
  out_nx: *mut f32,
  out_ny: *mut f32,
  out_px: *mut f32,
  out_py: *mut f32,
  out_depth: *mut f32,
) -> i32 {
  let w = unsafe { &*world };
  let h1 = unpack_col(c1);
  let pair = match w.narrow_phase.contact_pair(h1, unpack_col(c2)) {
    Some(p) => p,
    None => return 0,
  };
  let (manifold, contact) = match pair.find_deepest_contact() {
    Some(d) => d,
    None => return 0,
  };
  let normal = manifold.data.normal;
  // contact.local_p1 is in the *pair's* collider1 local frame (canonical handle order, not our query
  // order); lift it to world space (rotate by that collider's angle, then translate).
  let (wx, wy) = match w.colliders.get(pair.collider1) {
    Some(c) => {
      let pos = c.position();
      let (s, co) = pos.rotation.angle().sin_cos();
      let (lx, ly) = (contact.local_p1.x, contact.local_p1.y);
      (pos.translation.x + co * lx - s * ly, pos.translation.y + s * lx + co * ly)
    }
    None => (contact.local_p1.x, contact.local_p1.y),
  };
  unsafe {
    *out_nx = normal.x;
    *out_ny = normal.y;
    *out_px = wx;
    *out_py = wy;
    *out_depth = -contact.dist;
  }
  1
}

// ---- remaining body setters -------------------------------------------------------------------

/// Extra solver iterations for this body specifically (on top of the world's), for stiffer joints
/// or stacks on a high-priority object.
#[no_mangle]
pub extern "C" fn shim_body_set_additional_solver_iterations(world: *mut PhysicsWorld, handle: u64, iters: u32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    rb.set_additional_solver_iterations(iters as usize);
  }
}

/// Raw locked-axes bitmask: bit 0 = lock translation X, bit 1 = lock translation Y, bit 2 = lock
/// rotation. Combines the per-axis locks in one call.
#[no_mangle]
pub extern "C" fn shim_body_set_locked_axes(world: *mut PhysicsWorld, handle: u64, bits: u32) {
  let w = unsafe { &mut *world };
  if let Some(rb) = w.bodies.get_mut(unpack_body(handle)) {
    // Map our clean 2D bits (0=transX, 1=transY, 2=rotation) onto Rapier's 3D-layout flags.
    let mut axes = LockedAxes::empty();
    if bits & 0x1 != 0 {
      axes |= LockedAxes::TRANSLATION_LOCKED_X;
    }
    if bits & 0x2 != 0 {
      axes |= LockedAxes::TRANSLATION_LOCKED_Y;
    }
    if bits & 0x4 != 0 {
      axes |= LockedAxes::ROTATION_LOCKED;
    }
    rb.set_locked_axes(axes, true);
  }
}

// ---- remaining collider setters ---------------------------------------------------------------

/// A small extra contact margin (helps stability for thin/fast colliders).
#[no_mangle]
pub extern "C" fn shim_collider_set_contact_skin(world: *mut PhysicsWorld, handle: u64, skin: f32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_contact_skin(skin);
  }
}

fn combine_rule(r: i32) -> CoefficientCombineRule {
  match r {
    1 => CoefficientCombineRule::Min,
    2 => CoefficientCombineRule::Multiply,
    3 => CoefficientCombineRule::Max,
    _ => CoefficientCombineRule::Average,
  }
}

/// How this collider's friction combines with another's: 0=average, 1=min, 2=multiply, 3=max.
#[no_mangle]
pub extern "C" fn shim_collider_set_friction_combine_rule(world: *mut PhysicsWorld, handle: u64, rule: i32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_friction_combine_rule(combine_rule(rule));
  }
}

/// How this collider's restitution combines with another's: 0=average, 1=min, 2=multiply, 3=max.
#[no_mangle]
pub extern "C" fn shim_collider_set_restitution_combine_rule(world: *mut PhysicsWorld, handle: u64, rule: i32) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_restitution_combine_rule(combine_rule(rule));
  }
}

/// Set the collider's mass properties directly (mass, center of mass, scalar 2D inertia).
#[no_mangle]
pub extern "C" fn shim_collider_set_mass_properties(
  world: *mut PhysicsWorld,
  handle: u64,
  mass: f32,
  com_x: f32,
  com_y: f32,
  inertia: f32,
) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    c.set_mass_properties(MassProperties::new(Vector::new(com_x, com_y), mass, inertia));
  }
}

// ===============================================================================================
// Parity pass — capabilities the reference (rapier.js) binding also exposes: physics-hook pair
// filtering, gyroscopic forces, world snapshot/restore (serialization), and debug-render geometry.
// ===============================================================================================

// NOTE: gyroscopic forces (`RigidBody::enable_gyroscopic_forces`) are `#[cfg(feature = "dim3")]` in
// Rapier — the method does not exist in rapier2d, so it cannot be bound here (or in any 2D binding).

// ---- physics hooks ----------------------------------------------------------------------------

/// Install (or clear, with a null pointer) the contact-pair filter callback. The callback gets the
/// two packed collider handles and returns non-zero to allow the contact, 0 to suppress it. Only
/// consulted for colliders opted in via `shim_collider_set_active_hooks`.
#[no_mangle]
pub extern "C" fn shim_set_contact_filter(world: *mut PhysicsWorld, cb: Option<extern "C" fn(u64, u64) -> i32>) {
  let w = unsafe { &mut *world };
  w.hooks.contact_filter = cb;
}

/// Install (or clear) the intersection-pair (sensor) filter callback. Same contract as above.
#[no_mangle]
pub extern "C" fn shim_set_intersection_filter(world: *mut PhysicsWorld, cb: Option<extern "C" fn(u64, u64) -> i32>) {
  let w = unsafe { &mut *world };
  w.hooks.intersection_filter = cb;
}

/// Opt a collider into the hook callbacks (off by default, so hooks cost nothing unless requested).
#[no_mangle]
pub extern "C" fn shim_collider_set_active_hooks(world: *mut PhysicsWorld, handle: u64, contact: bool, intersection: bool) {
  let w = unsafe { &mut *world };
  if let Some(c) = w.colliders.get_mut(unpack_col(handle)) {
    let mut h = ActiveHooks::empty();
    if contact {
      h |= ActiveHooks::FILTER_CONTACT_PAIRS;
    }
    if intersection {
      h |= ActiveHooks::FILTER_INTERSECTION_PAIR;
    }
    c.set_active_hooks(h);
  }
}

// ---- world snapshot / restore (serialization) -------------------------------------------------

type Snapshot = (
  Vector,
  IntegrationParameters,
  IslandManager,
  DefaultBroadPhase,
  NarrowPhase,
  RigidBodySet,
  ColliderSet,
  ImpulseJointSet,
  MultibodyJointSet,
);

/// Serialize the full simulation state to a freshly-allocated byte buffer (length written to
/// `out_len`). Free it with `shim_buffer_free`. Pipelines/solver/hooks/event queues are transient
/// and not part of the snapshot. For deterministic save states & lockstep/rollback netcode.
#[no_mangle]
pub extern "C" fn shim_world_snapshot(world: *mut PhysicsWorld, out_len: *mut u32) -> *mut u8 {
  let w = unsafe { &*world };
  let snap = (
    &w.gravity,
    &w.integration_parameters,
    &w.islands,
    &w.broad_phase,
    &w.narrow_phase,
    &w.bodies,
    &w.colliders,
    &w.impulse_joints,
    &w.multibody_joints,
  );
  match bincode::serialize(&snap) {
    Ok(mut data) => {
      data.shrink_to_fit();
      let ptr = data.as_mut_ptr();
      unsafe { *out_len = data.len() as u32 };
      std::mem::forget(data);
      ptr
    }
    Err(_) => {
      unsafe { *out_len = 0 };
      std::ptr::null_mut()
    }
  }
}

/// Rebuild a world from a snapshot (see `shim_world_snapshot`). Returns a new world to free with
/// `shim_world_free`, or null if the bytes can't be decoded.
#[no_mangle]
pub extern "C" fn shim_world_restore(bytes: *const u8, len: u32) -> *mut PhysicsWorld {
  let slice = unsafe { std::slice::from_raw_parts(bytes, len as usize) };
  let snap: Snapshot = match bincode::deserialize(slice) {
    Ok(v) => v,
    Err(_) => return std::ptr::null_mut(),
  };
  let (gravity, integration_parameters, islands, broad_phase, narrow_phase, bodies, colliders, impulse_joints, multibody_joints) = snap;
  let world = PhysicsWorld {
    pipeline: PhysicsPipeline::new(),
    gravity,
    integration_parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver: CCDSolver::new(),
    events: CollisionCollector::default(),
    hooks: ShimHooks::default(),
    last_dt: 1.0 / 60.0,
    overlap_buf: Vec::new(),
  };
  Box::into_raw(Box::new(world))
}

/// Free a byte buffer returned by `shim_world_snapshot`.
#[no_mangle]
pub extern "C" fn shim_buffer_free(ptr: *mut u8, len: u32) {
  if !ptr.is_null() {
    unsafe { drop(Vec::from_raw_parts(ptr, len as usize, len as usize)) };
  }
}

// ---- debug render -----------------------------------------------------------------------------

/// Collects debug lines as a flat `[ax, ay, bx, by, ...]` vertex buffer.
struct LineCollector {
  verts: Vec<f32>,
}

impl DebugRenderBackend for LineCollector {
  fn draw_line(&mut self, _object: DebugRenderObject, a: Vector, b: Vector, _color: DebugColor) {
    self.verts.extend_from_slice(&[a.x, a.y, b.x, b.y]);
  }
}

/// Render the world's collision shapes (and joints) to a flat line-vertex buffer: `count` floats as
/// `[ax, ay, bx, by, ...]`, one line per 4 floats. Free with `shim_debug_buffer_free`. Lets a
/// renderer draw Rapier's own debug view instead of reconstructing shapes by hand.
#[no_mangle]
pub extern "C" fn shim_debug_render(world: *mut PhysicsWorld, out_count: *mut u32) -> *mut f32 {
  let w = unsafe { &*world };
  let mut backend = LineCollector { verts: Vec::new() };
  let mut pipeline = DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::all());
  pipeline.render(
    &mut backend,
    &w.bodies,
    &w.colliders,
    &w.impulse_joints,
    &w.multibody_joints,
    &w.narrow_phase,
  );
  let mut data = backend.verts;
  data.shrink_to_fit();
  let ptr = data.as_mut_ptr();
  unsafe { *out_count = data.len() as u32 };
  std::mem::forget(data);
  ptr
}

/// Free a vertex buffer returned by `shim_debug_render`.
#[no_mangle]
pub extern "C" fn shim_debug_buffer_free(ptr: *mut f32, count: u32) {
  if !ptr.is_null() {
    unsafe { drop(Vec::from_raw_parts(ptr, count as usize, count as usize)) };
  }
}
