// C-ABI shim over Rapier2D for project-ignis (called from LuaJIT FFI via src/util/physics).
//
// Everything crosses the boundary as opaque pointers + primitives + #[repr(C)] structs — never
// Rust types (no String/Vec/enums/generics), because LuaJIT's FFI binds the C ABI, not Rust's.
// Rapier handles are (index, generation); we pack them into a u64 for the boundary.
//
// Surface: world + step; bodies (dynamic/fixed/kinematic, velocity/impulse/force, damping, CCD);
// colliders (ball/cuboid/capsule, attached or static, sensors, collision groups); collision &
// sensor events (drained after each step); the query pipeline (raycast / point / overlap); and a
// kinematic character controller. Bound generously — the build cost is fixed regardless of count.

use rapier2d::control::KinematicCharacterController;
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

/// Accumulates collision events during a step; drained by the Lua adapter each frame.
/// Interior mutability (Mutex) because `EventHandler` methods take `&self` while the step
/// borrows the rest of the world mutably (disjoint field borrows).
#[derive(Default)]
struct CollisionCollector {
  events: Mutex<Vec<ContactRecord>>,
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
    _contact_pair: &ContactPair,
    _total_force_magnitude: Real,
  ) {
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
    &(),
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
