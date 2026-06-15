-- Raw LuaJIT FFI binding to the Rapier shim (native/rapier_shim -> lib/native/<platform>/).
--
-- This is the thin, header-mirroring layer: an `ffi.cdef` matching the shim's `extern "C"` surface
-- plus the per-platform `ffi.load`. Nothing idiomatic lives here — `rapier` (init.lua) wraps
-- this into a friendly Lua API; `rapier.system` is the game adapter. Regenerate the C header with
-- `cbindgen` (see native/rapier_shim/cbindgen.toml) and keep this cdef in sync with it.

local ffi = require("ffi")

ffi.cdef([[
typedef struct PhysicsWorld PhysicsWorld;

typedef struct { uint64_t a; uint64_t b; int32_t started; int32_t sensor; } ContactRecord;

typedef struct { uint64_t a; uint64_t b; float magnitude; } ContactForceRecord;

/* world */
PhysicsWorld* shim_world_new(void);
void          shim_world_free(PhysicsWorld* world);
void          shim_world_step(PhysicsWorld* world, float dt);

/* events */
uint32_t      shim_events_count(PhysicsWorld* world);
int32_t       shim_events_get(PhysicsWorld* world, uint32_t i, ContactRecord* out);
void          shim_events_clear(PhysicsWorld* world);

/* bodies (kind: 0=dynamic, 1=fixed, 2=kinematic-velocity) */
uint64_t      shim_body_create(PhysicsWorld* world, int32_t kind, float x, float y);
void          shim_body_remove(PhysicsWorld* world, uint64_t handle);
void          shim_body_position(PhysicsWorld* world, uint64_t handle, float* out_x, float* out_y);
void          shim_bodies_read_transforms(PhysicsWorld* world, const uint64_t* handles, uint32_t count, float* out);
float         shim_body_rotation(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_translation(PhysicsWorld* world, uint64_t handle, float x, float y);
void          shim_body_set_linvel(PhysicsWorld* world, uint64_t handle, float vx, float vy);
void          shim_body_linvel(PhysicsWorld* world, uint64_t handle, float* out_x, float* out_y);
void          shim_body_lock_rotations(PhysicsWorld* world, uint64_t handle, bool locked);
void          shim_body_set_linear_damping(PhysicsWorld* world, uint64_t handle, float damping);
void          shim_body_apply_impulse(PhysicsWorld* world, uint64_t handle, float x, float y);
void          shim_body_add_force(PhysicsWorld* world, uint64_t handle, float x, float y);
void          shim_body_enable_ccd(PhysicsWorld* world, uint64_t handle, bool enabled);

/* colliders (shape: 0=ball[a=radius], 1=cuboid[a=hx,b=hy], 2=capsule[a=half_h,b=radius]) */
uint64_t      shim_collider_attach(PhysicsWorld* world, uint64_t body, int32_t shape, float a, float b);
uint64_t      shim_collider_static(PhysicsWorld* world, int32_t shape, float x, float y, float a, float b);
void          shim_collider_remove(PhysicsWorld* world, uint64_t handle);
void          shim_collider_set_sensor(PhysicsWorld* world, uint64_t handle, bool sensor);
void          shim_collider_set_groups(PhysicsWorld* world, uint64_t handle, uint32_t memberships, uint32_t filter);

/* queries */
int32_t       shim_query_raycast(PhysicsWorld* world, float ox, float oy, float dx, float dy,
                                 float max_toi, uint64_t* out_collider, float* out_toi);
int32_t       shim_query_point(PhysicsWorld* world, float px, float py, uint64_t* out_collider);
uint32_t      shim_query_overlap_circle(PhysicsWorld* world, float x, float y, float radius);
uint32_t      shim_overlap_count(PhysicsWorld* world);
uint64_t      shim_overlap_get(PhysicsWorld* world, uint32_t i);

/* kinematic character controller */
void          shim_kcc_move_ball(PhysicsWorld* world, float x, float y, float radius,
                                 float dx, float dy, float* out_x, float* out_y);

/* ---- extended surface --------------------------------------------------------------------- */

/* world config */
void          shim_world_set_gravity(PhysicsWorld* world, float gx, float gy);
void          shim_world_set_solver_iterations(PhysicsWorld* world, uint32_t iters);

/* bodies: angular + extended control + properties */
float         shim_body_angvel(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_angvel(PhysicsWorld* world, uint64_t handle, float av);
void          shim_body_set_rotation(PhysicsWorld* world, uint64_t handle, float angle);
void          shim_body_apply_torque_impulse(PhysicsWorld* world, uint64_t handle, float torque);
void          shim_body_add_torque(PhysicsWorld* world, uint64_t handle, float torque);
void          shim_body_apply_impulse_at_point(PhysicsWorld* world, uint64_t handle,
                                               float ix, float iy, float px, float py);
void          shim_body_add_force_at_point(PhysicsWorld* world, uint64_t handle,
                                           float fx, float fy, float px, float py);
void          shim_body_set_angular_damping(PhysicsWorld* world, uint64_t handle, float damping);
void          shim_body_set_gravity_scale(PhysicsWorld* world, uint64_t handle, float scale);
void          shim_body_set_type(PhysicsWorld* world, uint64_t handle, int32_t kind);
void          shim_body_set_additional_mass(PhysicsWorld* world, uint64_t handle, float mass);
float         shim_body_mass(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_enabled(PhysicsWorld* world, uint64_t handle, bool enabled);
void          shim_body_wake_up(PhysicsWorld* world, uint64_t handle);
void          shim_body_sleep(PhysicsWorld* world, uint64_t handle);
int32_t       shim_body_is_sleeping(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_dominance(PhysicsWorld* world, uint64_t handle, int32_t group);
void          shim_body_set_next_kinematic_translation(PhysicsWorld* world, uint64_t handle,
                                                       float x, float y);
void          shim_body_set_next_kinematic_rotation(PhysicsWorld* world, uint64_t handle, float angle);

/* colliders: materials + config + offsets */
void          shim_collider_set_friction(PhysicsWorld* world, uint64_t handle, float friction);
void          shim_collider_set_restitution(PhysicsWorld* world, uint64_t handle, float restitution);
void          shim_collider_set_density(PhysicsWorld* world, uint64_t handle, float density);
void          shim_collider_set_mass(PhysicsWorld* world, uint64_t handle, float mass);
void          shim_collider_set_translation_wrt_parent(PhysicsWorld* world, uint64_t handle,
                                                       float x, float y);
void          shim_collider_set_rotation_wrt_parent(PhysicsWorld* world, uint64_t handle, float angle);
void          shim_collider_set_solver_groups(PhysicsWorld* world, uint64_t handle,
                                              uint32_t memberships, uint32_t filter);
void          shim_collider_set_enabled(PhysicsWorld* world, uint64_t handle, bool enabled);
void          shim_collider_position(PhysicsWorld* world, uint64_t handle, float* out_x, float* out_y);
void          shim_collider_set_translation(PhysicsWorld* world, uint64_t handle, float x, float y);

/* colliders: extended static shapes */
uint64_t      shim_collider_static_triangle(PhysicsWorld* world, float ax, float ay,
                                            float bx, float by, float cx, float cy);
uint64_t      shim_collider_static_segment(PhysicsWorld* world, float ax, float ay, float bx, float by);
uint64_t      shim_collider_static_polyline(PhysicsWorld* world, const float* points, uint32_t count);
uint64_t      shim_collider_static_convex_hull(PhysicsWorld* world, const float* points, uint32_t count);
uint64_t      shim_collider_static_heightfield(PhysicsWorld* world, const float* heights,
                                               uint32_t count, float scale_x, float scale_y);

/* joints */
uint64_t      shim_joint_fixed(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                               float a1x, float a1y, float a2x, float a2y);
uint64_t      shim_joint_revolute(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                                  float a1x, float a1y, float a2x, float a2y);
uint64_t      shim_joint_prismatic(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                                   float a1x, float a1y, float a2x, float a2y,
                                   float axis_x, float axis_y);
uint64_t      shim_joint_rope(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                              float a1x, float a1y, float a2x, float a2y, float max_dist);
uint64_t      shim_joint_spring(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                                float a1x, float a1y, float a2x, float a2y,
                                float rest_length, float stiffness, float damping);
void          shim_joint_remove(PhysicsWorld* world, uint64_t handle);

/* extended queries */
int32_t       shim_query_raycast_normal(PhysicsWorld* world, float ox, float oy, float dx, float dy,
                                        float max_toi, uint64_t* out_collider, float* out_toi,
                                        float* out_nx, float* out_ny);
int32_t       shim_query_shapecast(PhysicsWorld* world, int32_t shape, float a, float b,
                                   float ox, float oy, float angle, float dx, float dy,
                                   float max_toi, uint64_t* out_collider, float* out_toi);
int32_t       shim_query_project_point(PhysicsWorld* world, float px, float py, float max_dist,
                                       uint64_t* out_collider, float* out_x, float* out_y,
                                       int32_t* out_inside);
uint32_t      shim_query_overlap_shape(PhysicsWorld* world, int32_t shape, float a, float b,
                                       float x, float y, float angle);

/* generalized kinematic character controller */
void          shim_kcc_move(PhysicsWorld* world, int32_t shape, float a, float b,
                            float x, float y, float angle, float dx, float dy, float offset,
                            float* out_x, float* out_y, int32_t* out_grounded);

/* ---- completion pass ---------------------------------------------------------------------- */

/* joint motors + limits (axis: 0=linear-x, 1=linear-y, 2=angular) */
void          shim_joint_set_motor(PhysicsWorld* world, uint64_t joint, int32_t axis,
                                   float target_pos, float target_vel, float stiffness, float damping);
void          shim_joint_set_limits(PhysicsWorld* world, uint64_t joint, int32_t axis, float min, float max);
void          shim_joint_set_motor_max_force(PhysicsWorld* world, uint64_t joint, int32_t axis, float max_force);
void          shim_joint_set_contacts_enabled(PhysicsWorld* world, uint64_t joint, bool enabled);

/* multibody (articulated) joints */
uint64_t      shim_multibody_joint_fixed(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                                         float a1x, float a1y, float a2x, float a2y);
uint64_t      shim_multibody_joint_revolute(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                                            float a1x, float a1y, float a2x, float a2y);
uint64_t      shim_multibody_joint_prismatic(PhysicsWorld* world, uint64_t b1, uint64_t b2,
                                             float a1x, float a1y, float a2x, float a2y,
                                             float axis_x, float axis_y);
void          shim_multibody_joint_remove(PhysicsWorld* world, uint64_t handle);

/* bodies: locks, forces, full pose, mass properties, reads */
void          shim_body_lock_translations(PhysicsWorld* world, uint64_t handle, bool locked);
void          shim_body_set_enabled_translations(PhysicsWorld* world, uint64_t handle,
                                                 bool allow_x, bool allow_y);
void          shim_body_reset_forces(PhysicsWorld* world, uint64_t handle);
void          shim_body_reset_torques(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_position(PhysicsWorld* world, uint64_t handle, float x, float y, float angle);
int32_t       shim_body_type(PhysicsWorld* world, uint64_t handle);
int32_t       shim_body_is_enabled(PhysicsWorld* world, uint64_t handle);
void          shim_body_center_of_mass(PhysicsWorld* world, uint64_t handle, float* out_x, float* out_y);
void          shim_body_set_additional_mass_properties(PhysicsWorld* world, uint64_t handle,
                                                       float mass, float com_x, float com_y, float inertia);
void          shim_body_recompute_mass(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_soft_ccd_prediction(PhysicsWorld* world, uint64_t handle, float distance);

/* colliders: shape swap, events, reads */
void          shim_collider_set_active_events(PhysicsWorld* world, uint64_t handle,
                                              bool collision, bool contact_force);
void          shim_collider_set_active_collision_types(PhysicsWorld* world, uint64_t handle, uint32_t bits);
void          shim_collider_set_contact_force_threshold(PhysicsWorld* world, uint64_t handle, float threshold);
void          shim_collider_set_shape(PhysicsWorld* world, uint64_t handle, int32_t shape, float a, float b);
void          shim_collider_set_rotation(PhysicsWorld* world, uint64_t handle, float angle);
float         shim_collider_density(PhysicsWorld* world, uint64_t handle);
float         shim_collider_mass(PhysicsWorld* world, uint64_t handle);
float         shim_collider_volume(PhysicsWorld* world, uint64_t handle);
uint64_t      shim_collider_parent(PhysicsWorld* world, uint64_t handle);
int32_t       shim_collider_is_sensor(PhysicsWorld* world, uint64_t handle);

/* narrow-phase contact / intersection queries (fill the overlap scratch buffer) */
uint32_t      shim_contacts_with(PhysicsWorld* world, uint64_t collider);
uint32_t      shim_intersections_with(PhysicsWorld* world, uint64_t collider);

/* filtered raycast */
int32_t       shim_query_raycast_filtered(PhysicsWorld* world, float ox, float oy, float dx, float dy,
                                          float max_toi, uint32_t memberships, uint32_t filter,
                                          uint64_t exclude, uint64_t* out_collider, float* out_toi);

/* remaining world tuning */
void          shim_world_set_length_unit(PhysicsWorld* world, float length_unit);
void          shim_world_set_max_ccd_substeps(PhysicsWorld* world, uint32_t substeps);

/* ---- audit pass: contact-force events, contact geometry, last setters --------------------- */

/* contact-force events */
uint32_t      shim_force_events_count(PhysicsWorld* world);
int32_t       shim_force_events_get(PhysicsWorld* world, uint32_t i, ContactForceRecord* out);
void          shim_force_events_clear(PhysicsWorld* world);

/* contact-manifold geometry */
int32_t       shim_contact_pair_info(PhysicsWorld* world, uint64_t c1, uint64_t c2,
                                     float* out_nx, float* out_ny, float* out_px, float* out_py,
                                     float* out_depth);

/* remaining body + collider setters */
void          shim_body_set_additional_solver_iterations(PhysicsWorld* world, uint64_t handle, uint32_t iters);
void          shim_body_set_locked_axes(PhysicsWorld* world, uint64_t handle, uint32_t bits);
void          shim_collider_set_contact_skin(PhysicsWorld* world, uint64_t handle, float skin);
void          shim_collider_set_friction_combine_rule(PhysicsWorld* world, uint64_t handle, int32_t rule);
void          shim_collider_set_restitution_combine_rule(PhysicsWorld* world, uint64_t handle, int32_t rule);
void          shim_collider_set_mass_properties(PhysicsWorld* world, uint64_t handle,
                                                float mass, float com_x, float com_y, float inertia);

/* ---- parity pass: hooks, serialization, debug-render -------------------------------------- */

/* physics hooks — pair-filter callbacks (return non-zero to keep the pair, 0 to discard) */
void          shim_set_contact_filter(PhysicsWorld* world, int32_t (*cb)(uint64_t, uint64_t));
void          shim_set_intersection_filter(PhysicsWorld* world, int32_t (*cb)(uint64_t, uint64_t));
void          shim_collider_set_active_hooks(PhysicsWorld* world, uint64_t handle,
                                             bool contact, bool intersection);

/* world snapshot / restore (serialization) */
uint8_t*      shim_world_snapshot(PhysicsWorld* world, uint32_t* out_len);
PhysicsWorld* shim_world_restore(const uint8_t* bytes, uint32_t len);
void          shim_buffer_free(uint8_t* ptr, uint32_t len);

/* debug-render line geometry (flat [ax,ay,bx,by,...]) */
float*        shim_debug_render(PhysicsWorld* world, uint32_t* out_count);
void          shim_debug_buffer_free(float* ptr, uint32_t count);
]])

-- Resolve lib/native/<platform>/<libname> for the current OS+arch.
local function platformDir()
  local os, arch = jit.os, jit.arch
  if os == "OSX" then
    return arch == "arm64" and "macos-arm64" or "macos-x86_64"
  elseif os == "Windows" then
    return "windows-x64"
  elseif os == "Linux" then
    return "linux-x64"
  end
  error("rapier shim: unsupported platform " .. tostring(os) .. "/" .. tostring(arch))
end

local function libName()
  if jit.os == "OSX" then
    return "librapier_shim.dylib"
  elseif jit.os == "Windows" then
    return "rapier_shim.dll"
  end
  return "librapier_shim.so"
end

-- Base dir: LÖVE's project source when running the game, else the current dir (tests/CLI).
local function baseDir()
  if rawget(_G, "love") and love.filesystem and love.filesystem.getSource then
    return love.filesystem.getSource()
  end
  return os.getenv("RAPIER_ROOT") or "."
end

local path = baseDir() .. "/lib/native/" .. platformDir() .. "/" .. libName()
if jit.os == "Windows" then
  path = path:gsub("/", "\\") -- LoadLibrary is unreliable with forward slashes
end
local C = ffi.load(path)

-- shim_* come from the ffi.cdef above; LuaLS can't introspect cdef strings, so type the namespace
-- structurally (string keys → callables). Inline type (not a named @class), so it doesn't collide
-- when this file is vendored into a consumer and both copies are analyzed at once.
return C --[[@as table<string, function>]]
