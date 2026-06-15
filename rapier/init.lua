-- Idiomatic Lua wrapper over the raw Rapier FFI binding (rapier.ffi).
--
-- Turns the flat C surface into a `World` object with methods. Handles (bodies/colliders) are
-- returned as opaque `uint64_t` cdata — pass them straight back into the methods; don't do math on
-- them. The world frees itself via `ffi.gc`. This layer is generic (game-agnostic); the game-facing
-- fixed-timestep / event-dispatch / static-geometry adapter is `rapier.system`.
--
-- Shape codes: "ball" {radius}, "cuboid" {hx,hy}, "capsule" {half_height,radius}.
-- Body kinds:  "dynamic", "fixed", "kinematic".

local ffi = require("ffi")
local C = require("rapier.ffi")

local physics = {}

-- Reused scratch buffers so per-call queries don't allocate cdata every frame. The `@as` casts on
-- the ctype strings work around LuaLS's bundled FFI defs typing ffi.typeof's param too narrowly
-- (it rejects plain strings, though string ctypes are the canonical FFI usage).
local float1 = ffi.typeof("float[1]" --[[@as ffi.ctype*]])
local f2a = float1()
local f2b = float1()
local f2c = float1()
local f2d = float1()
local f2e = float1()
local u64 = ffi.typeof("uint64_t[1]" --[[@as ffi.ctype*]])()
local u32 = ffi.typeof("uint32_t[1]" --[[@as ffi.ctype*]])()
local i32 = ffi.typeof("int32_t[1]" --[[@as ffi.ctype*]])()
local rec = ffi.typeof("ContactRecord[1]" --[[@as ffi.ctype*]])()
local frec = ffi.typeof("ContactForceRecord[1]" --[[@as ffi.ctype*]])()

-- 0=dynamic, 1=fixed, 2=kinematic-velocity, 3=kinematic-position (matches the shim's codes).
local BODY_KIND = { dynamic = 0, fixed = 1, kinematic = 2, kinematicPosition = 3 }
local SHAPE = { ball = 0, cuboid = 1, capsule = 2 }

-- "No handle" sentinel (u64::MAX), matching the shim. Handle 0 is a *valid* handle (the first
-- body/collider packs to 0), so it can't double as null.
local NULL = 0xFFFFFFFFFFFFFFFFULL

---@class PhysicsWorld
---@field _w userdata  -- ffi cdata: PhysicsWorld* (gc-managed)
local World = {}
World.__index = World

--- Create a new physics world (top-down, zero gravity).
---@return PhysicsWorld
function physics.newWorld()
  local ptr = C.shim_world_new()
  return setmetatable({ _w = ffi.gc(ptr, C.shim_world_free) }, World)
end

--- Advance the simulation by `dt` seconds. Callers should drive this on a fixed timestep.
function World:step(dt)
  C.shim_world_step(self._w, dt)
end

-- ---- bodies ---------------------------------------------------------------------------------

--- Create a rigid body. `kind` is "dynamic" | "fixed" | "kinematic".
---@return userdata handle
function World:newBody(kind, x, y)
  return C.shim_body_create(self._w, BODY_KIND[kind] or 0, x, y)
end

function World:removeBody(h)
  C.shim_body_remove(self._w, h)
end

--- @return number x, number y
function World:position(h)
  C.shim_body_position(self._w, h, f2a, f2b)
  return f2a[0], f2b[0]
end

--- Batched position read — one FFI call for many bodies instead of one `position()` per body.
--- `handles` is a `uint64_t[count]` cdata of body handles, `out` a `float[2*count]` cdata; on return
--- `out[2*i], out[2*i+1]` are body i's x,y. Keep both buffers persistent across frames (no per-call
--- alloc). For large body counts this avoids O(N) boundary crossings.
function World:readTransforms(handles, count, out)
  C.shim_bodies_read_transforms(self._w, handles, count, out)
  return out
end

--- @return number radians
function World:rotation(h)
  return C.shim_body_rotation(self._w, h)
end

function World:setPosition(h, x, y)
  C.shim_body_set_translation(self._w, h, x, y)
end

function World:setLinvel(h, vx, vy)
  C.shim_body_set_linvel(self._w, h, vx, vy)
end

--- @return number vx, number vy
function World:linvel(h)
  C.shim_body_linvel(self._w, h, f2a, f2b)
  return f2a[0], f2b[0]
end

function World:lockRotations(h, locked)
  C.shim_body_lock_rotations(self._w, h, locked ~= false)
end

function World:setLinearDamping(h, damping)
  C.shim_body_set_linear_damping(self._w, h, damping)
end

--- Instantaneous impulse — knockback / explosion shove.
function World:applyImpulse(h, x, y)
  C.shim_body_apply_impulse(self._w, h, x, y)
end

--- Force applied over the next step — sustained push.
function World:addForce(h, x, y)
  C.shim_body_add_force(self._w, h, x, y)
end

function World:enableCcd(h, enabled)
  C.shim_body_enable_ccd(self._w, h, enabled ~= false)
end

-- ---- colliders ------------------------------------------------------------------------------

-- shapeArgs: ball -> (radius), cuboid -> (hx, hy), capsule -> (half_height, radius)
local function shapeAB(shape, a, b)
  return SHAPE[shape] or 0, a or 0.5, b or 0.0
end

--- Attach a collider to a body. Collision events are enabled by default.
---@return userdata handle
function World:attachCollider(body, shape, a, b)
  local s, av, bv = shapeAB(shape, a, b)
  return C.shim_collider_attach(self._w, body, s, av, bv)
end

--- Free-standing static collider at (x,y) — map walls/obstacles, no body.
---@return userdata handle
function World:staticCollider(shape, x, y, a, b)
  local s, av, bv = shapeAB(shape, a, b)
  return C.shim_collider_static(self._w, s, x, y, av, bv)
end

function World:removeCollider(h)
  C.shim_collider_remove(self._w, h)
end

function World:setSensor(h, sensor)
  C.shim_collider_set_sensor(self._w, h, sensor ~= false)
end

--- Collision filtering. `memberships`/`filter` are 32-bit group masks; two colliders interact iff
--- each is in the other's filter.
function World:setGroups(h, memberships, filter)
  C.shim_collider_set_groups(self._w, h, memberships, filter)
end

-- ---- queries --------------------------------------------------------------------------------
-- NOTE: spatial queries read the broad-phase BVH, which is (re)built during `step()`. A collider
-- added since the last step is not visible to a query until one `step()` has run (same contract as
-- raw Rapier's QueryPipeline.update). In a running game this is moot — you step every frame before
-- querying — but if you query a freshly-built, never-stepped world, step it once first.

--- Raycast. Returns (colliderHandle, distance) on hit, or nil.
function World:raycast(ox, oy, dx, dy, maxToi)
  local hit = C.shim_query_raycast(self._w, ox, oy, dx, dy, maxToi or 1e6, u64, f2a)
  if hit == 1 then
    return u64[0], f2a[0]
  end
  return nil
end

--- First collider containing the point, or nil.
function World:pointQuery(px, py)
  if C.shim_query_point(self._w, px, py, u64) == 1 then
    return u64[0]
  end
  return nil
end

--- All colliders overlapping a circle (AoE). Returns an array of collider handles.
function World:overlapCircle(x, y, radius)
  local n = tonumber(C.shim_query_overlap_circle(self._w, x, y, radius))
  local out = {}
  for i = 0, n - 1 do
    out[i + 1] = C.shim_overlap_get(self._w, i)
  end
  return out
end

--- Kinematic character move: corrected translation for a ball at (x,y) wanting to move (dx,dy).
--- Returns (allowedDx, allowedDy).
function World:moveBall(x, y, radius, dx, dy)
  C.shim_kcc_move_ball(self._w, x, y, radius, dx, dy, f2a, f2b)
  return f2a[0], f2b[0]
end

-- ---- events ---------------------------------------------------------------------------------

--- Drain queued collision/sensor events since the last drain. Returns an array of
--- `{ a, b, started, sensor }` (a/b are collider handles; started/sensor are booleans), and clears
--- the queue. Call once per frame after stepping.
function World:drainEvents()
  local n = tonumber(C.shim_events_count(self._w))
  local out = {}
  for i = 0, n - 1 do
    if C.shim_events_get(self._w, i, rec) == 1 then
      out[i + 1] = {
        a = rec[0].a,
        b = rec[0].b,
        started = rec[0].started == 1,
        sensor = rec[0].sensor == 1,
      }
    end
  end
  C.shim_events_clear(self._w)
  return out
end

-- ---- world config ---------------------------------------------------------------------------

--- Set gravity (default is zero — top-down). Side-scrollers use e.g. (0, 9.81).
function World:setGravity(gx, gy)
  C.shim_world_set_gravity(self._w, gx, gy)
end

--- Constraint solver iterations per step (higher = stiffer/more accurate, slower).
function World:setSolverIterations(n)
  C.shim_world_set_solver_iterations(self._w, n)
end

-- ---- bodies: angular + extended control + properties ----------------------------------------

--- @return number radians_per_second
function World:angvel(h)
  return C.shim_body_angvel(self._w, h)
end

function World:setAngvel(h, av)
  C.shim_body_set_angvel(self._w, h, av)
end

function World:setRotation(h, angle)
  C.shim_body_set_rotation(self._w, h, angle)
end

--- Instantaneous angular impulse (spin shove).
function World:applyTorqueImpulse(h, torque)
  C.shim_body_apply_torque_impulse(self._w, h, torque)
end

--- Torque applied over the next step (sustained spin).
function World:addTorque(h, torque)
  C.shim_body_add_torque(self._w, h, torque)
end

--- Off-center impulse at world point (px,py) — induces both linear and angular response.
function World:applyImpulseAtPoint(h, ix, iy, px, py)
  C.shim_body_apply_impulse_at_point(self._w, h, ix, iy, px, py)
end

--- Off-center force (over the next step) at world point (px,py).
function World:addForceAtPoint(h, fx, fy, px, py)
  C.shim_body_add_force_at_point(self._w, h, fx, fy, px, py)
end

function World:setAngularDamping(h, damping)
  C.shim_body_set_angular_damping(self._w, h, damping)
end

--- Per-body gravity multiplier (0 = unaffected by gravity, 2 = double, etc.).
function World:setGravityScale(h, scale)
  C.shim_body_set_gravity_scale(self._w, h, scale)
end

--- Change body type at runtime. `kind` is "dynamic" | "fixed" | "kinematic" | "kinematicPosition".
function World:setBodyType(h, kind)
  C.shim_body_set_type(self._w, h, BODY_KIND[kind] or 0)
end

--- Add to the body's computed (collider-derived) mass.
function World:setAdditionalMass(h, mass)
  C.shim_body_set_additional_mass(self._w, h, mass)
end

--- @return number
function World:mass(h)
  return C.shim_body_mass(self._w, h)
end

function World:setEnabled(h, enabled)
  C.shim_body_set_enabled(self._w, h, enabled ~= false)
end

function World:wakeUp(h)
  C.shim_body_wake_up(self._w, h)
end

function World:sleep(h)
  C.shim_body_sleep(self._w, h)
end

--- @return boolean
function World:isSleeping(h)
  return C.shim_body_is_sleeping(self._w, h) == 1
end

--- Dominance group (-127..127): a higher-dominance body is immovable by lower ones.
function World:setDominance(h, group)
  C.shim_body_set_dominance(self._w, h, group)
end

--- For kinematic-position bodies: the translation reached at the end of the next step (physics
--- interpolates and generates contacts along the way).
function World:setNextKinematicTranslation(h, x, y)
  C.shim_body_set_next_kinematic_translation(self._w, h, x, y)
end

function World:setNextKinematicRotation(h, angle)
  C.shim_body_set_next_kinematic_rotation(self._w, h, angle)
end

-- ---- colliders: materials + config + offsets ------------------------------------------------

function World:setFriction(h, friction)
  C.shim_collider_set_friction(self._w, h, friction)
end

function World:setRestitution(h, restitution)
  C.shim_collider_set_restitution(self._w, h, restitution)
end

function World:setDensity(h, density)
  C.shim_collider_set_density(self._w, h, density)
end

function World:setColliderMass(h, mass)
  C.shim_collider_set_mass(self._w, h, mass)
end

--- Offset of an attached collider relative to its parent body's frame.
function World:setTranslationWrtParent(h, x, y)
  C.shim_collider_set_translation_wrt_parent(self._w, h, x, y)
end

function World:setRotationWrtParent(h, angle)
  C.shim_collider_set_rotation_wrt_parent(self._w, h, angle)
end

--- Solver groups (which colliders exert contact *forces*), distinct from collision groups (which
--- generate contact *events*). Same bitmask semantics as `setGroups`.
function World:setSolverGroups(h, memberships, filter)
  C.shim_collider_set_solver_groups(self._w, h, memberships, filter)
end

function World:setColliderEnabled(h, enabled)
  C.shim_collider_set_enabled(self._w, h, enabled ~= false)
end

--- @return number x, number y
function World:colliderPosition(h)
  C.shim_collider_position(self._w, h, f2a, f2b)
  return f2a[0], f2b[0]
end

--- Teleport a free-standing (static) collider.
function World:setColliderTranslation(h, x, y)
  C.shim_collider_set_translation(self._w, h, x, y)
end

-- ---- colliders: extended static shapes (map geometry) ---------------------------------------

--- @return userdata handle
function World:staticTriangle(ax, ay, bx, by, cx, cy)
  return C.shim_collider_static_triangle(self._w, ax, ay, bx, by, cx, cy)
end

--- @return userdata handle
function World:staticSegment(ax, ay, bx, by)
  return C.shim_collider_static_segment(self._w, ax, ay, bx, by)
end

-- Pack an array of {x,y} points (or a flat {x1,y1,x2,y2,...}) into a float[2n] cdata + count.
local function packPoints(points)
  local flat = type(points[1]) == "table"
  local n = flat and #points or #points / 2
  local buf = ffi.new("float[?]", n * 2)
  if flat then
    for i = 1, n do
      buf[(i - 1) * 2] = points[i][1]
      buf[(i - 1) * 2 + 1] = points[i][2]
    end
  else
    for i = 0, n * 2 - 1 do buf[i] = points[i + 1] end
  end
  return buf, n
end

--- Open chain of segments from a list of points (`{{x,y},...}` or flat `{x1,y1,...}`). Map walls.
--- @return userdata handle
function World:staticPolyline(points)
  local buf, n = packPoints(points)
  return C.shim_collider_static_polyline(self._w, buf, n)
end

--- Convex hull of a point set. Returns nil if the hull is degenerate (collinear / too few points).
--- @return userdata|nil handle
function World:staticConvexHull(points)
  local buf, n = packPoints(points)
  local h = C.shim_collider_static_convex_hull(self._w, buf, n)
  if h == NULL then return nil end
  return h
end

--- Heightfield terrain: `heights` sampled evenly along x by `scaleX`, scaled in y by `scaleY`.
--- @return userdata handle
function World:staticHeightfield(heights, scaleX, scaleY)
  local n = #heights
  local buf = ffi.new("float[?]", n)
  for i = 0, n - 1 do buf[i] = heights[i + 1] end
  return C.shim_collider_static_heightfield(self._w, buf, n, scaleX, scaleY)
end

-- ---- joints ---------------------------------------------------------------------------------
-- Anchors are in each body's *local* frame. Returns an opaque joint handle (pass to removeJoint).

--- Weld two bodies rigidly. @return userdata joint
function World:fixedJoint(b1, b2, a1x, a1y, a2x, a2y)
  return C.shim_joint_fixed(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0)
end

--- Hinge: bodies rotate freely about the anchor. @return userdata joint
function World:revoluteJoint(b1, b2, a1x, a1y, a2x, a2y)
  return C.shim_joint_revolute(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0)
end

--- Slider along `axis` (no relative rotation). @return userdata joint
function World:prismaticJoint(b1, b2, a1x, a1y, a2x, a2y, axisX, axisY)
  return C.shim_joint_prismatic(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0, axisX, axisY)
end

--- Slack rope: caps the anchor distance at `maxDist`. @return userdata joint
function World:ropeJoint(b1, b2, maxDist, a1x, a1y, a2x, a2y)
  return C.shim_joint_rope(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0, maxDist)
end

--- Damped spring toward `restLength`. @return userdata joint
function World:springJoint(b1, b2, restLength, stiffness, damping, a1x, a1y, a2x, a2y)
  return C.shim_joint_spring(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0,
    restLength, stiffness, damping)
end

function World:removeJoint(h)
  C.shim_joint_remove(self._w, h)
end

-- ---- extended queries -----------------------------------------------------------------------

--- Raycast that also returns the hit surface normal: (collider, distance, nx, ny) or nil.
function World:raycastNormal(ox, oy, dx, dy, maxToi)
  local hit = C.shim_query_raycast_normal(self._w, ox, oy, dx, dy, maxToi or 1e6, u64, f2a, f2b, f2c)
  if hit == 1 then
    return u64[0], f2a[0], f2b[0], f2c[0]
  end
  return nil
end

--- Sweep a shape ("ball"/"cuboid"/"capsule") from (ox,oy)+angle along (dx,dy). Returns
--- (collider, timeOfImpact) on the first hit, or nil. `a`,`b` are the shape args (see attachCollider).
function World:shapeCast(shape, a, b, ox, oy, angle, dx, dy, maxToi)
  local hit = C.shim_query_shapecast(self._w, SHAPE[shape] or 0, a or 0.5, b or 0.0,
    ox, oy, angle or 0, dx, dy, maxToi or 1e6, u64, f2a)
  if hit == 1 then
    return u64[0], f2a[0]
  end
  return nil
end

--- Nearest collider to a point within `maxDist`: (collider, x, y, inside) or nil. `inside` is true
--- when the query point lies within the collider.
function World:projectPoint(px, py, maxDist)
  local hit = C.shim_query_project_point(self._w, px, py, maxDist or 1e6, u64, f2a, f2b, i32)
  if hit == 1 then
    return u64[0], f2a[0], f2b[0], i32[0] == 1
  end
  return nil
end

--- All colliders overlapping an arbitrary shape posed at (x,y)+angle (generalizes overlapCircle).
--- Returns an array of collider handles.
function World:overlapShape(shape, a, b, x, y, angle)
  local n = tonumber(C.shim_query_overlap_shape(self._w, SHAPE[shape] or 0, a or 0.5, b or 0.0,
    x, y, angle or 0))
  local out = {}
  for i = 0, n - 1 do
    out[i + 1] = C.shim_overlap_get(self._w, i)
  end
  return out
end

-- ---- generalized kinematic character controller --------------------------------------------

--- Collision-corrected move for any shape character. `offset` is a small skin gap (0 = default).
--- Returns (allowedDx, allowedDy, grounded).
function World:moveShape(shape, a, b, x, y, angle, dx, dy, offset)
  C.shim_kcc_move(self._w, SHAPE[shape] or 0, a or 0.5, b or 0.0, x, y, angle or 0, dx, dy,
    offset or 0, f2a, f2b, i32)
  return f2a[0], f2b[0], i32[0] == 1
end

-- ---- joint motors + limits ------------------------------------------------------------------
-- Joint axis: "linX" | "linY" | "ang" (the angular axis is what a revolute joint's motor drives).
local JOINT_AXIS = { linX = 0, linY = 1, ang = 2 }

--- Drive a joint axis toward a target with spring stiffness + damping. Stiffness 0 + a target
--- velocity gives a pure velocity motor (e.g. a powered hinge: axis "ang").
function World:setJointMotor(joint, axis, targetPos, targetVel, stiffness, damping)
  C.shim_joint_set_motor(self._w, joint, JOINT_AXIS[axis] or 2, targetPos, targetVel, stiffness, damping)
end

--- Clamp a joint axis to [min, max] (radians for "ang", world units for linear axes).
function World:setJointLimits(joint, axis, min, max)
  C.shim_joint_set_limits(self._w, joint, JOINT_AXIS[axis] or 2, min, max)
end

function World:setJointMotorMaxForce(joint, axis, maxForce)
  C.shim_joint_set_motor_max_force(self._w, joint, JOINT_AXIS[axis] or 2, maxForce)
end

--- Whether the two jointed bodies still collide with each other.
function World:setJointContactsEnabled(joint, enabled)
  C.shim_joint_set_contacts_enabled(self._w, joint, enabled ~= false)
end

-- ---- multibody (articulated) joints ---------------------------------------------------------
-- Reduced-coordinates joints: no positional drift, ideal for articulated chains. Anchors are in
-- each body's local frame. Returns an opaque handle (pass to removeMultibodyJoint), or nil if the
-- joint would create a loop the multibody solver can't represent.

local function mbHandle(h)
  if h == NULL then return nil end
  return h
end

--- @return userdata|nil joint
function World:multibodyFixedJoint(b1, b2, a1x, a1y, a2x, a2y)
  return mbHandle(C.shim_multibody_joint_fixed(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0))
end

--- @return userdata|nil joint
function World:multibodyRevoluteJoint(b1, b2, a1x, a1y, a2x, a2y)
  return mbHandle(C.shim_multibody_joint_revolute(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0))
end

--- @return userdata|nil joint
function World:multibodyPrismaticJoint(b1, b2, a1x, a1y, a2x, a2y, axisX, axisY)
  return mbHandle(C.shim_multibody_joint_prismatic(self._w, b1, b2, a1x or 0, a1y or 0, a2x or 0, a2y or 0,
    axisX, axisY))
end

function World:removeMultibodyJoint(h)
  C.shim_multibody_joint_remove(self._w, h)
end

-- ---- bodies: locks, forces, full pose, mass properties, reads -------------------------------

--- Freeze a body's position (it can still rotate). Pass false to release.
function World:lockTranslations(h, locked)
  C.shim_body_lock_translations(self._w, h, locked ~= false)
end

--- Allow/forbid translation per axis (e.g. lock y for a side-view constraint).
function World:setEnabledTranslations(h, allowX, allowY)
  C.shim_body_set_enabled_translations(self._w, h, allowX ~= false, allowY ~= false)
end

function World:resetForces(h)
  C.shim_body_reset_forces(self._w, h)
end

function World:resetTorques(h)
  C.shim_body_reset_torques(self._w, h)
end

--- Set translation + rotation in one call.
function World:setPose(h, x, y, angle)
  C.shim_body_set_position(self._w, h, x, y, angle or 0)
end

--- @return string "dynamic"|"fixed"|"kinematic"|"kinematicPosition"
function World:bodyType(h)
  local code = C.shim_body_type(self._w, h)
  return ({ [0] = "dynamic", [1] = "fixed", [2] = "kinematic", [3] = "kinematicPosition" })[code]
end

--- @return boolean
function World:isEnabled(h)
  return C.shim_body_is_enabled(self._w, h) == 1
end

--- @return number x, number y
function World:centerOfMass(h)
  C.shim_body_center_of_mass(self._w, h, f2a, f2b)
  return f2a[0], f2b[0]
end

--- Override added mass properties: total `mass`, center of mass (comX,comY), scalar 2D `inertia`.
function World:setAdditionalMassProperties(h, mass, comX, comY, inertia)
  C.shim_body_set_additional_mass_properties(self._w, h, mass, comX or 0, comY or 0, inertia or 0)
end

--- Recompute mass/inertia from the body's attached colliders (after changing collider density/mass).
function World:recomputeMass(h)
  C.shim_body_recompute_mass(self._w, h)
end

--- Soft-CCD prediction distance — cheaper anti-tunneling than full CCD for moderately fast bodies.
function World:setSoftCcdPrediction(h, distance)
  C.shim_body_set_soft_ccd_prediction(self._w, h, distance)
end

-- ---- colliders: shape swap, events, reads ---------------------------------------------------

--- Choose which events a collider emits.
function World:setActiveEvents(h, collision, contactForce)
  C.shim_collider_set_active_events(self._w, h, collision ~= false, contactForce == true)
end

--- Raw `ActiveCollisionTypes` bitmask (enable contacts between body-type pairs that default off,
--- e.g. fixed/kinematic). Most games never need this.
function World:setActiveCollisionTypes(h, bits)
  C.shim_collider_set_active_collision_types(self._w, h, bits)
end

function World:setContactForceThreshold(h, threshold)
  C.shim_collider_set_contact_force_threshold(self._w, h, threshold)
end

--- Swap a collider's shape in place ("ball"/"cuboid"/"capsule").
function World:setShape(h, shape, a, b)
  C.shim_collider_set_shape(self._w, h, SHAPE[shape] or 0, a or 0.5, b or 0.0)
end

--- Absolute rotation of a free-standing (static) collider.
function World:setColliderRotation(h, angle)
  C.shim_collider_set_rotation(self._w, h, angle)
end

--- @return number
function World:colliderDensity(h)
  return C.shim_collider_density(self._w, h)
end

--- @return number
function World:colliderMass(h)
  return C.shim_collider_mass(self._w, h)
end

--- @return number
function World:colliderVolume(h)
  return C.shim_collider_volume(self._w, h)
end

--- The body a collider is attached to, or nil if it is free-standing (static).
--- @return userdata|nil body
function World:colliderParent(h)
  local p = C.shim_collider_parent(self._w, h)
  if p == NULL then return nil end
  return p
end

--- @return boolean
function World:colliderIsSensor(h)
  return C.shim_collider_is_sensor(self._w, h) == 1
end

-- ---- narrow-phase contact / intersection queries --------------------------------------------

--- Colliders in solid contact with `h` as of the last step. Returns an array of collider handles.
function World:contactsWith(h)
  local n = tonumber(C.shim_contacts_with(self._w, h))
  local out = {}
  for i = 0, n - 1 do
    out[i + 1] = C.shim_overlap_get(self._w, i)
  end
  return out
end

--- Colliders intersecting `h` via sensor pairs as of the last step. Returns an array of handles.
function World:intersectionsWith(h)
  local n = tonumber(C.shim_intersections_with(self._w, h))
  local out = {}
  for i = 0, n - 1 do
    out[i + 1] = C.shim_overlap_get(self._w, i)
  end
  return out
end

-- ---- filtered raycast -----------------------------------------------------------------------

--- Raycast honoring collision groups, optionally excluding one collider. `memberships`/`filter`
--- are the ray's group masks; `exclude` is a collider handle to skip (or nil). Returns
--- (collider, distance) on hit, or nil.
function World:raycastFiltered(ox, oy, dx, dy, maxToi, memberships, filter, exclude)
  local hit = C.shim_query_raycast_filtered(self._w, ox, oy, dx, dy, maxToi or 1e6,
    memberships or 0xFFFF, filter or 0xFFFF, exclude or NULL, u64, f2a)
  if hit == 1 then
    return u64[0], f2a[0]
  end
  return nil
end

-- ---- remaining world tuning -----------------------------------------------------------------

--- The simulation length unit (≈ the size of a typical dynamic object). Set once after creating the
--- world if your scale isn't ~1 unit = 1 meter; it tunes internal tolerances.
function World:setLengthUnit(lengthUnit)
  C.shim_world_set_length_unit(self._w, lengthUnit)
end

function World:setMaxCcdSubsteps(substeps)
  C.shim_world_set_max_ccd_substeps(self._w, substeps)
end

-- ---- contact-force events -------------------------------------------------------------------

--- Drain queued contact-force events (fire when a collider's contact force exceeds its threshold —
--- see `setContactForceThreshold`). Returns an array of `{ a, b, magnitude }` and clears the queue.
function World:drainForceEvents()
  local n = tonumber(C.shim_force_events_count(self._w))
  local out = {}
  for i = 0, n - 1 do
    if C.shim_force_events_get(self._w, i, frec) == 1 then
      out[i + 1] = { a = frec[0].a, b = frec[0].b, magnitude = frec[0].magnitude }
    end
  end
  C.shim_force_events_clear(self._w)
  return out
end

-- ---- contact-manifold geometry --------------------------------------------------------------

--- Geometry of the deepest current contact between two colliders (as of the last step), for hit
--- effects: returns (normalX, normalY, pointX, pointY, depth) where depth > 0 means overlapping, or
--- nil if the pair isn't touching.
function World:contactInfo(c1, c2)
  if C.shim_contact_pair_info(self._w, c1, c2, f2a, f2b, f2c, f2d, f2e) == 1 then
    return f2a[0], f2b[0], f2c[0], f2d[0], f2e[0]
  end
  return nil
end

-- ---- remaining body + collider setters ------------------------------------------------------

--- Extra solver iterations for this body (stiffer joints / important stacks).
function World:setAdditionalSolverIterations(h, iters)
  C.shim_body_set_additional_solver_iterations(self._w, h, iters)
end

--- Raw locked-axes bitmask: bit 0 = lock translation X, bit 1 = lock translation Y, bit 2 = lock
--- rotation. (Convenience wrappers: lockTranslations, setEnabledTranslations, lockRotations.)
function World:setLockedAxes(h, bits)
  C.shim_body_set_locked_axes(self._w, h, bits)
end

function World:setContactSkin(h, skin)
  C.shim_collider_set_contact_skin(self._w, h, skin)
end

-- How a collider's friction/restitution combines with the other collider's in a contact.
local COMBINE = { average = 0, min = 1, multiply = 2, max = 3 }

--- `rule` is "average" | "min" | "multiply" | "max".
function World:setFrictionCombineRule(h, rule)
  C.shim_collider_set_friction_combine_rule(self._w, h, COMBINE[rule] or 0)
end

--- `rule` is "average" | "min" | "multiply" | "max".
function World:setRestitutionCombineRule(h, rule)
  C.shim_collider_set_restitution_combine_rule(self._w, h, COMBINE[rule] or 0)
end

--- Set a collider's mass properties directly: total `mass`, center of mass, scalar 2D `inertia`.
function World:setColliderMassProperties(h, mass, comX, comY, inertia)
  C.shim_collider_set_mass_properties(self._w, h, mass, comX or 0, comY or 0, inertia or 0)
end

-- ---- physics hooks (pair-filter callbacks) --------------------------------------------------
-- These run per candidate pair every step and call back into Lua — keep them cheap. Only colliders
-- opted in via setActiveHooks consult them. The cast callback is kept alive on the World (else it'd
-- be collected) and freed when replaced.

local FILTER_CB = "int32_t(*)(uint64_t, uint64_t)"

local function setFilter(self, field, install, fn)
  local old = self[field]
  if old then old:free() end
  if fn == nil then
    self[field] = nil
    install(self._w, nil)
    return
  end
  local cb = ffi.cast(FILTER_CB, function(a, b)
    return fn(a, b) and 1 or 0
  end)
  self[field] = cb
  install(self._w, cb)
end

--- Filter solid contacts: `fn(colliderA, colliderB)` returns truthy to keep the contact, falsy to
--- ignore it. Pass nil to clear. Both colliders must opt in via `setActiveHooks(h, true, ...)`.
function World:setContactFilter(fn)
  setFilter(self, "_contactCb", C.shim_set_contact_filter, fn)
end

--- Filter sensor intersections: `fn(colliderA, colliderB)` → truthy to keep. Pass nil to clear.
--- Colliders must opt in via `setActiveHooks(h, _, true)`.
function World:setIntersectionFilter(fn)
  setFilter(self, "_intersectCb", C.shim_set_intersection_filter, fn)
end

--- Opt a collider into the hook callbacks (off by default — hooks cost nothing unless requested).
function World:setActiveHooks(h, contact, intersection)
  C.shim_collider_set_active_hooks(self._w, h, contact == true, intersection == true)
end

-- ---- world snapshot / restore (serialization) -----------------------------------------------

--- Serialize the full simulation to a Lua string (bodies, colliders, joints, broad/narrow phase,
--- gravity, integration params). Restore with `rapier.restore`. For deterministic save states and
--- lockstep/rollback netcode. Returns nil if serialization fails.
function World:snapshot()
  local ptr = C.shim_world_snapshot(self._w, u32)
  if ptr == nil then return nil end
  local s = ffi.string(ptr, u32[0])
  C.shim_buffer_free(ptr, u32[0])
  return s
end

-- ---- debug render ---------------------------------------------------------------------------

--- Rapier's own debug geometry as a flat array of line vertices `{ ax, ay, bx, by, ... }` (one line
--- per 4 numbers). Draw with `love.graphics.line` segments. Cheaper than reconstructing shapes, and
--- includes joints. Reflects state as of the last step.
function World:debugLines()
  local ptr = C.shim_debug_render(self._w, u32)
  local n = u32[0]
  local out = {}
  for i = 0, n - 1 do
    out[i + 1] = ptr[i]
  end
  C.shim_debug_buffer_free(ptr, n)
  return out
end

physics.World = World

--- Rebuild a world from a `World:snapshot()` string. Returns a fresh World, or nil if the bytes
--- can't be decoded.
---@return PhysicsWorld|nil
function physics.restore(bytes)
  local ptr = C.shim_world_restore(bytes, #bytes)
  if ptr == nil then return nil end
  return setmetatable({ _w = ffi.gc(ptr, C.shim_world_free) }, World)
end

return physics
