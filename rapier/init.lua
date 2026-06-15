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

-- Reused scratch buffers so per-call queries don't allocate cdata every frame.
local f2a = ffi.new("float[1]")
local f2b = ffi.new("float[1]")
local u64 = ffi.new("uint64_t[1]")
local rec = ffi.new("ContactRecord[1]")

local BODY_KIND = { dynamic = 0, fixed = 1, kinematic = 2 }
local SHAPE = { ball = 0, cuboid = 1, capsule = 2 }

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

physics.World = World
return physics
