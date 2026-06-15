-- Game-facing physics adapter (a Manager: one instance, created at the composition root and
-- injected). Wraps the generic `rapier` World with the things the game needs:
--
--   * a FIXED-TIMESTEP accumulator so simulation is frame-rate independent and deterministic
--     (the reason we chose Rapier) — render between steps with the returned interpolation alpha;
--   * COLLISION/SENSOR EVENT dispatch — drained each step and pushed to subscribers (`onCollision`);
--   * STATIC WORLD COLLIDERS — register the map's blocking geometry once (distinct from the
--     navmesh, which only describes where agents may *path*);
--   * DEBUG DRAW — outline every collider for dev (love.graphics); not shipped.
--
-- Use `Physics.new(deps)` — no singletons. FFI/LÖVE-coupled, so verified via `lx run`, not unit
-- tests (same call as util.graphics.screenshot).

local physics = require("rapier")

---@class PhysicsSystem
---@field world PhysicsWorld
---@field _fixedDt number
---@field _accumulator number
---@field _alpha number               -- 0..1 interpolation remainder for rendering
---@field _subs (fun(event: table))[] -- collision/sensor subscribers
---@field _drawables table[]          -- recorded shapes for debug draw
local Physics = {}
Physics.__index = Physics

--- Create the physics system.
--- `deps.fixedDt` overrides the simulation step (default 1/60).
---@param deps? { fixedDt?: number }
---@return PhysicsSystem
function Physics.new(deps)
  deps = deps or {}
  local self = setmetatable({}, Physics) ---@type PhysicsSystem
  self.world = physics.newWorld()
  self._fixedDt = deps.fixedDt or (1 / 60)
  self._accumulator = 0
  self._alpha = 0
  self._subs = {}
  self._drawables = {}
  return self
end

-- ---- fixed-timestep loop --------------------------------------------------------------------

--- Advance physics on the fixed timestep. Steps zero or more times to consume `dt`, draining and
--- dispatching events after each step. Leftover time is exposed as `:alpha()` for render interp.
---@param dt number Seconds since the previous frame (the variable frame delta).
function Physics:update(dt)
  self._accumulator = self._accumulator + dt
  -- Clamp to avoid a spiral of death after a long stall (e.g. window drag).
  local maxSteps = 5
  local steps = 0
  while self._accumulator >= self._fixedDt and steps < maxSteps do
    self.world:step(self._fixedDt)
    local events = self.world:drainEvents()
    for _, e in ipairs(events) do
      self:_dispatch(e)
    end
    self._accumulator = self._accumulator - self._fixedDt
    steps = steps + 1
  end
  if self._accumulator > self._fixedDt then
    self._accumulator = self._accumulator % self._fixedDt -- shed backlog past the clamp
  end
  self._alpha = self._accumulator / self._fixedDt
end

--- Interpolation remainder (0..1) between the last and next fixed step, for smooth rendering.
function Physics:alpha()
  return self._alpha
end

-- ---- events ---------------------------------------------------------------------------------

--- Subscribe to collision/sensor events. `fn` receives `{ a, b, started, sensor }` (a/b are
--- collider handles). Returns an unsubscribe function.
---@param fn fun(event: table)
function Physics:onCollision(fn)
  self._subs[#self._subs + 1] = fn
  return function()
    for i, f in ipairs(self._subs) do
      if f == fn then
        table.remove(self._subs, i)
        return
      end
    end
  end
end

function Physics:_dispatch(event)
  for _, fn in ipairs(self._subs) do
    fn(event)
  end
end

-- ---- bodies / colliders (delegate to World, record for debug draw) --------------------------

--- Create a body with one attached collider in one call (the common case). `shape` is a table:
--- `{ kind="ball", radius=r }`, `{ kind="cuboid", hx=, hy= }`, or `{ kind="capsule", halfHeight=, radius= }`.
--- Returns the body and collider handles.
---@return userdata body, userdata collider
function Physics:newActor(bodyKind, x, y, shape)
  local body = self.world:newBody(bodyKind, x, y)
  local collider = self:_attach(body, shape)
  return body, collider
end

function Physics:_attach(body, shape)
  local a, b = self:_shapeArgs(shape)
  local collider = self.world:attachCollider(body, shape.kind, a, b)
  self._drawables[#self._drawables + 1] = { body = body, shape = shape }
  return collider
end

--- Register a single static collider (map wall/obstacle). `shape` as in `newActor`.
---@return userdata collider
function Physics:addStatic(x, y, shape)
  local a, b = self:_shapeArgs(shape)
  local collider = self.world:staticCollider(shape.kind, x, y, a, b)
  self._drawables[#self._drawables + 1] = { x = x, y = y, shape = shape }
  return collider
end

--- Register a batch of static world geometry: `{ {x, y, shape}, ... }`.
function Physics:addStaticGeometry(items)
  for _, it in ipairs(items) do
    self:addStatic(it.x or it[1], it.y or it[2], it.shape or it[3])
  end
end

function Physics:_shapeArgs(shape)
  local k = shape.kind
  if k == "cuboid" then
    return shape.hx, shape.hy
  elseif k == "capsule" then
    return shape.halfHeight, shape.radius
  end
  return shape.radius, 0.0 -- ball
end

-- ---- locomotion -----------------------------------------------------------------------------

--- Drive a body from a steering force: integrate it into the body's velocity, clamp to `maxSpeed`,
--- and apply. Call once per fixed step with that agent's steering output (`util.spatial.steering`).
--- The physics engine then resolves collisions/penetration, so steering stays collision-agnostic.
function Physics:drive(body, fx, fy, maxSpeed, dt)
  local vx, vy = self.world:linvel(body)
  vx, vy = vx + fx * dt, vy + fy * dt
  local sp = math.sqrt(vx * vx + vy * vy)
  if sp > maxSpeed and sp > 1e-9 then
    vx, vy = vx / sp * maxSpeed, vy / sp * maxSpeed
  end
  self.world:setLinvel(body, vx, vy)
end

-- ---- debug draw -----------------------------------------------------------------------------

--- Outline every collider (dev only). Call inside the camera transform so it lines up with the
--- world. Colors: static = dim, dynamic = bright.
function Physics:debugDraw()
  local lg = love.graphics
  local r, g, bl, al = lg.getColor()
  for _, d in ipairs(self._drawables) do
    local x, y, angle
    if d.body then
      x, y = self.world:position(d.body)
      angle = self.world:rotation(d.body)
      lg.setColor(0.3, 0.9, 0.4, 0.9)
    else
      x, y, angle = d.x, d.y, 0
      lg.setColor(0.5, 0.5, 0.6, 0.7)
    end
    lg.push()
    lg.translate(x, y)
    lg.rotate(angle)
    local s = d.shape
    if s.kind == "ball" then
      lg.circle("line", 0, 0, s.radius)
    elseif s.kind == "cuboid" then
      lg.rectangle("line", -s.hx, -s.hy, s.hx * 2, s.hy * 2)
    elseif s.kind == "capsule" then
      local hh, rad = s.halfHeight, s.radius
      lg.line(-rad, -hh, -rad, hh)
      lg.line(rad, -hh, rad, hh)
      lg.arc("line", "open", 0, -hh, rad, math.pi, 2 * math.pi)
      lg.arc("line", "open", 0, hh, rad, 0, math.pi)
    end
    lg.pop()
  end
  lg.setColor(r, g, bl, al)
end

return Physics
