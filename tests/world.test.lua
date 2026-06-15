-- Integration tests for the raw World API. These load the real native shim via FFI, so they must
-- run under LuaJIT from the repo root (the loader resolves ./lib/native/<platform>/).
local rapier = require("rapier")
local ffi = require("ffi")

-- A wall (vertical cuboid) at x, spanning a tall y range — the canonical obstacle.
local function wall(w, x)
  return w:staticCollider("cuboid", x, 0, 0.5, 5)
end

describe("rapier.World", function()
  describe("bodies + integration", function()
    it("moves a dynamic body under velocity (zero gravity, no damping)", function()
      local w = rapier.newWorld()
      local b = w:newBody("dynamic", 0, 0)
      w:setLinvel(b, 10, 0)
      for _ = 1, 60 do w:step(1 / 60) end
      local x, y = w:position(b)
      assert.is_true(math.abs(x - 10) < 0.05, "x ~= 10 (got " .. x .. ")")
      assert.is_true(math.abs(y) < 0.01)
    end)

    it("teleports with setPosition and reports linvel", function()
      local w = rapier.newWorld()
      local b = w:newBody("dynamic", 0, 0)
      w:setPosition(b, 3, -4)
      local x, y = w:position(b)
      assert.is_true(math.abs(x - 3) < 0.01 and math.abs(y + 4) < 0.01)
      w:setLinvel(b, 1, 2)
      local vx, vy = w:linvel(b)
      assert.is_true(math.abs(vx - 1) < 0.01 and math.abs(vy - 2) < 0.01)
    end)

    it("applies an instantaneous impulse (knockback)", function()
      local w = rapier.newWorld()
      local b = w:newBody("dynamic", 0, 0)
      w:attachCollider(b, "ball", 0.5)
      w:applyImpulse(b, 0, -10)
      w:step(1 / 60)
      local _, vy = w:linvel(b)
      assert.is_true(vy < -0.001, "impulse produced no -y velocity")
    end)

    it("accelerates under a sustained force (addForce), unlike a one-shot impulse", function()
      local w = rapier.newWorld()
      local b = w:newBody("dynamic", 0, 0)
      w:attachCollider(b, "ball", 0.5)
      local prev = 0
      for _ = 1, 30 do
        w:addForce(b, 5, 0) -- force must be re-applied each step
        w:step(1 / 60)
        local vx = select(1, w:linvel(b))
        assert.is_true(vx >= prev - 1e-4, "velocity should keep rising under force")
        prev = vx
      end
      assert.is_true(prev > 0.1, "no acceleration from sustained force")
    end)

    it("bleeds off velocity under linear damping", function()
      local function speedAfter(damping)
        local w = rapier.newWorld()
        local b = w:newBody("dynamic", 0, 0)
        w:attachCollider(b, "ball", 0.5)
        w:setLinearDamping(b, damping)
        w:setLinvel(b, 10, 0)
        for _ = 1, 60 do w:step(1 / 60) end
        return select(1, w:linvel(b))
      end
      assert.is_true(speedAfter(5.0) < speedAfter(0.0) - 1, "damping did not slow the body")
    end)

    it("frees handles without erroring (lifecycle)", function()
      local w = rapier.newWorld()
      local a = w:newBody("dynamic", 0, 0)
      local c = w:attachCollider(a, "ball", 0.5)
      local b = w:newBody("dynamic", 5, 0)
      w:removeCollider(c)
      w:removeBody(a)
      -- The surviving body still simulates.
      w:setLinvel(b, 1, 0)
      w:step(1 / 60)
      assert.is_true(select(1, w:position(b)) > 5)
    end)
  end)

  describe("colliders + collisions", function()
    it("blocks a dynamic body against a static wall and emits a collision event", function()
      local w = rapier.newWorld()
      wall(w, 5)
      local b = w:newBody("dynamic", 0, 0)
      w:attachCollider(b, "ball", 0.5)
      w:setLinvel(b, 20, 0)
      local started = 0
      for _ = 1, 120 do
        w:step(1 / 60)
        for _, e in ipairs(w:drainEvents()) do
          if e.started and not e.sensor then started = started + 1 end
        end
      end
      assert.is_true(select(1, w:position(b)) < 4.6, "ball passed through the wall")
      assert.is_true(started > 0, "no solid collision events")
    end)

    it("supports a capsule collider", function()
      local w = rapier.newWorld()
      wall(w, 5)
      local b = w:newBody("dynamic", 0, 0)
      w:attachCollider(b, "capsule", 1.0, 0.5) -- half_height, radius
      w:setLinvel(b, 20, 0)
      for _ = 1, 120 do w:step(1 / 60) end
      assert.is_true(select(1, w:position(b)) < 4.7, "capsule passed through the wall")
    end)

    it("reports overlap on a sensor without blocking the body", function()
      local w = rapier.newWorld()
      local s = wall(w, 5)
      w:setSensor(s, true)
      local b = w:newBody("dynamic", 0, 0)
      w:attachCollider(b, "ball", 0.5)
      w:setLinvel(b, 20, 0)
      local sensorEvents = 0
      for _ = 1, 120 do
        w:step(1 / 60)
        for _, e in ipairs(w:drainEvents()) do
          if e.sensor then sensorEvents = sensorEvents + 1 end
        end
      end
      assert.is_true(sensorEvents > 0, "sensor produced no intersection events")
      assert.is_true(select(1, w:position(b)) > 6, "sensor wrongly blocked the body")
    end)

    it("filters collisions by interaction groups", function()
      local w = rapier.newWorld()
      local wc = wall(w, 5)
      w:setGroups(wc, 0x0001, 0x0001)
      local b = w:newBody("dynamic", 0, 0)
      local bc = w:attachCollider(b, "ball", 0.5)
      w:setGroups(bc, 0x0002, 0x0002) -- disjoint membership/filter → no interaction
      w:setLinvel(b, 20, 0)
      for _ = 1, 120 do w:step(1 / 60) end
      assert.is_true(select(1, w:position(b)) > 6, "filtered ball should pass through")
    end)
  end)

  -- Spatial queries read the broad-phase BVH, which is (re)built during step(). Freshly-added
  -- geometry isn't visible to a query until one step has run — exactly like raw Rapier's
  -- QueryPipeline.update(). A running game steps every frame before it queries, so this is a
  -- non-issue in practice; here we step once after building the world to mirror that.
  describe("queries (after a step registers the geometry)", function()
    it("raycasts to the nearest collider", function()
      local w = rapier.newWorld()
      wall(w, 5)
      w:step(1 / 60)
      local c, toi = w:raycast(0, 0, 1, 0, 100)
      assert.is_not_nil(c)
      assert.is_true(toi > 3 and toi < 6, "ray toi out of range: " .. tostring(toi))
      assert.is_nil(w:raycast(0, 0, 0, 1, 100), "ray pointing away should miss")
    end)

    it("finds the collider containing a point", function()
      local w = rapier.newWorld()
      wall(w, 5)
      w:step(1 / 60)
      assert.is_not_nil(w:pointQuery(5, 0))
      assert.is_nil(w:pointQuery(50, 50))
    end)

    it("returns all colliders overlapping a circle (AoE)", function()
      local w = rapier.newWorld()
      wall(w, 5)
      w:step(1 / 60)
      assert.is_true(#w:overlapCircle(5, 0, 1) >= 1)
      assert.equals(0, #w:overlapCircle(50, 50, 1))
    end)

    it("corrects a kinematic character move against a wall (moveBall)", function()
      local w = rapier.newWorld()
      wall(w, 5)
      w:step(1 / 60)
      local dx = w:moveBall(0, 0, 0.5, 10, 0) -- want +10x, but the wall is at ~4.5
      assert.is_true(dx > 0 and dx < 9, "KCC did not clamp the move into the wall (dx=" .. dx .. ")")
      local free = w:moveBall(-50, 0, 0.5, 10, 0) -- open space → near-full travel
      assert.is_true(free > 9.5, "KCC clamped an unobstructed move (dx=" .. free .. ")")
    end)
  end)

  describe("determinism + batching", function()
    it("produces bit-identical results across two identical runs", function()
      local function run()
        local w = rapier.newWorld()
        wall(w, 6)
        local b = w:newBody("dynamic", 0, 0)
        w:attachCollider(b, "ball", 0.5)
        w:applyImpulse(b, 7.3, 2.1)
        for _ = 1, 180 do w:step(1 / 60) end
        return w:position(b)
      end
      local x1, y1 = run()
      local x2, y2 = run()
      assert.equals(x1, x2) -- exact equality: the reason we chose Rapier (enhanced-determinism)
      assert.equals(y1, y2)
    end)

    it("reads many transforms in one batched FFI call", function()
      local w = rapier.newWorld()
      local n = 4
      local hs = ffi.new("uint64_t[?]", n)
      for i = 0, n - 1 do hs[i] = w:newBody("dynamic", i * 10, i * 5) end
      local out = ffi.new("float[?]", n * 2)
      w:readTransforms(hs, n, out)
      for i = 0, n - 1 do
        assert.is_true(math.abs(out[i * 2] - i * 10) < 0.01)
        assert.is_true(math.abs(out[i * 2 + 1] - i * 5) < 0.01)
      end
    end)
  end)
end)
