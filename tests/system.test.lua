-- Tests for the game-facing `rapier.system` adapter: the fixed-timestep loop, event dispatch, the
-- newActor/addStatic helpers, and the `drive` steering integrator. Runs under LuaJIT (loads the
-- native shim). `debugDraw` is not covered here — it needs a live love.graphics context.
local Physics = require("rapier.system")

local BALL = { kind = "ball", radius = 0.5 }

describe("rapier.system", function()
  describe("fixed timestep", function()
    it("steps floor(dt / fixedDt) times and exposes the remainder as alpha", function()
      local p = Physics.new({ fixedDt = 1 / 60 })
      local body = p:newActor("dynamic", 0, 0, BALL)
      p.world:setLinvel(body, 60, 0) -- 60 u/s → 1 unit per fixed step
      p:update(2.5 / 60) -- 2 whole steps + 0.5 leftover
      local x = p.world:position(body)
      assert.is_true(math.abs(x - 2) < 0.05, "expected ~2 steps of travel, got x=" .. x)
      assert.is_true(math.abs(p:alpha() - 0.5) < 0.01, "alpha should be the 0.5-step remainder")
    end)

    it("does not step until a full fixedDt has accumulated", function()
      local p = Physics.new({ fixedDt = 1 / 60 })
      local body = p:newActor("dynamic", 0, 0, BALL)
      p.world:setLinvel(body, 60, 0)
      p:update(1 / 120) -- half a step
      assert.is_true(math.abs(p.world:position(body)) < 1e-6, "body moved before a full step")
      assert.is_true(math.abs(p:alpha() - 0.5) < 0.01)
    end)

    it("clamps the step count to avoid a spiral of death on a long stall", function()
      local p = Physics.new({ fixedDt = 1 / 60 })
      local body = p:newActor("dynamic", 0, 0, BALL)
      p.world:setLinvel(body, 60, 0)
      p:update(10) -- 600 steps worth; must be clamped to maxSteps (5)
      local x = p.world:position(body)
      assert.is_true(x < 6, "spiral-of-death clamp failed (x=" .. x .. ", expected <= ~5 steps)")
    end)
  end)

  describe("collision events", function()
    it("dispatches drained events to subscribers and stops after unsubscribe", function()
      local p = Physics.new()
      p:addStatic(5, 0, { kind = "cuboid", hx = 0.5, hy = 5 })
      local body = p:newActor("dynamic", 0, 0, BALL)
      p.world:setLinvel(body, 20, 0)

      local hits = 0
      local unsubscribe = p:onCollision(function() hits = hits + 1 end)
      for _ = 1, 120 do p:update(1 / 60) end
      assert.is_true(hits > 0, "subscriber never received a collision event")

      local afterUnsub = hits
      unsubscribe()
      -- Drive it back into the wall again; no further callbacks should land.
      p.world:setLinvel(body, -20, 0)
      for _ = 1, 60 do p:update(1 / 60) end
      p.world:setLinvel(body, 20, 0)
      for _ = 1, 120 do p:update(1 / 60) end
      assert.equals(afterUnsub, hits, "callback fired after unsubscribe")
    end)
  end)

  describe("actors + static geometry", function()
    it("creates an actor (body + collider) that simulates", function()
      local p = Physics.new()
      local body, collider = p:newActor("dynamic", 0, 0, BALL)
      assert.is_not_nil(body)
      assert.is_not_nil(collider)
      p.world:setLinvel(body, 5, 0)
      p:update(1 / 60)
      assert.is_true(p.world:position(body) > 0)
    end)

    it("blocks actors with batch-registered static geometry", function()
      local p = Physics.new()
      p:addStaticGeometry({
        { x = 5, y = 0, shape = { kind = "cuboid", hx = 0.5, hy = 5 } },
      })
      local body = p:newActor("dynamic", 0, 0, BALL)
      p.world:setLinvel(body, 20, 0)
      for _ = 1, 120 do p:update(1 / 60) end
      assert.is_true(p.world:position(body) < 4.6, "static geometry did not block the actor")
    end)
  end)

  describe("drive (steering integrator)", function()
    it("integrates a steering force into velocity and clamps to maxSpeed", function()
      local p = Physics.new()
      local body = p:newActor("dynamic", 0, 0, BALL)
      p:drive(body, 1000, 0, 5, 1 / 60) -- huge force, capped at 5 u/s
      local vx, vy = p.world:linvel(body)
      local speed = math.sqrt(vx * vx + vy * vy)
      assert.is_true(math.abs(speed - 5) < 0.01, "speed not clamped to maxSpeed (got " .. speed .. ")")
      assert.is_true(vx > 4.9, "drive should push along +x")
      assert.is_true(math.abs(vy) < 1e-6)
    end)

    it("leaves velocity below maxSpeed untouched by the clamp", function()
      local p = Physics.new()
      local body = p:newActor("dynamic", 0, 0, BALL)
      p:drive(body, 60, 0, 100, 1 / 60) -- 60 * 1/60 = 1 u/s, well under the cap
      local vx = p.world:linvel(body)
      assert.is_true(math.abs(vx - 1) < 0.01, "under-cap velocity was altered (got " .. vx .. ")")
    end)
  end)
end)
