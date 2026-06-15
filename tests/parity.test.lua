-- Tests for the parity-pass surface: physics-hook pair filters, world snapshot/restore
-- (serialization), and debug-render geometry. LuaJIT, from the repo root.
local rapier = require("rapier")

describe("rapier.World parity surface", function()
  describe("physics hooks", function()
    it("suppresses contacts via the contact filter", function()
      local function endX(useFilter)
        local w = rapier.newWorld()
        local wall = w:staticCollider("cuboid", 5, 0, 0.5, 5)
        local b = w:newBody("dynamic", 0, 0)
        local bc = w:attachCollider(b, "ball", 0.5)
        if useFilter then
          w:setActiveHooks(wall, true, false)
          w:setActiveHooks(bc, true, false)
          w:setContactFilter(function() return false end) -- never allow a contact
        end
        w:setLinvel(b, 20, 0)
        for _ = 1, 120 do w:step(1 / 60) end
        return w:position(b)
      end
      assert.is_true(endX(true) > 6, "contact filter should let the ball pass through")
      assert.is_true(endX(false) < 5, "without the filter it should be blocked")
    end)

    it("suppresses sensor intersections via the intersection filter", function()
      local w = rapier.newWorld()
      local sensor = w:staticCollider("cuboid", 0, 0, 2, 2)
      w:setSensor(sensor, true)
      local b = w:newBody("dynamic", 0, 0)
      local bc = w:attachCollider(b, "ball", 0.5)
      w:setActiveHooks(sensor, false, true)
      w:setActiveHooks(bc, false, true)
      w:setIntersectionFilter(function() return false end)
      w:step(1 / 60)
      assert.equals(0, #w:intersectionsWith(bc), "filter should suppress the intersection")
      w:setIntersectionFilter(nil) -- clear
      w:step(1 / 60)
      assert.is_true(#w:intersectionsWith(bc) >= 1, "cleared filter should restore intersections")
    end)
  end)

  describe("snapshot / restore", function()
    it("round-trips the simulation bit-identically", function()
      local w = rapier.newWorld()
      local b = w:newBody("dynamic", 0, 0)
      w:attachCollider(b, "ball", 0.5)
      w:applyImpulse(b, 5, 3)
      for _ = 1, 30 do w:step(1 / 60) end
      local xa, ya = w:position(b)

      local snap = w:snapshot()
      assert.is_not_nil(snap)
      assert.is_true(#snap > 0)

      for _ = 1, 30 do w:step(1 / 60) end -- diverge the original

      local w2 = rapier.restore(snap)
      assert.is_not_nil(w2)
      local x2, y2 = w2:position(b) -- handle indices are preserved across the snapshot
      assert.equals(xa, x2, "restored x should match the snapshot exactly")
      assert.equals(ya, y2, "restored y should match the snapshot exactly")

      -- And the restored world continues simulating.
      for _ = 1, 30 do w2:step(1 / 60) end
      assert.is_true(w2:position(b) > x2)
    end)

    it("returns nil on garbage input", function()
      assert.is_nil(rapier.restore("not a real snapshot"))
    end)
  end)

  describe("debug render", function()
    it("returns line geometry as groups of four", function()
      local w = rapier.newWorld()
      w:staticCollider("cuboid", 0, 0, 1, 1)
      local b = w:newBody("dynamic", 3, 0)
      w:attachCollider(b, "ball", 0.5)
      w:step(1 / 60)
      local lines = w:debugLines()
      assert.is_true(#lines > 0, "no debug geometry produced")
      assert.equals(0, #lines % 4, "each line is 4 numbers (ax, ay, bx, by)")
    end)
  end)
end)
