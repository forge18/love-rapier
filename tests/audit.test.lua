-- Tests for the audit-pass surface: contact-force events, contact-manifold geometry, and the last
-- body/collider setters (locked-axes mask, combine rules, contact skin, mass properties, solver
-- iterations). LuaJIT, from the repo root.
local rapier = require("rapier")

describe("rapier.World audit surface", function()
  it("delivers contact-force events past a threshold", function()
    local w = rapier.newWorld()
    w:setGravity(0, -30)
    local floor = w:staticCollider("cuboid", 0, -3, 5, 0.5)
    w:setActiveEvents(floor, true, true) -- collision + contact-force
    w:setContactForceThreshold(floor, 0.1)
    local b = w:newBody("dynamic", 0, 2)
    w:attachCollider(b, "ball", 0.5)
    local total, maxMag = 0, 0
    for _ = 1, 150 do
      w:step(1 / 60)
      for _, e in ipairs(w:drainForceEvents()) do
        total = total + 1
        maxMag = math.max(maxMag, e.magnitude)
      end
    end
    assert.is_true(total > 0, "no contact-force events fired")
    assert.is_true(maxMag > 0, "contact-force magnitude was zero")
  end)

  it("reports contact-manifold geometry between two colliders", function()
    local w = rapier.newWorld()
    local wall = w:staticCollider("cuboid", 5, 0, 0.5, 5)
    local b = w:newBody("dynamic", 4, 0)
    local bc = w:attachCollider(b, "ball", 0.5)
    for _ = 1, 40 do
      w:setLinvel(b, 5, 0) -- keep pressing into the wall
      w:step(1 / 60)
    end
    local nx, ny, px, _, depth = w:contactInfo(bc, wall)
    assert.is_not_nil(nx)
    assert.is_true(math.abs(nx) > 0.7, "contact normal should be ~horizontal (nx=" .. tostring(nx) .. ")")
    assert.is_true(math.abs(ny) < 0.5)
    assert.is_true(px > 4 and px < 5.1, "contact point near the wall face (px=" .. tostring(px) .. ")")
    assert.is_true(depth >= 0, "penetration depth should be non-negative")
    assert.is_nil(w:contactInfo(bc, w:staticCollider("ball", 99, 99, 0.5)), "non-touching pair → nil")
  end)

  it("locks rotation via the raw locked-axes mask", function()
    local w = rapier.newWorld()
    local b = w:newBody("dynamic", 0, 0)
    w:attachCollider(b, "cuboid", 0.5, 0.5)
    w:setLockedAxes(b, 0x4) -- bit 2 = rotation locked
    w:applyTorqueImpulse(b, 10)
    for _ = 1, 30 do w:step(1 / 60) end
    assert.is_true(math.abs(w:angvel(b)) < 0.01, "rotation lock failed (angvel=" .. w:angvel(b) .. ")")
  end)

  it("accepts combine rules, contact skin, solver iters, and collider mass properties", function()
    local w = rapier.newWorld()
    local b = w:newBody("dynamic", 0, 0)
    local c = w:attachCollider(b, "ball", 0.5)
    w:setFrictionCombineRule(c, "max")
    w:setRestitutionCombineRule(c, "min")
    w:setContactSkin(c, 0.02)
    w:setAdditionalSolverIterations(b, 4)
    w:setColliderMassProperties(c, 5, 0, 0, 1)
    assert.is_true(math.abs(w:colliderMass(c) - 5) < 0.01, "collider mass not set (" .. w:colliderMass(c) .. ")")
    w:setLinvel(b, 3, 0)
    w:step(1 / 60)
    assert.is_true(w:position(b) > 0, "sim still advances")
  end)
end)
