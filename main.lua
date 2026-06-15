-- love-rapier demo. `love .` from the repo root.
-- Drive the yellow ball with WASD/arrows; it collides with walls, a central block, and loose balls,
-- all resolved by Rapier through the FFI shim. Debug-draw outlines every collider.

local Physics = require("rapier.system")

local demo = {}
local R = 16

function love.load()
  love.graphics.setBackgroundColor(0.10, 0.10, 0.12)
  demo.font = love.graphics.newFont(15)
  demo.phys = Physics.new({ fixedDt = 1 / 60 })
  local p = demo.phys
  local W, H = love.graphics.getDimensions()
  local t = 20

  -- bounds + a central obstacle (static colliders)
  p:addStatic(W / 2, -t, { kind = "cuboid", hx = W / 2, hy = t })
  p:addStatic(W / 2, H + t, { kind = "cuboid", hx = W / 2, hy = t })
  p:addStatic(-t, H / 2, { kind = "cuboid", hx = t, hy = H / 2 })
  p:addStatic(W + t, H / 2, { kind = "cuboid", hx = t, hy = H / 2 })
  p:addStatic(W / 2, H / 2, { kind = "cuboid", hx = 60, hy = 60 })

  -- player (driven) + loose dynamic balls
  demo.player = p:newActor("dynamic", 150, 150, { kind = "ball", radius = R })
  p.world:lockRotations(demo.player, true)
  p.world:setLinearDamping(demo.player, 8)

  demo.balls = {}
  for i = 1, 6 do
    local b = p:newActor("dynamic", 320 + i * 36, 220, { kind = "ball", radius = R })
    p.world:setLinearDamping(b, 2)
    demo.balls[i] = b
  end
end

function love.update(dt)
  local p = demo.phys
  local dx, dy = 0, 0
  if love.keyboard.isDown("a", "left") then dx = dx - 1 end
  if love.keyboard.isDown("d", "right") then dx = dx + 1 end
  if love.keyboard.isDown("w", "up") then dy = dy - 1 end
  if love.keyboard.isDown("s", "down") then dy = dy + 1 end
  local l = math.sqrt(dx * dx + dy * dy)
  if l > 0 then dx, dy = dx / l, dy / l end
  p.world:setLinvel(demo.player, dx * 240, dy * 240)
  p:update(dt)
end

function love.draw()
  demo.phys:debugDraw()
  local function ball(body, r, g, b)
    local x, y = demo.phys.world:position(body)
    love.graphics.setColor(r, g, b)
    love.graphics.circle("fill", x, y, R)
  end
  for _, b in ipairs(demo.balls) do ball(b, 0.4, 0.7, 0.9) end
  ball(demo.player, 0.95, 0.8, 0.3)

  love.graphics.setFont(demo.font)
  love.graphics.setColor(0.8, 0.8, 0.85)
  love.graphics.print("love-rapier demo — WASD/arrows to drive; Rapier resolves collisions  ·  Esc quits", 12, 12)
  love.graphics.setColor(1, 1, 1)
end

function love.keypressed(k)
  if k == "escape" then love.event.quit() end
end
