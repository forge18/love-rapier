function love.conf(t)
  t.window.title = "love-rapier demo"
  t.window.width = 800
  t.window.height = 600
  t.modules.physics = false -- we use Rapier (FFI), not LÖVE's Box2D
end
