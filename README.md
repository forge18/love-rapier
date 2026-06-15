# love-rapier

A LÖVE 2D physics binding for **[Rapier2D](https://rapier.rs)** via LuaJIT FFI — deterministic
rigid-body physics (collision, forces/impulses, sensors, queries, a kinematic character controller)
for [LÖVE](https://love2d.org). Rapier runs as a native shim; the Lua side is a small idiomatic
wrapper. No `love.physics`/Box2D involved.

`love .` from this repo runs a demo (drive a ball; it collides with walls and other balls).

## Why Rapier

Cross-platform **determinism** (`enhanced-determinism`) — a step sequence yields bit-identical
results across runs and platforms — plus performance headroom. The shim pins `rapier2d = 0.33`.

## Install

This is a native binding, so you need two things in your game:

1. The Lua module: copy `rapier/` into your project so `require("rapier.system")` resolves (e.g.
   onto your require path). It's a folder module, so the path must include `?/init.lua` (LÖVE's
   default require path already does).
2. The native library for each platform you ship, dropped at `lib/native/<platform>/`:
   - `macos-arm64/librapier_shim.dylib`, `macos-x86_64/librapier_shim.dylib`
   - `linux-x64/librapier_shim.so`
   - `windows-x64/rapier_shim.dll`

   Prebuilt binaries are attached to each GitHub Release (and committed under `lib/native/`). The
   loader (`rapier/ffi.lua`) resolves `<source>/lib/native/<platform>/<libname>` — under LÖVE that's
   `love.filesystem.getSource()`; otherwise the cwd (or `$RAPIER_ROOT`).

## Usage

```lua
local Physics = require("rapier.system")   -- fixed-step adapter (events, debug-draw, locomotion)

function love.load()
  phys = Physics.new({ fixedDt = 1/60 })
  phys:addStatic(400, 300, { kind = "cuboid", hx = 60, hy = 60 })          -- map geometry
  player = phys:newActor("dynamic", 150, 150, { kind = "ball", radius = 16 })
  phys:onCollision(function(e) --[[ e.a, e.b colliders; e.started; e.sensor ]] end)
end

function love.update(dt)
  phys.world:setLinvel(player, vx, vy)   -- or :applyImpulse / :drive(force) for steering
  phys:update(dt)                        -- fixed-timestep accumulator; dispatches events
end

function love.draw() phys:debugDraw() end
```

Lower level: `local rapier = require("rapier"); local world = rapier.newWorld()` for the raw `World`
(bodies, colliders, filtering, forces, queries, KCC) without the fixed-step/debug-draw adapter.

## Building the native shim

The shim is a Rust crate (`native/rapier_shim`). `scripts/build-native.sh` cross-builds all four
platforms from one host (native cargo for macOS/Windows-mingw, `cargo-zigbuild` for Linux) into
`lib/native/<platform>/`. Requires Rust, `cargo-zigbuild` + zig, and mingw-w64.

## License

love-rapier (the wrapper + shim) is MIT — see `LICENSE`. The compiled binary links **Rapier2D**,
which is Apache-2.0; retain its notice when redistributing the binaries.
