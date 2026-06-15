# love-rapier

A [LÖVE](https://love2d.org) 2D physics binding for **[Rapier2D](https://rapier.rs)** via LuaJIT FFI.

> **⚠️ Alpha.** The binding is feature-complete — the full Rapier2D surface is bound — and the test
> suite runs in CI on macOS, Linux, and Windows. But it's new and hasn't been battle-tested across
> all platforms in real projects yet. Expect rough edges; please report anything you hit.

## Overview

Rapier runs as a native shim (a small Rust `cdylib`); the Lua side is a thin, idiomatic wrapper. No
`love.physics` / Box2D involved.

The reason to use it over Box2D is **determinism**: with Rapier's `enhanced-determinism`, a given
sequence of steps yields bit-identical results across runs and platforms — the foundation for
lockstep simulation, replays, and rollback netcode — plus more performance headroom. The shim pins
`rapier2d = 0.33`.

Run `love .` from this repo for a demo (drive a ball; it collides with walls and other balls).

## Functionality

The Rapier 2D surface is bound in full. By area:

- **Bodies** — dynamic / fixed / kinematic; linear & angular velocity, impulses, forces and torque
  (including at-a-point), damping, gravity scale, CCD / soft-CCD, sleeping, axis locks, full-pose
  set, mass and custom mass properties.
- **Colliders** — ball, cuboid, capsule, triangle, segment, polyline, convex hull, heightfield;
  friction / restitution / density / mass, sensors, collision & solver groups, parent offsets,
  in-place shape swap, configurable events.
- **Queries** — raycast (plus surface normal, plus group filtering), point, shape-cast, point
  projection, circle / shape overlap, narrow-phase contacts & intersections, and contact-manifold
  geometry (normal, point, depth).
- **Joints** — fixed, revolute, prismatic, rope, spring — as impulse *or* multibody — with motors
  and limits.
- **Character controller** — kinematic move-and-slide for any shape, with a grounded report.
- **Events** — collision, sensor, and contact-force events.
- **Physics hooks** — contact / intersection pair-filter callbacks.
- **Serialization** — full-world snapshot / restore (deterministic save states; rollback netcode).
- **Debug render** — Rapier's own debug line geometry.

The complete method-by-method reference is in **[docs/api.md](docs/api.md)**.

> **Not available in 2D: gyroscopic forces.** `RigidBody::enable_gyroscopic_forces` is gated
> `#[cfg(feature = "dim3")]` in Rapier's shared codebase, so it's compiled out of `rapier2d`
> entirely — the method doesn't exist in the 2D build, and no 2D binding can expose it. It's a
> 3D-only effect (precession of a spinning body); 2D angular motion is a single scalar axis where it
> has no meaning.

## Setup

It's a native binding, so a consuming game needs two things:

1. **The Lua module** — copy `rapier/` into your project so `require("rapier.system")` resolves.
   It's a folder module, so your require path must include `?/init.lua` (LÖVE's default already does).
2. **The native library** for each platform you ship, placed at `lib/native/<platform>/`:
   - `macos-arm64/librapier_shim.dylib`, `macos-x86_64/librapier_shim.dylib`
   - `linux-x64/librapier_shim.so`
   - `windows-x64/rapier_shim.dll`

   Prebuilt binaries are attached to each GitHub Release (and committed under `lib/native/`). The
   loader (`rapier/ffi.lua`) resolves `<source>/lib/native/<platform>/<libname>` — under LÖVE that's
   `love.filesystem.getSource()`, otherwise the cwd (or `$RAPIER_ROOT`).

## Usage

The `rapier.system` adapter adds a fixed-timestep loop, event dispatch, and debug drawing:

```lua
local Physics = require("rapier.system")

function love.load()
  phys = Physics.new({ fixedDt = 1/60 })
  phys:addStatic(400, 300, { kind = "cuboid", hx = 60, hy = 60 })       -- map geometry
  player = phys:newActor("dynamic", 150, 150, { kind = "ball", radius = 16 })
  phys:onCollision(function(e) --[[ e.a, e.b colliders; e.started; e.sensor ]] end)
end

function love.update(dt)
  phys.world:setLinvel(player, vx, vy)   -- or :applyImpulse / :drive(force) for steering
  phys:update(dt)                        -- fixed-timestep accumulator; dispatches events
end

function love.draw() phys:debugDraw() end
```

For the raw world without the adapter, use `require("rapier")` directly:
`local world = require("rapier").newWorld()`. See [docs/api.md](docs/api.md).

One gotcha: spatial queries reflect the broad phase **as of the last `step()`** — query a
freshly-built, never-stepped world and it sees nothing, so step once first (a running game steps
every frame).

## Building

The shim is a Rust crate (`native/rapier_shim`). `scripts/build-native.sh` cross-builds all four
platforms from one host — native cargo for macOS, mingw-w64 for Windows, `cargo-zigbuild` for Linux —
into `lib/native/<platform>/`. Requires Rust, `cargo-zigbuild` + zig, and mingw-w64. Build a single
platform with e.g. `scripts/build-native.sh macos-arm64`.

**Tests** (`tests/`) are integration-level — they load the real compiled binary over FFI and exercise
the whole surface (forces, collisions, sensors, queries, joints, the character controller, hooks,
snapshot/restore, and bit-identical determinism). They run under **LuaJIT** from the repo root:

```sh
scripts/build-native.sh macos-arm64   # build the host shim first
luajit tests/run.lua                  # dependency-free runner (no luarocks/busted needed)
```

(The tests are also busted-compatible — `busted` runs them too — but the bundled runner needs nothing
but LuaJIT, which sidesteps luarocks's manifest not loading under LuaJIT.)

CI (`.github/workflows/build.yml`) runs the suite on **all four platforms** — each building and
executing its own native binary — and gates the release build on it.

## License

love-rapier (the wrapper + shim) is MIT — see `LICENSE`. The compiled binary links **Rapier2D**,
which is Apache-2.0; retain its notice when redistributing the binaries.
