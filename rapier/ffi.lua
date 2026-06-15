-- Raw LuaJIT FFI binding to the Rapier shim (native/rapier_shim -> lib/native/<platform>/).
--
-- This is the thin, header-mirroring layer: an `ffi.cdef` matching the shim's `extern "C"` surface
-- plus the per-platform `ffi.load`. Nothing idiomatic lives here — `rapier` (init.lua) wraps
-- this into a friendly Lua API; `rapier.system` is the game adapter. Regenerate the C header with
-- `cbindgen` (see native/rapier_shim/cbindgen.toml) and keep this cdef in sync with it.

local ffi = require("ffi")

ffi.cdef([[
typedef struct PhysicsWorld PhysicsWorld;

typedef struct { uint64_t a; uint64_t b; int32_t started; int32_t sensor; } ContactRecord;

/* world */
PhysicsWorld* shim_world_new(void);
void          shim_world_free(PhysicsWorld* world);
void          shim_world_step(PhysicsWorld* world, float dt);

/* events */
uint32_t      shim_events_count(PhysicsWorld* world);
int32_t       shim_events_get(PhysicsWorld* world, uint32_t i, ContactRecord* out);
void          shim_events_clear(PhysicsWorld* world);

/* bodies (kind: 0=dynamic, 1=fixed, 2=kinematic-velocity) */
uint64_t      shim_body_create(PhysicsWorld* world, int32_t kind, float x, float y);
void          shim_body_remove(PhysicsWorld* world, uint64_t handle);
void          shim_body_position(PhysicsWorld* world, uint64_t handle, float* out_x, float* out_y);
void          shim_bodies_read_transforms(PhysicsWorld* world, const uint64_t* handles, uint32_t count, float* out);
float         shim_body_rotation(PhysicsWorld* world, uint64_t handle);
void          shim_body_set_translation(PhysicsWorld* world, uint64_t handle, float x, float y);
void          shim_body_set_linvel(PhysicsWorld* world, uint64_t handle, float vx, float vy);
void          shim_body_linvel(PhysicsWorld* world, uint64_t handle, float* out_x, float* out_y);
void          shim_body_lock_rotations(PhysicsWorld* world, uint64_t handle, bool locked);
void          shim_body_set_linear_damping(PhysicsWorld* world, uint64_t handle, float damping);
void          shim_body_apply_impulse(PhysicsWorld* world, uint64_t handle, float x, float y);
void          shim_body_add_force(PhysicsWorld* world, uint64_t handle, float x, float y);
void          shim_body_enable_ccd(PhysicsWorld* world, uint64_t handle, bool enabled);

/* colliders (shape: 0=ball[a=radius], 1=cuboid[a=hx,b=hy], 2=capsule[a=half_h,b=radius]) */
uint64_t      shim_collider_attach(PhysicsWorld* world, uint64_t body, int32_t shape, float a, float b);
uint64_t      shim_collider_static(PhysicsWorld* world, int32_t shape, float x, float y, float a, float b);
void          shim_collider_remove(PhysicsWorld* world, uint64_t handle);
void          shim_collider_set_sensor(PhysicsWorld* world, uint64_t handle, bool sensor);
void          shim_collider_set_groups(PhysicsWorld* world, uint64_t handle, uint32_t memberships, uint32_t filter);

/* queries */
int32_t       shim_query_raycast(PhysicsWorld* world, float ox, float oy, float dx, float dy,
                                 float max_toi, uint64_t* out_collider, float* out_toi);
int32_t       shim_query_point(PhysicsWorld* world, float px, float py, uint64_t* out_collider);
uint32_t      shim_query_overlap_circle(PhysicsWorld* world, float x, float y, float radius);
uint32_t      shim_overlap_count(PhysicsWorld* world);
uint64_t      shim_overlap_get(PhysicsWorld* world, uint32_t i);

/* kinematic character controller */
void          shim_kcc_move_ball(PhysicsWorld* world, float x, float y, float radius,
                                 float dx, float dy, float* out_x, float* out_y);
]])

-- Resolve lib/native/<platform>/<libname> for the current OS+arch.
local function platformDir()
  local os, arch = jit.os, jit.arch
  if os == "OSX" then
    return arch == "arm64" and "macos-arm64" or "macos-x86_64"
  elseif os == "Windows" then
    return "windows-x64"
  elseif os == "Linux" then
    return "linux-x64"
  end
  error("rapier shim: unsupported platform " .. tostring(os) .. "/" .. tostring(arch))
end

local function libName()
  if jit.os == "OSX" then
    return "librapier_shim.dylib"
  elseif jit.os == "Windows" then
    return "rapier_shim.dll"
  end
  return "librapier_shim.so"
end

-- Base dir: LÖVE's project source when running the game, else the current dir (tests/CLI).
local function baseDir()
  if rawget(_G, "love") and love.filesystem and love.filesystem.getSource then
    return love.filesystem.getSource()
  end
  return os.getenv("RAPIER_ROOT") or "."
end

local path = baseDir() .. "/lib/native/" .. platformDir() .. "/" .. libName()
local C = ffi.load(path)

return C
