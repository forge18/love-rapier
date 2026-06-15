# love-rapier — Lua API reference

Two modules:

- **`require("rapier")`** — the generic, game-agnostic physics world (`World`). Everything below
  under "World" lives here, plus the two module functions `rapier.newWorld()` and `rapier.restore()`.
- **`require("rapier.system")`** — an optional game-facing adapter (`Physics`): a fixed-timestep
  loop, event dispatch, static-geometry helpers, a steering integrator, and LÖVE debug drawing built
  on top of `World`. See "System adapter" at the end.

## Conventions

- **Handles** (bodies, colliders, joints) are opaque `uint64` cdata. Pass them straight back into
  methods; don't do arithmetic on them. The world owns them; it frees itself via GC.
- **Shapes** are named strings with up to two size args `(a, b)`:
  `"ball"` (a = radius), `"cuboid"` (a = half-width, b = half-height),
  `"capsule"` (a = half-height, b = radius).
- **Body kinds**: `"dynamic"`, `"fixed"`, `"kinematic"` (velocity-based), `"kinematicPosition"`.
- **Units / axes**: world units (≈ meters by default — see `setLengthUnit`), radians for angles,
  seconds for time. Gravity defaults to `(0, 0)` (top-down).
- **Queries reflect the last `step()`.** Spatial queries read the broad-phase as of the last step, so
  step once after building a world before querying it (a running game steps every frame).

---

## World — creation & stepping

| Method | Description |
|---|---|
| `rapier.newWorld()` → `World` | Create a world (top-down, zero gravity). |
| `World:step(dt)` | Advance the simulation by `dt` seconds. Drive on a fixed timestep. |
| `rapier.restore(bytes)` → `World`\|`nil` | Rebuild a world from a `snapshot()` string (see Serialization). |

### World config

| Method | Description |
|---|---|
| `World:setGravity(gx, gy)` | Set gravity (default `(0,0)`). |
| `World:setSolverIterations(n)` | Constraint solver iterations per step (stiffer/slower). |
| `World:setLengthUnit(u)` | Length unit (≈ typical object size); tunes internal tolerances. |
| `World:setMaxCcdSubsteps(n)` | Max continuous-collision substeps per step. |

---

## Bodies

### Create / remove / transform

| Method | Description |
|---|---|
| `World:newBody(kind, x, y)` → `body` | Create a rigid body of `kind` at `(x,y)`. |
| `World:removeBody(h)` | Remove a body (and its colliders/joints). |
| `World:position(h)` → `x, y` | Body translation. |
| `World:rotation(h)` → `radians` | Body rotation angle. |
| `World:readTransforms(handles, count, out)` | Batched read: fill `float[2*count]` with x,y per handle in one FFI call. |
| `World:setPosition(h, x, y)` | Teleport (no velocity change). |
| `World:setRotation(h, angle)` | Set rotation. |
| `World:setPose(h, x, y, angle)` | Set translation + rotation together. |

### Velocity / forces

| Method | Description |
|---|---|
| `World:linvel(h)` → `vx, vy` | Linear velocity. |
| `World:setLinvel(h, vx, vy)` | Set linear velocity. |
| `World:angvel(h)` → `rad/s` | Angular velocity. |
| `World:setAngvel(h, av)` | Set angular velocity. |
| `World:applyImpulse(h, x, y)` | Instantaneous linear impulse (knockback). |
| `World:addForce(h, x, y)` | Force over the next step (re-apply each step). |
| `World:applyTorqueImpulse(h, t)` | Instantaneous angular impulse. |
| `World:addTorque(h, t)` | Torque over the next step. |
| `World:applyImpulseAtPoint(h, ix, iy, px, py)` | Off-center impulse at a world point. |
| `World:addForceAtPoint(h, fx, fy, px, py)` | Off-center force at a world point. |
| `World:resetForces(h)` | Clear accumulated force. |
| `World:resetTorques(h)` | Clear accumulated torque. |

### Damping / gravity / type / enable / sleep

| Method | Description |
|---|---|
| `World:setLinearDamping(h, d)` | Linear velocity damping. |
| `World:setAngularDamping(h, d)` | Angular velocity damping. |
| `World:setGravityScale(h, s)` | Per-body gravity multiplier (0 = unaffected). |
| `World:setBodyType(h, kind)` | Change body type at runtime. |
| `World:bodyType(h)` → `string` | Read body type. |
| `World:setEnabled(h, enabled)` / `World:isEnabled(h)` → `bool` | Enable/disable a body. |
| `World:wakeUp(h)` / `World:sleep(h)` / `World:isSleeping(h)` → `bool` | Sleep control. |
| `World:setDominance(h, group)` | Dominance group (−127..127); higher wins. |
| `World:enableCcd(h, enabled)` | Continuous collision detection (anti-tunneling). |
| `World:setSoftCcdPrediction(h, dist)` | Cheaper anti-tunneling for moderately fast bodies. |

### Locks

| Method | Description |
|---|---|
| `World:lockRotations(h, locked)` | Lock/unlock rotation. |
| `World:lockTranslations(h, locked)` | Lock/unlock all translation. |
| `World:setEnabledTranslations(h, allowX, allowY)` | Allow/forbid translation per axis. |
| `World:setLockedAxes(h, bits)` | Raw mask: bit0 = transX, bit1 = transY, bit2 = rotation. |

### Mass

| Method | Description |
|---|---|
| `World:mass(h)` → `number` | Effective body mass. |
| `World:centerOfMass(h)` → `x, y` | Center of mass (world). |
| `World:setAdditionalMass(h, m)` | Add to collider-derived mass. |
| `World:setAdditionalMassProperties(h, mass, comX, comY, inertia)` | Add mass + COM + 2D inertia. |
| `World:recomputeMass(h)` | Recompute mass from attached colliders. |
| `World:setAdditionalSolverIterations(h, n)` | Extra per-body solver iterations. |

### Kinematic-position targets

| Method | Description |
|---|---|
| `World:setNextKinematicTranslation(h, x, y)` | Target translation at end of next step. |
| `World:setNextKinematicRotation(h, angle)` | Target rotation at end of next step. |

---

## Colliders

### Attach / static / remove

| Method | Description |
|---|---|
| `World:attachCollider(body, shape, a, b)` → `collider` | Attach a collider to a body. |
| `World:staticCollider(shape, x, y, a, b)` → `collider` | Free-standing static collider. |
| `World:removeCollider(h)` | Remove a collider. |
| `World:setShape(h, shape, a, b)` | Swap a collider's shape in place. |

### Extended static shapes (map geometry)

| Method | Description |
|---|---|
| `World:staticTriangle(ax, ay, bx, by, cx, cy)` → `collider` | Triangle. |
| `World:staticSegment(ax, ay, bx, by)` → `collider` | Segment. |
| `World:staticPolyline(points)` → `collider` | Open chain (`{{x,y},...}` or flat `{x1,y1,...}`). |
| `World:staticConvexHull(points)` → `collider`\|`nil` | Convex hull (nil if degenerate). |
| `World:staticHeightfield(heights, scaleX, scaleY)` → `collider` | Heightfield terrain. |

### Materials / config / offsets

| Method | Description |
|---|---|
| `World:setFriction(h, f)` / `World:setRestitution(h, r)` / `World:setDensity(h, d)` | Material coefficients. |
| `World:setColliderMass(h, m)` | Set collider mass. |
| `World:setColliderMassProperties(h, mass, comX, comY, inertia)` | Full collider mass properties. |
| `World:setFrictionCombineRule(h, rule)` / `World:setRestitutionCombineRule(h, rule)` | `"average"`\|`"min"`\|`"multiply"`\|`"max"`. |
| `World:setContactSkin(h, skin)` | Extra contact margin. |
| `World:setSensor(h, sensor)` / `World:colliderIsSensor(h)` → `bool` | Sensor (intersections, no response). |
| `World:setGroups(h, memberships, filter)` | Collision groups (event/geometry filtering). |
| `World:setSolverGroups(h, memberships, filter)` | Solver groups (contact-force filtering). |
| `World:setColliderEnabled(h, enabled)` | Enable/disable a collider. |
| `World:setActiveEvents(h, collision, contactForce)` | Which events the collider emits. |
| `World:setActiveCollisionTypes(h, bits)` | Raw `ActiveCollisionTypes` mask. |
| `World:setContactForceThreshold(h, threshold)` | Min force for contact-force events. |
| `World:setTranslationWrtParent(h, x, y)` / `World:setRotationWrtParent(h, angle)` | Offset from parent body. |
| `World:setColliderTranslation(h, x, y)` / `World:setColliderRotation(h, angle)` | Teleport a static collider. |

### Reads

| Method | Description |
|---|---|
| `World:colliderPosition(h)` → `x, y` | Absolute collider position. |
| `World:colliderDensity(h)` / `World:colliderMass(h)` / `World:colliderVolume(h)` → `number` | Collider properties. |
| `World:colliderParent(h)` → `body`\|`nil` | Parent body, or nil if free-standing. |

---

## Queries

> Reflect the broad phase as of the last `step()`.

| Method | Description |
|---|---|
| `World:raycast(ox, oy, dx, dy, maxToi)` → `collider, dist`\|`nil` | Nearest ray hit. |
| `World:raycastNormal(ox, oy, dx, dy, maxToi)` → `collider, dist, nx, ny`\|`nil` | Ray hit + surface normal. |
| `World:raycastFiltered(ox, oy, dx, dy, maxToi, memberships, filter, exclude)` → `collider, dist`\|`nil` | Ray honoring groups / excluding a collider. |
| `World:pointQuery(px, py)` → `collider`\|`nil` | First collider containing a point. |
| `World:projectPoint(px, py, maxDist)` → `collider, x, y, inside`\|`nil` | Nearest surface point. |
| `World:overlapCircle(x, y, radius)` → `{collider,...}` | Colliders overlapping a circle. |
| `World:overlapShape(shape, a, b, x, y, angle)` → `{collider,...}` | Colliders overlapping any shape. |
| `World:shapeCast(shape, a, b, ox, oy, angle, dx, dy, maxToi)` → `collider, toi`\|`nil` | Sweep a shape. |
| `World:contactsWith(h)` → `{collider,...}` | Colliders in solid contact (last step). |
| `World:intersectionsWith(h)` → `{collider,...}` | Colliders intersecting via sensor pairs (last step). |
| `World:contactInfo(c1, c2)` → `nx, ny, px, py, depth`\|`nil` | Deepest-contact geometry (depth > 0 = overlap). |

---

## Kinematic character controller

| Method | Description |
|---|---|
| `World:moveBall(x, y, radius, dx, dy)` → `allowedDx, allowedDy` | Collision-corrected move for a ball. |
| `World:moveShape(shape, a, b, x, y, angle, dx, dy, offset)` → `allowedDx, allowedDy, grounded` | Corrected move for any shape (+ grounded). |

---

## Joints

Anchors are in each body's **local** frame. Creators return an opaque joint handle.

| Method | Description |
|---|---|
| `World:fixedJoint(b1, b2, a1x, a1y, a2x, a2y)` → `joint` | Weld two bodies rigidly. |
| `World:revoluteJoint(b1, b2, a1x, a1y, a2x, a2y)` → `joint` | Hinge (free rotation about anchor). |
| `World:prismaticJoint(b1, b2, a1x, a1y, a2x, a2y, axisX, axisY)` → `joint` | Slider along an axis. |
| `World:ropeJoint(b1, b2, maxDist, a1x, a1y, a2x, a2y)` → `joint` | Cap anchor distance. |
| `World:springJoint(b1, b2, restLength, stiffness, damping, a1x, a1y, a2x, a2y)` → `joint` | Damped spring. |
| `World:removeJoint(h)` | Remove an (impulse) joint. |

### Motors & limits (joint axis: `"linX"` \| `"linY"` \| `"ang"`)

| Method | Description |
|---|---|
| `World:setJointMotor(joint, axis, targetPos, targetVel, stiffness, damping)` | Drive an axis toward a target. |
| `World:setJointLimits(joint, axis, min, max)` | Clamp an axis. |
| `World:setJointMotorMaxForce(joint, axis, maxForce)` | Cap motor force. |
| `World:setJointContactsEnabled(joint, enabled)` | Whether the jointed bodies collide. |

### Multibody (articulated; no positional drift)

| Method | Description |
|---|---|
| `World:multibodyFixedJoint(...)` / `multibodyRevoluteJoint(...)` / `multibodyPrismaticJoint(...)` → `joint`\|`nil` | Same args as the impulse variants; nil if it would form an unrepresentable loop. |
| `World:removeMultibodyJoint(h)` | Remove a multibody joint. |

---

## Events

Drain once per frame after stepping.

| Method | Description |
|---|---|
| `World:drainEvents()` → `{ {a, b, started, sensor}, ... }` | Collision/sensor events (a/b are collider handles; started/sensor are booleans). |
| `World:drainForceEvents()` → `{ {a, b, magnitude}, ... }` | Contact-force events (require `setActiveEvents(h, _, true)` + `setContactForceThreshold`). |

---

## Physics hooks

Pair-filter callbacks `fn(colliderA, colliderB)` returning truthy to keep the pair, falsy to discard.
They run per candidate pair **every step** and call back into Lua — keep them cheap. Only colliders
opted in via `setActiveHooks` consult them.

| Method | Description |
|---|---|
| `World:setActiveHooks(h, contact, intersection)` | Opt a collider into hook callbacks. |
| `World:setContactFilter(fn)` | Filter solid contacts (`nil` to clear). |
| `World:setIntersectionFilter(fn)` | Filter sensor intersections (`nil` to clear). |

---

## Serialization

| Method | Description |
|---|---|
| `World:snapshot()` → `string`\|`nil` | Serialize the full simulation to a byte string. |
| `rapier.restore(bytes)` → `World`\|`nil` | Rebuild a world from a snapshot (handle indices preserved). |

Deterministic save states / lockstep-rollback netcode. Pipelines, solver, hooks, and event queues
are transient and not part of the snapshot.

---

## Debug render

| Method | Description |
|---|---|
| `World:debugLines()` → `{ax, ay, bx, by, ...}` | Rapier's debug geometry as flat line vertices (4 numbers per line). Reflects the last step. |

---

## System adapter (`require("rapier.system")`)

A `Physics` manager wrapping `World` with the things a game loop needs. Create one at the composition
root and inject it.

| Method | Description |
|---|---|
| `Physics.new(deps?)` → `Physics` | `deps.fixedDt` overrides the step (default 1/60). Has a `.world` field. |
| `Physics:update(dt)` | Fixed-timestep accumulator: steps 0+ times, dispatches drained events, clamps against spiral-of-death. |
| `Physics:alpha()` → `0..1` | Interpolation remainder for rendering between steps. |
| `Physics:onCollision(fn)` → `unsubscribe` | Subscribe to `{a, b, started, sensor}` events. |
| `Physics:newActor(bodyKind, x, y, shape)` → `body, collider` | Body + one collider. `shape` is `{kind=, radius=/hx=,hy=/halfHeight=,radius=}`. |
| `Physics:addStatic(x, y, shape)` → `collider` | One static collider. |
| `Physics:addStaticGeometry(items)` | Batch of `{ {x, y, shape}, ... }`. |
| `Physics:drive(body, fx, fy, maxSpeed, dt)` | Integrate a steering force into velocity, clamped to `maxSpeed`. |
| `Physics:debugDraw()` | Outline every collider (dev; needs a live `love.graphics`). |
