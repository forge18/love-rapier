-- Minimal, dependency-free test runner for LuaJIT.
--
-- The suite is busted-style (describe/it/assert.*), but busted needs luarocks, and luarocks can't
-- load its manifest under LuaJIT (LuaJIT's 65536-constants-per-function limit). So rather than fight
-- that across four CI platforms, this provides the same small API surface the tests actually use and
-- runs them directly under LuaJIT. (The files remain busted-compatible — `busted` runs them too.)
--
-- Usage, from the repo root:  luajit tests/run.lua   (the FFI loader resolves ./lib/native/<plat>/)

package.path = "./?.lua;./?/init.lua;" .. package.path

-- ---- busted-compatible globals --------------------------------------------------------------

local queue = {}   -- ordered { name, fn }
local groups = {}  -- describe() name stack

function describe(name, fn)
  groups[#groups + 1] = name
  fn()
  groups[#groups] = nil
end

function it(name, fn)
  queue[#queue + 1] = { name = table.concat(groups, " › ") .. " › " .. name, fn = fn }
end

local function fail(msg, lvl)
  error(msg or "assertion failed", (lvl or 1) + 1)
end

local A = {
  is_true = function(v, m) if v ~= true then fail(m or ("expected true, got " .. tostring(v)), 1) end end,
  is_false = function(v, m) if v ~= false then fail(m or ("expected false, got " .. tostring(v)), 1) end end,
  is_nil = function(v, m) if v ~= nil then fail(m or ("expected nil, got " .. tostring(v)), 1) end end,
  is_not_nil = function(v, m) if v == nil then fail(m or "expected non-nil, got nil", 1) end end,
  is_boolean = function(v, m) if type(v) ~= "boolean" then fail(m or ("expected boolean, got " .. type(v)), 1) end end,
  equals = function(a, b, m)
    if a ~= b then fail(m or ("expected " .. tostring(a) .. " == " .. tostring(b)), 1) end
  end,
}
-- busted's `assert` is callable AND carries the matchers; mirror that.
assert = setmetatable(A, {
  __call = function(_, v, m) if not v then fail(m, 1) end return v end,
})

-- ---- discover + run -------------------------------------------------------------------------

-- Test files in run order (each dofile registers its describe/it blocks). Extra args override.
local files = { ... }
if #files == 0 then
  files = { "world", "system", "extended", "complete", "audit", "parity" }
end

for _, name in ipairs(files) do
  dofile("tests/" .. name .. ".test.lua")
end

local passed, failures = 0, {}
for _, t in ipairs(queue) do
  local ok, err = pcall(t.fn)
  if ok then
    passed = passed + 1
    io.write(".")
  else
    failures[#failures + 1] = { name = t.name, err = err }
    io.write("F")
  end
  -- Collect abandoned worlds/FFI-callbacks now, at a safe point between tests. Otherwise GC can run
  -- a world or callback finalizer *during* a step that's invoking a hook callback, which LuaJIT
  -- rejects with a "bad callback" panic (you can't free a callback from inside a callback).
  collectgarbage("collect")
end
io.write("\n")

for _, f in ipairs(failures) do
  io.write("\nFAIL: " .. f.name .. "\n  " .. tostring(f.err) .. "\n")
end

io.write(("\n%d passed, %d failed (%d total)\n"):format(passed, #failures, #queue))
os.exit(#failures == 0 and 0 or 1)
