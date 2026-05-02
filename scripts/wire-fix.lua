local algo_v18 = require("scripts.algo-v18")
local algo_segments = require("scripts.algo-segments")

local M = {}

local POLE_COPPER = defines.wire_connector_id.pole_copper
local WIRE_ORIGIN = defines.wire_origin.player

local config = {dev_mode = false}
local ok, local_config = pcall(require, "config-local")
if ok and type(local_config) == "table" then
  for k, v in pairs(local_config) do config[k] = v end
end

local function pair_key(a, b)
  if a < b then return a .. "_" .. b end
  return b .. "_" .. a
end

local function get_settings(player)
  local s = settings.get_player_settings(player)
  return s["wire-fix-max-conn"].value
end

local function notify(player, key, ...)
  player.create_local_flying_text{
    text = {key, ...},
    create_at_cursor = true,
  }
end

function M.run(player_index, entities)
  local player = game.get_player(player_index)
  if not player then return end

  local poles = {}
  for _, e in pairs(entities) do
    if e.valid and e.type == "electric-pole" then
      poles[#poles + 1] = e
    end
  end
  table.sort(poles, function(a, b) return a.unit_number < b.unit_number end)

  local n = #poles
  if n == 0 then
    notify(player, "wire-fix.no-poles")
    return
  end

  local p_total = game.create_profiler()

  -- Phase 1: classify existing internal connections. External (out-of-selection)
  -- wires aren't touched at all; internal ones survive the diff-apply if v18
  -- also wants them.
  local p_phase1 = game.create_profiler()
  local pole_set = {}
  for i = 1, n do pole_set[poles[i].unit_number] = true end

  local internal_pairs = {}
  for i = 1, n do
    local p = poles[i]
    local connector = p.get_wire_connector(POLE_COPPER, false)
    if connector then
      for _, conn in pairs(connector.connections) do
        local target = conn.target
        local other = target and target.owner
        if other and other.valid and pole_set[other.unit_number] then
          local k = pair_key(p.unit_number, other.unit_number)
          if not internal_pairs[k] then
            internal_pairs[k] = {p, other}
          end
        end
      end
    end
  end
  p_phase1.stop()

  -- Phase 2: segment-based pattern detection.
  -- NOTE: the per-user "max connections per pole" setting is NOT honored in
  -- v0.2 — the algorithm may produce more wires per pole than v0.1.0.
  -- Re-introducing the cap is tracked for a follow-up.
  -- algo_v18 is still required at the top of the file as a fallback; revert
  -- this commit to switch back to the midpoint-based algorithm.
  local p_algo = game.create_profiler()
  local points = {}
  local reaches = {}
  for i = 1, n do
    local p = poles[i]
    points[i] = { x = p.position.x, y = p.position.y }
    reaches[i] = p.prototype.get_max_wire_distance(p.quality)
  end
  local algo_edges, algo_accepted, algo_candidates, algo_stats =
    algo_segments.compute(points, reaches)
  p_algo.stop()

  -- Phase 3: build wanted set keyed by pair_key(unit_number).
  local wanted = {}
  for _, edge in ipairs(algo_edges) do
    local pa, pb = poles[edge[1]], poles[edge[2]]
    wanted[pair_key(pa.unit_number, pb.unit_number)] = { pa, pb }
  end

  -- Phase 4: diff-apply. Edges in both internal_pairs and wanted stay untouched.
  local p_diff = game.create_profiler()
  local disconnect_calls, connect_calls = 0, 0

  for k, pair in pairs(internal_pairs) do
    if not wanted[k] then
      local p1, p2 = pair[1], pair[2]
      if p1.valid and p2.valid then
        local c1 = p1.get_wire_connector(POLE_COPPER, false)
        local c2 = p2.get_wire_connector(POLE_COPPER, false)
        if c1 and c2 then
          c1.disconnect_from(c2, WIRE_ORIGIN)
          disconnect_calls = disconnect_calls + 1
        end
      end
    end
  end

  for k, pair in pairs(wanted) do
    if not internal_pairs[k] then
      local p1, p2 = pair[1], pair[2]
      if p1.valid and p2.valid then
        local c1 = p1.get_wire_connector(POLE_COPPER, true)
        local c2 = p2.get_wire_connector(POLE_COPPER, true)
        if c1 and c2 then
          c1.connect_to(c2, false, WIRE_ORIGIN)
          connect_calls = connect_calls + 1
        end
      end
    end
  end
  p_diff.stop()
  p_total.stop()

  notify(player, "wire-fix.done", n, #algo_edges)

  if config.dev_mode then
    local internal_count = 0
    for _ in pairs(internal_pairs) do internal_count = internal_count + 1 end
    player.print({"", "[Wire Fix v0.2/segments] total: ", p_total})
    player.print(string.format(
      "  poles=%d  internal_existing=%d  edges_wanted=%d  candidates=%d  accepted=%d  api: disconnect=%d connect=%d",
      n, internal_count, #algo_edges,
      algo_stats.candidate_count or 0, algo_stats.accepted_count or 0,
      disconnect_calls, connect_calls))
    player.print({"", "  phase1=", p_phase1, "  algo=", p_algo, "  diff=", p_diff})
    player.print({"",
      "  setup=", algo_stats.t_setup,
      "  pairs=", algo_stats.t_pairs,
      "  segments=", algo_stats.t_segments,
      "  index=", algo_stats.t_index})
    player.print({"",
      "  parls=", algo_stats.t_parls,
      "  accept=", algo_stats.t_accept,
      "  edges=", algo_stats.t_edges,
      "  mst=", algo_stats.t_mst})
    player.print(string.format(
      "  pair_count=%d  lines=%d  segments=%d  candidates=%d  accepted=%d  edges_from_parls=%d  total_edges=%d",
      algo_stats.pair_count, algo_stats.line_count,
      algo_stats.segment_count, algo_stats.candidate_count,
      algo_stats.accepted_count, algo_stats.edges_from_parls,
      algo_stats.total_edges))
  end
end

function M.disconnect_all(player_index, entities)
  local player = game.get_player(player_index)
  if not player then return end

  local poles = {}
  for _, e in pairs(entities) do
    if e.valid and e.type == "electric-pole" then
      poles[#poles + 1] = e
    end
  end

  local n = #poles
  if n == 0 then
    notify(player, "wire-fix.no-poles")
    return
  end

  local pole_set = {}
  for i = 1, n do pole_set[poles[i].unit_number] = true end

  local pairs_to_disconnect = {}
  for i = 1, n do
    local p = poles[i]
    local connector = p.get_wire_connector(POLE_COPPER, false)
    if connector then
      for _, conn in pairs(connector.connections) do
        local target = conn.target
        local other = target and target.owner
        if other and other.valid and pole_set[other.unit_number] then
          local k = pair_key(p.unit_number, other.unit_number)
          if not pairs_to_disconnect[k] then
            pairs_to_disconnect[k] = {p, other}
          end
        end
      end
    end
  end

  local count = 0
  for _, pair in pairs(pairs_to_disconnect) do
    local p1, p2 = pair[1], pair[2]
    if p1.valid and p2.valid then
      local c1 = p1.get_wire_connector(POLE_COPPER, false)
      local c2 = p2.get_wire_connector(POLE_COPPER, false)
      if c1 and c2 then
        c1.disconnect_from(c2, WIRE_ORIGIN)
        count = count + 1
      end
    end
  end

  notify(player, "wire-fix.disconnected", count, n)
end

return M
