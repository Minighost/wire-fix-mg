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

  local max_conn = get_settings(player)

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
  local conn_iters, find_calls = 0, 0
  local disconnect_calls, connect_calls = 0, 0

  local p_phase1 = game.create_profiler()

  -- Phase 1: classify connections
  local pole_set = {}
  for i = 1, n do pole_set[poles[i].unit_number] = true end

  local ext_count = {}
  local internal_pairs = {}
  for i = 1, n do
    local p = poles[i]
    ext_count[p.unit_number] = 0
    local connector = p.get_wire_connector(POLE_COPPER, false)
    if connector then
      for _, conn in pairs(connector.connections) do
        conn_iters = conn_iters + 1
        local target = conn.target
        local other = target and target.owner
        if other and other.valid then
          if pole_set[other.unit_number] then
            local k = pair_key(p.unit_number, other.unit_number)
            if not internal_pairs[k] then
              internal_pairs[k] = {p, other}
            end
          else
            ext_count[p.unit_number] = ext_count[p.unit_number] + 1
          end
        end
      end
    end
  end

  p_phase1.stop()
  local p_phase2 = game.create_profiler()

  -- Apply is deferred to Phase 4 — we compute the wanted internal-edge set
  -- in memory and then apply only the diff against the existing internal
  -- edges, so untouched wires aren't disturbed.

  local slot = {}
  local initial_slot = {}
  for i = 1, n do
    local u = poles[i].unit_number
    local s = max_conn - ext_count[u]
    if s < 0 then s = 0 end
    slot[u] = s
    initial_slot[u] = s
  end

  -- Phase 2: candidate edges (uses a Lua-side spatial hash — O(n × k) where
  -- k is the average number of poles within reach, instead of O(n²)).
  local edges = {}
  local reach_cache = {}
  for i = 1, n do
    local p = poles[i]
    reach_cache[p.unit_number] = p.prototype.get_max_wire_distance(p.quality)
  end

  local DEG = 180 / math.pi
  local function axis_key_of(dy, dx)
    -- Undirected axis in [0, 180), 1° buckets.
    local deg = math.atan2(dy, dx) * DEG
    local axis = deg % 180
    if axis < 0 then axis = axis + 180 end
    return math.floor(axis + 0.5) % 180
  end

  -- Lua-side spatial hash: bucket poles by chunk, then query candidates by
  -- iterating the chunks that intersect each pole's reach box.
  local CHUNK_SIZE = 8
  local chunks = {}
  for i = 1, n do
    local p = poles[i]
    local cx = math.floor(p.position.x / CHUNK_SIZE)
    local cy = math.floor(p.position.y / CHUNK_SIZE)
    local row = chunks[cx]
    if not row then row = {}; chunks[cx] = row end
    local cell = row[cy]
    if not cell then cell = {}; row[cy] = cell end
    cell[#cell + 1] = i
  end

  for i = 1, n do
    local p1 = poles[i]
    local u1 = p1.unit_number
    local pos1 = p1.position
    local r1 = reach_cache[u1]
    local cx_min = math.floor((pos1.x - r1) / CHUNK_SIZE)
    local cx_max = math.floor((pos1.x + r1) / CHUNK_SIZE)
    local cy_min = math.floor((pos1.y - r1) / CHUNK_SIZE)
    local cy_max = math.floor((pos1.y + r1) / CHUNK_SIZE)
    for cx = cx_min, cx_max do
      local row = chunks[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for _, idx in ipairs(cell) do
              if idx > i then
                local p2 = poles[idx]
                local r2 = reach_cache[p2.unit_number]
                local reach = r1 < r2 and r1 or r2
                local pos2 = p2.position
                local dx = pos2.x - pos1.x
                local dy = pos2.y - pos1.y
                local d2 = dx * dx + dy * dy
                if d2 <= reach * reach then
                  local d = math.sqrt(d2)
                  local dk = math.floor(d * 1000 + 0.5)
                  local ak = axis_key_of(dy, dx)
                  local au1, au2 = u1, p2.unit_number
                  if au1 > au2 then au1, au2 = au2, au1 end
                  edges[#edges + 1] = {
                    p1 = p1,
                    p2 = p2,
                    dist_key = dk,
                    pair_key = dk * 200 + ak,
                    u_low = au1,
                    u_high = au2,
                  }
                end
              end
            end
          end
        end
      end
    end
  end

  -- Stable initial order (dist, then canonical unit-number pair); sort_group
  -- re-orders each distance bucket by current slot priority before use.
  table.sort(edges, function(a, b)
    if a.dist_key ~= b.dist_key then return a.dist_key < b.dist_key end
    if a.u_low ~= b.u_low then return a.u_low < b.u_low end
    return a.u_high < b.u_high
  end)

  -- Per-pole candidate sets:
  --   pole_cand_pairs[u][pk] = true  (Step B hit filter)
  --   pole_cand_edges[u][pk] = list of edges from u at this (dist, axis) pair
  local pole_cand_pairs = {}
  local pole_cand_edges = {}
  for i = 1, n do
    local u = poles[i].unit_number
    pole_cand_pairs[u] = {}
    pole_cand_edges[u] = {}
  end
  for _, e in pairs(edges) do
    local pk = e.pair_key
    pole_cand_pairs[e.p1.unit_number][pk] = true
    pole_cand_pairs[e.p2.unit_number][pk] = true
    local m1 = pole_cand_edges[e.p1.unit_number]
    if not m1[pk] then m1[pk] = {} end
    m1[pk][#m1[pk] + 1] = e
    local m2 = pole_cand_edges[e.p2.unit_number]
    if not m2[pk] then m2[pk] = {} end
    m2[pk][#m2[pk] + 1] = e
  end

  -- Phase 3: MST + densification
  -- Step A: Kruskal MST over internal candidate edges (slot caps respected).
  -- Step B: cascading 1-hop axis-aware propagation. A (dist, axis) pair
  --         travels along an MST edge from v to u only if u has a candidate
  --         at the SAME (dist, axis). On acceptance u adds all of its
  --         candidate edges at that pair and propagation continues from u.
  --         Uses the post-A pair set as the propagation source.
  -- Step C: per-node distance densification — for each candidate edge at
  --         distance d, add it if either endpoint already has a wire at d.
  --         Distance-only check; cascades within a distance group.
  local parent = {}
  for i = 1, n do parent[poles[i].unit_number] = poles[i].unit_number end
  -- Union-find with path compression: each find() flattens the chain so
  -- subsequent calls are O(α(n)) ~ O(1) amortized.
  local function find(u)
    local root = u
    while parent[root] ~= root do root = parent[root] end
    while parent[u] ~= root do
      local pn = parent[u]
      parent[u] = root
      u = pn
    end
    return root
  end

  local node_dists = {}
  local node_dists_pair = {}
  local mst_neighbors = {}
  for i = 1, n do
    local u = poles[i].unit_number
    node_dists[u] = {}
    node_dists_pair[u] = {}
    mst_neighbors[u] = {}
  end

  local m = #edges
  local connected = 0

  p_phase2.stop()
  local p_step_a = game.create_profiler()

  -- Pure in-memory addition: marks an edge as wanted, updates union-find,
  -- slots, and distance/pair sets. No API calls — those happen in the diff
  -- phase below.
  local function try_connect(e)
    if e.added then return false end
    if not (e.p1.valid and e.p2.valid) then return false end
    local u1, u2 = e.p1.unit_number, e.p2.unit_number
    if slot[u1] <= 0 or slot[u2] <= 0 then return false end
    local r1, r2 = find(u1), find(u2)
    if r1 ~= r2 then
      parent[r1] = r2
      mst_neighbors[u1][u2] = true
      mst_neighbors[u2][u1] = true
    end
    slot[u1] = slot[u1] - 1
    slot[u2] = slot[u2] - 1
    node_dists[u1][e.dist_key] = true
    node_dists[u2][e.dist_key] = true
    node_dists_pair[u1][e.pair_key] = true
    node_dists_pair[u2][e.pair_key] = true
    e.added = true
    connected = connected + 1
    return true
  end

  local function sort_group(group)
    table.sort(group, function(a, b)
      local sa1, sa2 = slot[a.p1.unit_number], slot[a.p2.unit_number]
      local sb1, sb2 = slot[b.p1.unit_number], slot[b.p2.unit_number]
      local sa = sa1 < sa2 and sa1 or sa2
      local sb = sb1 < sb2 and sb1 or sb2
      if sa ~= sb then return sa < sb end
      local au1, au2 = a.p1.unit_number, a.p2.unit_number
      if au1 > au2 then au1, au2 = au2, au1 end
      local bu1, bu2 = b.p1.unit_number, b.p2.unit_number
      if bu1 > bu2 then bu1, bu2 = bu2, bu1 end
      if au1 ~= bu1 then return au1 < bu1 end
      return au2 < bu2
    end)
  end

  -- Step A: Kruskal MST over all distance groups
  local i = 1
  while i <= m do
    local j = i
    local dk = edges[i].dist_key
    while j <= m and edges[j].dist_key == dk do j = j + 1 end

    local group = {}
    for k = i, j - 1 do group[#group + 1] = edges[k] end
    sort_group(group)

    for _, e in ipairs(group) do
      if find(e.p1.unit_number) ~= find(e.p2.unit_number) then
        try_connect(e)
      end
    end
    i = j
  end

  p_step_a.stop()
  local mst_added = connected
  local p_step_b = game.create_profiler()

  -- Build sorted MST-neighbor lists for deterministic Step B iteration.
  local mst_neighbors_sorted = {}
  for i = 1, n do
    local u = poles[i].unit_number
    local arr = {}
    for v in pairs(mst_neighbors[u]) do arr[#arr + 1] = v end
    table.sort(arr)
    mst_neighbors_sorted[u] = arr
  end

  -- Step B: cascading 1-hop axis-aware propagation (BFS-based).
  -- Each pole only re-propagates when its pair set changes. Determinism is
  -- preserved by FIFO queue + sorted pair_keys / neighbor lists.
  do
    local in_queue = {}
    local queue = {}
    local qhead = 1
    local function enqueue(u)
      if not in_queue[u] then
        in_queue[u] = true
        queue[#queue + 1] = u
      end
    end

    -- Seed: every pole touched by the MST (deterministic by pole index).
    for i = 1, n do
      local u = poles[i].unit_number
      if next(node_dists_pair[u]) then enqueue(u) end
    end

    while qhead <= #queue do
      local u = queue[qhead]
      qhead = qhead + 1
      in_queue[u] = nil

      local u_pairs = {}
      for pk in pairs(node_dists_pair[u]) do u_pairs[#u_pairs + 1] = pk end
      table.sort(u_pairs)

      for _, v in ipairs(mst_neighbors_sorted[u]) do
        local cands_v = pole_cand_pairs[v]
        local edges_at_v = pole_cand_edges[v]
        local v_set = node_dists_pair[v]

        for _, pk in ipairs(u_pairs) do
          if cands_v[pk] and not v_set[pk] then
            v_set[pk] = true
            local list = edges_at_v[pk]
            if list then
              for _, e in ipairs(list) do
                if not e.added then
                  if try_connect(e) then
                    local other_u = (e.p1.unit_number == v)
                      and e.p2.unit_number or e.p1.unit_number
                    enqueue(other_u)
                  end
                end
              end
            end
            enqueue(v)
          end
        end
      end
    end
  end

  p_step_b.stop()
  local cascade_added = connected - mst_added
  local p_step_c = game.create_profiler()

  -- Step C: per-node distance densification.
  i = 1
  while i <= m do
    local j = i
    local dk = edges[i].dist_key
    while j <= m and edges[j].dist_key == dk do j = j + 1 end

    local group = {}
    for k = i, j - 1 do
      if not edges[k].added then group[#group + 1] = edges[k] end
    end
    sort_group(group)

    local changed = true
    while changed do
      changed = false
      for _, e in ipairs(group) do
        if not e.added then
          if node_dists[e.p1.unit_number][dk] or node_dists[e.p2.unit_number][dk] then
            if try_connect(e) then changed = true end
          end
        end
      end
    end
    i = j
  end

  p_step_c.stop()
  local densify_added = connected - mst_added - cascade_added
  local p_diff = game.create_profiler()

  -- Phase 4: diff-apply. Apply only the difference between current internal
  -- edges and the wanted set. Edges in both stay untouched (no API cost).
  local wanted = {}
  for _, e in ipairs(edges) do
    if e.added then
      wanted[pair_key(e.p1.unit_number, e.p2.unit_number)] = e
    end
  end

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

  for k, e in pairs(wanted) do
    if not internal_pairs[k] then
      if e.p1.valid and e.p2.valid then
        local c1 = e.p1.get_wire_connector(POLE_COPPER, true)
        local c2 = e.p2.get_wire_connector(POLE_COPPER, true)
        if c1 and c2 then
          c1.connect_to(c2, false, WIRE_ORIGIN)
          connect_calls = connect_calls + 1
        end
      end
    end
  end

  p_diff.stop()
  p_total.stop()

  notify(player, "wire-fix.done", n, connected)

  if config.dev_mode then
    local internal_count = 0
    for _ in pairs(internal_pairs) do internal_count = internal_count + 1 end

    -- Per-pole internal degree in the wanted set
    local degree = {}
    for i = 1, n do degree[poles[i].unit_number] = 0 end
    for k = 1, m do
      local e = edges[k]
      if e.added then
        degree[e.p1.unit_number] = degree[e.p1.unit_number] + 1
        degree[e.p2.unit_number] = degree[e.p2.unit_number] + 1
      end
    end

    -- "Isolated" = pole that COULD have had an internal connection (initial
    -- slot > 0 and reachable candidate exists) but ended up with 0 added.
    -- "Capped" = pole already saturated by external connections (initial
    -- slot == 0); the algorithm correctly skipped it, not a loss.
    local isolated, capped = 0, 0
    for i = 1, n do
      local u = poles[i].unit_number
      if initial_slot[u] == 0 then
        capped = capped + 1
      elseif degree[u] == 0 and next(pole_cand_pairs[u]) then
        isolated = isolated + 1
      end
    end

    -- Average / max degree
    local deg_sum, deg_max = 0, 0
    for i = 1, n do
      local d = degree[poles[i].unit_number]
      deg_sum = deg_sum + d
      if d > deg_max then deg_max = d end
    end

    -- Connected components in the wanted-graph (union-find roots after MST).
    -- Larger than the candidate-graph's natural component count = slot-blocked.
    local comp_roots = {}
    for i = 1, n do comp_roots[find(poles[i].unit_number)] = true end
    local comp_count = 0
    for _ in pairs(comp_roots) do comp_count = comp_count + 1 end

    -- Natural component count of the candidate graph (best possible).
    local nat_parent = {}
    for i = 1, n do nat_parent[poles[i].unit_number] = poles[i].unit_number end
    local function nat_find(u)
      local r = u
      while nat_parent[r] ~= r do r = nat_parent[r] end
      while nat_parent[u] ~= r do
        local pn = nat_parent[u]; nat_parent[u] = r; u = pn
      end
      return r
    end
    for k = 1, m do
      local e = edges[k]
      local r1 = nat_find(e.p1.unit_number)
      local r2 = nat_find(e.p2.unit_number)
      if r1 ~= r2 then nat_parent[r1] = r2 end
    end
    local nat_roots = {}
    for i = 1, n do nat_roots[nat_find(poles[i].unit_number)] = true end
    local nat_comp_count = 0
    for _ in pairs(nat_roots) do nat_comp_count = nat_comp_count + 1 end

    player.print({"", "[Wire Fix] total: ", p_total})
    player.print(string.format(
      "  poles=%d  internal=%d  candidates=%d  added: mst=%d cascade=%d densify=%d (total=%d)  isolated=%d  capped=%d  comps=%d/nat=%d  deg avg=%.2f max=%d  api: disconnect=%d connect=%d",
      n, internal_count, m,
      mst_added, cascade_added, densify_added, connected,
      isolated, capped, comp_count, nat_comp_count, deg_sum / n, deg_max,
      disconnect_calls, connect_calls))
    player.print({"", "  phase1=", p_phase1, "  phase2=", p_phase2,
      "  stepA=", p_step_a, "  stepB=", p_step_b, "  stepC=", p_step_c, "  diff=", p_diff})
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
