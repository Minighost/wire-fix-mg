-- v21 phase 1 prototype — baseline snapshot: 15ca79f (working tree of bc9c898).
--
-- Paradigm shift vs v20: replace midpoint-matching enumeration with
-- per-cell anchor + direct partner lookup. v20's hot loop is O(group²)
-- iteration over midpoint pair-pairs; on the 5493-pole layout this hits
-- 130M pair_checks. v21 phase 1 is O(1) per axis-rect candidate via
-- column-y partner lookup.
--
-- Phase 1 covers ONLY axis-aligned rectangles. Phases 2–4 (slanted,
-- vertical-anchor, rotated-axis) are not implemented yet — output is
-- expected to be incomplete vs v20 for non-axis-rect-dominant layouts.
-- MST closure picks up the remaining missing edges.
--
-- Algorithm (per scale s ∈ {32, 64, 128, 256}):
--   For each cell (s × s) of the layout grid:
--     For each alive pole P in the cell, ordered (y, x) ascending:
--       Walk P's horizontal chain; for each Q east of P on that chain
--       with x_Q − x_P ≤ s:
--         Find smallest y2 > y_P such that (x_P, y2) and (x_Q, y2) are
--         both alive poles AND y2 − y_P ≤ s (= smallest axis-rect anchored
--         at the (P, Q) bottom edge).
--         If found: validate the 4 chains (P→Q, Q→Q', Q'→P', P'→P), then
--         attempt SAT-accept + expand + mark_dead like v20.
--
-- Larger scales catch axis-rects whose bbox > previous scale; the all_seen
-- + SAT cumulative state prevents re-acceptance of what smaller scales
-- already produced.
--
-- Public API (same contract as algo-v20):
--   M.compute(points, reaches, opts) -> edges, parls, iso_tris, stats

local M = {}

local EPS = 1e-9
local EPS_GEOM = 1e-6
local EPS_DIAG = 1e-6
local ROUND_SCALE = 10000

local SCALES = { 32, 64, 128, 256 }
local SQRT3_2 = 0.86602540378443864676  -- √3/2, phase 2 lens β bound coefficient
local mfloor = math.floor
local mabs = math.abs
local msqrt = math.sqrt

local function vec_round_xy(x, y)
  local rx
  if x >= 0 then rx = mfloor(x * ROUND_SCALE + 0.5)
  else rx = -mfloor(-x * ROUND_SCALE + 0.5)
  end
  local ry
  if y >= 0 then ry = mfloor(y * ROUND_SCALE + 0.5)
  else ry = -mfloor(-y * ROUND_SCALE + 0.5)
  end
  return rx, ry
end

local function cross_2d(ax, ay, bx, by)
  return ax * by - ay * bx
end

local function dist(ax, ay, bx, by)
  local dx = ax - bx
  local dy = ay - by
  return msqrt(dx * dx + dy * dy)
end

local function edge_key(i, j)
  if i < j then return i .. "_" .. j
  else return j .. "_" .. i
  end
end

local function tri_key(a, b, c)
  local s = { a, b, c }
  table.sort(s)
  return s[1] .. "_" .. s[2] .. "_" .. s[3]
end

local function quad_key(corners)
  local s = { corners[1], corners[2], corners[3], corners[4] }
  table.sort(s)
  return s[1] .. "_" .. s[2] .. "_" .. s[3] .. "_" .. s[4]
end

local function segs_intersect(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y)
  if (p1x == p3x and p1y == p3y) or (p1x == p4x and p1y == p4y)
     or (p2x == p3x and p2y == p3y) or (p2x == p4x and p2y == p4y) then
    return false
  end
  local d1 = (p4x - p3x) * (p1y - p3y) - (p4y - p3y) * (p1x - p3x)
  local d2 = (p4x - p3x) * (p2y - p3y) - (p4y - p3y) * (p2x - p3x)
  local d3 = (p2x - p1x) * (p3y - p1y) - (p2y - p1y) * (p3x - p1x)
  local d4 = (p2x - p1x) * (p4y - p1y) - (p2y - p1y) * (p4x - p1x)
  if ((d1 > EPS and d2 < -EPS) or (d1 < -EPS and d2 > EPS)) and
     ((d3 > EPS and d4 < -EPS) or (d3 < -EPS and d4 > EPS)) then
    return true
  end
  return false
end

function M.compute(points, reaches, opts)
  local n = #points
  local stats = {}
  if n < 2 then return {}, {}, {}, stats end

  local pxs, pys = {}, {}
  for i = 1, n do
    pxs[i] = points[i].x
    pys[i] = points[i].y
  end

  stats.t_setup = game.create_profiler()

  local pos_to_idx = {}
  for i = 1, n do
    local rx, ry = vec_round_xy(points[i].x, points[i].y)
    local row = pos_to_idx[rx]
    if not row then row = {}; pos_to_idx[rx] = row end
    row[ry] = i
  end

  local function pair_reach(i, j)
    local ri, rj = reaches[i], reaches[j]
    return ri < rj and ri or rj
  end

  local max_reach = 0
  for i = 1, n do
    if reaches[i] > max_reach then max_reach = reaches[i] end
  end
  local cell_size = max_reach
  local point_bucket = {}
  for i = 1, n do
    local cx = mfloor(points[i].x / cell_size)
    local cy = mfloor(points[i].y / cell_size)
    local row = point_bucket[cx]
    if not row then row = {}; point_bucket[cx] = row end
    local cell = row[cy]
    if not cell then cell = {}; row[cy] = cell end
    cell[#cell + 1] = i
  end

  local pixs, piys = {}, {}
  for i = 1, n do
    local rx, ry = vec_round_xy(points[i].x, points[i].y)
    pixs[i] = rx
    piys[i] = ry
  end

  -- v21: column index. cols_by_rx[rx] = sorted-by-ry list of pole indices.
  -- col_pos[i] = position of pole i in its column. Used by phase 1's y2
  -- partner sweep to step "next pole above P in this column" in O(1).
  local cols_by_rx = {}
  for i = 1, n do
    local rx = pixs[i]
    local col = cols_by_rx[rx]
    if not col then col = {}; cols_by_rx[rx] = col end
    col[#col + 1] = i
  end
  for _, col in pairs(cols_by_rx) do
    table.sort(col, function(a, b) return piys[a] < piys[b] end)
  end
  local col_pos = {}
  for _, col in pairs(cols_by_rx) do
    for k = 1, #col do col_pos[col[k]] = k end
  end

  -- Lines bucket (same construction as v20). Phase 1 uses only the
  -- horizontal slice (ndy = 0) for chain Q-walk, but make_chain's general
  -- collinearity check needs the full bucket.
  local lines = {}
  for i = 1, n do
    local pix_i = pixs[i]
    local piy_i = piys[i]
    local px_i = points[i].x
    local py_i = points[i].y
    local ri = reaches[i]
    local cx_min = mfloor((px_i - ri) / cell_size)
    local cx_max = mfloor((px_i + ri) / cell_size)
    local cy_min = mfloor((py_i - ri) / cell_size)
    local cy_max = mfloor((py_i + ri) / cell_size)
    for cx = cx_min, cx_max do
      local row = point_bucket[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for ki = 1, #cell do
              local j = cell[ki]
              if j > i then
                local dxf = points[j].x - px_i
                local dyf = points[j].y - py_i
                local d2 = dxf * dxf + dyf * dyf
                local rj = reaches[j]
                local r = ri < rj and ri or rj
                if d2 <= r * r then
                  local idx = pixs[j] - pix_i
                  local idy = piys[j] - piy_i
                  local a = idx >= 0 and idx or -idx
                  local b = idy >= 0 and idy or -idy
                  local g
                  if a == 0 then g = b
                  elseif b == 0 then g = a
                  else
                    while b ~= 0 do a, b = b, a % b end
                    g = a
                  end
                  local ndx = idx / g
                  local ndy = idy / g
                  if ndx < 0 or (ndx == 0 and ndy < 0) then
                    ndx = -ndx; ndy = -ndy
                  end
                  local c = ndy * pix_i - ndx * piy_i
                  local row_a = lines[ndx]
                  if not row_a then row_a = {}; lines[ndx] = row_a end
                  local row_b = row_a[ndy]
                  if not row_b then row_b = {}; row_a[ndy] = row_b end
                  local line = row_b[c]
                  if not line then
                    line = { poles = {}, ndx = ndx, ndy = ndy,
                             anchor_x = pix_i, anchor_y = piy_i }
                    row_b[c] = line
                  end
                  line.poles[i] = true
                  line.poles[j] = true
                end
              end
            end
          end
        end
      end
    end
  end

  for _, row_a in pairs(lines) do
    for _, row_b in pairs(row_a) do
      for _, line in pairs(row_b) do
        local sorted = {}
        for p in pairs(line.poles) do sorted[#sorted + 1] = p end
        local ndx = line.ndx
        local ndy = line.ndy
        local ax = line.anchor_x
        local ay = line.anchor_y
        table.sort(sorted, function(aa, bb)
          return (pixs[aa] - ax) * ndx + (piys[aa] - ay) * ndy
               < (pixs[bb] - ax) * ndx + (piys[bb] - ay) * ndy
        end)
        line.poles = sorted
        local pos_in_line = {}
        for ki = 1, #sorted do pos_in_line[sorted[ki]] = ki end
        line.pos_in_line = pos_in_line
      end
    end
  end

  -- v21: for each pole, the horizontal line (ndx=1, ndy=0) it belongs to.
  -- nil if the pole has no reach-near horizontal neighbor (then it can't
  -- be the bottom-left anchor of any axis-rect).
  local horiz_line_of = {}
  do
    local row_a = lines[1]
    local row_b = row_a and row_a[0]
    if row_b then
      for _, line in pairs(row_b) do
        for ki = 1, #line.poles do
          horiz_line_of[line.poles[ki]] = line
        end
      end
    end
  end

  -- Phase 3 (xy-mirror of phase 2): vertical-edge anchor + lens of slanted
  -- third corner. vert_line_of[p] = ndx=0, ndy=1 line (canonicalized).
  local vert_line_of = {}
  do
    local row_a = lines[0]
    local row_b = row_a and row_a[1]
    if row_b then
      for _, line in pairs(row_b) do
        for ki = 1, #line.poles do
          vert_line_of[line.poles[ki]] = line
        end
      end
    end
  end

  -- Phase 4 (rotated frame): every non-axis-aligned line direction acts as
  -- the "horizontal" anchor of a phase-2-style lens search. We flatten such
  -- lines into a single list once so the per-scale loop just iterates this
  -- array instead of re-traversing the nested lines table.
  local diag_lines = {}
  for ndx, row_a in pairs(lines) do
    for ndy, row_b in pairs(row_a) do
      if not ((ndx == 1 and ndy == 0) or (ndx == 0 and ndy == 1)) then
        for _, line in pairs(row_b) do
          if #line.poles >= 2 then
            diag_lines[#diag_lines + 1] = line
          end
        end
      end
    end
  end

  stats.t_setup.stop()

  local pos_seg_count = 0
  local pos_seg_ks = {}

  local cache_hits = 0
  local cache_misses = 0

  local reaches_sq = {}
  for i = 1, n do reaches_sq[i] = reaches[i] * reaches[i] end
  local function pair_reach_sq(i, j)
    local ri, rj = reaches_sq[i], reaches_sq[j]
    return ri < rj and ri or rj
  end

  local function points_on_segment_strict(i, j)
    pos_seg_count = 0
    local idx = pixs[j] - pixs[i]
    local idy = piys[j] - piys[i]
    if idx == 0 and idy == 0 then return 0 end
    local a = idx >= 0 and idx or -idx
    local b = idy >= 0 and idy or -idy
    local g
    if a == 0 then g = b
    elseif b == 0 then g = a
    else
      while b ~= 0 do a, b = b, a % b end
      g = a
    end
    local ndx = idx / g
    local ndy = idy / g
    if ndx < 0 or (ndx == 0 and ndy < 0) then
      ndx = -ndx; ndy = -ndy
    end
    local c = ndy * pixs[i] - ndx * piys[i]
    local row_a = lines[ndx]
    if not row_a then return 0 end
    local row_b = row_a[ndy]
    if not row_b then return 0 end
    local line = row_b[c]
    if not line then return 0 end
    local pos_in_line = line.pos_in_line
    local p_i = pos_in_line[i]
    local p_j = pos_in_line[j]
    if not p_i or not p_j then return 0 end
    local poles = line.poles
    if p_i < p_j then
      for k = p_i + 1, p_j - 1 do
        pos_seg_count = pos_seg_count + 1
        pos_seg_ks[pos_seg_count] = poles[k]
      end
    else
      for k = p_i - 1, p_j + 1, -1 do
        pos_seg_count = pos_seg_count + 1
        pos_seg_ks[pos_seg_count] = poles[k]
      end
    end
    return pos_seg_count
  end

  local chain_cache = {}

  local function _make_chain_inner(lo, hi)
    local row = chain_cache[lo]
    if row then
      local cached = row[hi]
      if cached ~= nil then
        cache_hits = cache_hits + 1
        if cached == false then return nil end
        return cached
      end
    else
      row = {}; chain_cache[lo] = row
    end
    cache_misses = cache_misses + 1

    local count = points_on_segment_strict(lo, hi)
    local chain = { lo }
    for ki = 1, count do chain[#chain + 1] = pos_seg_ks[ki] end
    chain[#chain + 1] = hi

    for ci = 1, #chain - 1 do
      local a = chain[ci]
      local b = chain[ci + 1]
      local segdx = pxs[b] - pxs[a]
      local segdy = pys[b] - pys[a]
      if segdx * segdx + segdy * segdy > pair_reach_sq(a, b) then
        row[hi] = false
        return nil
      end
    end

    row[hi] = chain
    return chain
  end

  local function make_chain(i, j)
    local lo, hi = i, j
    if lo > hi then lo, hi = hi, lo end
    return _make_chain_inner(lo, hi)
  end

  -- v21: SAT (quads_overlap) and parl_contains_parl removed; conflict
  -- check is now chain-segment intersection vs edge_bucket — see
  -- parl_chains_intersect_existing further below.

  local accepted_iso_refl_tris = {}
  local seen_iso = {}
  local iso_pass = 0
  local iso_refl_emit = 0

  local primary_parls = {}
  local expansion_parls = {}
  local all_seen = {}
  local chain_calls = 0
  local chain_success = 0
  local pair_checks = 0    -- v21: count of (P, Q) tries (anchor walk steps)
  local angle_pass = 0     -- v21: not used by phase 1, kept for compat

  local function try_reflect_iso(side_a, side_b, opp)
    local mx = pxs[side_a] + pxs[side_b] - pxs[opp]
    local my = pys[side_a] + pys[side_b] - pys[opp]
    local rx, ry = vec_round_xy(mx, my)
    local row = pos_to_idx[rx]
    local X = row and row[ry]
    if not X or X == side_a or X == side_b or X == opp then return end
    local px_X, py_X = pxs[X], pys[X]
    local dax = pxs[side_a] - px_X
    local day = pys[side_a] - py_X
    local dbx = pxs[side_b] - px_X
    local dby = pys[side_b] - py_X
    if dax * dax + day * day > pair_reach_sq(side_a, X) then return end
    if dbx * dbx + dby * dby > pair_reach_sq(side_b, X) then return end
    local dabx = pxs[side_a] - pxs[side_b]
    local daby = pys[side_a] - pys[side_b]
    if dabx * dabx + daby * daby > pair_reach_sq(side_a, side_b) then return end
    local kr = tri_key(side_a, side_b, X)
    if seen_iso[kr] then return end
    seen_iso[kr] = true
    accepted_iso_refl_tris[#accepted_iso_refl_tris + 1] = { side_a, side_b, X }
    iso_refl_emit = iso_refl_emit + 1
  end

  local function try_iso_emit(P)
    local cc1, cc2, cc3, cc4 = P.corners[1], P.corners[2], P.corners[3], P.corners[4]
    local L1_dx = pxs[cc2] - pxs[cc1]
    local L1_dy = pys[cc2] - pys[cc1]
    local L2_dx = pxs[cc3] - pxs[cc2]
    local L2_dy = pys[cc3] - pys[cc2]
    local L1_sq = L1_dx * L1_dx + L1_dy * L1_dy
    local L2_sq = L2_dx * L2_dx + L2_dy * L2_dy
    local max_side_sq = L1_sq > L2_sq and L1_sq or L2_sq
    local d13x = pxs[cc3] - pxs[cc1]
    local d13y = pys[cc3] - pys[cc1]
    local d24x = pxs[cc4] - pxs[cc2]
    local d24y = pys[cc4] - pys[cc2]
    local d13_sq = d13x * d13x + d13y * d13y
    local d24_sq = d24x * d24x + d24y * d24y
    local short_diag_sq, short_a
    if d13_sq < d24_sq then short_diag_sq, short_a = d13_sq, cc1
    else short_diag_sq, short_a = d24_sq, cc2 end
    local diff = short_diag_sq - max_side_sq
    if diff < -EPS_DIAG or diff > EPS_DIAG then return end
    iso_pass = iso_pass + 1
    local long_is_L1 = L1_sq >= L2_sq
    if short_a == cc1 then
      if long_is_L1 then
        try_reflect_iso(cc1, cc2, cc3)
        try_reflect_iso(cc3, cc4, cc1)
      else
        try_reflect_iso(cc2, cc3, cc1)
        try_reflect_iso(cc4, cc1, cc3)
      end
    else
      if long_is_L1 then
        try_reflect_iso(cc1, cc2, cc4)
        try_reflect_iso(cc3, cc4, cc2)
      else
        try_reflect_iso(cc4, cc1, cc2)
        try_reflect_iso(cc2, cc3, cc4)
      end
    end
  end

  local pole_dead = {}
  local dead_count = 0

  local function point_in_parl_strict(p_idx, P)
    local px = pxs[p_idx]
    local py = pys[p_idx]
    local c = P.corners
    for i = 1, 4 do
      local a = c[i]
      local b = c[(i % 4) + 1]
      local ax, ay = pxs[a], pys[a]
      local bx, by = pxs[b], pys[b]
      local cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
      if cross <= EPS_GEOM then return false end
    end
    return true
  end

  local function mark_dead_in_parl(P)
    local cx_min = mfloor(P.bxmin / cell_size)
    local cx_max = mfloor(P.bxmax / cell_size)
    local cy_min = mfloor(P.bymin / cell_size)
    local cy_max = mfloor(P.bymax / cell_size)
    for cx = cx_min, cx_max do
      local row = point_bucket[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for ki = 1, #cell do
              local p = cell[ki]
              if not pole_dead[p] and point_in_parl_strict(p, P) then
                pole_dead[p] = true
                dead_count = dead_count + 1
              end
            end
          end
        end
      end
    end
  end

  -- v21 phase 1: build a candidate axis-rect from the (P, Q) bottom edge.
  -- Returns a candidate table (corners CCW, chains, area, bbox) or nil.
  -- Caller has already ensured P, Q are alive and on the same horizontal line.
  local function build_axis_rect_candidate(P, Q, max_dy)
    local rx_P = pixs[P]
    local ry_P = piys[P]
    local rx_Q = pixs[Q]

    -- Find smallest y2 (= ry above ry_P) such that (rx_P, y2) and (rx_Q, y2)
    -- are both alive poles.
    local col_P = cols_by_rx[rx_P]
    if not col_P then return nil end
    local pos_P = col_pos[P]
    local row_at_Q = pos_to_idx[rx_Q]
    if not row_at_Q then return nil end

    local Pp_idx, Qp_idx, ry_y2
    for k = pos_P + 1, #col_P do
      local cand = col_P[k]
      if pole_dead[cand] then
        -- dead poles inside the column don't break the search — just skip.
      else
        local cand_ry = piys[cand]
        if (cand_ry - ry_P) > max_dy * ROUND_SCALE then break end
        local q2 = row_at_Q[cand_ry]
        if q2 and not pole_dead[q2] then
          Pp_idx = cand
          Qp_idx = q2
          ry_y2 = cand_ry
          break
        end
      end
    end
    if not Pp_idx then return nil end

    -- CCW corners with cross_2d(v_a, v_b) > 0:
    --   v_a = P→Q  = (L, 0)  with L > 0
    --   v_b = P→P' = (0, dy) with dy > 0
    --   cross = L · dy > 0
    local cc1, cc2, cc3, cc4 = P, Q, Qp_idx, Pp_idx

    chain_calls = chain_calls + 1
    local ch1 = make_chain(cc1, cc2); if ch1 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch2 = make_chain(cc2, cc3); if ch2 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch3 = make_chain(cc3, cc4); if ch3 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch4 = make_chain(cc4, cc1); if ch4 == nil then return nil end
    chain_success = chain_success + 1

    local x1, x2, x3, x4 = pxs[cc1], pxs[cc2], pxs[cc3], pxs[cc4]
    local y1, y2, y3, y4 = pys[cc1], pys[cc2], pys[cc3], pys[cc4]
    local va_x = x2 - x1
    local va_y = y2 - y1
    local vb_x = x4 - x1
    local vb_y = y4 - y1
    local bxmin, bxmax = x1, x1
    if x2 < bxmin then bxmin = x2 end; if x2 > bxmax then bxmax = x2 end
    if x3 < bxmin then bxmin = x3 end; if x3 > bxmax then bxmax = x3 end
    if x4 < bxmin then bxmin = x4 end; if x4 > bxmax then bxmax = x4 end
    local bymin, bymax = y1, y1
    if y2 < bymin then bymin = y2 end; if y2 > bymax then bymax = y2 end
    if y3 < bymin then bymin = y3 end; if y3 > bymax then bymax = y3 end
    if y4 < bymin then bymin = y4 end; if y4 > bymax then bymax = y4 end
    return {
      corners = { cc1, cc2, cc3, cc4 },
      chains = { ch1, ch2, ch3, ch4 },
      area = mabs(cross_2d(va_x, va_y, vb_x, vb_y)),
      bxmin = bxmin, bxmax = bxmax,
      bymin = bymin, bymax = bymax,
    }
  end

  -- v21 phase 2: build a candidate slanted parl from a horizontal bottom
  -- (P, Q) and a third corner X. The 4th corner Y sits at (X.x + L, X.y);
  -- direct lookup via pos_to_idx. Caller has already lens-validated X.
  local function build_phase2_candidate(P, Q, X)
    local rx_Y = pixs[X] + (pixs[Q] - pixs[P])
    local ry_Y = piys[X]
    local row = pos_to_idx[rx_Y]
    local Y = row and row[ry_Y]
    if not Y or pole_dead[Y] or Y == P or Y == Q or Y == X then return nil end

    -- CCW: P (BL) → Q (BR) → Y (TR) → X (TL).
    --   v_a = Q-P = (L, 0)        with L > 0
    --   v_b = X-P = (αx, βy)      with βy > 0  (lens guarantees βy ≥ L·√3/2)
    --   cross_2d(v_a, v_b) = L·βy > 0  ✓
    local cc1, cc2, cc3, cc4 = P, Q, Y, X

    chain_calls = chain_calls + 1
    local ch1 = make_chain(cc1, cc2); if ch1 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch2 = make_chain(cc2, cc3); if ch2 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch3 = make_chain(cc3, cc4); if ch3 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch4 = make_chain(cc4, cc1); if ch4 == nil then return nil end
    chain_success = chain_success + 1

    local x1, x2, x3, x4 = pxs[cc1], pxs[cc2], pxs[cc3], pxs[cc4]
    local y1, y2, y3, y4 = pys[cc1], pys[cc2], pys[cc3], pys[cc4]
    local va_x = x2 - x1
    local va_y = y2 - y1
    local vb_x = x4 - x1
    local vb_y = y4 - y1
    local bxmin, bxmax = x1, x1
    if x2 < bxmin then bxmin = x2 end; if x2 > bxmax then bxmax = x2 end
    if x3 < bxmin then bxmin = x3 end; if x3 > bxmax then bxmax = x3 end
    if x4 < bxmin then bxmin = x4 end; if x4 > bxmax then bxmax = x4 end
    local bymin, bymax = y1, y1
    if y2 < bymin then bymin = y2 end; if y2 > bymax then bymax = y2 end
    if y3 < bymin then bymin = y3 end; if y3 > bymax then bymax = y3 end
    if y4 < bymin then bymin = y4 end; if y4 > bymax then bymax = y4 end
    return {
      corners = { cc1, cc2, cc3, cc4 },
      chains = { ch1, ch2, ch3, ch4 },
      area = mabs(cross_2d(va_x, va_y, vb_x, vb_y)),
      bxmin = bxmin, bxmax = bxmax,
      bymin = bymin, bymax = bymax,
    }
  end

  -- v21 phase 3: build a candidate slanted parl from a vertical left edge
  -- (P, Q) and a third corner X. The 4th corner Y sits at (X.x, X.y + L');
  -- direct lookup via pos_to_idx. Caller has already lens-validated X.
  local function build_phase3_candidate(P, Q, X)
    local rx_Y = pixs[X]
    local ry_Y = piys[X] + (piys[Q] - piys[P])
    local row = pos_to_idx[rx_Y]
    local Y = row and row[ry_Y]
    if not Y or pole_dead[Y] or Y == P or Y == Q or Y == X then return nil end

    -- CCW: P (TL) → X (TR) → Y (BR) → Q (BL).
    --   v_a = X-P = (αx, βy)      with αx > 0  (lens guarantees αx ≥ L'·√3/2)
    --   v_b = Q-P = (0, L')       with L' > 0
    --   cross_2d(v_a, v_b) = αx · L' > 0  ✓
    local cc1, cc2, cc3, cc4 = P, X, Y, Q

    chain_calls = chain_calls + 1
    local ch1 = make_chain(cc1, cc2); if ch1 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch2 = make_chain(cc2, cc3); if ch2 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch3 = make_chain(cc3, cc4); if ch3 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch4 = make_chain(cc4, cc1); if ch4 == nil then return nil end
    chain_success = chain_success + 1

    local x1, x2, x3, x4 = pxs[cc1], pxs[cc2], pxs[cc3], pxs[cc4]
    local y1, y2, y3, y4 = pys[cc1], pys[cc2], pys[cc3], pys[cc4]
    local va_x = x2 - x1
    local va_y = y2 - y1
    local vb_x = x4 - x1
    local vb_y = y4 - y1
    local bxmin, bxmax = x1, x1
    if x2 < bxmin then bxmin = x2 end; if x2 > bxmax then bxmax = x2 end
    if x3 < bxmin then bxmin = x3 end; if x3 > bxmax then bxmax = x3 end
    if x4 < bxmin then bxmin = x4 end; if x4 > bxmax then bxmax = x4 end
    local bymin, bymax = y1, y1
    if y2 < bymin then bymin = y2 end; if y2 > bymax then bymax = y2 end
    if y3 < bymin then bymin = y3 end; if y3 > bymax then bymax = y3 end
    if y4 < bymin then bymin = y4 end; if y4 > bymax then bymax = y4 end
    return {
      corners = { cc1, cc2, cc3, cc4 },
      chains = { ch1, ch2, ch3, ch4 },
      area = mabs(cross_2d(va_x, va_y, vb_x, vb_y)),
      bxmin = bxmin, bxmax = bxmax,
      bymin = bymin, bymax = bymax,
    }
  end

  -- v21 phase 4: build a candidate slanted parl from a non-axis bottom edge
  -- (P, Q) and a third corner X. The 4th corner Y sits at X + (Q − P) (i.e.
  -- the same direction-d shift Q underwent from P), looked up via pos_to_idx.
  -- Caller has already lens-validated X in the rotated (α, β) frame.
  local function build_phase4_candidate(P, Q, X)
    local rx_Y = pixs[X] + (pixs[Q] - pixs[P])
    local ry_Y = piys[X] + (piys[Q] - piys[P])
    local row = pos_to_idx[rx_Y]
    local Y = row and row[ry_Y]
    if not Y or pole_dead[Y] or Y == P or Y == Q or Y == X then return nil end

    -- CCW: P → Q → Y → X with v_a = Q-P (along d), v_b = X-P (= α·d + β·d_perp).
    -- cross_2d(v_a, v_b) = L · β > 0 since β > 0 by lens; same sign as phase 2.
    local cc1, cc2, cc3, cc4 = P, Q, Y, X

    chain_calls = chain_calls + 1
    local ch1 = make_chain(cc1, cc2); if ch1 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch2 = make_chain(cc2, cc3); if ch2 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch3 = make_chain(cc3, cc4); if ch3 == nil then return nil end
    chain_success = chain_success + 1
    chain_calls = chain_calls + 1
    local ch4 = make_chain(cc4, cc1); if ch4 == nil then return nil end
    chain_success = chain_success + 1

    local x1, x2, x3, x4 = pxs[cc1], pxs[cc2], pxs[cc3], pxs[cc4]
    local y1, y2, y3, y4 = pys[cc1], pys[cc2], pys[cc3], pys[cc4]
    local va_x = x2 - x1
    local va_y = y2 - y1
    local vb_x = x4 - x1
    local vb_y = y4 - y1
    local bxmin, bxmax = x1, x1
    if x2 < bxmin then bxmin = x2 end; if x2 > bxmax then bxmax = x2 end
    if x3 < bxmin then bxmin = x3 end; if x3 > bxmax then bxmax = x3 end
    if x4 < bxmin then bxmin = x4 end; if x4 > bxmax then bxmax = x4 end
    local bymin, bymax = y1, y1
    if y2 < bymin then bymin = y2 end; if y2 > bymax then bymax = y2 end
    if y3 < bymin then bymin = y3 end; if y3 > bymax then bymax = y3 end
    if y4 < bymin then bymin = y4 end; if y4 > bymax then bymax = y4 end
    return {
      corners = { cc1, cc2, cc3, cc4 },
      chains = { ch1, ch2, ch3, ch4 },
      area = mabs(cross_2d(va_x, va_y, vb_x, vb_y)),
      bxmin = bxmin, bxmax = bxmax,
      bymin = bymin, bymax = bymax,
    }
  end

  -- Phase classification (matches v20). 0 = axis-rect, 1 = horiz-aligned,
  -- 2 = vert-aligned, 3 = diagonal. Drives phase-priority dispatch.
  local function parl_phase(P)
    local p1 = points[P.corners[1]]
    local p2 = points[P.corners[2]]
    local p4 = points[P.corners[4]]
    local v_a_x = p2.x - p1.x
    local v_a_y = p2.y - p1.y
    local v_b_x = p4.x - p1.x
    local v_b_y = p4.y - p1.y
    local eps = 1e-6
    local a_horiz = mabs(v_a_y) < eps
    local a_vert  = mabs(v_a_x) < eps
    local b_horiz = mabs(v_b_y) < eps
    local b_vert  = mabs(v_b_x) < eps
    local has_horiz = a_horiz or b_horiz
    local has_vert  = a_vert  or b_vert
    if has_horiz and has_vert then return 0 end
    if has_horiz then return 1 end
    if has_vert  then return 2 end
    return 3
  end

  -- Counters for v21 phase 2 instrumentation.
  local p2_x_checks = 0     -- # of X poles in lens region considered
  local p2_lens_pass = 0    -- # of X poles that passed precise lens bounds
  local p2_y_lookups = 0    -- # of pos_to_idx Y lookups (lens-pass triggers one)

  -- v21 phase 1+2+3 enumerate. Per scale:
  --   * horizontal anchor (phase 1 axis-rect + phase 2 horiz-aligned slanted)
  --   * vertical anchor (phase 3 vert-aligned slanted; axis-rect already
  --     covered by phase 1's horizontal-anchor sweep, so phase 3 only emits
  --     non-axis-rect slanted parls)
  -- Candidates are filtered to bbox > prev_scale so a larger scale only
  -- emits parls that didn't fit a smaller scale (= step 12 of the v21 spec).
  local function enumerate_for_scale(scale, prev_scale)
    local candidates = {}

    -- Horizontal anchor sweep (phase 1 + 2).
    for P = 1, n do
      if not pole_dead[P] then
        local hline = horiz_line_of[P]
        if hline then
          local poles_in_line = hline.poles
          local pos_in_h = hline.pos_in_line[P]
          local px_P = pxs[P]
          local py_P = pys[P]
          for q_idx = pos_in_h + 1, #poles_in_line do
            local Q = poles_in_line[q_idx]
            if pole_dead[Q] then
              -- horizontal chain breaks at the first dead pole going east.
              break
            end
            local L = pxs[Q] - px_P
            if L > scale then break end
            pair_checks = pair_checks + 1

            -- Phase 1: smallest axis-rect on (P, Q).
            local c1 = build_axis_rect_candidate(P, Q, scale)
            if c1 then
              local w = c1.bxmax - c1.bxmin
              local h = c1.bymax - c1.bymin
              if w > prev_scale or h > prev_scale then
                candidates[#candidates + 1] = c1
              end
            end

            -- Phase 2: lens iterate. Search bbox is [P.x − L/2, P.x + L/2] ×
            -- [P.y + L·√3/2, P.y + scale]. Each X in there with the precise
            -- lens condition is a third-corner candidate; the 4th corner Y is
            -- at (X.x + L, X.y) via pos_to_idx (O(1)).
            local half_L = L * 0.5
            local lens_x_min = px_P - half_L
            local lens_x_max = px_P + half_L
            local lens_y_min = py_P + L * SQRT3_2
            local lens_y_max = py_P + scale
            if lens_y_min <= lens_y_max then
              local cx_min = mfloor(lens_x_min / cell_size)
              local cx_max = mfloor(lens_x_max / cell_size)
              local cy_min = mfloor(lens_y_min / cell_size)
              local cy_max = mfloor(lens_y_max / cell_size)
              for cx = cx_min, cx_max do
                local row_p = point_bucket[cx]
                if row_p then
                  for cy = cy_min, cy_max do
                    local cell = row_p[cy]
                    if cell then
                      for ki = 1, #cell do
                        local X = cell[ki]
                        if not pole_dead[X] and X ~= P and X ~= Q then
                          p2_x_checks = p2_x_checks + 1
                          local x_X = pxs[X]
                          local y_X = pys[X]
                          local off_x = x_X - px_P  -- = α in lens spec
                          local off_y = y_X - py_P  -- = β in lens spec
                          if mabs(off_x) <= half_L + EPS_GEOM
                             and off_y >= L * SQRT3_2 - EPS_GEOM
                             and off_y <= scale + EPS_GEOM then
                            p2_lens_pass = p2_lens_pass + 1
                            p2_y_lookups = p2_y_lookups + 1
                            local c2 = build_phase2_candidate(P, Q, X)
                            if c2 then
                              local w = c2.bxmax - c2.bxmin
                              local h = c2.bymax - c2.bymin
                              if w > prev_scale or h > prev_scale then
                                candidates[#candidates + 1] = c2
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end

    -- Vertical anchor sweep (phase 3 only — slanted with vertical left edge).
    -- xy-mirror of phase 2: chain walk goes south (Q below P, ry > P's ry),
    -- lens region is to the right of P (αx ≥ L'·√3/2) with |βy| ≤ L'/2.
    for P = 1, n do
      if not pole_dead[P] then
        local vline = vert_line_of[P]
        if vline then
          local poles_in_line = vline.poles
          local pos_in_v = vline.pos_in_line[P]
          local px_P = pxs[P]
          local py_P = pys[P]
          for q_idx = pos_in_v + 1, #poles_in_line do
            local Q = poles_in_line[q_idx]
            if pole_dead[Q] then break end
            local Lp = pys[Q] - py_P
            if Lp > scale then break end
            pair_checks = pair_checks + 1

            local half_Lp = Lp * 0.5
            local lens_x_min = px_P + Lp * SQRT3_2
            local lens_x_max = px_P + scale
            local lens_y_min = py_P - half_Lp
            local lens_y_max = py_P + half_Lp
            if lens_x_min <= lens_x_max then
              local cx_min = mfloor(lens_x_min / cell_size)
              local cx_max = mfloor(lens_x_max / cell_size)
              local cy_min = mfloor(lens_y_min / cell_size)
              local cy_max = mfloor(lens_y_max / cell_size)
              for cx = cx_min, cx_max do
                local row_p = point_bucket[cx]
                if row_p then
                  for cy = cy_min, cy_max do
                    local cell = row_p[cy]
                    if cell then
                      for ki = 1, #cell do
                        local X = cell[ki]
                        if not pole_dead[X] and X ~= P and X ~= Q then
                          p2_x_checks = p2_x_checks + 1
                          local x_X = pxs[X]
                          local y_X = pys[X]
                          local off_x = x_X - px_P  -- = α
                          local off_y = y_X - py_P  -- = β
                          if off_x >= Lp * SQRT3_2 - EPS_GEOM
                             and off_x <= scale + EPS_GEOM
                             and mabs(off_y) <= half_Lp + EPS_GEOM then
                            p2_lens_pass = p2_lens_pass + 1
                            p2_y_lookups = p2_y_lookups + 1
                            local c3 = build_phase3_candidate(P, Q, X)
                            if c3 then
                              local w = c3.bxmax - c3.bxmin
                              local h = c3.bymax - c3.bymin
                              if w > prev_scale or h > prev_scale then
                                candidates[#candidates + 1] = c3
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end

    -- Phase 4: rotated-frame anchor. For every non-axis line direction d,
    -- treat (P, Q) on that line as the bottom edge and search lens region
    -- in the rotated (α, β) frame where α is along d and β is along d_perp.
    -- Catches parls with no axis-aligned side. Phases 1–3 cover any parl
    -- that has ≥1 axis-aligned side.
    local scale_sq = scale * scale
    for li = 1, #diag_lines do
      local line = diag_lines[li]
      local poles_in_line = line.poles
      local n_in_line = #poles_in_line
      for p_idx = 1, n_in_line do
        local P = poles_in_line[p_idx]
        if not pole_dead[P] then
          local px_P = pxs[P]
          local py_P = pys[P]
          for q_idx = p_idx + 1, n_in_line do
            local Q = poles_in_line[q_idx]
            if pole_dead[Q] then break end
            local dx_PQ = pxs[Q] - px_P
            local dy_PQ = pys[Q] - py_P
            local L_sq = dx_PQ * dx_PQ + dy_PQ * dy_PQ
            if L_sq > scale_sq then break end
            pair_checks = pair_checks + 1

            local L = msqrt(L_sq)
            local dx = dx_PQ / L
            local dy = dy_PQ / L
            -- d_perp = (-dy, dx), 90° CCW from d.
            local half_L = L * 0.5
            local L_s32 = L * SQRT3_2

            -- Lens corners in (α, β) = {(±L/2, L·√3/2), (±L/2, scale)},
            -- transformed to xy via P + α·d + β·d_perp.
            local a1 = -half_L * dx + L_s32 * (-dy)
            local a2 =  half_L * dx + L_s32 * (-dy)
            local a3 =  half_L * dx + scale * (-dy)
            local a4 = -half_L * dx + scale * (-dy)
            local b1 = -half_L * dy + L_s32 * dx
            local b2 =  half_L * dy + L_s32 * dx
            local b3 =  half_L * dy + scale * dx
            local b4 = -half_L * dy + scale * dx
            local x_min = a1; if a2 < x_min then x_min = a2 end; if a3 < x_min then x_min = a3 end; if a4 < x_min then x_min = a4 end
            local x_max = a1; if a2 > x_max then x_max = a2 end; if a3 > x_max then x_max = a3 end; if a4 > x_max then x_max = a4 end
            local y_min = b1; if b2 < y_min then y_min = b2 end; if b3 < y_min then y_min = b3 end; if b4 < y_min then y_min = b4 end
            local y_max = b1; if b2 > y_max then y_max = b2 end; if b3 > y_max then y_max = b3 end; if b4 > y_max then y_max = b4 end
            x_min = x_min + px_P; x_max = x_max + px_P
            y_min = y_min + py_P; y_max = y_max + py_P

            local cx_min = mfloor(x_min / cell_size)
            local cx_max = mfloor(x_max / cell_size)
            local cy_min = mfloor(y_min / cell_size)
            local cy_max = mfloor(y_max / cell_size)
            for cx = cx_min, cx_max do
              local row_p = point_bucket[cx]
              if row_p then
                for cy = cy_min, cy_max do
                  local cell = row_p[cy]
                  if cell then
                    for ki = 1, #cell do
                      local X = cell[ki]
                      if not pole_dead[X] and X ~= P and X ~= Q then
                        p2_x_checks = p2_x_checks + 1
                        local vx = pxs[X] - px_P
                        local vy = pys[X] - py_P
                        -- Project onto rotated frame.
                        local alpha = vx * dx + vy * dy
                        local beta  = -vx * dy + vy * dx
                        if mabs(alpha) <= half_L + EPS_GEOM
                           and beta >= L_s32 - EPS_GEOM
                           and beta <= scale + EPS_GEOM then
                          p2_lens_pass = p2_lens_pass + 1
                          p2_y_lookups = p2_y_lookups + 1
                          local c4 = build_phase4_candidate(P, Q, X)
                          if c4 then
                            local w = c4.bxmax - c4.bxmin
                            local h = c4.bymax - c4.bymin
                            if w > prev_scale or h > prev_scale then
                              candidates[#candidates + 1] = c4
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end

    return candidates
  end

  -- v20-style cascade BFS expansion in 4 directions (±a, ±b).
  local function expand_parl(P)
    local p1 = points[P.corners[1]]
    local p2 = points[P.corners[2]]
    local p4 = points[P.corners[4]]
    local v_a_x = p2.x - p1.x
    local v_a_y = p2.y - p1.y
    local v_b_x = p4.x - p1.x
    local v_b_y = p4.y - p1.y
    local directions = {
      { x = v_a_x, y = v_a_y },
      { x = -v_a_x, y = -v_a_y },
      { x = v_b_x, y = v_b_y },
      { x = -v_b_x, y = -v_b_y },
    }
    local expanded = {}
    local seen = { [quad_key(P.corners)] = true }
    local queue = { P }
    local qhead = 1
    while qhead <= #queue do
      local cur = queue[qhead]
      qhead = qhead + 1
      for _, dir in ipairs(directions) do
        local new_corners = {}
        local ok = true
        for ci = 1, 4 do
          local cp = points[cur.corners[ci]]
          local rx, ry = vec_round_xy(cp.x + dir.x, cp.y + dir.y)
          local row = pos_to_idx[rx]
          local idx = row and row[ry]
          if not idx then ok = false; break end
          new_corners[ci] = idx
        end
        if ok then
          local k = quad_key(new_corners)
          if not seen[k] then
            local chains = {}
            local chains_ok = true
            for ci = 1, 4 do
              local ea = new_corners[ci]
              local eb = new_corners[(ci % 4) + 1]
              local ch = make_chain(ea, eb)
              if ch == nil then chains_ok = false; break end
              chains[ci] = ch
            end
            if chains_ok then
              local nc1 = new_corners[1]
              local nc2 = new_corners[2]
              local nc3 = new_corners[3]
              local nc4 = new_corners[4]
              local x1, x2, x3, x4 = pxs[nc1], pxs[nc2], pxs[nc3], pxs[nc4]
              local y1, y2, y3, y4 = pys[nc1], pys[nc2], pys[nc3], pys[nc4]
              local nva_x = x2 - x1
              local nva_y = y2 - y1
              local nvb_x = x4 - x1
              local nvb_y = y4 - y1
              local bxmin, bxmax = x1, x1
              if x2 < bxmin then bxmin = x2 end; if x2 > bxmax then bxmax = x2 end
              if x3 < bxmin then bxmin = x3 end; if x3 > bxmax then bxmax = x3 end
              if x4 < bxmin then bxmin = x4 end; if x4 > bxmax then bxmax = x4 end
              local bymin, bymax = y1, y1
              if y2 < bymin then bymin = y2 end; if y2 > bymax then bymax = y2 end
              if y3 < bymin then bymin = y3 end; if y3 > bymax then bymax = y3 end
              if y4 < bymin then bymin = y4 end; if y4 > bymax then bymax = y4 end
              local new_P = {
                corners = new_corners,
                chains = chains,
                area = mabs(cross_2d(nva_x, nva_y, nvb_x, nvb_y)),
                bxmin = bxmin, bxmax = bxmax,
                bymin = bymin, bymax = bymax,
              }
              seen[k] = true
              expanded[#expanded + 1] = new_P
              queue[#queue + 1] = new_P
            end
          end
        end
      end
    end
    return expanded
  end

  local accepted_parls = {}

  -- ---------------------------------------------------------------------
  -- Edge infrastructure (moved earlier vs v20 layout): the conflict check
  -- below uses chain-segment vs edge-bucket intersection, so the bucket
  -- and Union-Find must exist before the scale loop. MST closure later
  -- reuses the same structures.
  -- ---------------------------------------------------------------------
  local edges = {}
  local edge_set = {}
  local edge_segs = {}
  local edge_bucket = {}

  local parent = {}
  for i = 1, n do parent[i] = i end
  local function find(x)
    local root = x
    while parent[root] ~= root do root = parent[root] end
    while parent[x] ~= root do
      local pn = parent[x]; parent[x] = root; x = pn
    end
    return root
  end
  local function union(a, b)
    local ra, rb = find(a), find(b)
    if ra ~= rb then parent[ra] = rb; return true end
    return false
  end

  local function bbox_cells(x1, y1, x2, y2)
    local minx, maxx = x1, x2
    if minx > maxx then minx, maxx = maxx, minx end
    local miny, maxy = y1, y2
    if miny > maxy then miny, maxy = maxy, miny end
    return mfloor(minx / cell_size), mfloor(maxx / cell_size),
           mfloor(miny / cell_size), mfloor(maxy / cell_size)
  end

  local function bucket_add_edge(idx, x1, y1, x2, y2)
    local cx_min, cx_max, cy_min, cy_max = bbox_cells(x1, y1, x2, y2)
    for cx = cx_min, cx_max do
      local row = edge_bucket[cx]
      if not row then row = {}; edge_bucket[cx] = row end
      for cy = cy_min, cy_max do
        local cell = row[cy]
        if not cell then cell = {}; row[cy] = cell end
        cell[#cell + 1] = idx
      end
    end
  end

  local function bucket_any_intersect(x1, y1, x2, y2)
    local cx_min, cx_max, cy_min, cy_max = bbox_cells(x1, y1, x2, y2)
    local seen = {}
    for cx = cx_min, cx_max do
      local row = edge_bucket[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for _, idx in ipairs(cell) do
              if not seen[idx] then
                seen[idx] = true
                local old = edge_segs[idx]
                if segs_intersect(x1, y1, x2, y2,
                                  old[1], old[2], old[3], old[4]) then
                  return true
                end
              end
            end
          end
        end
      end
    end
    return false
  end

  local function add_edge_unchecked(a, b)
    local k = edge_key(a, b)
    if edge_set[k] then return end
    edge_set[k] = true
    edges[#edges + 1] = { a, b }
    local pa, pb = points[a], points[b]
    edge_segs[#edge_segs + 1] = { pa.x, pa.y, pb.x, pb.y }
    bucket_add_edge(#edge_segs, pa.x, pa.y, pb.x, pb.y)
    union(a, b)
  end

  local function add_edge_checked(a, b)
    local k = edge_key(a, b)
    if edge_set[k] then return false end
    local pa, pb = points[a], points[b]
    if bucket_any_intersect(pa.x, pa.y, pb.x, pb.y) then
      return false
    end
    edge_set[k] = true
    edges[#edges + 1] = { a, b }
    edge_segs[#edge_segs + 1] = { pa.x, pa.y, pb.x, pb.y }
    bucket_add_edge(#edge_segs, pa.x, pa.y, pb.x, pb.y)
    union(a, b)
    return true
  end

  -- v21 conflict-check counters.
  local edge_isect_calls = 0   -- # of bucket_any_intersect calls during accept
  local edge_isect_reject = 0  -- # of candidates rejected by intersection
  local chord_reject = 0       -- # of candidates rejected by chord check

  -- pole_parls[pole_idx] = { [parl_id] = true, ... }
  -- Records which accepted parls each pole sits on the boundary of (corner
  -- or chain intermediate). Used by the chord check below to detect new
  -- chain segments that lie inside an existing parl's interior.
  local pole_parls = {}
  local next_parl_id = 0

  -- Add a parl's chain segments to the edge bucket AND register every
  -- chain pole (corners + intermediates) under a fresh parl_id so future
  -- candidates can chord-check against it.
  local function add_parl_edges(P)
    next_parl_id = next_parl_id + 1
    local pid = next_parl_id
    for _, ch in ipairs(P.chains) do
      for ci = 1, #ch - 1 do
        add_edge_unchecked(ch[ci], ch[ci + 1])
      end
      for ki = 1, #ch do
        local p = ch[ki]
        local set = pole_parls[p]
        if not set then set = {}; pole_parls[p] = set end
        set[pid] = true
      end
    end
  end

  -- True iff a and b sit on the boundary of some same accepted parl Q
  -- (= both registered under the same parl_id in pole_parls).
  local function shares_accepted_parl(a, b)
    local pa = pole_parls[a]
    if not pa then return false end
    local pb = pole_parls[b]
    if not pb then return false end
    for q in pairs(pa) do
      if pb[q] then return true end
    end
    return false
  end

  -- Conflict check: returns true iff any chain segment of P should reject
  -- the candidate. Two independent reasons:
  --   1. CHORD — segment connects two boundary points of some existing Q
  --      and isn't a known chain edge. Such a segment lies inside Q's
  --      convex hull (interior), which SAT's non-overlap rule rejects.
  --   2. TRANSVERSAL — segment crosses an existing edge at a non-endpoint.
  -- Together (1) + (2) = SAT non-overlap with containment override:
  --   * containment (Q ⊂ P or P ⊂ Q): no chord, no transversal → pass
  --   * shared side / shared corner: edge_set has it / endpoint excl → pass
  --   * partial overlap with boundary cross: transversal → reject
  --   * partial overlap via chord (e.g., diagonal of Q): chord → reject
  local function parl_chains_intersect_existing(P)
    for _, ch in ipairs(P.chains) do
      for ci = 1, #ch - 1 do
        local a, b = ch[ci], ch[ci + 1]
        if not edge_set[edge_key(a, b)] and shares_accepted_parl(a, b) then
          chord_reject = chord_reject + 1
          return true
        end
        local pa, pb = points[a], points[b]
        edge_isect_calls = edge_isect_calls + 1
        if bucket_any_intersect(pa.x, pa.y, pb.x, pb.y) then
          return true
        end
      end
    end
    return false
  end

  -- v21 phase-priority dispatch: split candidates by phase, sort each phase
  -- by area asc, then process phase 0 → 1 → 2 → 3. Conflict via
  -- parl_chains_intersect_existing replaces v20's SAT loop.
  local function detect_phased_for_scale(candidates)
    if #candidates == 0 then return end
    local phases = { {}, {}, {}, {} }
    for _, P in ipairs(candidates) do
      local ph = parl_phase(P) + 1
      phases[ph][#phases[ph] + 1] = P
    end
    for ph = 1, 4 do
      table.sort(phases[ph], function(a, b) return a.area < b.area end)
    end

    for ph = 1, 4 do
      for _, P in ipairs(phases[ph]) do
        local p_key = quad_key(P.corners)
        if not all_seen[p_key] then
          if parl_chains_intersect_existing(P) then
            edge_isect_reject = edge_isect_reject + 1
          else
            primary_parls[#primary_parls + 1] = P
            all_seen[p_key] = true
            mark_dead_in_parl(P)
            add_parl_edges(P)
            -- Expansion lattice cascade. v20 trusts the cascade as
            -- conflict-free vs the parent, so we only dedup via all_seen
            -- and skip the intersection check on E. Edges still feed the
            -- bucket so later scales see them.
            for _, E in ipairs(expand_parl(P)) do
              local k = quad_key(E.corners)
              if not all_seen[k] then
                all_seen[k] = true
                expansion_parls[#expansion_parls + 1] = E
                mark_dead_in_parl(E)
                add_parl_edges(E)
              end
            end
          end
        end
      end
    end
  end

  stats.t_scale_total = game.create_profiler()
  stats.scale_candidates = {}
  stats.scale_dead_after = {}
  for s_idx = 1, #SCALES do
    local scale = SCALES[s_idx]
    local prev_scale = s_idx > 1 and SCALES[s_idx - 1] or 0
    local candidates = enumerate_for_scale(scale, prev_scale)
    stats.scale_candidates[s_idx] = #candidates
    detect_phased_for_scale(candidates)
    stats.scale_dead_after[s_idx] = dead_count
  end
  stats.t_scale_total.stop()
  stats.p2_x_checks = p2_x_checks
  stats.p2_lens_pass = p2_lens_pass
  stats.p2_y_lookups = p2_y_lookups
  stats.edge_isect_calls = edge_isect_calls
  stats.edge_isect_reject = edge_isect_reject
  stats.chord_reject = chord_reject

  for _, P in ipairs(primary_parls) do
    accepted_parls[#accepted_parls + 1] = P
  end
  for _, P in ipairs(expansion_parls) do
    accepted_parls[#accepted_parls + 1] = P
  end
  stats.primary = #primary_parls
  stats.expansion = #expansion_parls
  stats.candidates = #primary_parls + #expansion_parls
  stats.pair_checks = pair_checks
  stats.angle_pass = angle_pass
  stats.chain_calls = chain_calls
  stats.chain_success = chain_success
  stats.dead_count = dead_count

  stats.t_iso = game.create_profiler()
  for _, P in ipairs(primary_parls) do try_iso_emit(P) end
  for _, P in ipairs(expansion_parls) do try_iso_emit(P) end
  stats.t_iso.stop()
  stats.iso_pass = iso_pass
  stats.iso_refl_emit = iso_refl_emit
  stats.cache_hits = cache_hits
  stats.cache_misses = cache_misses

  -- Parl chain edges are already in edge_bucket / edges (added by
  -- add_parl_edges during accept). Only iso reflection tris need adding,
  -- with intersection check vs the planar parl edge graph.
  stats.t_pattern_edges = game.create_profiler()
  for _, T in ipairs(accepted_iso_refl_tris) do
    local sa, sb, x = T[1], T[2], T[3]
    add_edge_checked(sa, sb)
    add_edge_checked(sa, x)
    add_edge_checked(sb, x)
  end
  stats.t_pattern_edges.stop()

  stats.t_mst_cand = game.create_profiler()
  local mst_candidates = {}
  for i = 1, n do
    local pi = points[i]
    local ri = reaches[i]
    local cx_min = mfloor((pi.x - ri) / cell_size)
    local cx_max = mfloor((pi.x + ri) / cell_size)
    local cy_min = mfloor((pi.y - ri) / cell_size)
    local cy_max = mfloor((pi.y + ri) / cell_size)
    for cx = cx_min, cx_max do
      local row = point_bucket[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for _, j in ipairs(cell) do
              if j > i then
                local pj = points[j]
                local d = dist(pi.x, pi.y, pj.x, pj.y)
                if d <= pair_reach(i, j) then
                  mst_candidates[#mst_candidates + 1] = { d = d, i = i, j = j }
                end
              end
            end
          end
        end
      end
    end
  end
  table.sort(mst_candidates, function(a, b) return a.d < b.d end)
  stats.mst_cand_count = #mst_candidates
  stats.t_mst_cand.stop()

  stats.t_mst_closure = game.create_profiler()
  for _, e in ipairs(mst_candidates) do
    if find(e.i) ~= find(e.j) then
      add_edge_checked(e.i, e.j)
    end
  end
  stats.t_mst_closure.stop()

  return edges, accepted_parls, accepted_iso_refl_tris, stats
end

return M
