-- Pattern-aware wire layout: parallelogram detection + isosceles symmetry + MST.
--
-- v20 vs v19: keeps v19's parl confirmation rule and iso reflection
-- behavior, but replaces v19's single-pass enumerate-then-greedy with a
-- hierarchical, multi-scale, prune-as-you-go loop:
--
--   For each scale s ∈ {32, 64, 128, 256}:
--     1. Build mid_groups using only pole pairs (i, j) where
--        max(|x_i − x_j|, |y_i − y_j|) ≤ s AND neither pole is "dead".
--     2. Run the iterate hot loop on those groups → candidates fitting in
--        an s × s axis-aligned bbox.
--     3. Phase greedy + SAT non-overlap, with the SAT check pulling in all
--        accepted parls from prior scales (cumulative state).
--     4. For each newly accepted parl (primary + expansion), mark every
--        pole strictly inside its convex hull as "dead". Dead poles drop
--        out of the next scale's mid_groups insert.
--
-- Why hierarchical: smaller scales catch the dense, short-side parls
-- first; the resulting dead-pole set shrinks the pair set the larger
-- scales have to enumerate. v19 enumerated all ~155M midpoint pair_checks
-- on every run regardless of how few survived SAT.
--
-- Why "dead poles" not "dead chains": a pole strictly interior to an
-- accepted parl can never be a corner of a non-overlapping parl (its
-- corner would have to lie outside, contradiction). Dropping pairs that
-- include a dead pole prunes the search without geometric segment-vs-parl
-- intersection tests. Pole-on-boundary cases (chain intermediates on parl
-- sides) stay alive — they may legitimately participate in adjacent parls.
--
-- Iso reflection still fires per accepted parl (only when the parl is on
-- the equality boundary short_diag == max_side); the call moves to a
-- single pass after the scale loop so every accepted parl gets seen.
--
-- Public API:
--   M.compute(points, reaches, opts) -> edges, parls, iso_tris, stats
--     points     : 1-indexed array of {x = number, y = number}
--     reaches    : 1-indexed array of per-pole max wire distance.
--                  Pair-wise reach for any (i, j) is min(reaches[i], reaches[j]),
--                  matching how the Factorio engine validates a copper wire.
--     edges      : list of {i, j} (1-indexed pole indices)
--     parls      : list of accepted parallelograms (for visualization)
--     iso_tris   : list of accepted isosceles reflection triangles {a, b, c}

local M = {}

local EPS = 1e-9
local EPS_GEOM = 1e-6
local EPS_DIAG = 1e-6  -- tolerance for short_diag² == max(la², lb²) test
local ROUND_SCALE = 10000  -- 4-decimal precision

-- Hierarchical scale schedule for v20. At each step the iterate hot loop
-- only sees pole pairs whose axis-aligned bbox is ≤ this scale, so small
-- parls get found and prune the pair set before larger scales fire.
local SCALES = { 32, 64, 128, 256 }
local mfloor = math.floor
local mabs = math.abs
local msqrt = math.sqrt

-- Round (x, y) to fixed-point integers; returned as a (rx, ry) pair so
-- callers can use them as 2-level table keys without string allocation.
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
  -- Strict intersection (endpoints excluded).
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

  -- Measurement A/B switch: when opts.skip_chain_block is true, the iterate
  -- inner loop runs through pair_checks → distinct → vector → angle filter
  -- but bypasses the four make_chain calls and candidate registration.
  -- Output is broken (candidates = 0) but iterate time isolates non-chain
  -- work. Subtract from the full run to get the chain + register cost.
  local skip_chain_block = opts and opts.skip_chain_block

  -- Pack (i, j) pairs into a single int via i * K + j so the hot midpoint
  -- loop can store pairs as ints in a flat array, dropping ~n² per-pair
  -- {i, j} table allocations.
  local K = n + 1

  -- Flat coordinate arrays. The hot loops access pxs[i] / pys[i] (int-keyed
  -- array) instead of points[i].x (table-field hash lookup), which JITs into
  -- a direct memory load.
  local pxs, pys = {}, {}
  for i = 1, n do
    pxs[i] = points[i].x
    pys[i] = points[i].y
  end

  stats.t_setup = game.create_profiler()
  -- pos_to_idx: round positions for symmetric-target lookup. 2-level int
  -- table avoids per-build string allocation; callers index pos_to_idx[rx][ry].
  local pos_to_idx = {}
  for i = 1, n do
    local rx, ry = vec_round_xy(points[i].x, points[i].y)
    local row = pos_to_idx[rx]
    if not row then row = {}; pos_to_idx[rx] = row end
    row[ry] = i
  end

  -- Per-pair reach helper (Factorio rule: pair reach = min of endpoints).
  local function pair_reach(i, j)
    local ri, rj = reaches[i], reaches[j]
    return ri < rj and ri or rj
  end

  -- Spatial bucket sized by the max reach in the selection so cell queries
  -- can never miss a within-reach pair, even for the largest poles.
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

  -- Per-pole int coordinates (4-decimal fixed-point) for exact line keying.
  local pixs, piys = {}, {}
  for i = 1, n do
    local rx, ry = vec_round_xy(points[i].x, points[i].y)
    pixs[i] = rx
    piys[i] = ry
  end

  -- Pre-build line buckets so points_on_segment_strict can do O(1) lookup
  -- instead of cell-scanning the segment bbox. A "line" is keyed by its
  -- canonical direction (ndx, ndy) (gcd-reduced, sign-canonicalized) and
  -- offset c = ndy·pix − ndx·piy. Only reach-near pairs feed lines; any
  -- pole that participates in a valid chain is reach-near to at least one
  -- collinear neighbor, so it transitively shows up here. If lo or hi is
  -- absent from the shared line bucket, no reach-respecting chain spans
  -- them and make_chain will reject downstream.
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

  -- Sort each line's poles by canonical direction; build pos_in_line for
  -- O(1) "where on this line is pole P?" lookup.
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
        for ki = 1, #sorted do
          pos_in_line[sorted[ki]] = ki
        end
        line.pos_in_line = pos_in_line
      end
    end
  end
  stats.t_setup.stop()

  -- Reusable buffer for points_on_segment_strict's intermediate output.
  -- Cleared at the top of every call; caller reads pos_seg_ks[1..count].
  local pos_seg_count = 0
  local pos_seg_ks = {}

  -- Diagnostic counters (cheap, kept on). Reading them doesn't change
  -- algorithm output.
  local cache_hits = 0
  local cache_misses = 0

  -- Per-pole reach squared, so make_chain can compare squared lengths and
  -- skip the per-segment math.sqrt.
  local reaches_sq = {}
  for i = 1, n do reaches_sq[i] = reaches[i] * reaches[i] end
  local function pair_reach_sq(i, j)
    local ri, rj = reaches_sq[i], reaches_sq[j]
    return ri < rj and ri or rj
  end

  -- Fills pos_seg_ks[1..count] with collinear interior poles of segment i—j,
  -- in order from i toward j. Output identical to a cell-scan that catches
  -- any pole within EPS_GEOM of the line (Factorio's grid-aligned positions
  -- make the integer-key match strict and floating-point match coincide).
  --
  -- The pre-built line bucket gives O(1) lookup of all collinear poles in
  -- the (lo, hi) direction, then we slice the sorted poles between lo's and
  -- hi's positions on the line. If the line bucket entry is missing or
  -- either endpoint isn't in it, no chain can span lo→hi via reach-near
  -- hops, so we return 0 and let make_chain reject via the consecutive-gap
  -- check.
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

  -- Cache make_chain results so duplicate sides across adjacent parl
  -- candidates don't redo the segment scan. Keyed canonically by (lo, hi)
  -- because chain direction doesn't affect the resulting edge set.
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

    -- Squared-length validation — skips one math.sqrt per segment.
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

  -- SAT-based parallelogram-on-parallelogram overlap test.
  -- Boundary touch (separation distance == 0) is treated as separated.
  -- Hot-path SAT with no per-call allocation: corners are dereferenced into
  -- locals, the four axes are computed inline, and projections compare via
  -- chained min/max instead of an axes table + ipairs.
  local function quads_overlap(P_corners, Q_corners)
    local pp1 = points[P_corners[1]]
    local pp2 = points[P_corners[2]]
    local pp3 = points[P_corners[3]]
    local pp4 = points[P_corners[4]]
    local qp1 = points[Q_corners[1]]
    local qp2 = points[Q_corners[2]]
    local qp3 = points[Q_corners[3]]
    local qp4 = points[Q_corners[4]]
    local pp1x, pp1y = pp1.x, pp1.y
    local pp2x, pp2y = pp2.x, pp2.y
    local pp3x, pp3y = pp3.x, pp3.y
    local pp4x, pp4y = pp4.x, pp4.y
    local qp1x, qp1y = qp1.x, qp1.y
    local qp2x, qp2y = qp2.x, qp2.y
    local qp3x, qp3y = qp3.x, qp3.y
    local qp4x, qp4y = qp4.x, qp4.y

    for axis_idx = 1, 4 do
      local ax, ay
      if axis_idx == 1 then
        ax = pp2y - pp1y; ay = pp1x - pp2x
      elseif axis_idx == 2 then
        ax = pp3y - pp2y; ay = pp2x - pp3x
      elseif axis_idx == 3 then
        ax = qp2y - qp1y; ay = qp1x - qp2x
      else
        ax = qp3y - qp2y; ay = qp2x - qp3x
      end
      if ax * ax + ay * ay >= 1e-18 then
        local p1 = pp1x * ax + pp1y * ay
        local p2 = pp2x * ax + pp2y * ay
        local p3 = pp3x * ax + pp3y * ay
        local p4 = pp4x * ax + pp4y * ay
        local pmin, pmax = p1, p1
        if p2 < pmin then pmin = p2 end
        if p2 > pmax then pmax = p2 end
        if p3 < pmin then pmin = p3 end
        if p3 > pmax then pmax = p3 end
        if p4 < pmin then pmin = p4 end
        if p4 > pmax then pmax = p4 end

        local q1 = qp1x * ax + qp1y * ay
        local q2 = qp2x * ax + qp2y * ay
        local q3 = qp3x * ax + qp3y * ay
        local q4 = qp4x * ax + qp4y * ay
        local qmin, qmax = q1, q1
        if q2 < qmin then qmin = q2 end
        if q2 > qmax then qmax = q2 end
        if q3 < qmin then qmin = q3 end
        if q3 > qmax then qmax = q3 end
        if q4 < qmin then qmin = q4 end
        if q4 > qmax then qmax = q4 end

        if pmax < qmin + EPS or qmax < pmin + EPS then
          return false
        end
      end
    end
    return true
  end

  -- v19 iso emission state. Iso reflections fire per accepted parl whose
  -- short diagonal equals its longer side (the boundary of the side-length
  -- rule). For each of the two long parl sides, the opposite parl corner is
  -- reflected across the side's midpoint; the resulting triangle is emitted
  -- only if the reflected point exists as a pole and all three triangle
  -- sides fit a single direct hop. The (side_a, side_b) edge is also added
  -- by the parl chain, but if the parl side runs through intermediate
  -- poles its direct distance exceeds the per-pole reach and the reflection
  -- is rejected — matching v18's iso reflection convention.
  local accepted_iso_refl_tris = {}   -- {long_a, long_b, X} per reflection
  local seen_iso = {}                 -- tri_key dedup
  local iso_pass = 0
  local iso_refl_emit = 0

  -- v20 cumulative state across the scale loop. accepted parls accumulate
  -- here (primary + expansion lists are kept separate so iso emission can
  -- iterate them once at the end), and the iterate counters are upvalues
  -- so each enumerate_parls(scale) call adds to the same totals.
  local primary_parls = {}
  local expansion_parls = {}
  local all_seen = {}        -- quad_key dedup across primary + expansion
  local pair_checks = 0
  local angle_pass = 0
  local chain_calls = 0
  local chain_success = 0

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
    -- The (side_a, side_b) edge is also added as a direct wire, so it
    -- needs to fit a single hop too.
    local dabx = pxs[side_a] - pxs[side_b]
    local daby = pys[side_a] - pys[side_b]
    if dabx * dabx + daby * daby > pair_reach_sq(side_a, side_b) then return end
    local kr = tri_key(side_a, side_b, X)
    if seen_iso[kr] then return end
    seen_iso[kr] = true
    accepted_iso_refl_tris[#accepted_iso_refl_tris + 1] = { side_a, side_b, X }
    iso_refl_emit = iso_refl_emit + 1
  end

  -- Called per accepted parl (primary + expansion). When the parl satisfies
  -- short_diag² == max(L1², L2²) (within EPS_DIAG), reflect each of the two
  -- long sides outward across its midpoint. Otherwise no-op.
  local function try_iso_emit(P)
    local cc1, cc2, cc3, cc4 = P.corners[1], P.corners[2], P.corners[3], P.corners[4]
    -- CCW side type squared lengths.
    --   L1 = sides cc1-cc2 and cc3-cc4 (parallel pair).
    --   L2 = sides cc2-cc3 and cc4-cc1 (parallel pair).
    local L1_dx = pxs[cc2] - pxs[cc1]
    local L1_dy = pys[cc2] - pys[cc1]
    local L2_dx = pxs[cc3] - pxs[cc2]
    local L2_dy = pys[cc3] - pys[cc2]
    local L1_sq = L1_dx * L1_dx + L1_dy * L1_dy
    local L2_sq = L2_dx * L2_dx + L2_dy * L2_dy
    local max_side_sq = L1_sq > L2_sq and L1_sq or L2_sq
    -- Short diagonal squared: take the smaller of |cc3-cc1|² and |cc4-cc2|².
    local d13x = pxs[cc3] - pxs[cc1]
    local d13y = pys[cc3] - pys[cc1]
    local d24x = pxs[cc4] - pxs[cc2]
    local d24y = pys[cc4] - pys[cc2]
    local d13_sq = d13x * d13x + d13y * d13y
    local d24_sq = d24x * d24x + d24y * d24y
    local short_diag_sq, short_a
    if d13_sq < d24_sq then short_diag_sq, short_a = d13_sq, cc1
    else short_diag_sq, short_a = d24_sq, cc2 end
    -- Iso trigger: short_diag = max_side (== boundary of the parl rule).
    local diff = short_diag_sq - max_side_sq
    if diff < -EPS_DIAG or diff > EPS_DIAG then return end
    iso_pass = iso_pass + 1
    local long_is_L1 = L1_sq >= L2_sq
    -- Reflect across each long parl side. The "opp" vertex is the parl
    -- corner on the same half (relative to the short diagonal) as the long
    -- side; reflecting it across the long side's midpoint moves it outward.
    if short_a == cc1 then
      -- Short diag = cc1-cc3. Halves: {cc2} on one side, {cc4} on the other.
      if long_is_L1 then
        try_reflect_iso(cc1, cc2, cc3)
        try_reflect_iso(cc3, cc4, cc1)
      else
        try_reflect_iso(cc2, cc3, cc1)
        try_reflect_iso(cc4, cc1, cc3)
      end
    else
      -- Short diag = cc2-cc4. Halves: {cc1} on one side, {cc3} on the other.
      if long_is_L1 then
        try_reflect_iso(cc1, cc2, cc4)
        try_reflect_iso(cc3, cc4, cc2)
      else
        try_reflect_iso(cc4, cc1, cc2)
        try_reflect_iso(cc2, cc3, cc4)
      end
    end
  end

  -- v20 chain pruning state. pole_dead[i] = true once pole i is strictly
  -- inside an accepted parl's convex hull. Dead poles are filtered out of
  -- subsequent scales' mid_groups insert, so the next pass skips a pair
  -- the moment one of its endpoints belongs to a parl already accepted.
  -- Boundary poles (corners and chain intermediates on parl sides) stay
  -- alive — they may legitimately be corners of adjacent parls.
  local pole_dead = {}
  local dead_count = 0

  -- Strict point-in-parl test using CCW corner order. Returns true only
  -- when the pole sits strictly inside (positive signed area against every
  -- side, no boundary touches).
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

  -- Iterate poles inside P's bbox (via point_bucket) and mark interior
  -- ones as dead. Cheap because each parl's bbox is small relative to
  -- the layout and we only inspect cells the bbox actually touches.
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

  -- Enumerate parallelogram candidates restricted by scale and pole_dead.
  -- Two pairs sharing a midpoint = diagonals of a parallelogram.
  -- v19 confirmation rule: short_diag ≥ max(la, lb). In squared form, with
  -- A=la², B=lb², dot=v_a·v_b:  2·|dot| ≤ min(A, B). Iso reflections fire
  -- per accepted parl in try_iso_emit when 2·|dot| = min(A, B) exactly.
  -- (Compare to v18: 4·dot² ≤ A·B, i.e. 60°≤θ≤120°.)
  --
  -- v20: pairs with bbox > scale and pairs touching a dead pole are
  -- skipped at the insert step; the iterate hot loop never sees them.
  local function enumerate_parls(scale)
    if n < 4 then return {} end

    stats.t_enum_insert = game.create_profiler()
    -- v20 hot loop: iterate j candidates spatially via point_bucket, with
    -- bbox(scale) + pole_dead filters applied at insert time so the
    -- iterate stage only sees alive, in-scale pairs.
    --   * 2-level int tables avoid string-key allocation.
    --   * Pairs are packed as a single int (i * K + j); flat array storage.
    --   * Singleton midpoints stay inline in mid_first; a group table is
    --     allocated only when a second pair lands on the same midpoint.
    --   * pixs / piys are int-keyed flat arrays in the canonical 4-decimal
    --     fixed-point representation. The midpoint key is the integer sum
    --     (pixs[i] + pixs[j], piys[i] + piys[j]) — exactly twice the
    --     midpoint in fixed-point.
    local mid_first = {}
    local mid_groups = {}
    for i = 1, n do
      if not pole_dead[i] then
        local pix = pixs[i]
        local piy = piys[i]
        local i_packed = i * K
        local pi_x = pxs[i]
        local pi_y = pys[i]
        local cx_min = mfloor((pi_x - scale) / cell_size)
        local cx_max = mfloor((pi_x + scale) / cell_size)
        local cy_min = mfloor((pi_y - scale) / cell_size)
        local cy_max = mfloor((pi_y + scale) / cell_size)
        for cx = cx_min, cx_max do
          local row_p = point_bucket[cx]
          if row_p then
            for cy = cy_min, cy_max do
              local cell = row_p[cy]
              if cell then
                for ki = 1, #cell do
                  local j = cell[ki]
                  if j > i and not pole_dead[j] then
                    local dx = pxs[j] - pi_x
                    local dy = pys[j] - pi_y
                    local adx = dx >= 0 and dx or -dx
                    local ady = dy >= 0 and dy or -dy
                    if adx <= scale and ady <= scale then
                      local rx = pix + pixs[j]
                      local ry = piy + piys[j]
                      local packed = i_packed + j
                      local row_f = mid_first[rx]
                      if not row_f then row_f = {}; mid_first[rx] = row_f end
                      local existing = row_f[ry]
                      if not existing then
                        row_f[ry] = packed
                      else
                        local row_g = mid_groups[rx]
                        if not row_g then row_g = {}; mid_groups[rx] = row_g end
                        local g = row_g[ry]
                        if not g then
                          g = { existing, packed }
                          row_g[ry] = g
                        else
                          g[#g + 1] = packed
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
    stats.t_enum_insert.stop()

    stats.t_enum_iterate = game.create_profiler()
    local candidates = {}
    for _, ry_row in pairs(mid_groups) do
      for _, grp in pairs(ry_row) do
        local glen = #grp
        if glen >= 2 then
          for pi = 1, glen do
            local pa = grp[pi]
            local a = mfloor(pa / K)
            local b = pa - a * K
            for pj = pi + 1, glen do
              pair_checks = pair_checks + 1
              local pb = grp[pj]
              local c = mfloor(pb / K)
              local d = pb - c * K
              if a ~= c and a ~= d and b ~= c and b ~= d then
                local i, j, k = a, c, d  -- l = b
                local pix = pxs[i]; local piy = pys[i]
                local v_a_x = pxs[j] - pix
                local v_a_y = pys[j] - piy
                local v_b_x = pxs[k] - pix
                local v_b_y = pys[k] - piy
                local la2 = v_a_x * v_a_x + v_a_y * v_a_y
                local lb2 = v_b_x * v_b_x + v_b_y * v_b_y
                if la2 >= 1e-18 and lb2 >= 1e-18 then
                  local dotv = v_a_x * v_b_x + v_a_y * v_b_y
                  -- v19 confirmation rule: short_diag ≥ max(la, lb).
                  --   short_diag² = la² + lb² − 2·|dot|.
                  --   confirmed parl  ⇔  2·|dot| ≤ min(la², lb²)
                  -- The == boundary of this rule is later detected per
                  -- accepted parl in try_iso_emit and produces reflections.
                  local two_abs_dot = dotv >= 0 and 2 * dotv or -2 * dotv
                  local lmin = la2 < lb2 and la2 or lb2
                  if two_abs_dot <= lmin + EPS_DIAG then
                    angle_pass = angle_pass + 1
                    if not skip_chain_block then
                    -- Lazy alloc: keep CCW corner ids in locals while the
                    -- four make_chain calls run; only allocate corners_ccw
                    -- and chains tables when all four sides validate, so
                    -- the ~83% of angle-pass candidates that fail the
                    -- chain check don't pay for two table allocations.
                    local cc1, cc2, cc3, cc4
                    if cross_2d(v_a_x, v_a_y, v_b_x, v_b_y) > 0 then
                      cc1, cc2, cc3, cc4 = i, j, b, k
                    else
                      cc1, cc2, cc3, cc4 = i, k, b, j
                    end
                    chain_calls = chain_calls + 1
                    local ch1 = make_chain(cc1, cc2)
                    if ch1 ~= nil then
                      chain_success = chain_success + 1
                      chain_calls = chain_calls + 1
                      local ch2 = make_chain(cc2, cc3)
                      if ch2 ~= nil then
                        chain_success = chain_success + 1
                        chain_calls = chain_calls + 1
                        local ch3 = make_chain(cc3, cc4)
                        if ch3 ~= nil then
                          chain_success = chain_success + 1
                          chain_calls = chain_calls + 1
                          local ch4 = make_chain(cc4, cc1)
                          if ch4 ~= nil then
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
                            candidates[#candidates + 1] = {
                              corners = { cc1, cc2, cc3, cc4 },
                              chains = { ch1, ch2, ch3, ch4 },
                              area = mabs(cross_2d(va_x, va_y, vb_x, vb_y)),
                              bxmin = bxmin, bxmax = bxmax,
                              bymin = bymin, bymax = bymax,
                            }
                          end
                        end
                      end
                    end
                    end  -- if not skip_chain_block
                  end
                end
              end
            end
          end
        end
      end
    end
    stats.t_enum_iterate.stop()
    return candidates
  end

  -- Phase classification: 0=axis-rect, 1=horiz-aligned, 2=vert-aligned, 3=diagonal.
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

  -- Cascading BFS expansion in 4 directions (±a, ±b).
  -- Skips overlap with other accepted P (lattice cascade).
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

  -- v20 phase-ordered detection per scale. Reads candidates emitted by
  -- enumerate_parls(scale); SAT-checks each against the cumulative
  -- primary_parls and expansion_parls (across all prior scales), and
  -- when a primary is accepted runs cascade BFS expansion + dead-pole
  -- marking to prune the next scale's mid_groups insert.
  --
  -- Iso reflection is deferred to a single post-loop pass so every
  -- accepted parl from every scale gets a chance regardless of when it
  -- was accepted.
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
        local conflict = false
        local p_bxmin, p_bxmax = P.bxmin, P.bxmax
        local p_bymin, p_bymax = P.bymin, P.bymax
        for _, Q in ipairs(primary_parls) do
          if p_bxmin <= Q.bxmax and p_bxmax >= Q.bxmin
             and p_bymin <= Q.bymax and p_bymax >= Q.bymin then
            if quads_overlap(P.corners, Q.corners) then conflict = true; break end
          end
        end
        if not conflict then
          for _, Q in ipairs(expansion_parls) do
            if p_bxmin <= Q.bxmax and p_bxmax >= Q.bxmin
               and p_bymin <= Q.bymax and p_bymax >= Q.bymin then
              if quads_overlap(P.corners, Q.corners) then conflict = true; break end
            end
          end
        end
        if not conflict then
          primary_parls[#primary_parls + 1] = P
          all_seen[quad_key(P.corners)] = true
          mark_dead_in_parl(P)
          for _, E in ipairs(expand_parl(P)) do
            local k = quad_key(E.corners)
            if not all_seen[k] then
              all_seen[k] = true
              expansion_parls[#expansion_parls + 1] = E
              mark_dead_in_parl(E)
            end
          end
        end
      end
    end
  end

  stats.t_scale_total = game.create_profiler()
  stats.scale_candidates = {}      -- per-scale candidate counts
  stats.scale_dead_after = {}      -- per-scale cumulative dead pole count
  for s_idx = 1, #SCALES do
    local scale = SCALES[s_idx]
    local candidates = enumerate_parls(scale)
    stats.scale_candidates[s_idx] = #candidates
    detect_phased_for_scale(candidates)
    stats.scale_dead_after[s_idx] = dead_count
  end
  stats.t_scale_total.stop()

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
  for _, P in ipairs(primary_parls) do
    try_iso_emit(P)
  end
  for _, P in ipairs(expansion_parls) do
    try_iso_emit(P)
  end
  stats.t_iso.stop()

  stats.iso_pass = iso_pass
  stats.iso_refl_emit = iso_refl_emit
  stats.cache_hits = cache_hits
  stats.cache_misses = cache_misses

  -- =========================================================
  -- Phase B: collect edges + Kruskal MST closure (with intersection check)
  -- =========================================================

  local edges = {}
  local edge_set = {}
  local edge_segs = {}
  -- Spatial bucket for the planar edge graph. Edge length is bounded by
  -- max_reach == cell_size, so each edge touches at most 4 cells; the
  -- intersection check only iterates edges in the new edge's bbox cells.
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

  stats.t_pattern_edges = game.create_profiler()
  -- Parallelogram chain edges (no cross check — accepted P's are non-overlapping
  -- by SAT, so chain edges don't cross each other).
  for _, P in ipairs(accepted_parls) do
    for _, chain in ipairs(P.chains) do
      for ci = 1, #chain - 1 do
        add_edge_unchecked(chain[ci], chain[ci + 1])
      end
    end
  end

  -- v19 iso reflection triangles — three direct edges each, cross-checked
  -- against existing edges. The (sa, sb) edge is the parl long side, which
  -- the parent parl already wired via add_edge_unchecked (single segment
  -- when the long side fits a direct hop, which is the only case the
  -- reflection's reach check accepts). edge_set dedup makes the redundant
  -- add_edge_checked call here a cheap no-op.
  for _, T in ipairs(accepted_iso_refl_tris) do
    local sa, sb, x = T[1], T[2], T[3]
    add_edge_checked(sa, sb)
    add_edge_checked(sa, x)
    add_edge_checked(sb, x)
  end
  stats.t_pattern_edges.stop()

  -- MST closure: connect remaining components by Kruskal, shortest first.
  -- Spatial-bucket the candidate enumeration: cell_size == max_reach, so any
  -- valid pair lies within the box of cells around pole i, giving O(n × k)
  -- instead of O(n²).
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
