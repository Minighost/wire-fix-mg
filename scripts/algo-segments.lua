-- Segment-based parallelogram detection.
--
-- Pipeline:
--   1. Detect maximal collinear segments via exact integer line keying
--      (no angle/offset quantization). Pole positions are cast to fixed-point
--      ints, pair direction is reduced via gcd to (ndx, ndy) with canonical
--      sign, and the line offset c = ndy·x_anchor − ndx·y_anchor is integer.
--   2. Enumerate parallelogram candidates by walking (A, B, D) triples where
--      A is shared by two through-segments and B/D follow A in their
--      segments. The 4th corner C = B + D − A must exist as a real pole and
--      lie on the matching cross segments.
--   3. Filter by 60°–120° interior angle.
--   4. Within phase priority (rectangle / horiz-aligned / vert-aligned /
--      diagonal), accept smallest-area parls first, rejecting those that
--      overlap an already-accepted one (axis-aligned bbox prefilter + SAT).
--   5. Emit parl-side edges (each side traces the consecutive poles on its
--      segment between the two corners).
--   6. Kruskal MST closure connects any remaining components by shortest
--      edge with an intersection check against the accepted edge set.

local M = {}

local mfloor = math.floor
local mabs = math.abs
local msqrt = math.sqrt

local EPS = 1e-9
local ROUND_SCALE = 10000

local function int_coord(x)
  if x >= 0 then return mfloor(x * ROUND_SCALE + 0.5)
  else return -mfloor(-x * ROUND_SCALE + 0.5) end
end

-- ============================================================================
-- Stage 1: segment detection
-- ============================================================================

local function detect_segments_impl(points, reaches, n, stats)
  stats.t_setup = game.create_profiler()
  local max_reach = 0
  for i = 1, n do
    if reaches[i] > max_reach then max_reach = reaches[i] end
  end
  local cell_size = max_reach
  local pxs, pys = {}, {}
  for i = 1, n do
    pxs[i] = points[i].x
    pys[i] = points[i].y
  end
  local pixs, piys = {}, {}
  for i = 1, n do
    pixs[i] = int_coord(points[i].x)
    piys[i] = int_coord(points[i].y)
  end
  local point_bucket = {}
  for i = 1, n do
    local cx = mfloor(pxs[i] / cell_size)
    local cy = mfloor(pys[i] / cell_size)
    local row = point_bucket[cx]
    if not row then row = {}; point_bucket[cx] = row end
    local cell = row[cy]
    if not cell then cell = {}; row[cy] = cell end
    cell[#cell + 1] = i
  end
  stats.t_setup.stop()

  stats.t_pairs = game.create_profiler()
  local lines = {}
  local pair_count = 0
  for i = 1, n do
    local pix_f = pxs[i]
    local piy_f = pys[i]
    local pix_i = pixs[i]
    local piy_i = piys[i]
    local ri = reaches[i]
    local cx_min = mfloor((pix_f - ri) / cell_size)
    local cx_max = mfloor((pix_f + ri) / cell_size)
    local cy_min = mfloor((piy_f - ri) / cell_size)
    local cy_max = mfloor((piy_f + ri) / cell_size)
    for cx = cx_min, cx_max do
      local row = point_bucket[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for ki = 1, #cell do
              local j = cell[ki]
              if j > i then
                local dxf = pxs[j] - pix_f
                local dyf = pys[j] - piy_f
                local d2 = dxf * dxf + dyf * dyf
                local rj = reaches[j]
                local r = ri < rj and ri or rj
                if d2 <= r * r then
                  pair_count = pair_count + 1
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
                    ndx = -ndx
                    ndy = -ndy
                  end
                  local c = ndy * pix_i - ndx * piy_i
                  local row_a = lines[ndx]
                  if not row_a then row_a = {}; lines[ndx] = row_a end
                  local row_b = row_a[ndy]
                  if not row_b then row_b = {}; row_a[ndy] = row_b end
                  local line = row_b[c]
                  if not line then
                    line = {
                      poles = {},
                      ndx = ndx, ndy = ndy,
                      anchor_x = pix_i, anchor_y = piy_i,
                    }
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
  stats.t_pairs.stop()
  stats.pair_count = pair_count

  stats.t_segments = game.create_profiler()
  local segments = {}
  local line_count = 0
  for _, row_a in pairs(lines) do
    for _, row_b in pairs(row_a) do
      for _, line in pairs(row_b) do
        line_count = line_count + 1
        local poles_list = {}
        for p in pairs(line.poles) do poles_list[#poles_list + 1] = p end
        if #poles_list >= 2 then
          local ndx = line.ndx
          local ndy = line.ndy
          local ax = line.anchor_x
          local ay = line.anchor_y
          table.sort(poles_list, function(aa, bb)
            local sa = (pixs[aa] - ax) * ndx + (piys[aa] - ay) * ndy
            local sb = (pixs[bb] - ax) * ndx + (piys[bb] - ay) * ndy
            return sa < sb
          end)
          local current = { poles_list[1] }
          for k = 2, #poles_list do
            local prev = poles_list[k - 1]
            local curr = poles_list[k]
            local sdx = pxs[curr] - pxs[prev]
            local sdy = pys[curr] - pys[prev]
            local sd2 = sdx * sdx + sdy * sdy
            local rp = reaches[prev]
            local rc = reaches[curr]
            local rr = rp < rc and rp or rc
            if sd2 <= rr * rr then
              current[#current + 1] = curr
            else
              if #current >= 2 then
                segments[#segments + 1] = {
                  poles = current, ndx = ndx, ndy = ndy,
                }
              end
              current = { curr }
            end
          end
          if #current >= 2 then
            segments[#segments + 1] = {
              poles = current, ndx = ndx, ndy = ndy,
            }
          end
        end
      end
    end
  end
  stats.t_segments.stop()

  stats.line_count = line_count
  stats.segment_count = #segments

  return segments, pxs, pys, pixs, piys, cell_size, point_bucket
end

-- ============================================================================
-- Stage 2-6: full pipeline
-- ============================================================================

local function quads_overlap(corners_P, corners_Q, pxs, pys)
  local pp1 = corners_P[1]; local pp2 = corners_P[2]
  local pp3 = corners_P[3]; local pp4 = corners_P[4]
  local qp1 = corners_Q[1]; local qp2 = corners_Q[2]
  local qp3 = corners_Q[3]; local qp4 = corners_Q[4]
  local pp1x, pp1y = pxs[pp1], pys[pp1]
  local pp2x, pp2y = pxs[pp2], pys[pp2]
  local pp3x, pp3y = pxs[pp3], pys[pp3]
  local pp4x, pp4y = pxs[pp4], pys[pp4]
  local qp1x, qp1y = pxs[qp1], pys[qp1]
  local qp2x, qp2y = pxs[qp2], pys[qp2]
  local qp3x, qp3y = pxs[qp3], pys[qp3]
  local qp4x, qp4y = pxs[qp4], pys[qp4]
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

function M.compute(points, reaches)
  local n = #points
  local stats = {}
  if n < 2 then return {}, {}, {}, stats end

  local segments, pxs, pys, pixs, piys, cell_size, point_bucket =
    detect_segments_impl(points, reaches, n, stats)

  -- ---- index ----
  stats.t_index = game.create_profiler()
  local pos_to_idx = {}
  for i = 1, n do
    local row = pos_to_idx[pixs[i]]
    if not row then row = {}; pos_to_idx[pixs[i]] = row end
    row[piys[i]] = i
  end
  local segs_by_dir = {}
  local pole_to_pos = {}
  for s_idx = 1, #segments do
    local seg = segments[s_idx]
    local dir_key = seg.ndx .. ":" .. seg.ndy
    for pos_idx, pole in ipairs(seg.poles) do
      local row = segs_by_dir[pole]
      if not row then row = {}; segs_by_dir[pole] = row end
      row[dir_key] = s_idx
      local row2 = pole_to_pos[pole]
      if not row2 then row2 = {}; pole_to_pos[pole] = row2 end
      row2[s_idx] = pos_idx
    end
  end
  stats.t_index.stop()

  -- ---- enumerate parl candidates ----
  stats.t_parls = game.create_profiler()
  local candidates = {}
  local seen_parls = {}
  for A = 1, n do
    local segs_A = segs_by_dir[A]
    if segs_A then
      local through_A = {}
      for dk, sidx in pairs(segs_A) do
        through_A[#through_A + 1] = { sidx = sidx, dk = dk }
      end
      local nA = #through_A
      for ii = 1, nA do
        local s1 = through_A[ii]
        local seg1 = segments[s1.sidx]
        local pos_A_1 = pole_to_pos[A][s1.sidx]
        for jj = ii + 1, nA do
          local s2 = through_A[jj]
          local seg2 = segments[s2.sidx]
          local pos_A_2 = pole_to_pos[A][s2.sidx]
          local poles1 = seg1.poles
          local poles2 = seg2.poles
          for b_pos = pos_A_1 + 1, #poles1 do
            local B = poles1[b_pos]
            for d_pos = pos_A_2 + 1, #poles2 do
              local D = poles2[d_pos]
              local cix = pixs[B] + pixs[D] - pixs[A]
              local ciy = piys[B] + piys[D] - piys[A]
              local row_C = pos_to_idx[cix]
              local C = row_C and row_C[ciy]
              if C and C ~= A and C ~= B and C ~= D then
                local seg_BC = segs_by_dir[B] and segs_by_dir[B][s2.dk]
                if seg_BC then
                  local pos_C_BC = pole_to_pos[C] and pole_to_pos[C][seg_BC]
                  if pos_C_BC then
                    local seg_DC = segs_by_dir[D] and segs_by_dir[D][s1.dk]
                    if seg_DC then
                      local pos_C_DC = pole_to_pos[C] and pole_to_pos[C][seg_DC]
                      if pos_C_DC then
                        local v_ab_x = pxs[B] - pxs[A]
                        local v_ab_y = pys[B] - pys[A]
                        local v_ad_x = pxs[D] - pxs[A]
                        local v_ad_y = pys[D] - pys[A]
                        local la2 = v_ab_x * v_ab_x + v_ab_y * v_ab_y
                        local lb2 = v_ad_x * v_ad_x + v_ad_y * v_ad_y
                        local dotv = v_ab_x * v_ad_x + v_ab_y * v_ad_y
                        if 4 * dotv * dotv <= la2 * lb2 then
                          local s = { A, B, C, D }
                          table.sort(s)
                          local key = s[1] .. "_" .. s[2] .. "_" .. s[3] .. "_" .. s[4]
                          if not seen_parls[key] then
                            seen_parls[key] = true
                            -- CCW corner ordering.
                            local cross = v_ab_x * v_ad_y - v_ab_y * v_ad_x
                            local corners
                            local seg_AB = s1.sidx
                            local seg_AD = s2.sidx
                            -- side_segments[k] = segment of side from corners[k] to corners[k%4+1]
                            local side_segments
                            if cross > 0 then
                              corners = { A, B, C, D }
                              -- A→B on seg_AB, B→C on seg_BC, C→D on seg_DC, D→A on seg_AD
                              side_segments = { seg_AB, seg_BC, seg_DC, seg_AD }
                            else
                              corners = { A, D, C, B }
                              -- A→D on seg_AD, D→C on seg_DC, C→B on seg_BC, B→A on seg_AB
                              side_segments = { seg_AD, seg_DC, seg_BC, seg_AB }
                            end
                            -- bbox
                            local x1, y1 = pxs[corners[1]], pys[corners[1]]
                            local x2, y2 = pxs[corners[2]], pys[corners[2]]
                            local x3, y3 = pxs[corners[3]], pys[corners[3]]
                            local x4, y4 = pxs[corners[4]], pys[corners[4]]
                            local bxmin, bxmax = x1, x1
                            if x2 < bxmin then bxmin = x2 end; if x2 > bxmax then bxmax = x2 end
                            if x3 < bxmin then bxmin = x3 end; if x3 > bxmax then bxmax = x3 end
                            if x4 < bxmin then bxmin = x4 end; if x4 > bxmax then bxmax = x4 end
                            local bymin, bymax = y1, y1
                            if y2 < bymin then bymin = y2 end; if y2 > bymax then bymax = y2 end
                            if y3 < bymin then bymin = y3 end; if y3 > bymax then bymax = y3 end
                            if y4 < bymin then bymin = y4 end; if y4 > bymax then bymax = y4 end
                            -- phase
                            local ax = x2 - x1
                            local ay = y2 - y1
                            local bx = x4 - x1
                            local by = y4 - y1
                            local eps_h = 1e-6
                            local a_h = mabs(ay) < eps_h
                            local a_v = mabs(ax) < eps_h
                            local b_h = mabs(by) < eps_h
                            local b_v = mabs(bx) < eps_h
                            local has_h = a_h or b_h
                            local has_v = a_v or b_v
                            local phase
                            if has_h and has_v then phase = 0
                            elseif has_h then phase = 1
                            elseif has_v then phase = 2
                            else phase = 3 end
                            local area = mabs(ax * by - ay * bx)
                            candidates[#candidates + 1] = {
                              corners = corners,
                              side_segments = side_segments,
                              area = area,
                              phase = phase,
                              bxmin = bxmin, bxmax = bxmax,
                              bymin = bymin, bymax = bymax,
                            }
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
  stats.t_parls.stop()
  stats.candidate_count = #candidates

  -- ---- phase loop / acceptance ----
  stats.t_accept = game.create_profiler()
  local phases = { {}, {}, {}, {} }
  for _, p in ipairs(candidates) do
    local ph = p.phase + 1
    phases[ph][#phases[ph] + 1] = p
  end
  for ph = 1, 4 do
    table.sort(phases[ph], function(a, b) return a.area < b.area end)
  end
  local accepted = {}
  for ph = 1, 4 do
    for _, P in ipairs(phases[ph]) do
      local conflict = false
      local p_bxmin, p_bxmax = P.bxmin, P.bxmax
      local p_bymin, p_bymax = P.bymin, P.bymax
      for _, Q in ipairs(accepted) do
        if p_bxmin <= Q.bxmax and p_bxmax >= Q.bxmin
           and p_bymin <= Q.bymax and p_bymax >= Q.bymin then
          if quads_overlap(P.corners, Q.corners, pxs, pys) then
            conflict = true
            break
          end
        end
      end
      if not conflict then
        accepted[#accepted + 1] = P
      end
    end
  end
  stats.t_accept.stop()
  stats.accepted_count = #accepted

  -- ---- edge generation from accepted parl sides ----
  stats.t_edges = game.create_profiler()
  local edges = {}
  local edge_set = {}
  local edge_segs = {}
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
  local function add_edge(a, b)
    local k = a < b and (a .. "_" .. b) or (b .. "_" .. a)
    if edge_set[k] then return false end
    edge_set[k] = true
    edges[#edges + 1] = { a, b }
    edge_segs[#edge_segs + 1] = { pxs[a], pys[a], pxs[b], pys[b] }
    local ra, rb = find(a), find(b)
    if ra ~= rb then parent[ra] = rb end
    return true
  end

  for _, P in ipairs(accepted) do
    for k = 1, 4 do
      local a = P.corners[k]
      local b = P.corners[(k % 4) + 1]
      local seg_idx = P.side_segments[k]
      local seg = segments[seg_idx]
      local pa = pole_to_pos[a][seg_idx]
      local pb = pole_to_pos[b][seg_idx]
      if pa > pb then pa, pb = pb, pa end
      for q = pa, pb - 1 do
        add_edge(seg.poles[q], seg.poles[q + 1])
      end
    end
  end
  stats.t_edges.stop()
  stats.edges_from_parls = #edges

  -- ---- MST closure for connectivity ----
  stats.t_mst = game.create_profiler()
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
                local dx = pj.x - pi.x
                local dy = pj.y - pi.y
                local d2 = dx * dx + dy * dy
                local rj = reaches[j]
                local r = ri < rj and ri or rj
                if d2 <= r * r then
                  mst_candidates[#mst_candidates + 1] =
                    { d2 = d2, i = i, j = j }
                end
              end
            end
          end
        end
      end
    end
  end
  table.sort(mst_candidates, function(a, b) return a.d2 < b.d2 end)

  for _, e in ipairs(mst_candidates) do
    if find(e.i) ~= find(e.j) then
      local k = e.i < e.j and (e.i .. "_" .. e.j) or (e.j .. "_" .. e.i)
      if not edge_set[k] then
        local nx1, ny1 = pxs[e.i], pys[e.i]
        local nx2, ny2 = pxs[e.j], pys[e.j]
        local crosses = false
        for _, old in ipairs(edge_segs) do
          if segs_intersect(nx1, ny1, nx2, ny2,
                            old[1], old[2], old[3], old[4]) then
            crosses = true; break
          end
        end
        if not crosses then add_edge(e.i, e.j) end
      end
    end
  end
  stats.t_mst.stop()
  stats.total_edges = #edges

  return edges, accepted, candidates, stats
end

return M
