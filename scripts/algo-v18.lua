-- Pattern-aware wire layout: parallelogram detection + isosceles symmetry + MST.
-- Port of Python prototype algo_v18.py.
--
-- Differences from algo_v18.py:
--   - Dead code (split_parl, split_parl_to_data, point_in_quad_strict,
--     point_in_or_on_quad, points_in_or_on_quad) was not ported — those
--     functions are defined but never reached from detect_phased's flow.
--   - Indices are 1-based throughout (Lua convention).
--   - vec_round / edge_key / quad_key use string keys (Lua table lookup).
--
-- Public API:
--   M.compute(points, reaches) -> edges, parls, iso_tris
--     points     : 1-indexed array of {x = number, y = number}
--     reaches    : 1-indexed array of per-pole max wire distance.
--                  Pair-wise reach for any (i, j) is min(reaches[i], reaches[j]),
--                  matching how the Factorio engine validates a copper wire.
--     edges      : list of {i, j} (1-indexed pole indices)
--     parls      : list of accepted parallelograms (for visualization)
--     iso_tris   : list of accepted isosceles triangles {a, b, c}

local M = {}

local EPS = 1e-9
local EPS_GEOM = 1e-6
local ROUND_SCALE = 10000  -- 4-decimal precision

local function vec_round(x, y)
  local rx
  if x >= 0 then rx = math.floor(x * ROUND_SCALE + 0.5)
  else rx = -math.floor(-x * ROUND_SCALE + 0.5)
  end
  local ry
  if y >= 0 then ry = math.floor(y * ROUND_SCALE + 0.5)
  else ry = -math.floor(-y * ROUND_SCALE + 0.5)
  end
  return rx .. ":" .. ry
end

local function cross_2d(ax, ay, bx, by)
  return ax * by - ay * bx
end

local function dist(ax, ay, bx, by)
  local dx = ax - bx
  local dy = ay - by
  return math.sqrt(dx * dx + dy * dy)
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

function M.compute(points, reaches)
  local n = #points
  if n < 2 then return {}, {}, {} end

  -- pos_to_idx: round positions for symmetric-target lookup
  local pos_to_idx = {}
  for i = 1, n do
    pos_to_idx[vec_round(points[i].x, points[i].y)] = i
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
    local cx = math.floor(points[i].x / cell_size)
    local cy = math.floor(points[i].y / cell_size)
    local row = point_bucket[cx]
    if not row then row = {}; point_bucket[cx] = row end
    local cell = row[cy]
    if not cell then cell = {}; row[cy] = cell end
    cell[#cell + 1] = i
  end

  -- Returns points strictly between i and j on the segment (sorted by t).
  local function points_on_segment_strict(i, j)
    local pa, pb = points[i], points[j]
    local dx = pb.x - pa.x
    local dy = pb.y - pa.y
    local L2 = dx * dx + dy * dy
    if L2 < 1e-12 then return {} end
    local minx, maxx = pa.x, pb.x
    if minx > maxx then minx, maxx = maxx, minx end
    local miny, maxy = pa.y, pb.y
    if miny > maxy then miny, maxy = maxy, miny end
    local cx1 = math.floor(minx / cell_size)
    local cx2 = math.floor(maxx / cell_size)
    local cy1 = math.floor(miny / cell_size)
    local cy2 = math.floor(maxy / cell_size)
    local result = {}
    local seen = {}
    for cx = cx1, cx2 do
      local row = point_bucket[cx]
      if row then
        for cy = cy1, cy2 do
          local cell = row[cy]
          if cell then
            for _, k in ipairs(cell) do
              if k ~= i and k ~= j and not seen[k] then
                seen[k] = true
                local pk = points[k]
                local cv = (pk.x - pa.x) * dy - (pk.y - pa.y) * dx
                if math.abs(cv) <= EPS_GEOM then
                  local t = ((pk.x - pa.x) * dx + (pk.y - pa.y) * dy) / L2
                  if t > 1e-6 and t < 1 - 1e-6 then
                    result[#result + 1] = { t = t, k = k }
                  end
                end
              end
            end
          end
        end
      end
    end
    table.sort(result, function(a, b) return a.t < b.t end)
    return result
  end

  -- Chain i -> mids -> j; nil if any sub-segment exceeds its pair reach.
  local function make_chain(i, j)
    local mids = points_on_segment_strict(i, j)
    local chain = { i }
    for _, m in ipairs(mids) do chain[#chain + 1] = m.k end
    chain[#chain + 1] = j
    for ci = 1, #chain - 1 do
      local a = chain[ci]
      local b = chain[ci + 1]
      local pa = points[a]
      local pb = points[b]
      if dist(pa.x, pa.y, pb.x, pb.y) > pair_reach(a, b) + 1e-9 then
        return nil
      end
    end
    return chain
  end

  -- SAT-based parallelogram-on-parallelogram overlap test.
  -- Boundary touch (separation distance == 0) is treated as separated.
  local function quads_overlap(P_corners, Q_corners)
    local P_pts = {
      points[P_corners[1]], points[P_corners[2]],
      points[P_corners[3]], points[P_corners[4]],
    }
    local Q_pts = {
      points[Q_corners[1]], points[Q_corners[2]],
      points[Q_corners[3]], points[Q_corners[4]],
    }
    -- 4 axes total — 2 unique side-normals per parallelogram.
    local axes = {}
    for poly_idx = 1, 2 do
      local poly = (poly_idx == 1) and P_pts or Q_pts
      for i = 1, 2 do
        local pa = poly[i]
        local pb = poly[i + 1]
        axes[#axes + 1] = { x = pb.y - pa.y, y = -(pb.x - pa.x) }
      end
    end
    for _, axis in ipairs(axes) do
      local ax, ay = axis.x, axis.y
      if ax * ax + ay * ay >= 1e-18 then
        local p_min, p_max = math.huge, -math.huge
        for ci = 1, 4 do
          local pt = P_pts[ci]
          local proj = pt.x * ax + pt.y * ay
          if proj < p_min then p_min = proj end
          if proj > p_max then p_max = proj end
        end
        local q_min, q_max = math.huge, -math.huge
        for ci = 1, 4 do
          local pt = Q_pts[ci]
          local proj = pt.x * ax + pt.y * ay
          if proj < q_min then q_min = proj end
          if proj > q_max then q_max = proj end
        end
        if p_max < q_min + EPS or q_max < p_min + EPS then
          return false  -- separating axis found
        end
      end
    end
    return true
  end

  -- Enumerate parallelogram candidates from a set of point indices.
  -- Two pairs sharing a midpoint = diagonals of a parallelogram.
  -- Filter by side-angle in [60°, 120°] (4·dot² ≤ |a|²|b|²).
  local function enumerate_parls(point_indices_set)
    local pts_idx = {}
    for k in pairs(point_indices_set) do pts_idx[#pts_idx + 1] = k end
    table.sort(pts_idx)
    local m = #pts_idx
    if m < 4 then return {} end

    local mid_groups = {}
    for ii = 1, m do
      local i = pts_idx[ii]
      local pix, piy = points[i].x, points[i].y
      for jj = ii + 1, m do
        local j = pts_idx[jj]
        local mx = pix + points[j].x
        local my = piy + points[j].y
        local k = vec_round(mx, my)
        local g = mid_groups[k]
        if not g then g = {}; mid_groups[k] = g end
        g[#g + 1] = { i, j }
      end
    end

    local candidates = {}
    for _, grp in pairs(mid_groups) do
      local glen = #grp
      if glen >= 2 then
        for pi = 1, glen do
          local a, b = grp[pi][1], grp[pi][2]
          for pj = pi + 1, glen do
            local c, d = grp[pj][1], grp[pj][2]
            if a ~= c and a ~= d and b ~= c and b ~= d then
              local i, j, k = a, c, d  -- l = b
              local v_a_x = points[j].x - points[i].x
              local v_a_y = points[j].y - points[i].y
              local v_b_x = points[k].x - points[i].x
              local v_b_y = points[k].y - points[i].y
              local la2 = v_a_x * v_a_x + v_a_y * v_a_y
              local lb2 = v_b_x * v_b_x + v_b_y * v_b_y
              if la2 >= 1e-18 and lb2 >= 1e-18 then
                local dotv = v_a_x * v_b_x + v_a_y * v_b_y
                if 4 * dotv * dotv <= la2 * lb2 then
                  local corners_ccw
                  if cross_2d(v_a_x, v_a_y, v_b_x, v_b_y) > 0 then
                    corners_ccw = { i, j, b, k }
                  else
                    corners_ccw = { i, k, b, j }
                  end
                  local chains = {}
                  local ok = true
                  for ci = 1, 4 do
                    local ea = corners_ccw[ci]
                    local eb = corners_ccw[(ci % 4) + 1]
                    local ch = make_chain(ea, eb)
                    if ch == nil then ok = false; break end
                    chains[ci] = ch
                  end
                  if ok then
                    local p1 = points[corners_ccw[1]]
                    local p2 = points[corners_ccw[2]]
                    local p4 = points[corners_ccw[4]]
                    local va_x = p2.x - p1.x
                    local va_y = p2.y - p1.y
                    local vb_x = p4.x - p1.x
                    local vb_y = p4.y - p1.y
                    candidates[#candidates + 1] = {
                      corners = corners_ccw,
                      chains = chains,
                      area = math.abs(cross_2d(va_x, va_y, vb_x, vb_y)),
                    }
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
    local a_horiz = math.abs(v_a_y) < eps
    local a_vert  = math.abs(v_a_x) < eps
    local b_horiz = math.abs(v_b_y) < eps
    local b_vert  = math.abs(v_b_x) < eps
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
          local key = vec_round(cp.x + dir.x, cp.y + dir.y)
          local idx = pos_to_idx[key]
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
              local q1 = points[new_corners[1]]
              local q2 = points[new_corners[2]]
              local q4 = points[new_corners[4]]
              local nva_x = q2.x - q1.x
              local nva_y = q2.y - q1.y
              local nvb_x = q4.x - q1.x
              local nvb_y = q4.y - q1.y
              local new_P = {
                corners = new_corners,
                chains = chains,
                area = math.abs(cross_2d(nva_x, nva_y, nvb_x, nvb_y)),
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

  -- If short diagonal == one of the sides, split P into two isosceles
  -- triangles. For each triangle, try point-symmetric reflection across
  -- each outer side to find a sibling triangle.
  local function try_isosceles_split(P)
    local c0, c1, c2, c3 = P.corners[1], P.corners[2], P.corners[3], P.corners[4]
    local p0 = points[c0]
    local p1 = points[c1]
    local p2 = points[c2]
    local p3 = points[c3]
    local la = dist(p0.x, p0.y, p1.x, p1.y)
    local lb = dist(p0.x, p0.y, p3.x, p3.y)
    local d_diag1 = dist(p0.x, p0.y, p2.x, p2.y)
    local d_diag2 = dist(p1.x, p1.y, p3.x, p3.y)
    local short_a, short_b, diag_len
    if d_diag1 <= d_diag2 then
      short_a, short_b, diag_len = c0, c2, d_diag1
    else
      short_a, short_b, diag_len = c1, c3, d_diag2
    end
    if not (math.abs(diag_len - la) < 1e-4 or math.abs(diag_len - lb) < 1e-4) then
      return {}
    end

    local tri1, tri2
    if short_a == c0 then
      tri1 = { c0, c1, c2 }
      tri2 = { c0, c2, c3 }
    else
      tri1 = { c1, c2, c3 }
      tri2 = { c0, c1, c3 }
    end

    local result = { tri1, tri2 }
    local triangles = { tri1, tri2 }

    for _, T in ipairs(triangles) do
      local t1, t2, t3 = T[1], T[2], T[3]
      local q1, q2, q3 = points[t1], points[t2], points[t3]
      local d01 = dist(q1.x, q1.y, q2.x, q2.y)
      local d12 = dist(q2.x, q2.y, q3.x, q3.y)
      local d02 = dist(q1.x, q1.y, q3.x, q3.y)
      local iso_v, base_a, base_b
      if math.abs(d01 - d12) < 1e-4 then
        iso_v, base_a, base_b = t2, t1, t3
      elseif math.abs(d01 - d02) < 1e-4 then
        iso_v, base_a, base_b = t1, t2, t3
      elseif math.abs(d12 - d02) < 1e-4 then
        iso_v, base_a, base_b = t3, t1, t2
      end
      if iso_v then
        for pair_idx = 1, 2 do
          local out_v_a = iso_v
          local out_v_b, opp
          if pair_idx == 1 then out_v_b, opp = base_a, base_b
          else out_v_b, opp = base_b, base_a
          end
          local oa, ob, op = points[out_v_a], points[out_v_b], points[opp]
          local mx = oa.x + ob.x - op.x
          local my = oa.y + ob.y - op.y
          local key = vec_round(mx, my)
          local X = pos_to_idx[key]
          if X and X ~= out_v_a and X ~= out_v_b and X ~= opp then
            local px = points[X]
            local d_oa_X = dist(oa.x, oa.y, px.x, px.y)
            local d_ob_X = dist(ob.x, ob.y, px.x, px.y)
            local d_oa_ob = dist(oa.x, oa.y, ob.x, ob.y)
            if d_oa_X <= pair_reach(out_v_a, X)
               and d_ob_X <= pair_reach(out_v_b, X)
               and d_oa_ob <= pair_reach(out_v_a, out_v_b) then
              result[#result + 1] = { out_v_a, out_v_b, X }
            end
          end
        end
      end
    end
    return result
  end

  local accepted_parls = {}
  local accepted_iso_tris = {}
  local seen_iso = {}

  -- Phase-ordered detection: rectangles → axis-aligned → diagonal.
  -- Within each phase, area asc; primary acceptance requires no overlap
  -- with prior primaries OR expansions; expansion is cascading and skips
  -- overlap check (only deduped by quad_key).
  local function detect_phased(point_indices_set)
    local candidates = enumerate_parls(point_indices_set)
    if #candidates == 0 then return end

    local phases = { {}, {}, {}, {} }
    for _, P in ipairs(candidates) do
      local ph = parl_phase(P) + 1
      phases[ph][#phases[ph] + 1] = P
    end
    for ph = 1, 4 do
      table.sort(phases[ph], function(a, b) return a.area < b.area end)
    end

    local primary_parls = {}
    local expansion_parls = {}
    local all_seen = {}

    for ph = 1, 4 do
      for _, P in ipairs(phases[ph]) do
        local conflict = false
        for _, Q in ipairs(primary_parls) do
          if quads_overlap(P.corners, Q.corners) then conflict = true; break end
        end
        if not conflict then
          for _, Q in ipairs(expansion_parls) do
            if quads_overlap(P.corners, Q.corners) then conflict = true; break end
          end
        end
        if not conflict then
          primary_parls[#primary_parls + 1] = P
          all_seen[quad_key(P.corners)] = true
          for _, E in ipairs(expand_parl(P)) do
            local k = quad_key(E.corners)
            if not all_seen[k] then
              all_seen[k] = true
              expansion_parls[#expansion_parls + 1] = E
            end
          end
        end
      end
    end

    for _, P in ipairs(primary_parls) do
      accepted_parls[#accepted_parls + 1] = P
    end
    for _, P in ipairs(expansion_parls) do
      accepted_parls[#accepted_parls + 1] = P
    end

    for _, P in ipairs(primary_parls) do
      for _, T in ipairs(try_isosceles_split(P)) do
        local k = tri_key(T[1], T[2], T[3])
        if not seen_iso[k] then
          seen_iso[k] = true
          accepted_iso_tris[#accepted_iso_tris + 1] = T
        end
      end
    end
    for _, P in ipairs(expansion_parls) do
      for _, T in ipairs(try_isosceles_split(P)) do
        local k = tri_key(T[1], T[2], T[3])
        if not seen_iso[k] then
          seen_iso[k] = true
          accepted_iso_tris[#accepted_iso_tris + 1] = T
        end
      end
    end
  end

  local all_set = {}
  for i = 1, n do all_set[i] = true end
  detect_phased(all_set)

  -- =========================================================
  -- Phase B: collect edges + Kruskal MST closure (with intersection check)
  -- =========================================================

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
  local function union(a, b)
    local ra, rb = find(a), find(b)
    if ra ~= rb then parent[ra] = rb; return true end
    return false
  end

  local function add_edge_unchecked(a, b)
    local k = edge_key(a, b)
    if edge_set[k] then return end
    edge_set[k] = true
    edges[#edges + 1] = { a, b }
    local pa, pb = points[a], points[b]
    edge_segs[#edge_segs + 1] = { pa.x, pa.y, pb.x, pb.y }
    union(a, b)
  end

  local function add_edge_checked(a, b)
    local k = edge_key(a, b)
    if edge_set[k] then return false end
    local pa, pb = points[a], points[b]
    for _, old in ipairs(edge_segs) do
      if segs_intersect(pa.x, pa.y, pb.x, pb.y,
                        old[1], old[2], old[3], old[4]) then
        return false
      end
    end
    edge_set[k] = true
    edges[#edges + 1] = { a, b }
    edge_segs[#edge_segs + 1] = { pa.x, pa.y, pb.x, pb.y }
    union(a, b)
    return true
  end

  -- Parallelogram chain edges (no cross check — accepted P's are non-overlapping
  -- by SAT, so chain edges don't cross each other).
  for _, P in ipairs(accepted_parls) do
    for _, chain in ipairs(P.chains) do
      for ci = 1, #chain - 1 do
        add_edge_unchecked(chain[ci], chain[ci + 1])
      end
    end
  end

  -- Isosceles triangle edges (with cross check against existing edges).
  for _, T in ipairs(accepted_iso_tris) do
    local a, b, c = T[1], T[2], T[3]
    add_edge_checked(a, b)
    add_edge_checked(b, c)
    add_edge_checked(a, c)
  end

  -- MST closure: connect remaining components by Kruskal, shortest first.
  local mst_candidates = {}
  for i = 1, n do
    local pi = points[i]
    for j = i + 1, n do
      local pj = points[j]
      local d = dist(pi.x, pi.y, pj.x, pj.y)
      if d <= pair_reach(i, j) then
        mst_candidates[#mst_candidates + 1] = { d = d, i = i, j = j }
      end
    end
  end
  table.sort(mst_candidates, function(a, b) return a.d < b.d end)

  for _, e in ipairs(mst_candidates) do
    if find(e.i) ~= find(e.j) then
      add_edge_checked(e.i, e.j)
    end
  end

  -- Connectivity safety net: if components remain separated because every
  -- bridging candidate crossed a pattern edge, fall back to shortest-first
  -- without the intersection check. Electrical connectivity beats avoiding
  -- a wire crossing.
  for _, e in ipairs(mst_candidates) do
    if find(e.i) ~= find(e.j) then
      add_edge_unchecked(e.i, e.j)
    end
  end

  return edges, accepted_parls, accepted_iso_tris
end

return M
