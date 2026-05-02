-- Test-only module: segment-based parallelogram detection.
--
-- Stage 1 (detect_segments): exact integer line keying. Pole positions are
-- cast to fixed-point ints, pair direction is reduced via gcd to (ndx, ndy)
-- with canonical sign, and the line offset c = ndy·x_anchor − ndx·y_anchor
-- is computed in integers. Same line ⇔ same (ndx, ndy, c) triple.
--
-- Stage 2 (detect_parl_candidates): enumerate parallelogram candidates from
-- the segment graph. For each pole A, iterate pairs of segments through A
-- (different directions). For each (B in S1 after A) × (D in S2 after A),
-- compute the 4th corner C = B + D − A. Accept the (A, B, C, D) quadruple
-- iff:
--   - C exists as a real pole (pos_to_idx hit)
--   - B and C share a segment in S2's direction
--   - D and C share a segment in S1's direction
--   - The (A→B, A→D) angle is in [60°, 120°] (4·dot² ≤ |a|²·|b|²)
-- Each parl is enumerated 4 times (once per corner-as-A); dedupe by sorted
-- 4-tuple of pole indices.

local M = {}

local mfloor = math.floor
local mabs = math.abs

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
  local max_seg_len = 0
  local total_segment_poles = 0
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
                total_segment_poles = total_segment_poles + #current
                if #current > max_seg_len then max_seg_len = #current end
              end
              current = { curr }
            end
          end
          if #current >= 2 then
            segments[#segments + 1] = {
              poles = current, ndx = ndx, ndy = ndy,
            }
            total_segment_poles = total_segment_poles + #current
            if #current > max_seg_len then max_seg_len = #current end
          end
        end
      end
    end
  end
  stats.t_segments.stop()

  stats.line_count = line_count
  stats.segment_count = #segments
  stats.max_seg_len = max_seg_len
  stats.total_segment_poles = total_segment_poles

  return segments, pxs, pys, pixs, piys
end

function M.detect_segments(points, reaches)
  local n = #points
  local stats = {}
  detect_segments_impl(points, reaches, n, stats)
  return stats
end

-- ============================================================================
-- Stage 2: parl candidate enumeration
-- ============================================================================

function M.compute(points, reaches)
  local n = #points
  local stats = {}
  local segments, pxs, pys, pixs, piys =
    detect_segments_impl(points, reaches, n, stats)

  stats.t_index = game.create_profiler()
  -- pos_to_idx: integer (pix, piy) → pole index. 2-level table.
  local pos_to_idx = {}
  for i = 1, n do
    local row = pos_to_idx[pixs[i]]
    if not row then row = {}; pos_to_idx[pixs[i]] = row end
    row[piys[i]] = i
  end
  -- segs_by_dir[pole][dir_key] = seg_idx. dir_key packs (ndx, ndy) into a
  -- string; one segment per (pole, direction).
  -- pole_to_pos_in_seg[pole][seg_idx] = position index in segment.
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

  stats.t_parls = game.create_profiler()
  -- For each pole A, iterate pairs of segments through A (different directions),
  -- then iterate (B in S1 after A) × (D in S2 after A).
  -- Compute C = B + D − A, check it's a real pole on the right segments,
  -- check angle 60–120°, dedupe via sorted 4-tuple.
  local candidates = {}
  local seen_parls = {}  -- canonical "p1_p2_p3_p4" key → true
  local counter_triples = 0
  local counter_C_real = 0
  local counter_C_bc_ok = 0
  local counter_C_dc_ok = 0
  local counter_angle_ok = 0
  local counter_unique = 0

  for A = 1, n do
    local segs_A = segs_by_dir[A]
    if segs_A then
      -- Collect segments through A (with their direction keys).
      local through_A = {}  -- list of {seg_idx, dir_key}
      for dk, sidx in pairs(segs_A) do
        through_A[#through_A + 1] = { sidx = sidx, dk = dk }
      end
      local nA = #through_A
      for ii = 1, nA do
        local s1 = through_A[ii]
        local seg1 = segments[s1.sidx]
        local pos_A_in_s1 = pole_to_pos[A][s1.sidx]
        for jj = ii + 1, nA do
          local s2 = through_A[jj]
          local seg2 = segments[s2.sidx]
          local pos_A_in_s2 = pole_to_pos[A][s2.sidx]
          -- For each B in seg1 after A, each D in seg2 after A.
          local poles1 = seg1.poles
          local poles2 = seg2.poles
          for b_pos = pos_A_in_s1 + 1, #poles1 do
            local B = poles1[b_pos]
            for d_pos = pos_A_in_s2 + 1, #poles2 do
              local D = poles2[d_pos]
              counter_triples = counter_triples + 1
              -- Compute C = B + D − A in integer space.
              local cix = pixs[B] + pixs[D] - pixs[A]
              local ciy = piys[B] + piys[D] - piys[A]
              local row_C = pos_to_idx[cix]
              local C = row_C and row_C[ciy]
              if C and C ~= A and C ~= B and C ~= D then
                counter_C_real = counter_C_real + 1
                -- Check (B, C) on a segment in s2's direction.
                local seg_BC = segs_by_dir[B] and segs_by_dir[B][s2.dk]
                if seg_BC then
                  local pos_C_in_BC = pole_to_pos[C] and pole_to_pos[C][seg_BC]
                  if pos_C_in_BC then
                    counter_C_bc_ok = counter_C_bc_ok + 1
                    -- Check (D, C) on a segment in s1's direction.
                    local seg_DC = segs_by_dir[D] and segs_by_dir[D][s1.dk]
                    if seg_DC then
                      local pos_C_in_DC = pole_to_pos[C] and pole_to_pos[C][seg_DC]
                      if pos_C_in_DC then
                        counter_C_dc_ok = counter_C_dc_ok + 1
                        -- Angle filter (60°–120°): 4·dot² ≤ |a|²·|b|².
                        local v_ab_x = pxs[B] - pxs[A]
                        local v_ab_y = pys[B] - pys[A]
                        local v_ad_x = pxs[D] - pxs[A]
                        local v_ad_y = pys[D] - pys[A]
                        local la2 = v_ab_x * v_ab_x + v_ab_y * v_ab_y
                        local lb2 = v_ad_x * v_ad_x + v_ad_y * v_ad_y
                        local dotv = v_ab_x * v_ad_x + v_ab_y * v_ad_y
                        if 4 * dotv * dotv <= la2 * lb2 then
                          counter_angle_ok = counter_angle_ok + 1
                          -- Canonical 4-tuple key (sorted) to dedupe.
                          local s = { A, B, C, D }
                          table.sort(s)
                          local key = s[1] .. "_" .. s[2] .. "_" .. s[3] .. "_" .. s[4]
                          if not seen_parls[key] then
                            seen_parls[key] = true
                            counter_unique = counter_unique + 1
                            -- (For now, just count. Full corners CCW + phase
                            -- + bbox + acceptance comes in the next commit.)
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

  stats.triples = counter_triples
  stats.C_real = counter_C_real
  stats.C_bc_ok = counter_C_bc_ok
  stats.C_dc_ok = counter_C_dc_ok
  stats.angle_ok = counter_angle_ok
  stats.unique_parls = counter_unique

  return candidates, stats
end

return M
