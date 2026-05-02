-- Test-only module: detect maximal collinear segments among the pole set
-- so we can measure whether the segment-based approach is fast enough to
-- justify replacing v18's midpoint-pair enumeration.
--
-- Algorithm:
--   1. For each pole i, query the spatial bucket for poles j within reach.
--   2. For each (i, j) pair within reach, compute a canonical line key
--      (quantized angle modulo π, quantized perpendicular offset rho) and
--      add both poles to that line's pole set.
--   3. For each line, sort the poles along the line direction and split
--      into maximal segments where consecutive poles fit in pair_reach.
--
-- This module is intentionally standalone — no integration with v18, no
-- candidate-parl construction. Just enough to time step (1) + (2) + (3)
-- and see if it's competitive with v18's 3 s enumerate.

local M = {}

local mfloor = math.floor
local matan2 = math.atan2
local msqrt = math.sqrt
local mpi = math.pi

local ANGLE_SCALE = 10000  -- ~0.0057 degree precision (1 / 10000 rad)
local RHO_SCALE = 10000    -- 1/10000 unit precision

function M.detect_segments(points, reaches)
  local n = #points
  local stats = {}

  stats.t_setup = game.create_profiler()
  -- Spatial bucket (cell_size = max reach so any reachable pair lies
  -- within the box of cells around either endpoint).
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
  -- Step 1+2: walk reachable pairs, group into line buckets.
  -- Each line = (quantized angle mod π, quantized perpendicular offset rho).
  -- Two-level table keyed by angle_q then rho_q.
  local lines = {}
  local pair_count = 0
  for i = 1, n do
    local pix = pxs[i]
    local piy = pys[i]
    local ri = reaches[i]
    local cx_min = mfloor((pix - ri) / cell_size)
    local cx_max = mfloor((pix + ri) / cell_size)
    local cy_min = mfloor((piy - ri) / cell_size)
    local cy_max = mfloor((piy + ri) / cell_size)
    for cx = cx_min, cx_max do
      local row = point_bucket[cx]
      if row then
        for cy = cy_min, cy_max do
          local cell = row[cy]
          if cell then
            for ki = 1, #cell do
              local j = cell[ki]
              if j > i then
                local pjx = pxs[j]
                local pjy = pys[j]
                local dx = pjx - pix
                local dy = pjy - piy
                local d2 = dx * dx + dy * dy
                local rj = reaches[j]
                local r = ri < rj and ri or rj
                if d2 <= r * r then
                  pair_count = pair_count + 1
                  -- Canonical angle in [0, π).
                  local angle = matan2(dy, dx)
                  if angle < 0 then angle = angle + mpi end
                  if angle >= mpi then angle = angle - mpi end
                  local L = msqrt(d2)
                  -- Perpendicular offset (signed distance from origin to line).
                  local rho = (pix * (-dy) + piy * dx) / L
                  local angle_q = mfloor(angle * ANGLE_SCALE + 0.5)
                  local rho_q
                  if rho >= 0 then rho_q = mfloor(rho * RHO_SCALE + 0.5)
                  else rho_q = -mfloor(-rho * RHO_SCALE + 0.5) end
                  local row_a = lines[angle_q]
                  if not row_a then row_a = {}; lines[angle_q] = row_a end
                  local line = row_a[rho_q]
                  if not line then
                    line = { poles = {}, dx = dx, dy = dy, ax = pix, ay = piy }
                    row_a[rho_q] = line
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
  -- Step 3: for each line, sort poles along direction and split into
  -- maximal segments (consecutive poles within pair_reach).
  local segments = {}
  local line_count = 0
  local max_seg_len = 0
  local total_segment_poles = 0
  for _, row_a in pairs(lines) do
    for _, line in pairs(row_a) do
      line_count = line_count + 1
      local poles_list = {}
      for p in pairs(line.poles) do poles_list[#poles_list + 1] = p end
      if #poles_list >= 2 then
        local dx = line.dx
        local dy = line.dy
        local ax = line.ax
        local ay = line.ay
        table.sort(poles_list, function(a, b)
          local sa = (pxs[a] - ax) * dx + (pys[a] - ay) * dy
          local sb = (pxs[b] - ax) * dx + (pys[b] - ay) * dy
          return sa < sb
        end)
        local segment = { poles_list[1] }
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
            segment[#segment + 1] = curr
          else
            if #segment >= 2 then
              segments[#segments + 1] = segment
              total_segment_poles = total_segment_poles + #segment
              if #segment > max_seg_len then max_seg_len = #segment end
            end
            segment = { curr }
          end
        end
        if #segment >= 2 then
          segments[#segments + 1] = segment
          total_segment_poles = total_segment_poles + #segment
          if #segment > max_seg_len then max_seg_len = #segment end
        end
      end
    end
  end
  stats.t_segments.stop()

  stats.line_count = line_count
  stats.segment_count = #segments
  stats.max_seg_len = max_seg_len
  stats.total_segment_poles = total_segment_poles

  return segments, stats
end

return M
