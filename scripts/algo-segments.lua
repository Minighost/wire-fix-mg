-- Test-only module: detect maximal collinear segments among the pole set.
--
-- Line detection uses exact integer arithmetic (no angle/offset quantization).
-- Pole positions are cast to fixed-point integers (× ROUND_SCALE), pair
-- direction is reduced via gcd to (ndx, ndy) with canonical sign, and the
-- line offset c = ndy·x_anchor − ndx·y_anchor is computed in integers. Two
-- pairs lie on exactly the same line iff their (ndx, ndy, c) triples are
-- identical. No tolerance, no merging of slightly-bent pairs.

local M = {}

local mfloor = math.floor

local ROUND_SCALE = 10000  -- 4-decimal precision; positions become exact ints

local function int_coord(x)
  if x >= 0 then return mfloor(x * ROUND_SCALE + 0.5)
  else return -mfloor(-x * ROUND_SCALE + 0.5) end
end

function M.detect_segments(points, reaches)
  local n = #points
  local stats = {}

  stats.t_setup = game.create_profiler()
  local max_reach = 0
  for i = 1, n do
    if reaches[i] > max_reach then max_reach = reaches[i] end
  end
  local cell_size = max_reach
  -- Float coords for spatial bucketing / reach checks.
  local pxs, pys = {}, {}
  for i = 1, n do
    pxs[i] = points[i].x
    pys[i] = points[i].y
  end
  -- Integer coords for exact line keying / sorting.
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
  -- 3-level table keyed on (ndx, ndy, c). Same line ⇔ same triple.
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

                  -- Exact integer line key. idx, idy are integer differences.
                  local idx = pixs[j] - pix_i
                  local idy = piys[j] - piy_i
                  -- gcd(|idx|, |idy|) via Euclid; both nonzero unless poles
                  -- coincide (which they don't for distinct i, j).
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
                  -- Canonical sign: ndx > 0, or (ndx == 0 ⇒ ndy > 0).
                  if ndx < 0 or (ndx == 0 and ndy < 0) then
                    ndx = -ndx
                    ndy = -ndy
                  end
                  -- c = ndy · x_i − ndx · y_i (constant along the line).
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
  -- For each line, sort poles by exact integer position along direction
  -- (ndx, ndy) and split into maximal segments by pair_reach gaps.
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
          table.sort(poles_list, function(a, b)
            local sa = (pixs[a] - ax) * ndx + (piys[a] - ay) * ndy
            local sb = (pixs[b] - ax) * ndx + (piys[b] - ay) * ndy
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
  end
  stats.t_segments.stop()

  stats.line_count = line_count
  stats.segment_count = #segments
  stats.max_seg_len = max_seg_len
  stats.total_segment_poles = total_segment_poles

  return segments, stats
end

return M
