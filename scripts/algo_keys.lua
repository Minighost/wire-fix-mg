local M = {}

M.DEFAULT_MAX_GRID = 128

function M.build_scales(max_grid)
  local scales = {}
  local s = 32
  while s <= max_grid do
    scales[#scales + 1] = s
    s = s * 2
  end
  if #scales == 0 then scales[1] = 32 end
  return scales
end

function M.edge_key(i, j)
  if i < j then return i .. "_" .. j
  else return j .. "_" .. i
  end
end

function M.tri_key(a, b, c)
  local s = { a, b, c }
  table.sort(s)
  return s[1] .. "_" .. s[2] .. "_" .. s[3]
end

function M.quad_key(corners)
  local s = { corners[1], corners[2], corners[3], corners[4] }
  table.sort(s)
  return s[1] .. "_" .. s[2] .. "_" .. s[3] .. "_" .. s[4]
end

return M
