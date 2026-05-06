local M = {}

function M.find(uf, i)
  while uf[i] ~= i do
    uf[i] = uf[uf[i]]
    i = uf[i]
  end
  return i
end

function M.union(uf, a, b)
  local ra, rb = M.find(uf, a), M.find(uf, b)
  if ra ~= rb then uf[ra] = rb end
end

return M
