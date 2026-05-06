local M = {}

M.EPS = 1e-9
M.ROUND_SCALE = 10000

local mfloor = math.floor
local msqrt = math.sqrt
local ROUND_SCALE = M.ROUND_SCALE
local EPS = M.EPS

function M.vec_round_xy(x, y)
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

function M.cross_2d(ax, ay, bx, by)
  return ax * by - ay * bx
end

function M.dist(ax, ay, bx, by)
  local dx = ax - bx
  local dy = ay - by
  return msqrt(dx * dx + dy * dy)
end

function M.segs_intersect(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y)
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

return M
