local wire_fix = require("scripts.wire-fix")

local TOOL_NAME = "wire-fix-tool"

script.on_event(defines.events.on_player_selected_area, function(event)
  if event.item ~= TOOL_NAME then return end
  wire_fix.run(event.player_index, event.entities, {cross_network = false})
end)

script.on_event(defines.events.on_player_alt_selected_area, function(event)
  if event.item ~= TOOL_NAME then return end
  wire_fix.run(event.player_index, event.entities, {cross_network = true})
end)

script.on_event(defines.events.on_player_reverse_selected_area, function(event)
  if event.item ~= TOOL_NAME then return end
  wire_fix.disconnect_all(event.player_index, event.entities)
end)
