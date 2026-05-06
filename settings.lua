data:extend{
  {
    type = "string-setting",
    name = "wire-fix-max-grid",
    setting_type = "runtime-per-user",
    default_value = "128",
    allowed_values = {"32", "64", "128", "256", "512", "1024", "2048", "4096", "8192"},
    order = "a",
  },
  {
    type = "bool-setting",
    name = "wire-fix-include-ghosts",
    setting_type = "runtime-per-user",
    default_value = true,
    order = "b",
  },
}
