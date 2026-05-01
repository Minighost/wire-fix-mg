# Wire Fix

A small Factorio 2.0 utility mod that tidies copper wire connections between electric poles in a selected area.

After hours of building and rebuilding, copper wires between your electric poles often end up tangled — long crossings, redundant links, or poles wired to the wrong neighbors. **Wire Fix** rebuilds them based on distance, while preserving every connection that leaves your selection.

## Features

- **Distance-based rebuild** — picks the shortest valid wires, expands evenly along consistent axes, and keeps the network connected.
- **Preserves external connections** — wires going to poles outside your selection are never touched.
- **Per-pole connection cap** — limit how many copper wires each pole keeps (default 4, configurable 1–8).
- **Reverse-select to disconnect** — right-click drag to remove all internal copper wires in the selection.
- **Multiplayer safe** — deterministic algorithm, no global state.
- **Localized** — 26 languages: Czech, Danish, Dutch, English, Finnish, French, German, Greek, Hungarian, Italian, Japanese, Korean, Norwegian, Polish, Portuguese (BR/PT), Romanian, Russian, Simplified Chinese, Spanish (ES/MX), Swedish, Traditional Chinese, Turkish, Ukrainian, Vietnamese.

## Usage

1. Press **ALT + W** (or click the shortcut bar icon) to get the Wire Fix tool in your cursor.
2. **Left-click and drag** over an area containing electric poles to rebuild the copper wires inside that selection.
3. **Right-click and drag** to disconnect all copper wires between poles in the selection.

A small flying-text message reports how many poles were processed and how many wires were placed.

## Settings

**Max connections per pole** (per-player runtime setting, default `4`)
Each pole keeps at most this many copper wire connections after cleanup. Connections leaving the selection count against this cap, so a pole already saturated by external wires won't get internal ones added.

## How it works

Wire Fix solves a constrained graph problem on the poles inside your selection:

1. **Classify** existing copper connections as internal (both endpoints in selection) or external (preserved as-is).
2. **Build candidate edges** between every pair of poles within reach, using a spatial hash so it stays fast on large selections.
3. **Kruskal MST** by distance — the shortest set of wires that keeps the network connected, respecting per-pole slot caps.
4. **Cascading axis-aware densification** — propagates wires of the same distance and angle along the MST so parallel rows of poles get parallel wires, not zig-zag patterns.
5. **Diff-apply** — only the wires that actually changed are added or removed via the API. Existing correct wires stay untouched.

The result is stable, looks tidy, and works on selections from a handful of poles to the thousands.

## Compatibility

- **Factorio**: 2.0+
- **Dependencies**: `base` only
- Works with any mod that adds electric poles (Bob's, Krastorio 2, etc.) — uses each pole's own max wire distance.
- No prototype changes; safe to add to or remove from existing saves.

## License

MIT — see [LICENSE](LICENSE).

## Source

Issues and contributions welcome.
