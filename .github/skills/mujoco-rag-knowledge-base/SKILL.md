---
name: mujoco-rag-knowledge-base
description: >-
  Guide for researching the MuJoCo physics simulation library source code.
  Use this skill when asked any question about MuJoCo functions, structs,
  rendering, physics, UI, XML model loading, or the Python bindings. This
  skill provides step-by-step instructions for querying a local RAG knowledge
  base via the mujoco-knowledge MCP server. Do not read MuJoCo source files
  directly; use the MCP tools described below instead.
---

To research a question about the MuJoCo codebase, follow this process using
tools provided by the `mujoco-knowledge` MCP server.

## Step 1 — Choose the right tool for the question

| Question type | Start with | Escalate to |
|---------------|-----------|-------------|
| "What does this file do?" | `search_summaries` | `search_tiered` |
| "How do I call function X?" | `search_tiered` (default) | `get_symbol_card_tool` |
| "What fields does struct X have?" | `search_cards` with `kind="struct"` | `search_knowledge` |
| "Show all functions related to X" | `search_cards` with `kind="function"` | — |
| "How does subsystem X work?" | `search_tiered` with `detail="full"` | `search_knowledge` |
| "Give me the exact source code" | `search_tiered` with `detail="full"` | `get_evidence` |

**Default starting call for any MuJoCo question:**

```
search_tiered(query="<your question>", detail="cards")
```

This returns file-level summaries (L1) + symbol signatures with docstrings (L2)
in one fast call without loading raw source. Only escalate to `detail="full"` if
you need verbatim code or line numbers.

---

## Step 2 — Formulate the query

- Use exact C symbol names when known: `mjv_initGeom`, `mjrContext`, `mjvScene`
- Use descriptive phrases for concept questions: `"rendering pipeline for visualization geoms"`
- Ask one concept per call — split compound questions into multiple tool calls
- MuJoCo symbol prefix conventions:

| Prefix | Subsystem |
|--------|-----------|
| `mj_` | Core simulation API (`mj_step`, `mj_forward`, `mj_loadXML`) |
| `mjv_` | Visualization (`mjv_initGeom`, `mjv_makeScene`, `mjv_updateScene`) |
| `mjr_` | Rendering/OpenGL (`mjr_render`, `mjr_makeContext`, `mjr_readPixels`) |
| `mju_` | Utilities (`mju_error`, `mju_warning`, `mju_str2num`) |
| `mjt_` | Types and enums (`mjtGeom`, `mjtObj`, `mjtSolver`) |
| `mjui_` | UI widgets (`mjui_update`, `mjui_render`, `mjui_add`) |
| `mjs_` | Spec/build API (`mjs_addBody`, `mjs_compile`) |

---

## Step 3 — Call the tools

### `search_tiered` — all-purpose tiered search

```
search_tiered(query, detail="cards", top_k=10)
```

- `detail="summary"` — file paths + short summaries only (fastest)
- `detail="cards"` — files + symbol signatures + docstrings (default, use this first)
- `detail="full"` — files + symbols + verbatim code chunks (use when code is needed)

Returns `l1_summaries` (file paths and summaries), `l2_cards` (symbol name,
kind, file, line, signature, docstring), and `l3_chunks` (raw code with line numbers).

### `search_summaries` — file-level orientation

```
search_summaries(query, top_k=10)
```

Returns `[{"file_path": "...", "summary_text": "..."}]`. Use to identify
which files are relevant before going deeper.

### `search_cards` — symbol lookup by name or kind

```
search_cards(query, file_path="", kind="", top_k=10)
```

- `kind`: `"function"`, `"struct"`, `"class"`, `"typedef"`, `"enum"`
- `file_path`: substring filter (e.g. `"engine_vis"` matches any path containing it)

Returns symbol signatures and extracted docstrings — no raw code.

### `search_knowledge` — direct hybrid chunk search

```
search_knowledge(query, top_k=10, mode="hybrid")
```

- `mode`: `"hybrid"` (BM25 + vector, default), `"lexical"` (keyword only), `"vector"` (semantic only)

Use when `search_tiered(detail="full")` doesn't return enough detail.

### `get_symbol_card_tool` — full card for one symbol

```
get_symbol_card_tool(symbol_id)
```

Use when you have a `symbol_id` from a prior search result and need its full
details: signature, docstring, start/end line, parent symbol, child count.

### `get_evidence` — verbatim source lines for one chunk

```
get_evidence(chunk_id)
```

Use when you have a `chunk_id` from a prior search result and need the exact
raw source text with file path and line numbers.

---

## Step 4 — Interpret the results

- **`file_path`**: relative path within the MuJoCo repository
- **`start_line` / `end_line`**: 1-indexed source line range
- **`commit_sha`**: Git commit of the indexed snapshot — always cite this when making factual claims
- **`inclusion_reason`**: `"fts"` = keyword match, `"vector"` = semantic match, `"neighbor_expand"` = structurally adjacent code
- **`summary_text`**: format is `{filename}: {first lines of file} | Symbols: {top-10 symbol names}`
- **`docstring`**: the comment block immediately above the symbol definition in source

---

## Step 5 — Cite your sources

Always include `file_path`, `start_line`, and `commit_sha` when stating facts
about MuJoCo behavior or API contracts. Use `propose_fact_tool` to record
verified, evidence-backed findings.

```
propose_fact_tool(
    fact_type="api_contract",
    subject_key="mjv_initGeom",
    statement="...",
    confidence=0.95,
    impact_level="medium",
    evidence_chunk_ids=["<chunk_id from search result>"],
    created_by="agent"
)
```

Only call `propose_fact_tool` with real `evidence_chunk_ids` returned by a
previous tool call. Never fabricate chunk IDs.

---

## Key subsystem reference

| Subsystem | Primary files | Key symbols |
|-----------|--------------|-------------|
| Visualization | `src/engine/engine_vis_visualize.c`, `include/mujoco/mjvisualize.h` | `mjvGeom`, `mjvScene`, `mjv_initGeom`, `mjv_makeScene`, `mjv_updateScene` |
| Rendering | `src/render/render_context.c`, `include/mujoco/mjrender.h` | `mjrContext`, `mjr_makeContext`, `mjr_render`, `mjr_readPixels` |
| Physics core | `src/engine/engine_core_smooth.c` | `mj_step`, `mj_forward`, `mj_kinematics`, `mj_collision` |
| XML / model loading | `src/xml/xml_native_reader.cc` | `mj_loadXML`, `mj_saveLastXML`, `mjSpec` |
| Simulation viewer | `simulate/simulate.cc`, `simulate/simulate.h` | `Simulate`, `user_scn`, `UiEvent` |
| UI widgets | `simulate/uitools.cc`, `include/mujoco/mjui.h` | `mjUI`, `mjuiDef`, `mjui_add`, `mjui_render` |
| Collision | `src/engine/engine_collision_driver.c` | `mj_collision`, `mjContact`, `mj_contactForce` |
| Python bindings | `python/mujoco/` | `MjModel`, `MjData`, `Renderer` |

---

## Rules

- **Do not** read MuJoCo source files directly — always use the MCP tools above
- **Do not** fabricate symbol names, file paths, line numbers, or chunk IDs
- **Do not** claim certainty about behavior that is not in a retrieved chunk
- **Do not** assume the Python API at `python/mujoco/` mirrors the C API — verify separately
- **Do** cite `file_path`, `start_line`, and `commit_sha` for every factual claim
- **Do** start with `search_tiered(detail="cards")` before escalating to `detail="full"`
