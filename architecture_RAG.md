# Architecture for a Local-First RAG Knowledge System

This document defines the architecture for a **local-only knowledge system** that other AI agents can use to retrieve, verify, maintain, and retire facts about a codebase. It is written as an implementation specification for an AI coding agent, not as a research note.

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [User Constraints and Design Goals](#user-constraints-and-design-goals)
3. [Non-Goals](#non-goals)
4. [Recommended Technology Stack](#recommended-technology-stack)
5. [System Overview](#system-overview)
6. [Core Data Model](#core-data-model)
7. [Knowledge Layers](#knowledge-layers)
8. [Main Workflows](#main-workflows)
9. [Verification and Trust Model](#verification-and-trust-model)
10. [Archival and Deletion Policy](#archival-and-deletion-policy)
11. [Retrieval Architecture](#retrieval-architecture)
12. [MCP Interface for Other Agents](#mcp-interface-for-other-agents)
13. [Suggested Project Structure](#suggested-project-structure)
14. [Implementation Phases](#implementation-phases)
15. [Library and Repository Notes](#library-and-repository-notes)
16. [Open Decisions](#open-decisions)

## Executive Summary

Build a **local-first MCP server** backed by **SQLite** that stores codebase knowledge in multiple layers: routing metadata, short summaries, navigational summaries, contextualized chunks, and raw evidence. The system should support three first-class use cases from day one:

1. answer repository questions with citations
2. prepare grounded context for code changes and reviews
3. maintain a trusted, long-lived fact database for other local AI agents

The first implementation should **not** use a distributed architecture. Use **SQLite + FTS5 + sqlite-vec** for storage and search, **Tree-sitter** for structural parsing, **LlamaIndex** only for retrieval composition where helpful, and **DeepEval** or **Ragas** for regression-style evaluation.

## User Constraints and Design Goals

The architecture must satisfy these constraints:

- runs on **one local machine only**
- used by **the user and other AI agents running on the same machine**
- optimized for **personal use**, not multi-user cloud deployment
- knowledge must be:
  - loaded from the codebase
  - checked for correctness
  - marked active only after verification rules pass
  - archived or deleted when stale or wrong
- automated checks are preferred, but **human review is required for low-confidence or high-impact facts**
- implementation should reuse existing libraries and repositories rather than starting from scratch

The architecture must optimize for simplicity, observability, and correctness over scale.

## Non-Goals

The first version should explicitly avoid:

- cloud deployment
- distributed vector databases
- multi-machine synchronization
- GraphRAG-style graph indexing as a mandatory dependency
- full autonomous fact activation with no provenance
- large workflow orchestration systems

These can be added later only if the local-first version proves insufficient.

## Recommended Technology Stack

| Concern | Recommendation | Why |
|---|---|---|
| Agent protocol | `modelcontextprotocol/python-sdk` | Clean tool and resource interface for other agents |
| Primary database | SQLite | Single-file local database, durable, simple deployment |
| Lexical search | SQLite FTS5 | Exact identifiers, file names, symbols, macros, error strings |
| Vector search | `sqlite-vec` | Keeps vector search inside SQLite instead of adding another service |
| Structural parsing | Tree-sitter | Incremental, language-aware parsing with C-friendly runtime |
| Retrieval composition | LlamaIndex | Reuse coarse-to-fine retrieval helpers and AutoMergingRetriever ideas |
| Automated evaluation | DeepEval | Strong RAG, agent, and MCP evaluation support |
| Evaluation alternative | Ragas | Useful for retrieval metrics and dataset generation |
| Optional versioned DB upgrade | Dolt | Useful later if fact-level branching and diff become important |
| Optional local vector DB alternative | LanceDB or Qdrant local mode | Useful only if SQLite-based search becomes limiting |

## System Overview

```text
        Git repo checkout
              |
              v
      Change detection layer
      (commit SHA + file hashes)
              |
              v
      Structural extraction
      (Tree-sitter + file metadata)
              |
      +-------+--------+-------------------+
      |                |                   |
      v                v                   v
  symbol metadata   summaries        raw evidence chunks
  deps / parents    L1 / L2          L3 / L4
      |                |                   |
      +----------------+-------------------+
                       |
                       v
                 SQLite database
      +-----------------------------------------+
      | metadata tables                         |
      | facts + evidence                        |
      | review + verification state             |
      | archive history                         |
      | FTS5 lexical index                      |
      | sqlite-vec embeddings                   |
      +-----------------------------------------+
                       |
                       v
                  Local MCP server
      +-----------------------------------------+
      | search_knowledge                        |
      | get_symbol_card                         |
      | get_evidence                            |
      | propose_fact                            |
      | verify_pending                          |
      | archive_stale                           |
      | diff_knowledge                          |
      +-----------------------------------------+
                       |
                       v
                Other local AI agents
```

## Core Data Model

Use SQLite as the source of truth. Do not split structured data and vector data across different systems in v1.

### Required tables

#### `snapshots`

Tracks repository indexing checkpoints.

Suggested fields:

- `id`
- `repo_root`
- `branch_name`
- `commit_sha`
- `created_at`
- `indexer_version`
- `status`

#### `sources`

Tracks file-level source state.

Suggested fields:

- `id`
- `snapshot_id`
- `path`
- `language`
- `content_hash`
- `last_modified`
- `parse_status`
- `is_deleted`

#### `symbols`

Tracks code structure.

Suggested fields:

- `id`
- `source_id`
- `symbol_name`
- `kind`
- `qualified_name`
- `parent_symbol_id`
- `signature`
- `start_line`
- `end_line`
- `visibility`

#### `edges`

Tracks structural relationships.

Suggested fields:

- `id`
- `snapshot_id`
- `from_symbol_id`
- `to_symbol_id`
- `edge_type`

Example `edge_type` values:

- `imports`
- `calls`
- `inherits`
- `implements`
- `declared_in`
- `tested_by`
- `documents`

#### `chunks`

Stores retrieval units.

Suggested fields:

- `id`
- `snapshot_id`
- `source_id`
- `symbol_id`
- `chunk_type`
- `context_header`
- `raw_text`
- `raw_text_hash`
- `start_line`
- `end_line`
- `embedding`
- `fts_text`
- `status`

#### `facts`

Stores normalized knowledge statements.

Suggested fields:

- `id`
- `snapshot_id`
- `fact_type`
- `subject_key`
- `statement`
- `confidence`
- `status`
- `impact_level`
- `created_by`
- `created_at`
- `supersedes_fact_id`

Suggested `status` values:

- `pending`
- `active`
- `rejected`
- `superseded`
- `archived`
- `deleted`

#### `fact_evidence`

Maps facts to their supporting evidence.

Suggested fields:

- `fact_id`
- `chunk_id`
- `source_id`
- `symbol_id`
- `evidence_rank`

#### `reviews`

Stores automated and manual review outcomes.

Suggested fields:

- `id`
- `fact_id`
- `review_type`
- `reviewer`
- `result`
- `reason`
- `created_at`

Example `review_type` values:

- `auto_source_exists`
- `auto_hash_match`
- `auto_symbol_exists`
- `auto_duplicate_check`
- `auto_conflict_check`
- `manual_review`

## Knowledge Layers

The system should implement five knowledge layers.

### Layer 0: Routing metadata

Purpose:

- fast filtering
- branch and snapshot isolation
- symbol and dependency lookup
- freshness control

Examples:

- file path
- language
- commit SHA
- symbol kind
- parent-child relationships
- related tests

### Layer 1: Global summaries

Purpose:

- answer high-level questions
- route architecture and subsystem queries

Examples:

- one- or two-sentence summary per repository
- one- or two-sentence summary per subsystem

### Layer 2: Navigational summaries

Purpose:

- provide module, file, class, and function cards
- help agents find the right place before raw retrieval

Each card should capture:

- responsibility
- important APIs
- dependencies
- invariants or gotchas
- sample questions answered

### Layer 3: Contextualized chunks

Purpose:

- optimize search recall
- improve semantic and lexical retrieval

Each chunk should contain:

- structural boundary-respecting raw text
- a short generated context header
- exact line range
- source and symbol links

Chunking guidance:

- prefer function, method, class, file section, test block, or config block boundaries
- preserve imports, signatures, and docstrings where relevant
- avoid tiny arbitrary token windows

### Layer 4: Raw evidence

Purpose:

- provide exact support for final answers and facts
- support citations and verification

This layer must always preserve:

- source file path
- line range
- commit SHA
- raw text hash

Final answers produced for agents should be grounded in Layer 4 evidence, even if routing started from higher layers.

## Main Workflows

### 1. Initial ingestion

1. detect current repository state
2. create a new `snapshot`
3. enumerate source files to include
4. parse files with Tree-sitter
5. extract symbols and edges
6. build summaries and contextualized chunks
7. write chunks, facts, and evidence as `pending`
8. run verification pipeline
9. promote eligible facts to `active`

### 2. Incremental update

1. compare current Git SHA and file hashes to last indexed snapshot
2. identify added, modified, deleted files
3. reprocess only affected files
4. update symbols, edges, chunks, and facts for affected areas
5. mark displaced facts as `superseded`
6. archive or delete obsolete generated artifacts according to policy

### 3. Retrieval for answering questions

1. classify query type
2. select entry layer
3. retrieve candidates using FTS5 + vector search + metadata filters
4. expand neighbors using parent symbols, imports, tests, and related files
5. rerank and trim context
6. return evidence-backed results

### 4. Fact proposal

1. agent proposes statement plus evidence refs
2. system writes fact as `pending`
3. automated review runs
4. fact becomes:
   - `active` if checks pass and impact is low
   - `pending_manual_review` logically represented as `pending` plus review records if confidence or impact requires manual confirmation
   - `rejected` if checks fail

### 5. Fact retirement

1. detect stale or conflicting fact
2. verify whether supporting evidence changed
3. if replaced, mark old fact `superseded`
4. if no longer useful but worth keeping, mark `archived`
5. hard delete only generated artifacts that are explicitly safe to remove

## Verification and Trust Model

Verification is a first-class subsystem, not a post-processing detail.

### Automated verification checks

Required checks for new or updated facts:

1. source file still exists
2. line range still exists
3. evidence text hash still matches
4. referenced symbol still exists
5. statement is not a duplicate of an already active fact
6. statement does not contradict a higher-confidence active fact
7. summary-level facts remain grounded in raw evidence

### Manual review triggers

Require human review when any of these are true:

- confidence below threshold
- fact affects architecture, safety, or public API interpretation
- automated conflict detection finds contradictory evidence
- fact is synthesized from multiple sources and cannot be validated by one local chunk

### Confidence model

Use a simple numeric score in v1. Example inputs:

- number of evidence items
- evidence freshness
- parser confidence
- summary-to-evidence consistency
- retrieval/reranking agreement

Do not use confidence alone to activate high-impact facts.

## Archival and Deletion Policy

Default policy: **archive first, delete second**.

### Archive when

- a fact was once valid but has been replaced
- a chunk belongs to an old snapshot
- a summary is stale but useful for debugging
- a review record should remain part of the audit trail

### Hard delete when

- embeddings are obsolete and reproducible
- temporary chunking artifacts are no longer referenced
- rejected generated content should not be reused
- the user explicitly requests cleanup

### Retention guidance

Keep:

- active facts
- review history
- evidence mappings
- snapshot metadata

Delete aggressively only for:

- regenerated embeddings
- discarded chunk caches
- orphaned intermediate artifacts

## Retrieval Architecture

Use a coarse-to-fine retrieval flow.

```text
User query
   |
   v
Query classifier
   |
   +--> exact symbol / error / config key
   |      -> Layer 0 + Layer 3
   |
   +--> implementation question
   |      -> Layer 2 -> Layer 3 -> Layer 4
   |
   +--> architecture or subsystem question
   |      -> Layer 1 -> Layer 2 -> Layer 4
   |
   +--> change-impact question
          -> Layer 0/2 + edges + related tests -> Layer 4
```

### Search strategy

Always combine:

- metadata filtering
- lexical search
- vector search

Then apply:

- neighbor expansion
- reranking
- context trimming

### Neighbor expansion rules

Use repository structure rather than similarity-only retrieval.

Preferred neighbors:

- parent symbol
- sibling symbols
- imported files
- callers/callees
- related tests
- related docs

Avoid over-retrieving "similar code" from unrelated areas.

## MCP Interface for Other Agents

Expose a small, stable tool surface.

### Required tools

#### `search_knowledge`

Purpose:

- search facts, summaries, chunks, and evidence

Inputs:

- query
- mode
- snapshot or branch filter
- top_k

Outputs:

- ranked results with evidence refs

#### `get_symbol_card`

Purpose:

- return Layer 2 card for a file, class, function, or module

#### `get_evidence`

Purpose:

- return exact raw evidence by chunk, file path, or symbol

#### `propose_fact`

Purpose:

- allow an agent to submit a fact candidate with evidence refs

#### `verify_pending`

Purpose:

- run automated review checks for pending facts

#### `archive_stale`

Purpose:

- archive or retire stale facts and chunks based on latest snapshot

#### `diff_knowledge`

Purpose:

- compare two snapshots and show knowledge changes

### Optional resources

Expose read-only MCP resources for:

- latest active repository summary
- subsystem cards
- symbol catalogs
- knowledge health report

## Suggested Project Structure

This is a suggested layout for the future implementation. It is not required to match the current repository structure.

```text
rag_knowledge/
  README.md
  pyproject.toml
  src/
    rag_knowledge/
      app.py
      config.py
      db/
        schema.sql
        migrations/
        repositories/
      ingest/
        scanner.py
        parser.py
        summarizer.py
        chunker.py
        embeddings.py
      graph/
        edges.py
      retrieval/
        classifier.py
        hybrid_search.py
        rerank.py
        neighbor_expand.py
      verification/
        checks.py
        conflict_detector.py
        review_policy.py
      lifecycle/
        archive.py
        delete.py
        snapshot_diff.py
      mcp_server/
        tools.py
        resources.py
      evals/
        deepeval_suite.py
        ragas_suite.py
  tests/
    unit/
    integration/
    golden/
```

## Implementation Phases

### Phase 1: Local searchable knowledge base

Deliver:

- SQLite schema
- file scanning
- Tree-sitter parsing
- symbol extraction
- chunk storage
- FTS5 search
- simple MCP server with read-only tools

Do not include:

- graph index
- complex verification
- advanced reranking

### Phase 2: Trusted fact lifecycle

Deliver:

- `facts`, `fact_evidence`, and `reviews` tables
- automated verification checks
- fact activation rules
- archival flow
- snapshot diffing

### Phase 3: Better retrieval quality

Deliver:

- contextualized chunks
- sqlite-vec embeddings
- hybrid search
- neighbor expansion
- reranking
- symbol cards

### Phase 4: Agent-facing maintenance tools

Deliver:

- `propose_fact`
- `verify_pending`
- `archive_stale`
- `diff_knowledge`
- health and audit views

### Phase 5: Optional upgrades

Candidates:

- graph-assisted impact analysis
- Dolt-backed fact history
- LanceDB or Qdrant local migration if SQLite vector search becomes limiting

## Library and Repository Notes

### Recommended for v1

#### `modelcontextprotocol/python-sdk`

Use this to expose the local knowledge system to other agents through MCP tools and resources.

Repository:

- https://github.com/modelcontextprotocol/python-sdk

#### `sqlite-vec`

Use for vector search inside SQLite. This keeps deployment simple.

Repository:

- https://github.com/asg017/sqlite-vec

#### Tree-sitter

Use for structural parsing and symbol-aware chunking.

Repository:

- https://github.com/tree-sitter/tree-sitter

#### LlamaIndex

Do not use it as the primary data store. Use it selectively for retrieval composition ideas, especially coarse-to-fine retrieval and parent expansion behavior.

References:

- https://developers.llamaindex.ai/python/framework/integrations/retrievers/auto_merging_retriever/
- https://github.com/run-llama/llama_index

#### DeepEval

Use for regression-style evaluation of agent and RAG behavior.

Repository:

- https://github.com/confident-ai/deepeval

#### Ragas

Use if testset generation and retrieval-focused metrics are needed.

Repository:

- https://github.com/explodinggradients/ragas

### Useful alternatives, not required in v1

#### Dolt

Good later if table-level version control becomes important.

Repository:

- https://github.com/dolthub/dolt

#### LanceDB

Good later if a richer local vector store is needed.

Repository:

- https://github.com/lancedb/lancedb

#### Qdrant local mode

Good later if a file-backed local vector DB is preferred over SQLite-based vector search.

Repository:

- https://github.com/qdrant/qdrant-client

#### CocoIndex

Interesting for incremental indexing and lineage, but heavier than needed for the first local-only implementation.

Repository:

- https://github.com/cocoindex-io/cocoindex

## Open Decisions

These items should be resolved by the implementing agent before coding begins:

1. exact embedding model to use locally
2. whether summaries are generated online or cached from prior runs
3. whether Tree-sitter alone is sufficient for C/C++ symbol extraction or if clang-based enrichment is needed
4. whether fact activation thresholds should be configurable per fact type
5. whether archived embeddings should be retained or regenerated on demand

## Final Recommendation

The first implementation should be a **single local Python application** with:

- one SQLite database file
- FTS5 and sqlite-vec enabled
- Tree-sitter-based parsing
- MCP tool surface for other agents
- explicit fact lifecycle with verification and archive-first retirement

This design is small enough to build and maintain on one machine, but structured enough to support multiple local AI agents and a trusted long-lived codebase knowledge store.
