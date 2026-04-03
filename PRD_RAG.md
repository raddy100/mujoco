# Product Requirements Document: Local-First RAG Knowledge System for MuJoCo

This document translates `architecture_RAG.md` into an implementation-oriented product requirements document. It is intended to be read by another AI agent that will generate a detailed task list. It should be treated as the primary product specification for version 1.

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Product Intent](#product-intent)
3. [Version 1 Scope](#version-1-scope)
4. [Target Users and Primary Use Cases](#target-users-and-primary-use-cases)
5. [Constraints and Assumptions](#constraints-and-assumptions)
6. [Required Technology Stack](#required-technology-stack)
7. [MuJoCo Repository Scope](#mujoco-repository-scope)
8. [Functional Requirements](#functional-requirements)
9. [Architecture Outline](#architecture-outline)
10. [Knowledge Model Requirements](#knowledge-model-requirements)
11. [Verification and Trust Requirements](#verification-and-trust-requirements)
12. [Retention, Archival, and Deletion Requirements](#retention-archival-and-deletion-requirements)
13. [Interfaces and Interaction Model](#interfaces-and-interaction-model)
14. [Validation and Test Requirements](#validation-and-test-requirements)
15. [Acceptance Criteria](#acceptance-criteria)
16. [Deferred Features](#deferred-features)
17. [Open Decisions for the Implementing Agent](#open-decisions-for-the-implementing-agent)

## Executive Summary

Build a **fully local/offline**, **Python-based**, **MuJoCo-first** RAG knowledge system that stores trusted codebase knowledge in a local database and exposes it to other local AI agents through an MCP server. The product must support three first-class capabilities from version 1:

1. answer repository questions with citations
2. prepare grounded context for code changes and reviews
3. maintain a trusted, long-lived fact database that can be updated, verified, archived, and queried by local agents

The implementation must reuse existing libraries rather than building every subsystem from scratch. The preferred stack is **SQLite + FTS5 + sqlite-vec + Tree-sitter + MCP Python SDK**, with local-only model dependencies.

## Product Intent

The product exists to reduce repeated long searches through the MuJoCo codebase while preserving trust. It should allow local AI agents to reuse validated repository knowledge across sessions, but only when that knowledge is traceable back to exact source evidence and can be retired when it becomes stale.

This is **not** a generic hosted RAG platform in version 1. It is a **local developer tool** for one machine, one primary user, and multiple local AI agents.

## Version 1 Scope

### In scope

- MuJoCo-first implementation, structured so other repositories can be supported later
- local-only database and index files
- local-only model execution; no required cloud APIs
- codebase ingestion and incremental re-indexing
- evidence-backed question answering for local agents
- fact proposal, verification, activation, and retirement lifecycle
- MCP tool interface for local AI agents
- CLI or scriptable local workflows for indexing, review, and maintenance

### Out of scope

- multi-user support
- cloud deployment
- distributed vector databases
- mandatory graph database
- browser UI or desktop UI in v1
- direct raw database writes by external agents
- generic multi-repo onboarding workflow from day one

## Target Users and Primary Use Cases

### Primary human user

- a local developer with C++ embedded experience
- some experience with the MuJoCo repository
- wants AI agents to use a persistent, trustworthy local knowledge base

### Primary machine users

- local AI agents running on the same machine
- agents that need grounded repository context, not just ad hoc search results

### Primary use cases

#### Use Case 1: Repository question answering

An AI agent should be able to answer questions such as:

- where is `mjModel` defined and used?
- where is the plugin system implemented?
- what files participate in building the simulator?

The answer must include evidence references such as file paths, symbols, and line ranges.

#### Use Case 2: Grounded implementation context

An AI agent preparing a code change or review should be able to retrieve:

- relevant files
- related tests
- symbol cards
- dependencies and nearby context

The retrieval should prefer repository structure over unrelated "similar code."

#### Use Case 3: Long-lived trusted fact store

An AI agent should be able to store repository facts such as:

- subsystem summaries
- symbol responsibilities
- build or test commands
- invariants and integration notes

These facts must remain inactive until verification rules pass, and must be archived or superseded when the repository changes.

## Constraints and Assumptions

### Hard constraints

- fully local/offline only
- Python is acceptable as the main implementation language
- database must remain local to the machine
- all required functionality must work without internet access
- the product must be able to operate on the MuJoCo repository on Windows

### Assumptions

- the main runtime will be a single local Python application
- the MCP server will be the primary interface for other agents
- CLI commands are sufficient for human review and maintenance in v1
- the local database will be stored in a gitignored directory, preferably under the repo root such as `.rag_knowledge\`
- the system will run with a single writer and multiple readers

### Design implications

- no required OpenAI, Anthropic, or hosted embedding APIs
- any summarization, embedding, or evaluation model must run locally
- if a local generative model is not configured, the product must still provide useful extractive indexing and retrieval

## Required Technology Stack

The implementing agent should treat the following as the default v1 stack unless there is a strong, documented reason to deviate.

### Required libraries and tools

| Concern | Required Choice | Notes |
|---|---|---|
| Main language | Python 3.11+ | Main implementation language |
| Agent protocol | `modelcontextprotocol/python-sdk` | MCP server for local agents |
| Database | SQLite | Primary source of truth |
| Lexical search | SQLite FTS5 | Required for identifiers and exact matching |
| Vector search | `sqlite-vec` | Keep embeddings inside SQLite |
| Structural parsing | `tree-sitter` | Primary parser framework |
| Multi-language Tree-sitter support | individual grammars or language pack | Must support MuJoCo-first languages |
| Local embeddings | `sentence-transformers` | Default local embedding path |
| CLI | `typer` or equivalent | Human/operator workflows |
| Config validation | `pydantic` or `pydantic-settings` | Structured config |
| Testing | `pytest` | Standard test harness |

### Preferred local model layer

The product must remain usable even if no local generative model is configured. However, if a local model is available, use a pluggable provider.

Preferred approach:

- embeddings: `sentence-transformers`
- optional local generation and summarization: `llama-cpp-python`

### Default local embedding model

Use a lightweight local default model for v1, such as:

- `sentence-transformers/all-MiniLM-L6-v2`

This is acceptable because lexical search and repository structure will carry much of the retrieval burden in version 1.

### Evaluation libraries

At least one of the following should be integrated during implementation:

- `deepeval`
- `ragas`

If both are too heavy for early phases, the product must still include deterministic offline tests and golden query fixtures.

### Useful optional libraries

These are allowed but not required in v1:

- `GitPython` for git-state queries, or subprocess-based git commands
- `watchdog` for file watching in later iterations
- `networkx` if graph reasoning becomes necessary later

### Explicitly not required in version 1

- Postgres
- Docker
- cloud vector DBs
- cloud-hosted LLM APIs
- distributed search services

## MuJoCo Repository Scope

Version 1 is MuJoCo-first. The indexer should prioritize parts of the repository that are most valuable for code understanding, build/test reasoning, and implementation work.

### Priority 1 paths

- `include\`
- `src\`
- `test\`
- `plugin\`
- `python\`
- `cmake\`
- `CMakeLists.txt`
- `README.md`
- `CONTRIBUTING.md`
- `STYLEGUIDE.md`

### Priority 2 paths

- `simulate\`
- `sample\`
- `doc\`
- `model\`
- root-level `*.xml` model files

### Priority 3 paths

- `unity\`
- `mjx\`
- notebooks and auxiliary examples

### Excluded in version 1

- `build\`
- `dist\`
- generated artifacts
- binary assets
- images not needed for text retrieval

### Required language/file handling in version 1

| Type | Requirement |
|---|---|
| C / headers | Required, structural parsing |
| C++ | Required, structural parsing |
| Python | Required, structural parsing |
| CMake | Required, text + lightweight structural parsing |
| Markdown | Required, document chunking |
| XML / MJCF | Required, text/XML chunking with metadata |
| C# | Optional early, better later |

## Functional Requirements

### FR1: Local repository indexing

The product must ingest repository content into a local database and produce a searchable knowledge base.

Must support:

- initial full indexing
- incremental indexing based on changed files
- storage of snapshot metadata tied to Git state
- storage of structural metadata, summaries, chunks, and evidence

### FR2: Layered knowledge storage

The product must store knowledge in distinct layers:

1. routing metadata
2. global summaries
3. navigational symbol/file cards
4. contextualized retrieval chunks
5. raw evidence

These layers must have separate purposes and must not be collapsed into a single flat store.

### FR3: Hybrid retrieval

The product must support hybrid retrieval over:

- lexical search
- vector search
- metadata filters

The system must also support neighbor expansion using repository structure such as:

- parent symbols
- sibling symbols
- imported files
- callers/callees
- related tests
- related docs

### FR4: Evidence-backed responses

Every response intended for agent consumption must include enough evidence to trace the answer back to repository sources. At minimum, returned evidence must include:

- file path
- line range or symbol range
- snapshot or commit reference

### FR5: Fact proposal and lifecycle

The product must allow an agent to propose a fact candidate and store it as pending. Facts must then move through a lifecycle:

- pending
- active
- rejected
- superseded
- archived
- deleted

No fact should become active without evidence and verification.

### FR6: Automated verification

The product must run automated verification checks for new or updated facts. Required checks include:

- source exists
- line range exists
- evidence hash still matches
- referenced symbol still exists
- duplicate detection
- contradiction detection against active facts where possible

### FR7: Manual review for risky facts

Low-confidence or high-impact facts must require human review before activation. In v1, human review may happen through CLI commands or MCP tools; no GUI is required.

### FR8: Archival and retirement

When repository changes invalidate knowledge, the system must:

- supersede or archive old facts
- avoid returning stale active facts
- preserve auditability where appropriate

### FR9: MCP server for local agents

The product must expose a local MCP server that other local AI agents can use. It must support at least:

- search
- symbol-card retrieval
- evidence retrieval
- fact proposal
- verification execution
- stale knowledge archival
- knowledge diffing between snapshots

### FR10: Offline-first operation

Version 1 must work with no network access. This includes:

- indexing
- storage
- retrieval
- verification logic
- tests

If local LLM-based summarization is unavailable, the system must degrade gracefully to extractive or rule-based summaries.

## Architecture Outline

The product should follow this high-level shape.

```text
MuJoCo working tree
      |
      v
Git state + file scanning
      |
      v
Tree-sitter parsers + file handlers
      |
      +--> symbols and structural edges
      +--> summaries and symbol cards
      +--> contextualized chunks
      +--> raw evidence windows
      |
      v
SQLite database
  - snapshots
  - sources
  - symbols
  - edges
  - chunks
  - facts
  - evidence
  - reviews
  - FTS5
  - sqlite-vec
      |
      v
Local MCP server + CLI
      |
      v
Other local AI agents and the human operator
```

### Ownership model

The system should use a **single writer, multiple readers** design.

- the main local application owns schema changes and write operations
- external agents should not write directly to SQLite
- agents interact through MCP tools or controlled CLI entry points

### Storage location

Default storage should be in a gitignored local directory, for example:

- `<repo_root>\.rag_knowledge\`

The implementing agent may choose a different default if it remains local, configurable, and easy for other local agents to access.

## Knowledge Model Requirements

The implementing agent should derive detailed schema tasks from these required entities.

### Required logical entities

- repository snapshots
- source files
- symbols
- structural edges
- retrieval chunks
- facts
- fact-to-evidence mappings
- reviews

### Required fact fields

At minimum, each fact must carry:

- subject key
- fact type
- statement
- confidence
- lifecycle status
- snapshot or commit reference
- creator

### Required evidence fields

At minimum, each evidence item must carry:

- source file path
- line range
- raw text hash
- symbol or chunk reference where applicable

### Required summary/card content

Each symbol or file card should capture:

- purpose
- important APIs or symbols
- dependencies or neighbors
- invariants or gotchas
- likely questions it answers

## Verification and Trust Requirements

### Trust model

The system is designed around **trust through provenance and lifecycle**, not trust through a one-time summary.

### Required verification behavior

- facts default to pending
- automated checks run before activation
- low-confidence or high-impact facts require manual approval
- active facts must remain traceable to exact evidence
- stale facts must be retired when the repository changes

### High-impact fact examples

Facts should be treated as high-impact when they concern:

- build commands
- public APIs
- architecture-level summaries
- behavioral invariants
- safety-sensitive code paths

### Manual review model in v1

Manual review may be command-line driven. A GUI is not required. The product only needs a workable review flow such as:

- list pending risky facts
- inspect evidence
- approve
- reject
- mark superseded

## Retention, Archival, and Deletion Requirements

### Default policy

Use **archive first, delete second**.

### Archive

Archive when:

- a fact was valid but replaced
- a summary is stale but still useful for audit or debugging
- an old snapshot should remain inspectable

### Delete

Hard delete only when:

- an artifact is generated and reproducible
- it is no longer referenced
- it is explicitly safe to remove

Examples:

- obsolete embeddings
- temporary chunk caches
- rejected generated summaries if they should never be reused

## Interfaces and Interaction Model

### Required MCP tools

The product must expose at least these tools:

- `search_knowledge`
- `get_symbol_card`
- `get_evidence`
- `propose_fact`
- `verify_pending`
- `archive_stale`
- `diff_knowledge`

### Recommended CLI commands

The product should also provide local operator commands such as:

- `index`
- `sync`
- `query`
- `review-pending`
- `approve-fact`
- `reject-fact`
- `archive-stale`
- `diff-snapshots`

### Query behavior

Query handling should distinguish at least these categories:

- exact symbol / identifier / error lookup
- implementation questions
- architecture questions
- change-impact questions

### Response requirements

Responses returned to other agents should include:

- answer text or summary
- evidence references
- confidence or trust metadata where useful
- snapshot or commit context

## Validation and Test Requirements

The implementing agent should treat testability as a core product requirement.

### Required test categories

#### Unit tests

Must cover:

- file selection and exclusion rules
- chunking logic
- symbol extraction adapters
- fact lifecycle transitions
- verification checks
- archive/delete rules

#### Integration tests

Must cover:

- full indexing into SQLite
- FTS5 and sqlite-vec queries
- MCP tool execution
- incremental re-indexing after file changes
- fact proposal and verification flows

#### Golden tests

Must include a set of representative MuJoCo queries and expected evidence-bearing results.

Examples:

- locate the public definition of `mjModel`
- locate the plugin extension interface
- identify where tests for XML or engine behavior live
- identify build configuration entry points

#### Offline enforcement

Tests must run with network access disabled or simulated as unavailable. Version 1 cannot depend on internet access.

### Fallback behavior tests

The product must have tests proving that it still works when:

- no local generative model is configured
- only lexical retrieval is available
- an indexed file is deleted or moved
- a fact’s evidence no longer matches current source text

## Acceptance Criteria

Version 1 is complete when all of the following are true.

### Repository understanding

- the system can index the MuJoCo repository without requiring cloud services
- the system stores structured knowledge for prioritized MuJoCo paths
- the system excludes build artifacts and generated output

### Retrieval

- a local agent can query repository knowledge through MCP
- returned results include file paths and evidence ranges
- retrieval uses both lexical and vector-capable paths, even if vector quality is initially basic

### Fact lifecycle

- an agent can propose a fact with evidence
- the fact is stored as pending
- automated checks run
- the system can activate, reject, supersede, or archive the fact

### Incremental maintenance

- changing a source file does not force a full re-index
- outdated facts can be detected and retired
- active facts remain tied to a specific snapshot or commit

### Local operation

- the system runs fully offline
- the database remains local
- required workflows are available through MCP and CLI

### Reviewability

- a human can inspect pending risky facts
- evidence can be reviewed before approval
- the audit trail for facts and reviews is preserved

## Deferred Features

These are explicitly deferred unless the implementing agent documents a compelling reason to include them earlier.

- browser-based review UI
- graph database or GraphRAG dependency
- multi-user sharing
- multi-repo first-class onboarding
- cloud-hosted model providers
- advanced graph-assisted impact analysis
- heavy semantic reranking before baseline retrieval is proven useful

## Open Decisions for the Implementing Agent

The implementing agent may need to resolve these during task planning, but should not change the product intent without documenting why.

1. whether to use Tree-sitter only for C/C++ or add clang-based enrichment later
2. the exact local model configuration for optional summarization
3. whether XML should use a dedicated XML parser in addition to text chunking
4. the exact threshold policy for low-confidence vs high-impact facts
5. whether vector embeddings are stored directly in SQLite rows or via an auxiliary `sqlite-vec` virtual table structure

## Handoff Note to the Task-Generation Agent

Generate implementation tasks from this PRD with these priorities:

1. get a usable local index and query path working first
2. keep the system offline and simple
3. add trust and fact lifecycle before advanced retrieval tricks
4. optimize MuJoCo-specific value before generic multi-repo expansion

If tradeoffs are needed, prefer:

- correctness over recall
- provenance over clever summarization
- incremental updates over full rebuilds
- simple local dependencies over heavyweight infrastructure
