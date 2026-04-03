# Layered Knowledge Architecture for Codebase RAG

## Executive Summary

A simple `1-sentence -> 4-sentence -> full detail` stack is directionally right, but the strongest current pattern is a five-part retrieval stack: routing metadata, global summaries, navigational summaries, contextualized chunks, and raw evidence, with an optional graph/community index for architecture and multi-hop questions.[^1][^2][^3][^4][^5] Hierarchical methods such as RAPTOR and GraphRAG help the system "zoom out" for global questions, while Anthropic's contextual retrieval work shows that chunk-local context is often more important than generic summaries for fine-grained retrieval.[^1][^3][^4][^5] Code-specific research adds two important constraints: repository structure matters more than single-file proximity, and blindly retrieving "similar code" can hurt because it introduces noise; imports, parent classes, APIs, and other structurally related artifacts are the high-value context.[^6][^7][^8] The best practical design today is therefore coarse-to-fine: route with summaries, retrieve with hybrid lexical+semantic search, expand through repository structure, rerank, and answer only from raw code, tests, configs, and docs.[^1][^2][^11][^13]

## First decision: do you need RAG at all?

If the relevant slice of the codebase fits comfortably inside model context, start with whole-context prompting or prompt caching before building a complicated retrieval system. Anthropic explicitly recommends trying full-context prompting first for corpora under roughly 200,000 tokens, because the simplest solution can outperform a more complex RAG stack when the data is small enough.[^1]

## Direct answer to your proposed layers

Your three layers are a good core, but I would not stop at three.

1. Keep the **single-sentence layer**, but use it only for repo/subsystem routing, not as the final factual source.[^3][^4][^5]
2. Keep the **four-sentence layer**, but use it as a module/file/symbol card with explicit fields such as purpose, dependencies, invariants, and likely questions answered.[^2][^7]
3. Keep the **full-detail layer**, but make it raw evidence windows retrieved on demand rather than huge unsliced files.[^1][^2][^9]
4. Add a **metadata/routing layer** under everything, because branch, commit, path, language, symbol kind, and dependency edges determine whether retrieval stays fresh and precise.[^2][^7]
5. Add a **chunk-local context layer** between summaries and raw evidence, because current evidence suggests contextualized chunks outperform generic summary-only indexing for retrieval quality.[^1]

The key idea is that layers should have **different jobs**, not just different lengths.

## Recommended layer model

### Layer 0 - Routing metadata (no summarization)

Store unsummarized facts for every retrievable unit: repo, branch, commit SHA, path, language, package/module, symbol name, symbol kind, parent symbol, imports, callers/callees, related tests, related docs, timestamps, and permissions or visibility labels if relevant.[^2][^7] This layer is primarily for filtering, freshness, and query routing rather than generation. Azure's guidance explicitly emphasizes metadata extraction, versioning, hierarchical index organization, and incremental updates, while code-retrieval papers show that repository structure outside the current file is essential context.[^2][^7]

### Layer 1 - Global memory / one-sentence summaries

Create one- or two-sentence summaries for the repository and each major subsystem. These should answer "what is this?" and "how does this fit into the whole?" rather than restating detailed implementation. RAPTOR and GraphRAG both use bottom-up hierarchical summarization because global questions are poorly served by flat chunk retrieval.[^3][^4][^5] MemoRAG is an emerging variation on the same theme: keep a separate global memory over the corpus and use it to guide later retrieval when the question is underspecified or exploratory.[^10] This layer is best for routing questions such as "What are the main subsystems?" or "Where should I look for authentication, rendering, or data loading?"[^3][^4]

### Layer 2 - Navigational summaries / four-sentence cards

This is where your four-sentence idea fits best. Use 3-6 sentences per package, module, file, class, or function card. Each card should capture:

- responsibility and purpose
- key inputs, outputs, or public API
- important dependencies and neighbors
- invariants, side effects, or "gotchas"
- sample questions the node can answer

Azure recommends alignment optimization by attaching sample questions to chunks, and repository-level code papers show that retrieval quality improves when the retriever has access to repository relationships such as imports, parent classes, and sibling files instead of only the active file.[^2][^7] In practice, this layer is the best entry point for "Explain how module X works" style questions.

### Layer 3 - Contextualized retrieval units

This layer should contain raw code or documentation chunks split at meaningful boundaries, but with a short generated context header prepended to each chunk before indexing. Anthropic calls this contextual retrieval and reports that contextual embeddings plus contextual BM25 reduced top-20 retrieval failures by 49%, and adding reranking reduced failures by 67%.[^1] Anthropic also notes that generic document summaries and summary-based indexing gave much smaller gains than chunk-specific contextualization, which is a strong argument against using only `1 sentence + 4 sentences + raw chunk` as the retrieval substrate.[^1]

For code, the chunking rule should be structural rather than purely token-based:

- prefer function, method, class, module, test, or config-block boundaries
- if a symbol is too large, split internally but preserve signature, docstring, and import context
- keep exact identifiers and error strings intact for lexical search

This recommendation is consistent with repository-level code retrieval work, which benefits from structural context across files rather than arbitrary windows.[^6][^7][^8]

### Layer 4 - Raw evidence windows (no summarization)

This layer holds the exact code, tests, configs, docs, or schema fragments that support the final answer. It should be retrievable as surrounding windows such as `symbol -> class -> file -> package`, not just as isolated micro-chunks.[^2][^9][^11] Final answers should be composed from this layer, even when routing began at higher summary layers, because code questions are often decided by exact identifiers, control flow, default values, side effects, or tests.[^1][^8]

LongRAG is relevant here: once the search space has been narrowed, larger retrieval units can outperform many tiny fragments because they preserve context and reduce search burden.[^9] LlamaIndex's AutoMergingRetriever demonstrates the same principle operationally by retrieving leaf chunks and then merging them back up to parents when several sibling hits land in the same area.[^11]

### Optional graph/community layer

If the system must answer cross-cutting architecture or change-impact questions, add a graph layer over imports, calls, inheritance, ownership, config references, and entity relationships. GraphRAG shows that graph communities and community summaries are particularly useful for global questions and multi-hop reasoning where flat top-k text retrieval fails.[^3][^4] For code, this layer is especially valuable for:

- "What breaks if I change this interface?"
- "Which subsystems participate in request handling?"
- "How does data move from config to runtime behavior?"

I would treat the graph layer as optional-but-powerful rather than mandatory for every repository.[^3][^4]

## Recommended retrieval architecture

```text
                       User question
                             |
                             v
                    +------------------+
                    | Query classifier |
                    +------------------+
                       |      |      |
          exact/symbol |      | architecture/global
                       |      v
                       |   +-------------------+
                       |   | L1 + graph search |
                       |   +-------------------+
                       |             |
                       v             v
                +---------------------------+
                | L2 navigational retrieval |
                +---------------------------+
                             |
                             v
                +---------------------------+
                | L3 hybrid retrieval       |
                | BM25 + dense + metadata   |
                +---------------------------+
                             |
                             v
                +---------------------------+
                | neighbor expansion        |
                | parents/imports/tests     |
                | Small2Big / auto-merge    |
                +---------------------------+
                             |
                             v
                +---------------------------+
                | rerank and trim context   |
                +---------------------------+
                             |
                             v
                +---------------------------+
                | answer from L4 evidence   |
                +---------------------------+
```

A good query router should choose different entry points:

- **Exact symbol / error / config key / test name** -> start with Layer 0 + Layer 3, with BM25 or hybrid search weighted heavily toward lexical matches.[^1][^13]
- **Implementation question** -> start at Layer 2, then descend to Layer 3 and Layer 4.[^2][^11]
- **Architecture / "what are the main pieces?"** -> start with Layer 1 and the graph/community layer, then verify against Layer 4 if the answer names concrete files or interfaces.[^3][^4][^5]
- **Impact / dependency / multi-hop question** -> start with graph + Layer 2, then expand into Layer 4 along imports, callers/callees, tests, and configs.[^3][^4][^7]

A practical runtime pattern is to retrieve broadly, then rerank and trim. Anthropic's example pipeline retrieves a larger candidate set first and then reranks down to a smaller grounded context window, which is a good default for codebase assistants too.[^1]

## What to generate at ingestion time

I would generate five artifacts for every code unit:

1. **routing metadata**
   - repo, branch, commit, path, language, symbol kind, ownership, graph edges[^2]

2. **one-sentence routing summary**
   - "This module handles X and is the main entry point for Y."[^3][^5]

3. **four-sentence navigational card**
   - purpose
   - key APIs or symbols
   - dependencies or neighbors
   - invariants or side effects
   - questions it answers[^2][^7]

4. **context header for each raw chunk**
   - 1-2 sentences or roughly 50-100 tokens situating the chunk inside the file, module, or repository[^1]

5. **raw evidence**
   - exact chunk text plus parent windows, with stable references back to file and commit[^1][^2]

If you want one concrete schema, use fields like:

- `node_id`
- `repo`
- `branch`
- `commit_sha`
- `path`
- `language`
- `symbol`
- `kind`
- `parent_id`
- `imports[]`
- `callers[]`
- `callees[]`
- `tests[]`
- `one_sentence_summary`
- `navigational_card`
- `sample_questions[]`
- `context_header`
- `raw_text_ref`
- `embedding_id`
- `bm25_text`
- `updated_at`

That schema is a synthesis, but it directly reflects Azure's metadata, versioning, and sample-question advice, Anthropic's contextualized-chunk indexing, and repository-level code retrieval's need for structural relationships.[^1][^2][^7]

## Update and freshness strategy

The layer stack only works if it stays fresh. Azure's advanced RAG guidance is clear that production systems need incremental updates, selective reindexing, versioning, and often hybrid strategies rather than full rebuilds on every change.[^2] For codebases, the update rule should be:

1. changed file -> rebuild its Layer 3 chunks and Layer 4 evidence refs
2. regenerate Layer 2 cards for affected symbols or files
3. invalidate and recompute parent Layer 1 summaries only for impacted subsystems
4. refresh graph edges for changed imports, calls, or inheritance
5. keep commit SHA on every artifact so answers can be tied to a specific revision[^2]

This makes the hierarchy practical for active repositories.

## Best practices that appear consistently across sources

1. **Do not rely on a flat vector index alone.** Hybrid lexical + semantic retrieval repeatedly appears as the robust default for technical data because exact identifiers and semantic descriptions both matter.[^1][^13]

2. **Use summaries for routing, not as the last word.** RAPTOR and GraphRAG show the value of high-level summaries, but Anthropic's results strongly suggest that chunk-local context plus raw evidence is what closes retrieval gaps for actual answering.[^1][^3][^4][^5]

3. **Prefer structural neighbors over "similar code."** Repository-level prompting and retrieval work value imports, APIs, parent classes, siblings, and other structural relationships; AllianceCoder reports that similar-code retrieval can reduce performance by up to 15% when it injects noise.[^7][^8]

4. **Use larger windows only after narrowing the search space.** LongRAG and AutoMergingRetriever both support the pattern of coarse retrieval first, then expanding to larger coherent contexts once you know where to look.[^9][^11]

5. **Attach questions to chunks or cards.** Azure's alignment optimization recommendation is useful for codebases too: "what question does this unit answer?" is often a better retrieval surface than the raw text alone.[^2]

6. **Keep tests, configs, and docs in the same retrieval graph.** CodeRAG-Bench shows useful context can come from multiple sources, not just source code files, and many codebase facts live in tests and documentation rather than implementations.[^12]

## Anti-patterns

- **Only three layers with no metadata layer.** This makes routing, freshness, and branch isolation much weaker.[^2]
- **Generic document summaries attached everywhere.** Anthropic explicitly reports limited gains from generic summaries and stronger gains from chunk-specific contextualization.[^1]
- **Uniform tiny chunks for code.** This loses structure and exact context that repository-level code work depends on.[^6][^7]
- **Vector-only retrieval.** Exact identifiers, error codes, and API names are too important to leave lexical search out.[^1][^13]
- **Retrieving lots of similar snippets from unrelated areas.** This is now a known failure mode in code RAG.[^8]
- **Always using graph retrieval.** Graph/community indexes are most valuable for global or multi-hop queries, not every lookup.[^3][^4]

## A concrete recommendation if I were building this today

If I were implementing a codebase knowledge system today, I would use this exact stack:

1. **Layer 0 metadata index**
   - symbol table, file table, dependency edges, commit/version data

2. **Layer 1 repo/subsystem cards**
   - 1-2 sentences each

3. **Layer 2 module/file/symbol cards**
   - 3-6 sentences each + sample questions + dependencies

4. **Layer 3 contextualized chunks**
   - symbol-aware raw chunks with 50-100 token context headers
   - indexed in BM25 and dense vector search

5. **Layer 4 evidence store**
   - raw code, tests, docs, and configs with parent-window expansion

6. **Optional graph/community index**
   - imports, calls, inheritance, config references, service/data-flow entities

Then I would route queries as follows:

- global architecture -> Layer 1 + graph
- module explanation -> Layer 2 -> Layer 4
- exact symbol lookup -> Layer 0 + Layer 3 -> Layer 4
- change impact -> graph + Layer 2 + tests in Layer 4

That preserves your original intuition, but upgrades it into a retrieval architecture that matches current evidence.

## Evaluation plan

I would evaluate the system at four levels:

1. **Routing accuracy**
   - did the query start in the right layer or index?

2. **Retrieval quality**
   - recall@k or miss-rate for the needed evidence chunks
   - Anthropic uses `1 - recall@20` as a retrieval failure metric[^1]

3. **Grounded answer quality**
   - can every factual claim be traced to Layer 4 evidence?
   - Azure recommends golden datasets, assessment pipelines, logging, and root-cause analysis for this reason.[^2]

4. **Code task quality**
   - for code assistance, measure end-to-end completion or task success as papers like RepoCoder and CodeRAG-Bench do, not just text similarity.[^6][^12]

Track latency and freshness too, since hierarchical systems can become over-engineered if they improve accuracy but become too stale or too slow.[^2][^4]

## Bottom line

The best current design is **not** "three summaries of the same object at increasing length." It is **layered retrieval with different jobs at each layer**:

- routing and filtering
- global understanding
- navigational understanding
- precise retrieval
- final evidence

So: **yes to a one-sentence layer, yes to a four-sentence layer, yes to a raw-detail layer - but add metadata underneath them and contextualized chunks between summaries and raw evidence, with an optional graph/community layer for architecture and multi-hop questions.**[^1][^2][^3][^4][^5]

## Confidence Assessment

High confidence:

- Hierarchical summaries help with global and multi-hop questions.[^3][^4][^5]
- Chunk-local contextualization, hybrid retrieval, and reranking materially improve fine-grained retrieval.[^1][^13]
- Repository-level code retrieval must use structural context across files, not just the current file.[^6][^7]
- Similar-code retrieval can be harmful when it is not structurally relevant.[^8]
- Incremental updates and version-aware indexing are necessary in production.[^2]

Moderate confidence:

- The exact number of layers that is "best" is not universally fixed in the literature. The 5-layer recommendation here is a synthesis of current evidence, not a standard with broad consensus.[^1][^2][^3][^5]
- Whether you need the optional graph layer depends heavily on your query mix; not every codebase assistant benefits enough to justify the extra indexing cost.[^3][^4]

## Footnotes

[^1]: Anthropic, "Contextual Retrieval," explains contextual embeddings + contextual BM25, reports a 49% top-20 retrieval-failure reduction and 67% with reranking, and notes limited gains from generic document summaries and summary indexing. https://www.anthropic.com/news/contextual-retrieval
[^2]: Microsoft Learn, "Build Advanced Retrieval-Augmented Generation Systems," recommends metadata extraction, hierarchical indexes, Small2Big, sample questions or alignment optimization, incremental updates, versioning, and golden datasets or assessment pipelines. https://learn.microsoft.com/en-us/azure/developer/ai/advanced-retrieval-augmented-generation
[^3]: Microsoft GraphRAG docs describe GraphRAG as a structured, hierarchical RAG approach that builds a knowledge graph, clusters communities, generates bottom-up summaries, and supports global, local, and DRIFT search. https://microsoft.github.io/graphrag/
[^4]: Microsoft Research, "GraphRAG: New tool for complex data discovery now on GitHub," explains hierarchical community summaries for global questions and reports strong gains over naive RAG on comprehensiveness and diversity with favorable token costs. https://www.microsoft.com/en-us/research/blog/graphrag-new-tool-for-complex-data-discovery-now-on-github/
[^5]: Sarthi et al., "RAPTOR: Recursive Abstractive Processing for Tree-Organized Retrieval," arXiv:2401.18059, describes recursively embedding, clustering, and summarizing chunks into a retrieval tree with multiple abstraction levels. https://arxiv.org/abs/2401.18059
[^6]: Zhang et al., "RepoCoder: Repository-Level Code Completion Through Iterative Retrieval and Generation," arXiv:2303.12570, shows iterative repo-level retrieval/generation improves code completion by over 10% over in-file baselines and outperforms vanilla retrieval-augmented completion. https://arxiv.org/abs/2303.12570
[^7]: Shrivastava et al., "Repository-Level Prompt Generation for Large Language Models of Code," arXiv:2206.12839, uses repository context including imports and parent class files and reports a 36% relative improvement over Codex. https://arxiv.org/abs/2206.12839
[^8]: Gu et al., "What to Retrieve for Effective Retrieval-Augmented Code Generation," arXiv:2503.20589, reports that contextual code and API information help, while retrieved similar code can degrade results by up to 15%. https://arxiv.org/abs/2503.20589
[^9]: Jiang et al., "LongRAG: Enhancing Retrieval-Augmented Generation with Long-context LLMs," arXiv:2406.15319, argues that larger retrieval units can reduce search burden and preserve context once the right area is found. https://arxiv.org/abs/2406.15319
[^10]: Qian et al., "MemoRAG: Moving towards Next-Gen RAG Via Memory-Inspired Knowledge Discovery," arXiv:2409.05591, proposes a global-memory layer plus targeted retrieval for long-context tasks; useful as evidence that top-level memory can be separate from raw retrieval. https://arxiv.org/abs/2409.05591
[^11]: LlamaIndex AutoMergingRetriever docs show a hierarchical node parser and a retriever that merges leaf hits back to parent nodes for more coherent context. https://developers.llamaindex.ai/python/framework/integrations/retrievers/auto_merging_retriever/
[^12]: Wang et al., "CodeRAG-Bench: Can Retrieval Augment Code Generation?" arXiv:2406.14497, benchmarks code RAG across basic, open-domain, and repository-level tasks using multiple context sources, showing both the value and current limits of code retrieval. https://arxiv.org/abs/2406.14497
[^13]: Microsoft, "Common Retrieval-Augmented Generation (RAG) Techniques Explained," summarizes current common practices including full-text search, vector search, chunking, hybrid search, query rewriting, and re-ranking. https://www.microsoft.com/en-us/microsoft-cloud/blog/2025/02/04/common-retrieval-augmented-generation-rag-techniques-explained/
