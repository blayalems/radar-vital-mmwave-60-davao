## 2024-05-26 - [Angular Template Method Calls]\n**Learning:** [Angular Change Detection overhead is high when expensive functions are called directly from templates (@if, @for). Arrays, allocations and sorting (like median FPS calculation or bias bucketing) run on every CD cycle. Using Angular Signals `computed()` function caches the results and prevents unnecessary recomputation unless underlying payload signals change.]\n**Action:** [When encountering expensive parameter-less methods called from Angular templates, refactor them into `computed` properties, especially for high-frequency telemetry dashboards.]
## 2024-05-18 - Template Loading Optimization
**Learning:** Loading large static resources (like HTML templates) from disk on every request can cause significant I/O overhead (e.g., 5.7 seconds for 1000 iterations).
**Action:** Use memory caching techniques like `@functools.lru_cache(maxsize=1)` for static or infrequently changing resources to improve performance.

## 2025-02-23 - Avoid Repeated File System Access in Monolith Configuration
**Learning:** Caching static file path resolution, especially those doing directory traversal during server requests, drastically cuts latency. Even minimal filesystem checks can accumulate latency rapidly in loops or high-frequency invocations.
**Action:** When a path resolution or read does not change during application lifecycle, always wrap it with `functools.lru_cache(maxsize=1)` or store the result globally.

## 2026-05-26 - Map Update Performance Optimization
**Learning:** In loops processing large numbers of items, unconditional object allocation and Map.set operations per iteration add measurable hashing and mapping overhead. Since objects are passed by reference in JavaScript, updating an existing map entry's properties modifies it in-place.
**Action:** Avoid redundant `Map.set` operations on existing references. Instead, fetch the entry using `Map.get`, update it if it exists, and only allocate a new object and call `Map.set` when creating a new entry.
