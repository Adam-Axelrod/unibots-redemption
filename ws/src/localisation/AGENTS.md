# ws/src/localisation/AGENTS.md

## General principles

- Use British English in code comments, documentation, identifiers and filenames where appropriate.
- Prefer simple, explicit implementations over unnecessary abstractions.
- Keep code modular, with each module having a clear responsibility.
- Avoid speculative features, excessive configurability and premature generalisation.
- Reuse existing functionality rather than duplicating it.
- Follow the repository's established structure and conventions.
- Make the smallest changes necessary to complete the requested task.

## Code structure

- Separate mathematical logic from framework-specific integration.
- Prefer small, cohesive modules with reusable functions and well-defined interfaces.
- Avoid introducing classes when simple functions or dataclasses are sufficient.
- Keep configuration, computation and application logic separate where useful.
- Do not introduce dependencies without a clear benefit.
- Do not create empty scaffolding or placeholder modules unless explicitly requested.

## Configuration

- Prefer small, readable YAML files with clear parameter names.
- Never use inline comments in YAML.
- Use standalone comments only when they explain something non-obvious.
- Keep each comment sentence on one line.
- Do not repeat parameter names, units or obvious facts in comments.
- Avoid duplicating values that can be derived or obtained at runtime.
- Choose sensible provisional defaults for algorithm parameters.
- Distinguish provisional values from measured hardware parameters.
- Use `null` only when a genuinely required value is unknown.
- Store each configuration value in one authoritative location.
- Do not preserve unused configuration fields purely for future possibilities.

## Documentation

- Keep documentation concise and technically precise.
- Explain design decisions, interfaces, assumptions and important limitations.
- Avoid restating information already available in authoritative documentation.
- Prefer references to existing documents over copying their contents.
- Avoid repetitive comments and excessive explanatory text.

## Implementation workflow

- Inspect relevant existing code before making changes.
- Respect the requested scope; do not implement future milestones prematurely.
- Prefer independently testable mathematical components.
- Add focused tests for meaningful behaviour and edge cases.
- Run relevant checks and report actual results.
- Do not claim something has been tested if it has not.
- Do not commit, push, merge or open pull requests unless explicitly requested.

## Communication

- Keep implementation summaries concise.
- Report what changed, why it changed, tests performed and unresolved issues.
- Highlight important architectural trade-offs without lengthy explanations of routine changes.