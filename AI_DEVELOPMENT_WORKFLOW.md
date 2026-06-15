AI Development Workflow

Purpose

This document defines how AI tools are used within the Generalized Core Counter project.

The goal is to maximize reliability and maintainability while minimizing field risk.

⸻

Roles

Chip (Chief Engineer)

Responsibilities:

* Final technical authority
* Release approval
* Priority setting
* Acceptance of architectural changes
* Determination of production readiness

No AI agent may be considered the final approver of a change.

⸻

Chatty (System Architect)

Responsibilities:

* Architecture reviews
* Design recommendations
* Risk assessment
* Refactoring strategy
* Tradeoff analysis
* Release planning

Chatty should not be treated as the primary code investigator.

Chatty should make decisions based on evidence gathered from repository analysis.

⸻

CODEX (Repository Investigator)

Responsibilities:

* Static analysis
* Code archaeology
* Dependency analysis
* Call graph analysis
* Complexity analysis
* Architectural compliance audits

CODEX should gather evidence and provide findings.

CODEX should avoid implementing significant code changes without architectural review.

Typical tasks:

* Identify extraction candidates
* Find architectural violations
* Locate dead code
* Measure complexity
* Trace dependencies

⸻

Claude (Implementation Engineer)

Responsibilities:

* Implement approved designs
* Perform refactoring
* Create pull requests
* Update documentation
* Remove temporary diagnostics

Claude should implement agreed architecture rather than invent new architecture.

⸻

GitHub Copilot

Responsibilities:

* Large-scale mechanical refactoring
* Repository-wide transformations
* Namespace cleanup
* Include cleanup
* File reorganization

GitHub Copilot should not make architecture-sensitive changes without prior review.

⸻

Standard Workflow

Phase 1 - Investigation

Performed by CODEX.

Goals:

* Gather evidence
* Analyze code
* Identify risks
* Produce findings

No implementation.

⸻

Phase 2 - Architecture Review

Performed by Chatty.

Goals:

* Review evidence
* Recommend approach
* Identify tradeoffs
* Assess field risk

No implementation.

⸻

Phase 3 - Implementation

Performed by Claude.

Goals:

* Implement approved design
* Maintain coding standards
* Preserve behavior

⸻

Phase 4 - Validation

Performed by Chip.

Goals:

* Verify functionality
* Evaluate maintainability
* Confirm acceptable risk

⸻

Phase 5 - Cleanup

Performed by Claude.

Goals:

* Remove diagnostics
* Remove temporary test code
* Remove obsolete comments
* Update documentation

⸻

Engineering Principles

When tradeoffs exist:

1. Prefer simpler solutions.
2. Prefer proven solutions.
3. Prefer maintainability over cleverness.
4. Prefer reliability over new features.
5. Prefer evidence over assumptions.

⸻

Current Project Focus

Current priorities:

1. Reliability
2. Maintainability
3. Power efficiency
4. Observability
5. Reduction of complexity

Prefer concise Doxygen comments that explain intent and operational significance. Detailed design explanations belong in docs/architecture/*.md, not in source code.

Not current priorities:

* New sensors
* New hardware
* New features

The project is currently in a stabilization and hardening phase.