# `mesh2solid` Architecture Checkpoint

This document captures where the project is today and the architecture we are
actively moving toward.

## Why We Are Changing Direction

The current exporter has been successful because it solved real geometry
families quickly:

- simple boxes and brackets
- through-bores
- blind bores
- counterbores
- countersinks
- bosses and standoffs
- obround slots
- mixed cylindrical features on planar bodies
- smooth side-profile bands on cap-pair bodies

That work proved the geometric direction is right, but it also produced too many
feature-specific detectors and STEP writers. The failure mode is predictable:
every new mechanical combination risks adding another bespoke code path.

We do not want to reverse engineer Fusion 360 commands one by one. The same
final body can come from many different CAD histories, and the mesh does not
preserve the original timeline anyway.

The new architecture is therefore geometry-first, not UI-history-first.

## Core Goal

Recover a small set of canonical geometric generators and emit a clean analytic
B-rep from them.

For the mechanical scope we care about, that means:

- planar half-space reconstruction for polyhedral bodies
- prismatic sweep reconstruction for sketch-plus-extrude bodies
- revolved/revolution-like reconstruction for cylinders, cones, and similar
  bodies
- blend recovery for common smooth transitions such as fillets
- a faceted fallback for cases that do not admit a confident analytic model yet

## Target Geometry Model

### 1. Planar Branch

Use the existing mesh repair, plane fitting, regularization, and constraint
graph pipeline to recover clean planar faces and their adjacency.

This branch is the right solution for:

- boxes
- brackets
- enclosures
- planar shells
- polyhedral cutouts

### 2. Prismatic Sweep Branch

This is the main architectural investment for common mechanical parts.

A prismatic body is treated as:

- one or more planar cap pairs
- an extrusion axis for each cap pair
- 2D profile loops in a basis orthogonal to that axis
- axial event planes where topology or radius changes
- analytic side surfaces implied by sweeping those 2D curves along the axis

The important abstraction is not “bore”, “counterbore”, or “countersink”.
Those are all specific loop-evolution patterns inside one generic section model.

We want to represent them as:

- constant-radius axial segments -> cylinders
- linearly varying radius segments -> cones
- stepped axial segments -> planar annular step faces
- line/arc/spline profile segments -> planar, cylindrical, or translational
  spline side faces

### 3. Revolved Branch

Treat axisymmetric geometry as a profile rotated about an axis. This is how we
will eventually generalize beyond simple cylinders and cones toward more
revolved mechanical bodies.

### 4. Blend Branch

Fillets and similar smooth transitions should be handled as blends layered on
top of the base body model, not as independent ad hoc feature families.

For common mechanical cases, many faceted “fillets” are still simple extruded
arcs that belong in the prismatic sweep branch. True edge blends belong here.

### 5. Fallback Branch

When we cannot confidently recover a clean analytic model, keep the current
repaired closed-mesh fallback path. It is still valuable, but it should stay a
fallback rather than the organizing principle of the geometry engine.

## What “Generalized” Means In Practice

For the prismatic branch, we are standardizing on a section/sweep model:

- detect the dominant cap pair
- build a 2D section basis
- identify inner and outer loop families
- classify each loop family by its axial evolution
- emit a single analytic writer from that canonical description

That lets multiple feature classes share one representation:

- bore
- repeated bore pattern
- countersink
- counterbore
- mixed bore + countersink
- mixed bore + counterbore
- future mixed shared-cap combinations

The same idea extends later to:

- blind feature columns
- bosses / posts / standoffs
- obround and other profile-derived slots
- extruded arc bands and similar fillet-like side walls

## Current Checkpoint

Today the codebase has:

- a strong planar reconstruction pipeline
- optional CGAL plane regularization
- Open CASCADE-assisted fallback and export support
- a large but better-organized STEP export layer split into focused fragments
- an existing shared-cap axial through-feature writer that already behaves like a
  small canonical model

This existing through-feature path is the seed of the new architecture.

## Implementation Strategy

We are rebuilding incrementally rather than throwing away the working code.

### Phase 1: Canonical Shared-Cap Axial Model

Unify the current through-feature family under one internal model driven by:

- a shared cap pair
- outer cap loops
- one or more axial feature columns
- per-column analytic segments

This phase should subsume:

- single bore
- repeated bore pattern
- single counterbore
- single countersink
- mixed through-feature combinations

### Phase 2: Canonical Blind / Support Feature Models

Apply the same modeling idea to:

- blind bores
- blind counterbores
- blind countersinks
- bosses
- repeated bosses
- standoffs

### Phase 3: Profile-Derived Side Walls

Move extruded profile bands, obround slots, and fillet-like extruded arcs onto
the same “recover 2D profile curves, then sweep them” foundation.

### Phase 4: Blend And Revolved Expansion

Add higher-level treatment for:

- conical seats beyond simple axial countersinks
- more general revolved bodies
- true blends/fillets that are not just extruded profile arcs

## Success Criteria

We still evaluate on the example fixtures and benchmark models already in the
repo, but those examples are not the architecture.

The architecture is successful when:

- new mechanical feature combinations can often be expressed by adapting a
  generic section model rather than adding a new one-off writer
- simple fixtures remain stable
- hard public benchmarks do not regress
- the code gets smaller or at least flatter in conceptual complexity even as
  capability improves
- new models succeed because they match the geometry model, not because the code
  explicitly recognizes their filename or topology pattern

## Non-Goal

This project is not trying to perfectly reconstruct a user’s original Fusion 360
timeline. It is trying to recover a robust analytic solid representation that is
good enough to edit, inspect, and reuse downstream.
