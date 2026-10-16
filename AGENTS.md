# Repository agent rules

## Development compatibility policy

Lost Signal is under active development. Backward compatibility with earlier internal APIs, data layouts,
configuration shapes, save files, or unreleased extension contracts is not required unless the user explicitly
requests it for a specific boundary.

When an architecture or algorithm contract is wrong or ambiguous:

- replace it with the clearest correct contract;
- update every in-repository caller, test, asset, schema, and document in the same change;
- prefer one canonical representation over compatibility shims or parallel old/new representations;
- increase an exposed API or schema version when its meaning changes;
- do not add deprecated aliases, silent field reinterpretation, migration adapters, or optional legacy behavior
  solely to preserve unreleased code.

External standards and future bridges remain explicit boundaries. Compatibility work begins only when the project
declares a released public contract or the user asks for a particular compatibility target.
