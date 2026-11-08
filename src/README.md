# Source of the package

The package is split into submodules, one per responsibility, each in its own directory
`src/<Name>/<Name>.jl` (ct-conventions `MOD1`). A manifest `<Name>.jl` only declares: its
imports, its files, its exports (`MOD2`). The definitions are in the other files of the
directory.

## Submodules

In dependency order: a submodule uses only those above it (`MOD7`).

| Submodule | Holds |
| --- | --- |
| `Modelings` | the backend types and the modeling table `MODELING_SPECS`, the single place where a modeling is declared |
| `Metadata` | the list of the problems (the files of `ext/MetaData/`), the schema `METADATA_SCHEMA`, loading and checking the metadata, reading it (`problem_metadata`, `parameters_data`, `initial_guess_data`…), the analytic solutions used by the metadata files |
| `References` | the reference objectives and known solutions (`ext/MetaData/references/`), `reference_objective`, `solution_kind`, `reference_solution_data`, `SAME_SOLUTION`; the references as values (#409): `AbstractReference`, `NumericalReference` (a known solution and how it was obtained), `AnalyticReference`, `references`, `best_reference`, `analytic_reference`, `objective(ref)` |
| `Catalogue` | one function per problem (`beam`…), whose methods the extensions define, `problems`, `status`, the fingerprint of the files that define a problem |
| `Problems` | `Problem`, `features`, `solution`, and the accessors shared with OptimalControl (`state`, `objective`…) |

## What is public

- The **package exports** its main API (`problems`, the backends, the problem functions,
  `Problem`, the accessors…). Unlike the internal CT packages, it is used directly
  (decision of #413), so `using OptimalControlProblems` brings them into scope.
- Each **submodule exports** its public API, and the package loads every submodule, so all of
  it is reachable from the package: `OptimalControlProblems.status(:beam)`,
  `OptimalControlProblems.reference_objective(:beam)`.
- What a submodule does **not** export is internal, reached through the submodule:
  `OptimalControlProblems.Metadata.METADATA_SCHEMA`,
  `OptimalControlProblems.Catalogue._warn_if_draft`. The extensions and the tests call these
  internals this way.

`test/suite/core/test_modules.jl` checks this layout: a directory per submodule, every
export of the package defined in a submodule, every export of a submodule reachable from the
package.

## Add a file, a function, a submodule

- A file: in the directory of its submodule, included by the manifest. The API reference of
  the documentation reads the files of each directory (`docs/api_reference.jl`).
- A public function: exported by its submodule; also exported by the package if users call
  it unqualified.
- A submodule: `src/<Name>/<Name>.jl`, included and loaded (`using .<Name>`) by
  `src/OptimalControlProblems.jl` after the submodules it uses.

A file inside a submodule is never named after the submodule in another case
(`metadata.jl` in `Metadata/`): on macOS and Windows, file names ignore case, and it would
overwrite the manifest.
