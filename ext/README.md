# The problems and their modelings

How the problems are stored, and how to add a problem or a modeling. Nothing here lists the
problems: the lists are computed from the files.

## Layout

```text
ext/
├── MetaData/<problem>.jl          # metadata of every problem: one file each
├── MetaData/references/<problem>.toml  # reference objectives, generated (tools/references)
├── Descriptions/<problem>.md      # description of every problem: one file each
├── <Modeling>Models/<problem>.jl  # the problem in a modeling, one directory per modeling
├── JuMPModels.jl                  # extension of the JuMP modeling
└── OptimalControlModels.jl        # extension of the OptimalControl modelings
```

- A problem **exists** in a modeling when its file `<Modeling>Models/<problem>.jl` does. This is
  the only source of truth: `problems(backend)` reads it, and no list of problems is written by
  hand anywhere (a test checks it).
- The modelings are listed once, in `MODELING_SPECS` (`src/OptimalControlProblems.jl`): name,
  backend type, directory, weak dependency.
- Each modeling returns the most abstract object it can represent: OptimalControl returns the
  continuous problem (`Problem`), JuMP a discretized model.

## Add a problem

1. **Metadata**, `MetaData/<problem>.jl`: an `OrderedDict` named `<problem>_meta` with
   - `:grid_size`, the default number of steps;
   - `:parameters`, a `NamedTuple` of the default parameters, or `nothing`;
   - `:status`, one entry per modeling where the problem has a file, `:draft` to start with;
   - `:init`, the initial guess shared by every modeling, a function of the parameters
     returning `(state=…, control=…, variable=…)`;
   - optionally `:notes` (why a modeling is a draft) and `:reference_solution` (the analytic
     solution of the continuous problem, see `src/solutions.jl`).

   The schema is `METADATA_SCHEMA` in `src/OptimalControlProblems.jl`, checked when the
   package loads.
2. **Description**, `Descriptions/<problem>.md`: the problem in words and its mathematical
   formulation, and a `### References` section with at least one entry (checked).
3. **Models**: one file per modeling, `<Modeling>Models/<problem>.jl`. Copy the file of a
   problem with the same features (kind of cost, free times, constraints) in the same modeling:
   - OptimalControl: a method `OptimalControlProblems.<problem>(::OptimalControlBackend;
     parameters, init)` that writes the model with `@def` and returns
     `OptimalControlProblems._problem(…)`;
   - scalarized OptimalControl: the same model with scalar components only;
   - JuMP: a method that declares the problem as `ProblemData` (continuous functions, no
     discretization) and calls `build`; the schemes are written once, in
     `JuMPModels/schemes/`.
4. **Test it**: `Pkg.test(; test_args=["problem=<problem>"])` runs every test on this problem
   only (see `test/README.md`). The consistency tests check the files and the metadata; the
   smoke tests that it builds and runs; the equivalence tests that two modelings give the same
   NLP.
5. **Valid**: once the tests pass in a modeling, set its status to `:valid`. A problem valid in
   one modeling only needs a reference objective (the comparison rule, checked).
6. **Reference objective**: computed from the convergence crons by
   `tools/references/update.jl` (see `tools/references/README.md`).

The documentation page of the problem is generated from these files: nothing to write in
`docs/`.

## Add a modeling

1. A backend type and an entry in `MODELING_SPECS` (`src/OptimalControlProblems.jl`).
2. A directory `ext/<Modeling>Models/` and the extension that includes its files, declared in
   `Project.toml` (`[weakdeps]`, `[extensions]`).
3. A test environment, `test/suite/<Modeling>/` (see `test/README.md`).
4. A **documentation adapter** in `docs/modelings.jl`: display name, one-sentence summary,
   packages to load, the code to get and solve a problem, the first line of the model in a
   problem file, and the modeling it is checked equal to. Every page loops over the
   modelings: the new one then appears on the Installation page, on every problem page (its
   own tab, and in the comparison) and on the Status page, without other change. The build
   fails if a modeling has no adapter.
