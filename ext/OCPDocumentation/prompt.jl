"""
$(TYPEDSIGNATURES)

Generates a well-structured and precise prompt to produce Julia docstrings in the Documenter.jl style, using provided code, tests, and context.

# Arguments

- `code_text::String`: The Julia code (structs and functions) to document.
- `complement_text::String`: Optional complement to the prompt.
- `tests_text::String`: Optional related tests for improving examples.
- `context_text::String`: Additional domain knowledge or technical explanation to improve doc quality.

# Returns

- `prompt::String`: A clear prompt ready for use with a language model like ChatGPT or Mistral.

# Example

```julia-repl
julia> code = "function square(x); x^2; end"
julia> OptimalControlProblems.generate_prompt(code, "Write in UK english.", "", "")
"Your task is to write docstrings for the following Julia code..."
```
"""
function OptimalControlProblems.generate_prompt(
    code_text::String, complement_text::String, tests_text::String, context_text::String
)
    prompt = """
You are a Julia expert. Your task is to generate complete and idiomatic Julia docstrings for each `struct` or `function` in the code below.

Follow **Documenter.jl** standards precisely.

$complement_text

---

## ✅ What to do

- Place the docstring **immediately above** the corresponding declaration.
- For **structs**, start the docstring with `\"\"\"` and `\$(TYPEDEF)`.
- For **functions**, start the docstring with `\"\"\"` and `\$(TYPEDSIGNATURES)`.
- Write a **clear, concise** description of what the item does.
- For a **struct**, include a `# Fields` section (name, type, short description).
- For a **function**, include:
  - `# Arguments`: List and explain each argument.
  - `# Returns`: Describe what is returned (if applicable).
- Add a `# Example` with a `julia-repl` block showing basic usage.

---

## 🚫 What *not* to do

- ❌ Do not invent new code, functions, structs, or examples.
- ❌ Do not move or reorder the code.
- ❌ Do not rename arguments or fields.
- ❌ Do not include unrelated commentary or headers.

---

## 📦 Templates

### Template A — Structs

```julia
\"\"\"
\$(TYPEDEF)

[One-sentence description of what this struct represents.]

# Fields

- `[field_name]::[Type]`: [Short description.]

# Example

```julia-repl
julia> [usage example]
[expected output]
```
\"\"\"
[struct declaration]
```

---

### Template B — Functions

```julia
\"\"\"
\$(TYPEDSIGNATURES)

[One-sentence description of what this function does.]

# Arguments

- `[arg_name]::[Type]`: [Description.]

# Returns

- `[return_value]::[Type]`: [Description.]

# Example

```julia-repl
julia> [usage example]
[expected output]
```
\"\"\"
[function declaration]
```

---

## 🔧 Input

### BEGIN CODE
$code_text
### END CODE
"""

    if !isempty(tests_text)
        prompt *= """

### BEGIN TESTS
$tests_text
### END TESTS
"""
    end

    if !isempty(context_text)
        prompt *= """

### BEGIN CONTEXT
$context_text
### END CONTEXT
"""
    end

    prompt *= """

---

🧾 Return your answer in a single Julia code cell using **four backticks and the `julia` language tag**, like this:

\`\`\`\`julia
# your documented code here
\`\`\`\`

Strictly follow the structure above and complete a docstring for **each element** (struct or function).
Place each docstring **immediately above** its corresponding declaration.
Do **not** alter the provided code.
"""

    return prompt
end