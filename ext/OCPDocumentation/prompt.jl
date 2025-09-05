function OptimalControlProblems.generate_prompt(problem::String)
    code_url = "https://raw.githubusercontent.com/control-toolbox/OptimalControlProblems.jl/main/ext/OptimalControlModels/$(problem).jl"
    metadata_url = "https://raw.githubusercontent.com/control-toolbox/OptimalControlProblems.jl/main/ext/MetaData/$(problem).jl"

    prompt = """
You are an expert in optimal control and scientific writing.  
Your task is to generate a **problem description** for the OptimalControlProblems.jl library.

The description must follow exactly the style and structure used in the existing problem descriptions, for example:
- https://raw.githubusercontent.com/control-toolbox/OptimalControlProblems.jl/110-general-review-the-documentation-of-the-problems/ext/Descriptions/chain.md
- https://raw.githubusercontent.com/control-toolbox/OptimalControlProblems.jl/110-general-review-the-documentation-of-the-problems/ext/Descriptions/dielectrophoretic_particle.md

---

## ✅ What to do

1. Carefully analyze the provided Julia problem definition code.  
   - Source: $code_url
2. Use the metadata file to determine important characteristics such as the **final time** (fixed or free).  
   - Metadata: $metadata_url
3. Write a clear, structured description in **Markdown** with the following sections:

### Problem description  
Explain the physical or mathematical system and the control objective.  

### Mathematical formulation  
Write the optimal control problem using math notation:
- Dynamics
- Objective (Mayer/Lagrange/Bolza)
- Initial and terminal conditions
- Constraints  

### System parameters  
List all parameters defined in the code, with symbol, type/unit, and meaning.  

### Qualitative behaviour  
Explain qualitatively how the solution behaves depending on parameter values.  

### Characteristics  
Summarize the structural properties of the problem:
- Linear or nonlinear
- Free or fixed final time
- State constraints present?
- Control constraints present?  

### References  
Provide 2–3 scientific references (articles, textbooks, benchmark studies).  
For each reference, explain its relevance to the problem.  

---

## 🚫 What *not* to do

- ❌ Do not invent new dynamics, parameters, or objectives not in the code.  
- ❌ Do not change variable names from the code.  
- ❌ Do not add unrelated references.  

---

## 🔧 Input

Use the following problem definition and metadata:

- Problem code: $code_url  
- Problem metadata: $metadata_url  

---

## 📦 Output format

🧾 Return your answer in a single Markdown code cell using **four backticks**, with sections:

- [description]  
- Mathematical formulation (remark: h3 title)
- System parameters (remark: h3 title)
- Qualitative behaviour (remark: h3 title)
- Characteristics (remark: h3 title)
- References (remark: h3 title)

Strictly follow the formatting conventions of the existing descriptions.
"""

    return prompt
end
