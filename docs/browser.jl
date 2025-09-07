using JSON
using DataFrames
using Tables
using OptimalControlProblems
using NLPModels
using CTModels
using FilePathsBase  # optional, for robust path handling
using Base.Filesystem: rm, mktemp

function generate_constraint_buttons_html(constraint_dims::NamedTuple)
    total_constraints = sum(values(constraint_dims))
    buttons_html = join([begin
        """<button class="constraint-btn" data-type="$key" data-dim="$dim">$key</button>"""
    end for (key, dim) in pairs(constraint_dims)], "\n")
    return """<span class='constraints-wrapper' data-order='$total_constraints'>
                $buttons_html <strong style='margin-left:5px;'>($total_constraints)</strong>
              </span>"""
end

# ---------------------------
# --- Data Collection ---
# ---------------------------
function collect_ocp_data()
    data_ocp_rows = [
        let ocp = ocp_model(eval(problem_sym)(OptimalControlBackend()))
            
            # Logic to determine the cost type
            cost = if has_mayer_cost(ocp) && has_lagrange_cost(ocp)
                "Bolza"
            elseif has_mayer_cost(ocp)
                "Mayer"
            else
                "Lagrange"
            end

            # Determine if the final time is fixed or free
            final_time = has_fixed_final_time(ocp) ? "fixed" : "free"

            # Get dimensions for each constraint type
            dims = (
                x=CTModels.dim_state_constraints_box(ocp),
                u=CTModels.dim_control_constraints_box(ocp),
                v=CTModels.dim_variable_constraints_box(ocp),
                c=CTModels.dim_path_constraints_nl(ocp),
                b=CTModels.dim_boundary_constraints_nl(ocp)
            )

            # Calculate the total number of constraints
            total_constraints = sum(values(dims))

            # Generate HTML for the constraint buttons
            constraint_buttons_html = generate_constraint_buttons_html(dims)

            # Return a NamedTuple with all the data
            (
                Problem=string(problem_sym),
                State=state_dimension(ocp),
                Control=control_dimension(ocp),
                Variable=variable_dimension(ocp),
                Cost=cost,
                FinalTime=final_time,
                DimStateConstraint=dims.x,
                DimControlConstraint=dims.u,
                DimVariableConstraint=dims.v,
                DimPathConstraint=dims.c,
                DimBoundaryConstraint=dims.b,
                TotalConstraints=total_constraints,
                ConstraintButtonsHtml=constraint_buttons_html
            )
        end
        for problem_sym in problems()
    ]
    
    # Convert the vector of named tuples to a DataFrame and return it
    return DataFrame(data_ocp_rows)
end

# ---------------------------
# --- Generate JSON ---
# ---------------------------
function ocp_data_to_json(data_ocp::DataFrame)
    return JSON.json(Tables.columntable(data_ocp))
end

# ---------------------------
# --- Helper: Insert table CSS ---
# ---------------------------
function insert_table_style(io)
    write(io, """
<style>
:root {
    /* ==============================
       Base Palette
       ============================== */
    --color-dark-blue:  #003d4d;   /* very dark blue */
    --color-deep-blue:  #005f73;   /* deep blue */
    --color-bright-blue:#0096a0;   /* bright blue */
    --color-orange:     #f18f01;   /* orange / ochre */
    --color-soft-red:   #d72638;   /* soft red */
    --color-deep-violet:#6a0572;   /* deep violet */
    --color-bootstrap-blue: #007BFF;
    --color-light-gray: #ddd;
    --color-gray-text:  #666;
    --color-table-bg:   #f0f4f8;
    --color-table-bg-alt:#e6f2ff;
    --color-soft-green: #2F8F3F;
    --color-soft-green-lighter: #246B32;

    /* ==============================
       Semantic Colors
       ============================== */
    --color-problem: var(--color-dark-blue);
    --color-state:   var(--color-dark-blue);
    --color-control: var(--color-dark-blue);
    --color-variable:var(--color-dark-blue);
    --color-cost:    var(--color-dark-blue);
    --color-final:   var(--color-dark-blue);
    --color-constraints: var(--color-dark-blue);

    /* Constraint buttons */
    --btn-x: #ddd;
    --btn-u: var(--color-light-gray);
    --btn-v: var(--color-light-gray);
    --btn-c: var(--color-light-gray);
    --btn-b: var(--color-light-gray);

    /* DataTables Filter buttons */
    --btn-filters-active: var(--color-soft-green);
    --btn-filters-disabled: var(--color-light-gray);
    --btn-filters-enabled: var(--color-bootstrap-blue);
    --btn-filters-hover: var(--color-soft-green-lighter);

    /* DataTables Constraints buttons */
    --btn-constraints-active: var(--color-soft-green);
    --btn-constraints-disabled: var(--color-light-gray);
    --btn-constraints-enabled: var(--color-bootstrap-blue);
    --btn-constraints-hover: var(--color-soft-green-lighter);
}

/* ==============================
   Table Styles
   ============================== */
#problems-table {
    width: 100%;
    border-collapse: collapse;
}

#problems-table thead th {
    background: linear-gradient(to bottom, var(--color-table-bg), var(--color-table-bg-alt));
    text-align: center;
    padding: 6px 8px;
    border-bottom: 2px solid var(--color-light-gray);
}

#problems-table thead th:nth-child(1) { color: var(--color-problem); }
#problems-table thead th:nth-child(2) { color: var(--color-state); }
#problems-table thead th:nth-child(3) { color: var(--color-control); }
#problems-table thead th:nth-child(4) { color: var(--color-variable); }
#problems-table thead th:nth-child(5) { color: var(--color-cost); }
#problems-table thead th:nth-child(6) { color: var(--color-final); }
#problems-table thead th:nth-child(7) { color: var(--color-constraints); }

#problems-table tbody td {
    padding: 6px 8px;
    text-align: left;
}

#problems-table tbody tr:nth-child(even) {
    background-color: var(--color-table-bg-alt);
}

#problems-table tbody tr:hover {
    background-color: var(--color-table-bg-alt);
    cursor: pointer;
}

/* ==============================
   DataTables Controls
   ============================== */
div.dataTables_wrapper div.dataTables_length {
    margin-bottom: 8px;
}

.dt-top-buttons {
    margin-bottom: 6px;
}

.dt-top-buttons .dt-buttons {
    display: flex;
    gap: 8px;
    align-items: center;
    margin-bottom: 20px;
}

.dt-top-buttons .dt-buttons button {
    margin: 0;
    padding: 6px 10px;
}

.dt-buttons button i {
    font-size: 1.2em;
    vertical-align: middle;
}

.dt-buttons button {
    padding: 4px 8px!important;
}

/* Export buttons colors */
.dt-buttons .buttons-copy { color: #0096a0!important;}
.dt-buttons .buttons-csv  { color: #f18f01!important;}
.dt-buttons .buttons-excel{ color: #2F8F3F!important;}
.dt-buttons .buttons-pdf  { color: #d72638!important;}
.dt-buttons .buttons-print{ color: #6a0572!important;}

.dt-buttons button:hover {
    opacity: 0.85;
    transform: scale(1.03);
}

.dt-top-controls {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 12px;
    margin-bottom: 8px;
    width: 100%;
}

.dt-top-controls .dataTables_length {
    margin: 0;
}

.dt-top-controls .dataTables_length label {
    margin: 0;
    display: flex;
    align-items: center;
    gap: 6px;
}

.dt-top-controls .dataTables_length select {
    min-width: 70px;
}

.dt-top-controls .dataTables_filter {
    margin: 0;
}

.dt-top-controls .dataTables_filter label {
    margin: 0;
    display: flex;
    align-items: center;
    gap: 6px;
}

.dt-top-controls .dataTables_filter input {
    width: 220px;
    max-width: 40vw;
    padding: 6px 8px;
    box-sizing: border-box;
}

/* responsive: stack controls vertically on narrow screens */
@media (max-width: 680px) {
    .dt-top-controls {
        flex-direction: column;
        align-items: stretch;
    }
    .dt-top-controls .dataTables_filter input {
        width: 100%;
    }
}

/* ==============================
   Constraint Buttons (Table Rows)
   ============================== */
.constraint-btn {
    border-radius: 12px;
    padding: 4px 8px;
    margin: 0px;
    font-size: 0.85em;
    font-weight: bold;
    border: none;
    cursor: pointer;
    transition: 0.2s;
}

.constraint-btn[data-dim="0"] { background-color: var(--btn-constraints-disabled); color: var(--color-gray-text); }
.constraint-btn[data-dim]:not([data-dim="0"]) { background-color: var(--btn-constraints-enabled); color: white; }

.constraint-btn[data-type="x"] { background-color: var(--btn-x); }
.constraint-btn[data-type="u"] { background-color: var(--btn-u); }
.constraint-btn[data-type="v"] { background-color: var(--btn-v); }
.constraint-btn[data-type="c"] { background-color: var(--btn-c); }
.constraint-btn[data-type="b"] { background-color: var(--btn-b); color: white; }

.constraints-wrapper strong {
    padding: 0px 0px;
    border-radius: 4px;
    font-size: 0.95em;
    font-weight: bold;
    transition: background-color 0.3s, transform 0.2s;
}

/* ==============================
   Constraint Filter Buttons (Header)
   ============================== */
.constraint-filter-btn {
    border-radius: 6px;
    padding: 3px 7px;
    margin: 1px;
    font-size: 0.85em;
    font-weight: bold;
    border: none;
    transition: background-color 0.2s, transform 0.1s;
    cursor: pointer;
    background-color: var(--btn-filters-disabled);
    color: #333;
}

.constraint-filter-btn:hover {
    background-color: var(--btn-filters-hover);
    color: white;
    transform: scale(1.05);
}

.constraint-filter-btn.active {
    background-color: var(--btn-filters-active);
    color: white;
}

/* ==============================
   Filters (Numeric Inputs / Selects)
   ============================== */
#problems-table thead input[type="text"] {
    width: 90%!important;
    max-width: 55px;
    padding: 2px 4px;
    font-size: 0.85em;
    border: 1px solid var(--color-light-gray);
    border-radius: 4px;
    text-align: center;
}

#problems-table thead select {
    width: 95%;
    max-width: 80px;
    padding: 2px 2px;
    font-size: 0.85em;
    border: 1px solid var(--color-light-gray);
    border-radius: 4px;
    background: #fff;
    text-align: center;
}

/* Constraint Filter Container */
#constraints-filter {
    text-align: center;
}

#constraints-filter > div:first-child {
    display: flex!important;
    flex-direction: column;
    align-items: center;
    justify-content: center!important;
    padding: 2px;
}

#constraints-filter select {
    margin-left: 4px;
    padding: 2px 5px;
    font-size: 0.85em;
    border-radius: 4px;
    margin-bottom: 5px;
}

#constraints-filter > div > div {
    display: flex!important;
    justify-content: left!important;
}

#constraints-filter .btn-label {
    font-size: 0.75em;
    line-height: 1.1;
    margin-top: 2px;
    color: #333;
}

/* Reduce spacing of sort arrows (before and after) */
table.dataTable thead .sorting::before,
table.dataTable thead .sorting::after,
table.dataTable thead .sorting_asc::before,
table.dataTable thead .sorting_asc::after,
table.dataTable thead .sorting_desc::before,
table.dataTable thead .sorting_desc::after,
table.dataTable thead .sorting_asc_disabled::before,
table.dataTable thead .sorting_asc_disabled::after,
table.dataTable thead .sorting_desc_disabled::before,
table.dataTable thead .sorting_desc_disabled::after {
    right: 2px !important;   /* adjust distance from right edge */
}
</style>
""")
end

function insert_logic_script(io)
    write(io, """
<script>
document.addEventListener("DOMContentLoaded", function() {
    const data = JSON.parse(document.getElementById("problems-json").textContent);

    const table = \$('#problems-table').DataTable({
        orderCellsTop: true,
        fixedHeader: true,
        responsive: true,
        pageLength: 25,
        lengthMenu: [ [10, 25, 50, -1], [10, 25, 50, "All"] ],
        dom: '<"dt-top-buttons"B><"dt-top-controls"lf>rt<"bottom"ip><"clear">', 
        buttons: ['copy', 'csv', 'excel', 'pdf', 'print'],
        autoWidth: false,
        columns: [
            { data: 'Problem', width: '20%' },
            { data: 'State', width: '10%' },
            { data: 'Control', width: '12%' },
            { data: 'Variable', width: '12%' },
            { data: 'Cost', width: '10%' },
            { data: 'FinalTime', width: '14%' },
            { data: 'ConstraintButtonsHtml',
              render: function(data, type, row) {
                  if (type === 'sort' || type === 'type') return Number(row.TotalConstraints);
                  return data;
              }
            }
        ],
        buttons: [
            { extend: 'copy', text: '<i class="fas fa-copy"></i>', titleAttr: 'Copy to clipboard' },
            { extend: 'csv',  text: '<i class="fas fa-file-csv"></i>', titleAttr: 'Download CSV' },
            { extend: 'excel', text: '<i class="fas fa-file-excel"></i>', titleAttr: 'Download Excel' },
            { extend: 'pdf', text: '<i class="fas fa-file-pdf"></i>', titleAttr: 'Download PDF' },
            { extend: 'print', text: '<i class="fas fa-print"></i>', titleAttr: 'Print table' }
        ]
    });

    (function() {
        const topButtons = \$('.dt-top-buttons').first();
        const dtButtons = \$('.dt-buttons').first();
        if(topButtons.length && dtButtons.length && dtButtons.parent().get(0) !== topButtons.get(0)) {
            topButtons.empty().append(dtButtons);
        }
        const topControls = \$('.dt-top-controls').first();
        if(topControls.length) {
            const length = \$('.dataTables_length').first();
            const filter = \$('.dataTables_filter').first();
            if(length.length && length.parent().get(0) !== topControls.get(0)) topControls.append(length);
            if(filter.length && filter.parent().get(0) !== topControls.get(0)) topControls.append(filter);
            const inp = topControls.find('.dataTables_filter input').first();
            if(inp.length) {
                inp.attr('placeholder', 'Search...');
                inp.css({ 'width': '220px', 'display': 'inline-block' });
            }
        }
    })();

    const constraintTypes = ["x","u","v","c","b"];

    \$('#problems-table tbody').on('mouseenter', '.constraint-btn', function(){
        const count = \$(this).data('dim');
        \$(this).attr('title', `\${\$(this).data('type').toUpperCase()}: \${count} constraints`);
    });

    // --- Build table rows with clickable detail ---
    data.Problem.forEach((_, i) => {
        const totalConstraints = data.TotalConstraints[i];
        const buttonsWrapperHtml = data.ConstraintButtonsHtml[i];

        const rowNode = table.row.add({
            Problem: data.Problem[i],
            State: data.State[i],
            Control: data.Control[i],
            Variable: data.Variable[i],
            Cost: data.Cost[i],
            FinalTime: data.FinalTime[i],
            ConstraintButtonsHtml: buttonsWrapperHtml,
            TotalConstraints: totalConstraints,
            DimStateConstraint: data.DimStateConstraint[i],
            DimControlConstraint: data.DimControlConstraint[i],
            DimVariableConstraint: data.DimVariableConstraint[i],
            DimPathConstraint: data.DimPathConstraint[i],
            DimBoundaryConstraint: data.DimBoundaryConstraint[i]
        }).draw(false).node();

        // --- Clickable row: show/hide constraint details ---
        \$(rowNode).on('click', function() {
            const row = table.row(this);
            if (row.child.isShown()) {
                row.child.hide();
            } else {
            const detailHtml = `<div style="display:flex; gap:4px; align-items:center;">
                <div style="width:60px;background:#003d4d;color:white;text-align:center;border-radius:4px;">x: \${data.DimStateConstraint[i]}</div>
                <div style="width:60px;background:#005f73;color:white;text-align:center;border-radius:4px;">u: \${data.DimControlConstraint[i]}</div>
                <div style="width:60px;background:#0096a0;color:white;text-align:center;border-radius:4px;">v: \${data.DimVariableConstraint[i]}</div>
                <div style="width:60px;background:#f18f01;color:white;text-align:center;border-radius:4px;">c: \${data.DimPathConstraint[i]}</div>
                <div style="width:60px;background:#d72638;color:white;text-align:center;border-radius:4px;">b: \${data.DimBoundaryConstraint[i]}</div>
                <div style="font-weight:bold;margin-left:10px;">Total: \${totalConstraints}</div>
            </div>`;
                row.child(detailHtml).show();
            }
        });
    });

    // --- Filters and constraint header buttons ---
    \$('#problems-table thead tr#filters th').each(function(i) {
        if(i===4){
            \$(this).html('<select><option value="">All</option><option>Mayer</option><option>Lagrange</option><option>Bolza</option></select>');
        } else if(i===5){
            \$(this).html('<select><option value="">All</option><option>fixed</option><option>free</option></select>');
        } else if(i>0 && i<6){
            \$(this).html('<input type="text" placeholder="min-max" style="width:100%"/>');
        }
        \$('input, select', this).on('keyup change', function(){
            const val = \$(this).val();
            if(val.includes('-')){
                const [min,max] = val.split('-').map(Number);
                \$.fn.dataTable.ext.search.push(function(settings, data_row){
                    const v = parseFloat(data_row[i])||0;
                    return v>=min && v<=max;
                });
                table.draw();
                \$.fn.dataTable.ext.search.pop();
            } else {
                table.column(i).search(val).draw();
            }
        });
    });

    // --- Constraint filter buttons ---
    const filterContainer = \$('#constraints-filter');
    let html = `<div class="constraint-filter-wrapper">
                    <div class="constraint-logic">
                        Logic:
                        <select id="constraints-logic">
                            <option value="OR">OR</option>
                            <option value="AND">AND</option>
                        </select>
                    </div>
                    <div class="constraint-buttons-row">`;
    constraintTypes.forEach(c => {
        html += `<button class="constraint-filter-btn" data-type="\${c}">\${c}</button>`;
    });
    html += `</div></div>`;
    filterContainer.append(html);

    \$(document).on('click', '.constraint-filter-btn', function() {
        \$(this).toggleClass('active');
        applyConstraintFilter();
    });

    function applyConstraintFilter() {
        const active = [];
        \$('.constraint-filter-btn.active').each(function(){ active.push(\$(this).data('type')); });
        const logic = \$('#constraints-logic').val();

        \$.fn.dataTable.ext.search.push(function(settings, data_row, index){
            if(active.length === 0) return true;
            const rowNode = table.row(index).node();
            const hasConstraints = active.map(c=>{
                const btn = \$(rowNode).find(`button[data-type="\${c}"]`)[0];
                return btn && \$(btn).data('dim') > 0;
            });
            return logic === "AND" ? hasConstraints.every(v=>v) : hasConstraints.some(v=>v);
        });
        table.draw();
        \$.fn.dataTable.ext.search.pop();
    }

    \$(document).on('change', '#constraints-logic', applyConstraintFilter);
});
</script>
""")
end

function generate_html_text_debut(io)
write(io, """
# Problems Browser

The table below provides an overview of all **optimal control problems** and allows interactive exploration, filtering, and export.  

!!! tip "Quick guide to the problems table"

    ```@raw html
    <details><summary>Click to unfold and see the quick guide for the table.</summary>
    ```

    ## Table Overview

    - **Problem:** The name of the optimal control problem.  
    - **State:** Number of state variables in the system.  
    - **Control:** Number of control inputs.  
    - **Variable:** Number of additional optimisation variables (if any).  
    - **Cost:** Type of cost functional:  
        - **Mayer:** a **point cost** depending on the initial and final states and optional variables.
        - **Lagrange:** integral over time  
        - **Bolza:** combination of Mayer + Lagrange  
    - **FinalTime:** Whether the final time is **fixed** or **free**.  
    - **Constraints:** Buttons representing each constraint type:  
        - **x:** state box constraints  
        - **u:** control box constraints  
        - **v:** variable box constraints  
        - **c:** nonlinear path constraints  
        - **b:** nonlinear boundary constraints  

    The number next to the buttons shows the **total number of constraints**. Hover over buttons to see the exact count. Click a row to see a detailed list of constraints.

    ## Interactivity & Filters

    - **Sorting & Search:** Click column headers to sort. Use the search box to filter by text.  
    - **Numeric Filters:** Enter `min-max` ranges in numeric columns to filter values.  
    - **Cost & FinalTime Filters:** Use dropdown menus to filter by cost type or whether the final time is fixed/free.  
    - **Constraint Filtering:** Use the buttons above the Constraints column to filter problems by constraint type. Choose **AND/OR logic** to combine multiple constraint types.  
    - **Export Buttons:** Use the top buttons to copy the table or export it to CSV, Excel, PDF, or print. Hover over each button to see its function.  

    ```@raw html
    </details>
    ```

---

Scroll through the table or use filters to quickly find problems of interest, inspect their constraints, and export data for further analysis.
""")
end

# ---------------------------
# --- Generate HTML + JS ---
# ---------------------------
function generate_html(md_path::AbstractString, json_str::String)
    mkpath(dirname(md_path))
    open(md_path, "w") do io

        generate_html_text_debut(io)
        
        write(io, """
```@raw html
""")

insert_table_style(io)

write(io, """
<div>
    <table id="problems-table" class="display nowrap" style="width:100%">
        <thead>
            <tr>
                <th>Problem</th>
                <th>State</th>
                <th>Control</th>
                <th>Variable</th>
                <th>Cost</th>
                <th>FinalTime</th>
                <th>Constraints</th>
            </tr>
            <tr id="filters">
                <th></th><th></th><th></th><th></th>
                <th></th><th></th>
                <th id="constraints-filter"></th>
            </tr>
        </thead>
        <tbody></tbody>
    </table>
</div>

<link rel="stylesheet" href="https://cdn.datatables.net/1.13.6/css/jquery.dataTables.min.css">
<link rel="stylesheet" href="https://cdn.datatables.net/buttons/2.4.1/css/buttons.dataTables.min.css">
<link rel="stylesheet" href="https://cdn.datatables.net/responsive/2.5.0/css/responsive.dataTables.min.css">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.6.0/css/all.min.css">

<script src="https://code.jquery.com/jquery-3.7.1.min.js"></script>
<script>var define=undefined;</script>
<script src="https://cdn.datatables.net/1.13.6/js/jquery.dataTables.min.js"></script>
<script src="https://cdn.datatables.net/buttons/2.4.1/js/dataTables.buttons.min.js"></script>
<script src="https://cdn.datatables.net/buttons/2.4.1/js/buttons.html5.min.js"></script>
<script src="https://cdn.datatables.net/buttons/2.4.1/js/buttons.print.min.js"></script>
<script src="https://cdn.datatables.net/responsive/2.5.0/js/dataTables.responsive.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/jszip/3.10.1/jszip.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/pdfmake/0.2.7/pdfmake.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/pdfmake/0.2.7/vfs_fonts.js"></script>

<script id="problems-json" type="application/json">
$json_str
</script>
""")

insert_logic_script(io)

write(io, """
```
""")
end
end

# ---------------------------
# --- Main function ---
# ---------------------------
function generate_problems_browser(md_path::AbstractString=joinpath(@__DIR__, "src", "problems_browser.md"))
    data_ocp = collect_ocp_data()
    json_str = ocp_data_to_json(data_ocp)
    generate_html(md_path, json_str)
    println("Problems browser page written to: $md_path")
    return md_path
end

# ---------------------------
# Example: generate, use, and remove
# ---------------------------
function with_problems_browser(f::Function)
    path = generate_problems_browser()  # generate file
    try
        return f(path)  # run your function using the generated file
    finally
        isfile(path) && rm(path)  # remove file if it exists
        println("Temporary problems browser file removed: $path")
    end
end
