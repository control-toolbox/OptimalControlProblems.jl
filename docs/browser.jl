using JSON
using DataFrames
using Tables
using OptimalControlProblems
using NLPModels
using CTModels
using FilePathsBase  # optional, for robust path handling
using Base.Filesystem: rm, mktemp

const BROWSER_FILE = "problems_browser.md"
const BROWSER_PATH = joinpath(@__DIR__, "src", BROWSER_FILE)

# -------------------------------
# MD + CSS + JS constants
# -------------------------------
const TABLE_PRESENTATION = """
# [Problems browser](@id problems-browser)

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
"""

const TABLE_STYLE = """
<style>
:root {
    /* ==============================
       Base Palette
       ============================== */
    --color-dark-blue:  #003d4d;
    --color-deep-blue:  #005f73;
    --color-bright-blue:#0096a0;
    --color-orange:     #f18f01;
    --color-soft-red:   #d72638;
    --color-deep-violet:#6a0572;
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

    /* Buttons */
    --btn-filters-active: var(--color-soft-green);
    --btn-filters-disabled: var(--color-light-gray);
    --btn-filters-enabled: var(--color-bootstrap-blue);
    --btn-filters-hover: var(--color-soft-green-lighter);

    --btn-constraints-disabled: var(--color-light-gray);
}

/* ==============================
   Table Styles
   ============================== */
#problems-table {
    width: 100%;
    border-collapse: collapse;
    opacity: 0; /* fade-in on load */
    transition: opacity 0.5s ease;
}
#problems-table.visible { opacity: 1; }

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
    padding: 4px 8px!important;
}
.dt-buttons button i { font-size: 1.2em; vertical-align: middle; }
.dt-buttons button:hover {
    opacity: 0.85;
    transform: scale(1.03);
}

/* Export buttons (semantic classes) */
.dt-buttons .buttons-copy  { color: var(--color-bright-blue) !important; }
.dt-buttons .buttons-csv   { color: var(--color-orange) !important; }
.dt-buttons .buttons-excel { color: var(--color-soft-green) !important; }
.dt-buttons .buttons-pdf   { color: var(--color-soft-red) !important; }
.dt-buttons .buttons-print { color: var(--color-deep-violet) !important; }

/* Also target icons inside buttons, in case <i> inherits default color */
.dt-buttons button i {
    color: inherit !important;
}

.dt-top-controls {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 12px;
    margin-bottom: 8px;
    width: 100%;
}
.dt-top-controls .dataTables_length,
.dt-top-controls .dataTables_filter { margin: 0; }
.dt-top-controls .dataTables_length label,
.dt-top-controls .dataTables_filter label {
    display: flex;
    align-items: center;
    gap: 6px;
    margin: 0;
}
.dt-top-controls .dataTables_length select { min-width: 70px; }
.dt-top-controls .dataTables_filter input {
    width: 220px;
    max-width: 40vw;
    padding: 6px 8px;
    box-sizing: border-box;
}
@media (max-width: 680px) {
    .dt-top-controls { flex-direction: column; align-items: stretch; }
    .dt-top-controls .dataTables_filter input { width: 100%; }
}

/* ==============================
   Constraint Buttons
   ============================== */
.constraint-btn {
    border: none;
    border-radius: 12px;
    padding: 4px 8px;
    font-weight: bold;
    cursor: pointer;
    transition: 0.2s;
}
.constraint-btn.small { font-size: 0.75em; }
.constraint-btn.normal { font-size: 0.85em; }

/* State (zero vs nonzero) */
.constraint-btn.dim-zero {
    background-color: var(--btn-constraints-disabled);
    color: var(--color-gray-text);
}
.constraint-btn.dim-nonzero { color: white; }

/* Type-specific (nonzero only) */
.constraint-btn.constraint-x.dim-nonzero { background-color: var(--color-dark-blue); }
.constraint-btn.constraint-u.dim-nonzero { background-color: var(--color-deep-blue); }
.constraint-btn.constraint-v.dim-nonzero { background-color: var(--color-bright-blue); }
.constraint-btn.constraint-c.dim-nonzero { background-color: var(--color-orange); }
.constraint-btn.constraint-b.dim-nonzero { background-color: var(--color-soft-red); }

.constraints-wrapper strong {
    font-size: 0.95em;
    font-weight: bold;
    border-radius: 4px;
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
    cursor: pointer;
    background-color: var(--btn-filters-disabled);
    color: #333;
    transition: background-color 0.2s, transform 0.1s;
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

.constraint-filter-btn { background: #ccc; color: #333; } /* default gray */
.constraint-filter-btn.positive { background: #4caf50; color: white; } /* green */
.constraint-filter-btn.negative { background: #f44336; color: white; } /* red */

/* ==============================
   Filters (Inputs / Selects)
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
#constraints-filter {
    text-align: center;
}
#constraints-filter > div:first-child {
    display: flex!important;
    flex-direction: column;
    align-items: center;
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
    margin-top: 2px;
    color: #333;
}

/* ==============================
   Sorting arrows spacing
   ============================== */
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
    right: 2px !important;
}

/* ==============================
   Loading Overlay
   ============================== */
#loading-overlay {
  position: fixed;
  top: 0; left: 0;
  width: 100%; height: 100%;
  background: rgba(255,255,255,0.9);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 9999;
  transition: opacity 0.5s ease;
}
#loading-overlay.hidden {
  opacity: 0;
  pointer-events: none;
}
.spinner {
  border: 6px solid #f3f3f3;
  border-top: 6px solid var(--color-deep-blue);
  border-radius: 50%;
  width: 50px; height: 50px;
  animation: spin 1s linear infinite;
}
@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}
</style>
"""

const TABLE_LOGIC = """
<div id="loading-overlay"><div class="spinner"></div></div>
<script>
// ==============================
// Constraint Helper Functions (CSS class-based)
// ==============================
const ConstraintHelpers = (() => {

    const constraintMap = {
        x: 'DimStateConstraint',
        u: 'DimControlConstraint',
        v: 'DimVariableConstraint',
        c: 'DimPathConstraint',
        b: 'DimBoundaryConstraint'
    };

    function getConstraintParts(rowData) {
        return Object.entries(constraintMap)
            .filter(([letter, col]) => rowData[col] && Number(rowData[col]) > 0)
            .map(([letter]) => letter);
    }

    function summary(rowData) {
        return getConstraintParts(rowData).join(' ') + ` (\${rowData.TotalConstraints})`;
    }

    function buttonHTML(type, dim, small = false, print_val = true) {
        const classes = [
            "constraint-btn",
            `constraint-\${type}`,
            small ? "small" : "normal",
            dim === 0 ? "dim-zero" : "dim-nonzero"
        ].join(" ");
        const text = print_val ? `\${type}: \${dim}` : type;
        return `<button class="\${classes}">\${text}</button>`;
    }

    function rowSummaryHTML(rowData, print_val=false) {
        return Object.keys(constraintMap)
            .map(k => buttonHTML(k, rowData[constraintMap[k]], true, print_val))
            .join(' ') + ` <strong style="margin-left:5px;">(\${rowData.TotalConstraints})</strong>`;
    }

    function detailHTML(rowData) {
        return `<div style="display:flex; gap:4px; align-items:center;">\${rowSummaryHTML(rowData, true)}</div>`;
    }

    return {summary, rowSummaryHTML, detailHTML};
})();

// ==============================
// Table Initialization
// ==============================
document.addEventListener('DOMContentLoaded', function() {
    setTimeout(initProblemsTable, 0);
});

function initProblemsTable() {
    const data = JSON.parse(document.getElementById("problems-json").textContent);
    let constraintFilter = null; // placeholder for constraint filter

    const table = \$('#problems-table').DataTable({
        deferRender: true,
        orderCellsTop: true,
        fixedHeader: true,
        responsive: false,
        pageLength: 25,
        lengthMenu: [ [10, 25, 50, -1], [10, 25, 50, "All"] ],
        dom: '<"dt-top-buttons"B><"dt-top-controls"lf>rt<"bottom"ip><"clear">',
        autoWidth: false,
        columns: [
            { data: 'Problem', width: '20%' },
            { data: 'State', width: '10%' },
            { data: 'Control', width: '12%' },
            { data: 'Variable', width: '12%' },
            { data: 'Cost', width: '10%' },
            { data: 'FinalTime', width: '14%' },
            {
                data: 'ConstraintButtonsHtml',
                render: function(data, type, row) {
                    if (type === 'sort' || type === 'type') return Number(row.TotalConstraints);
                    return ConstraintHelpers.rowSummaryHTML(row);
                }
            }
        ],
        buttons: ['copy','csv','excel','pdf','print'].map(ext => ({
            extend: ext,
            text: {
                copy:  '<i class="fas fa-copy"></i>',
                csv:   '<i class="fas fa-file-csv"></i>',
                excel: '<i class="fas fa-file-excel"></i>',
                pdf:   '<i class="fas fa-file-pdf"></i>',
                print: '<i class="fas fa-print"></i>'
            }[ext],
            titleAttr: {
                copy:  'Copy to clipboard',
                csv:   'Download CSV',
                excel: 'Download Excel',
                pdf:   'Download PDF',
                print: 'Print table'
            }[ext],
            exportOptions: {
                columns: [0,1,2,3,4,5,6],
                format: {
                    body: function(data, rowIdx, colIdx, node) {
                        if(colIdx === 6) {
                            const rowData = table.row(rowIdx).data();
                            return ConstraintHelpers.summary(rowData);
                        }
                        return data;
                    }
                }
            }
        })),
        initComplete: function() {
            const topButtons = \$('.dt-top-buttons').first();
            const dtButtons = \$('.dt-buttons').first();
            if(topButtons.length && dtButtons.length && dtButtons.parent().get(0) !== topButtons.get(0)) topButtons.empty().append(dtButtons);

            const topControls = \$('.dt-top-controls').first();
            if(topControls.length){
                const length = \$('.dataTables_length').first();
                const filter = \$('.dataTables_filter').first();
                if(length.length && length.parent().get(0) !== topControls.get(0)) topControls.append(length);
                if(filter.length && filter.parent().get(0) !== topControls.get(0)) topControls.append(filter);
                const inp = topControls.find('.dataTables_filter input').first();
                if(inp.length){
                    inp.attr('placeholder', 'Search...');
                    inp.css({ 'width': '220px', 'display': 'inline-block' });
                }
            }
        }
    });

    // -------------------------
    // Add rows
    // -------------------------
    const rows = [];
    data.Problem.forEach((_, i) => {
        rows.push({
            Problem: `<a href="problems/\${data.Problem[i]}.html" class="problem-link">\${data.Problem[i]}</a>`,
            State: data.State[i],
            Control: data.Control[i],
            Variable: data.Variable[i],
            Cost: data.Cost[i],
            FinalTime: data.FinalTime[i],
            ConstraintButtonsHtml: data.ConstraintButtonsHtml[i],
            TotalConstraints: data.TotalConstraints[i],
            DimStateConstraint: data.DimStateConstraint[i],
            DimControlConstraint: data.DimControlConstraint[i],
            DimVariableConstraint: data.DimVariableConstraint[i],
            DimPathConstraint: data.DimPathConstraint[i],
            DimBoundaryConstraint: data.DimBoundaryConstraint[i]
        });
    });
    table.rows.add(rows).draw();

    \$('#loading-overlay').addClass('hidden');
    \$('#problems-table').addClass('visible');

    // -------------------------
    // Hover tooltip
    // -------------------------
    \$('#problems-table tbody').off('mouseenter', '.constraint-btn').on('mouseenter', '.constraint-btn', function(){
        let type = Array.from(this.classList).find(c => c.startsWith('constraint-'))?.split('-')[1] || '?';
        let dim = \$(this).hasClass('dim-zero') ? 0 : '?';
        \$(this).attr('title', `\${type.toUpperCase()}: \${dim} constraints`);
    });

    // Row click: toggle child detail
    \$('#problems-table tbody').off('click', 'tr').on('click', 'tr', function(e) {
        if (\$(e.target).closest('.problem-link').length) return;
        const row = table.row(this);
        if (row.child.isShown()) row.child.hide();
        else row.child(ConstraintHelpers.detailHTML(row.data())).show();
    });

    \$(document).on('click', '.problem-link', function(e) { e.stopPropagation(); });

    // -------------------------
    // Filters (numeric + dropdown)
    // -------------------------
    \$('#problems-table thead tr#filters th').each(function(i){
        if(i===4){
            \$(this).html('<select><option value="">All</option><option>Mayer</option><option>Lagrange</option><option>Bolza</option></select>');
        } else if(i===5){
            \$(this).html('<select><option value="">All</option><option>fixed</option><option>free</option></select>');
        } else if(i>0 && i<6){
            \$(this).html('<input type="text" placeholder="min-max" style="width:100%"/>');
        }
        \$('input, select', this).on('keyup change', function(){ table.draw(); });
    });

    function combinedFilter(settings, data, dataIndex){
        const rowData = table.row(dataIndex).data();
        let pass = true;

        \$('#problems-table thead tr#filters th').each(function(i){
            const input = \$('input', this);
            const select = \$('select', this);

            // Numeric input filter
            if(input.length){
                const val = input.val().trim();
                if(val !== ''){
                    const colName = ["Problem","State","Control","Variable","Cost","FinalTime","Constraints"][i];
                    const v = Number(rowData[colName]) || 0;
                    if(val.includes('-')){
                        const [minStr,maxStr] = val.split('-').map(s=>s.trim());
                        const min = minStr === '' ? -Infinity : Number(minStr);
                        const max = maxStr === '' ? Infinity : Number(maxStr);
                        if(v < min || v > max) pass = false;
                    } else {
                        const num = Number(val);
                        if(v !== num) pass = false;
                    }
                }
            }

            // Dropdown selects filter
            if(select.length){
                const val = select.val();
                if(val !== ''){
                    const colName = i===4 ? "Cost" : i===5 ? "FinalTime" : null;
                    if(colName && rowData[colName] !== val) pass = false;
                }
            }
        });
        return pass;
    }

    \$.fn.dataTable.ext.search = [];
    \$.fn.dataTable.ext.search.push(combinedFilter);

    // -------------------------
    // Constraint filter buttons
    // -------------------------
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
    ["x","u","v","c","b"].forEach(c => {
        html += `<button class="constraint-filter-btn" data-type="\${c}">\${c}</button>`;
    });
    html += `</div></div>`;
    filterContainer.append(html);

    \$(document).on('click', '.constraint-filter-btn', function() {
        if (\$(this).hasClass('positive')) {
            \$(this).removeClass('positive').addClass('negative'); // green → red
        } else if (\$(this).hasClass('negative')) {
            \$(this).removeClass('negative'); // red → gray
        } else {
            \$(this).addClass('positive'); // gray → green
        }
        applyConstraintFilter();
    });

    function applyConstraintFilter() {
        const activeConditions = [];

        \$('.constraint-filter-btn.positive').each(function(){
            const c = \$(this).data('type');
            activeConditions.push({type: c, check: 'positive'});
        });
        \$('.constraint-filter-btn.negative').each(function(){
            const c = \$(this).data('type');
            activeConditions.push({type: c, check: 'negative'});
        });

        const logic = \$('#constraints-logic').val();

        // Remove old constraint filter
        \$.fn.dataTable.ext.search = \$.fn.dataTable.ext.search.filter(f => f !== constraintFilter);

        constraintFilter = function(settings, data, dataIndex){
            const rowData = table.row(dataIndex).data();
            const map = {
                x:"StateConstraint", u:"ControlConstraint",
                v:"VariableConstraint", c:"PathConstraint", b:"BoundaryConstraint"
            };

            // Build conditions
            const results = activeConditions.map(cond => {
                const val = rowData["Dim"+map[cond.type]];
                return cond.check === 'positive' ? (val > 0) : (val === 0);
            });

            if (results.length === 0) return true;

            return logic === "AND" ? results.every(v=>v) : results.some(v=>v);
        };

        \$.fn.dataTable.ext.search.push(constraintFilter);
        table.draw();
    }


    \$(document).on('change', '#constraints-logic', applyConstraintFilter);
}
</script>
"""

const TABLE = (presentation=TABLE_PRESENTATION, style=TABLE_STYLE, logic=TABLE_LOGIC)

# -------------------------------
# Helpers
# -------------------------------
function sum_namedtuple(nt::NamedTuple)
    sum(values(nt))
end

function ocp_data_to_json(data_ocp::DataFrame)
    return JSON.json(Tables.columntable(data_ocp))
end

function write_block(io, content)
    write(io, content)
end

function generate_constraint_buttons_html(constraint_dims::NamedTuple)
    return constraint_dims
end

function build_table_html(json_str::String)
    return """
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
"""
end

# -------------------------------
# Data collection
# -------------------------------
function collect_problem_data(problem_sym::Symbol)
    ocp = ocp_model(eval(problem_sym)(OptimalControlBackend()))

    cost = if has_mayer_cost(ocp) && has_lagrange_cost(ocp)
        "Bolza"
    elseif has_mayer_cost(ocp)
        "Mayer"
    else
        "Lagrange"
    end

    final_time = has_fixed_final_time(ocp) ? "fixed" : "free"

    dims = (
        x=CTModels.dim_state_constraints_box(ocp),
        u=CTModels.dim_control_constraints_box(ocp),
        v=CTModels.dim_variable_constraints_box(ocp),
        c=CTModels.dim_path_constraints_nl(ocp),
        b=CTModels.dim_boundary_constraints_nl(ocp),
    )

    total = sum_namedtuple(dims)

    return (
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
        TotalConstraints=total,
        ConstraintButtonsHtml=generate_constraint_buttons_html(dims),
    )
end

"Collects OCP data and returns a DataFrame"
function collect_problems_data()
    return DataFrame([collect_problem_data(sym) for sym in problems()])
end

### MAIN GENERATION PROCESS

# ---------------------------
# Layer 1: Data collection
# ---------------------------
"Convert OCP data into JSON string"
function collect_problems_data_json()
    df = collect_problems_data()
    return ocp_data_to_json(df)
end

# ---------------------------
# Layer 2: HTML assembly
# ---------------------------
function assemble_problems_browser_html(json_str)
    html_parts = [
        TABLE_PRESENTATION,
        "```@raw html",
        TABLE_STYLE,
        build_table_html(json_str),
        TABLE_LOGIC,
        "```",
    ]
    return join(html_parts, "\n")
end

# ---------------------------
# Layer 3: File writing
# ---------------------------
function generate_problems_browser!(path::AbstractString)
    mkpath(dirname(path))
    json_str = collect_problems_data_json()
    html = assemble_problems_browser_html(json_str)
    open(path, "w") do io
        write(io, html)
    end
    return path
end

# ---------------------------
# Temporary browser context using fixed path
# ---------------------------
"""
with_problems_browser(f::Function)

Generates the problems browser at the fixed path, passes the filename
to `f`, and removes the file after `f` finishes.
"""
function with_problems_browser(f::Function)
    # Generate the problems browser at the fixed path
    generate_problems_browser!(BROWSER_PATH)

    try
        # Pass the generated file path to the user function
        return f(BROWSER_FILE)
    finally
        # Remove the file after usage
        if isfile(BROWSER_PATH)
            rm(BROWSER_PATH)
            println("Temporary problems browser file removed: $BROWSER_PATH")
        end
    end
end
