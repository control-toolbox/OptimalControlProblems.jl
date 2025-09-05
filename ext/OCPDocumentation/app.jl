function html_header()
html = """
<head>
        <meta charset='UTF-8'>
        <title>OCP Doc Helper</title>
        <!-- Prism CSS: light and dark themes -->
        <link id="prism-theme" rel="stylesheet" href="https://cdn.jsdelivr.net/npm/prismjs@1.29.0/themes/prism-tomorrow.css" />
        <style>
            :root {
                --bg-dark: #23272f;
                --bg-container-dark: #2d333b;
                --text-dark: #ececf1;
                --border-dark: #444950;
                --bg-output-dark: #1f242b;
                --scrollbar-thumb-dark: #10a37f;

                --bg-light: #fafafa;
                --bg-container-light: #ffffff;
                --text-light: #222222;
                --border-light: #ccc;
                --bg-output-light: #f5f5f5;
                --scrollbar-thumb-light: #10a37f;
            }

            body {
                background: var(--bg-dark);
                color: var(--text-dark);
                font-family: 'Segoe UI', Arial, sans-serif;
                margin: 0;
                padding: 0;
                transition: background 0.3s, color 0.3s;
            }
            body.light {
                background: var(--bg-light);
                color: var(--text-light);
            }
            body.light input[type="password"] {
                background: var(--bg-light);
                color: var(--text-light);
                border: 1px solid var(--border-light);
            }
            .container {
                max-width: 900px;
                margin: 40px auto;
                background: var(--bg-container-dark);
                border-radius: 12px;
                box-shadow: 0 4px 24px #0006;
                padding: 2em 2.5em 3em 2.5em; /* extra bottom padding for copy button */
                transition: background 0.3s;
                position: relative;
            }
            body.light .container {
                background: var(--bg-container-light);
                box-shadow: 0 4px 24px #aaa;
            }
            h2 {
                text-align: center;
                font-weight: 600;
                margin-bottom: 1.5em;
                color: #10a37f;
                letter-spacing: 1px;
            }
            label {
                font-weight: 500;
                color: inherit;
            }
            .tabs {
                display: flex;
                gap: 0.5em;
                margin-bottom: 1em;
                border-bottom: 2px solid var(--border-dark);
            }
            body.light .tabs {
                border-color: var(--border-light);
            }
            .tab {
                flex: 1;
                text-align: center;
                padding: 0.5em 0;
                cursor: pointer;
                background: var(--bg-container-dark);
                color: var(--text-dark);
                border: 1px solid var(--border-dark);
                border-bottom: none;
                border-radius: 6px 6px 0 0;
                user-select: none;
                transition: background 0.3s, color 0.3s, border-color 0.3s;
            }
            body.light .tab {
                background: var(--bg-container-light);
                color: var(--text-light);
                border-color: var(--border-light);
            }
            .tab.active {
                background: var(--bg-output-dark);
                color: #10a37f;
                font-weight: 600;
                border-color: #10a37f;
                border-bottom: 2px solid var(--bg-output-dark);
            }
            body.light .tab.active {
                background: var(--bg-output-light);
                border-color: #10a37f;
                color: #10a37f;
                border-bottom: 2px solid var(--bg-output-light);
            }
            input[type="password"], textarea {
                width: 100%;
                background: var(--bg-dark);
                color: var(--text-dark);
                border: 1px solid var(--border-dark);
                border-radius: 6px;
                padding: 0.7em;
                font-size: 1em;
                resize: none;
                transition: background 0.3s, color 0.3s, border 0.3s;
                min-height: 150px;
                max-height: 600px;
                overflow-y: auto;
                box-sizing: border-box;
                /* HIDE all textareas by default */
                display: none;
            }
            input[type="password"] {
                min-width: 200px;
                max-width: 200px;
                min-height: 30px;
                max-height: 100px;
                padding-left: 10px;
                padding-right: 10px;
                padding-top: 0px;
                padding-bottom: 0px;
                display: inline;
            }
            textarea.active {
                /* SHOW only the active textarea */
                display: block;
            }
            body.light textarea {
                background: var(--bg-light);
                color: var(--text-light);
                border: 1px solid var(--border-light);
            }
            input[type="password"]:focus, textarea:focus {
                border: 1.5px solid #10a37f;
                outline: none;
            }
            .flex-row {
                display: flex;
                gap: 1em;
                flex-wrap: wrap;
                margin-top: 0.5em;
            }
            .flex-row.justify-between {
                justify-content: space-between;
                align-items: center;
            }
            .left-buttons,
            .right-buttons {
                display: flex;
                gap: 0.8em;
            }
            @media (max-width: 600px) {
                .flex-row.justify-between {
                    flex-direction: column;
                    align-items: stretch;
                }

                .left-buttons,
                .right-buttons {
                    justify-content: space-between;
                }
            }
            button {
                background: #10a37f;
                color: #fff;
                border: none;
                border-radius: 6px;
                padding: 0.5em 1.0em;
                font-size: 1.1em;
                font-weight: 600;
                cursor: pointer;
                transition: background 0.2s, box-shadow 0.2s;
                margin-top: 0.5em;
            }
            button:hover, button:focus {
                background: #13c08a;
                outline: none;
                box-shadow: 0 0 5px #13c08a;
            }
            .output {
                margin-top: 1.5em;
                background: var(--bg-output-dark);
                border: 1px solid var(--border-dark);
                border-radius: 8px;
                padding: 1.5em;
                font-family: 'Fira Mono', 'Consolas', monospace;
                font-size: 1.1em;
                line-height: 1.5;
                color: var(--text-dark);
                min-height: 150px;
                max-height: 800px;
                overflow-y: auto;
                box-sizing: border-box;
                user-select: text;
                white-space: pre-wrap;
                word-wrap: break-word;
                transition: background 0.3s, color 0.3s, border 0.3s;
                position: relative;
            }
            body.light .output {
                background: var(--bg-output-light);
                border: 1px solid var(--border-light);
                color: var(--text-light);
                box-shadow: 0 0 10px #ccc inset;
            }
            .output.generating {
                font-style: italic;
                color: #a0a0a0;
            }
            hr {
                border: 0;
                border-top: 1px solid #444;
                margin: 2em 0;
                transition: border-color 0.3s;
            }
            body.light hr {
                border-top-color: #ccc;
            }
            @media (max-width: 900px) {
                .flex-row { flex-direction: column; }
            }
            /* Scrollbar styling */
            .output::-webkit-scrollbar {
                width: 10px;
            }
            .output::-webkit-scrollbar-track {
                background: var(--bg-output-dark);
                border-radius: 8px;
            }
            body.light .output::-webkit-scrollbar-track {
                background: var(--bg-output-light);
            }
            .output::-webkit-scrollbar-thumb {
                background: var(--scrollbar-thumb-dark);
                border-radius: 8px;
            }
            body.light .output::-webkit-scrollbar-thumb {
                background: var(--scrollbar-thumb-light);
            }
            /* Firefox */
            .output {
                scrollbar-width: thin;
                scrollbar-color: var(--scrollbar-thumb-dark) var(--bg-output-dark);
            }
            body.light .output {
                scrollbar-color: var(--scrollbar-thumb-light) var(--bg-output-light);
            }

            #output {
                position: relative;   /* So the copy button can be absolutely positioned inside */
                padding-top: 2.5em;   /* Add padding at top so text doesn't go under button */
                /* Other styles you have */
                border-radius: 4px;
                overflow: auto;
                max-height: 400px;
                font-family: monospace;
            }

            #copy {
                position: absolute;
                top: 0.5em;
                right: 0.5em;
                padding: 0.4em 0.9em;
                font-size: 0.9em;
                margin: 0;
                cursor: pointer;
                user-select: none;
                border-radius: 3px;
            }

            #copy-check {
                margin-left: 0.5em;
                font-weight: bold;
                color: #10a37f;
            }

            /* Toggle theme button top right inside container */
            #toggle-theme {
                position: absolute;
                top: 1em;
                right: 1em;
                padding: 0.4em 1em;
                font-size: 0.9em;
                margin-top: 0;
            }
        </style>
    </head>
"""
return html
end

function html_code_doc_app()
    # build the <option> list from available problems
    options = join(["<option value=\"$(p)\">$(string(p))</option>" for p in OptimalControlProblems.problems()], "\n")

    html = """
    <!DOCTYPE html>
    <html lang='en'>
    """ * html_header() * """
    <body>
        <div class="container">
            <h2>OCP Prompt Generator</h2>

            <form id="form" novalidate>
                <label for="problem">Choose a problem:</label>
                <select id="problem" name="problem">
                    $options
                </select>

                <div class="flex-row justify-between">
                    <div class="left-buttons">
                        <button type="submit">Generate</button>
                    </div>
                    <div class="right-buttons">
                        <button type="button" id="quit">Quit</button>
                    </div>
                </div>
            </form>

            <hr>

            <div class="output-container" style="position: relative;">
                <pre class="output language-julia" id="output" tabindex="0" aria-live="polite"></pre>
                <button type="button" id="copy" aria-label="Copy prompt output">
                    Copy <span id="copy-check" style="display:none;">✓</span>
                </button>
            </div>
            <button type="button" id="toggle-theme" aria-label="Toggle dark/light mode">Toggle Dark/Light Mode</button>
        </div>

        <!-- Prism.js core -->
        <script src="https://cdn.jsdelivr.net/npm/prismjs@1.29.0/prism.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/prismjs@1.29.0/components/prism-julia.min.js"></script>

        <script>
            async function generateOutput() {
                const problem = document.getElementById('problem').value;
                const outputEl = document.getElementById('output');

                outputEl.textContent = "Generating...";
                outputEl.classList.add("generating");

                try {
                    const resp = await fetch('/run', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ problem })
                    });
                    const text = await resp.text();
                    outputEl.textContent = text;
                    Prism.highlightElement(outputEl);
                } catch (err) {
                    outputEl.textContent = "Error during request: " + err;
                } finally {
                    outputEl.classList.remove("generating");
                }
            }

            document.getElementById('form').onsubmit = function(e) {
                e.preventDefault();
                generateOutput();
            };

            document.getElementById('quit').onclick = function () {
                const outputEl = document.getElementById('output');
                outputEl.textContent = "Session cleared. Closing...";
                fetch('/quit').then(() => {
                    setTimeout(() => window.close(), 700);
                });
            };

            const copyButton = document.getElementById('copy');
            const copyCheck = document.getElementById('copy-check');
            const outputEl = document.getElementById('output');

            copyButton.onclick = function () {
                const text = outputEl.textContent;
                if (text.trim().length === 0) return;
                navigator.clipboard.writeText(text).then(() => {
                    copyCheck.style.display = "inline";
                    copyButton.disabled = true;
                    setTimeout(() => {
                        copyCheck.style.display = "none";
                        copyButton.disabled = false;
                    }, 1500);
                });
            };

            function applyTheme(theme) {
                const isLight = theme === "light";
                document.body.classList.toggle("light", isLight);
                const prismLink = document.getElementById("prism-theme");
                prismLink.href = isLight
                    ? "https://cdn.jsdelivr.net/npm/prismjs@1.29.0/themes/prism.css"
                    : "https://cdn.jsdelivr.net/npm/prismjs@1.29.0/themes/prism-tomorrow.css";
            }

            const toggleThemeBtn = document.getElementById('toggle-theme');
            toggleThemeBtn.onclick = () => {
                const newTheme = document.body.classList.contains("light") ? "dark" : "light";
                localStorage.setItem("ocp-theme", newTheme);
                applyTheme(newTheme);
            };

            document.addEventListener('DOMContentLoaded', () => {
                const savedTheme = localStorage.getItem("ocp-theme") || "dark";
                applyTheme(savedTheme);
            });
        </script>
    </body>
    </html>
    """
    return html
end

"""
$(TYPEDSIGNATURES)

Handles HTTP requests to serve the web app interface, process user-submitted code, and shut down the server.

# Arguments

- `req`: An HTTP request object representing an incoming request to the server.

# Returns

- `response::HTTP.Response`: An appropriate HTTP response depending on the request path and method.

# Example

```julia-repl
julia> using HTTP
julia> req = HTTP.Request("GET", "/");
julia> resp = handled_doc_app(req)
HTTP.Response(200 OK)
```
"""
function handled_doc_app(req)
    if req.target == "/"
        return HTTP.Response(200, ["Content-Type" => "text/html"], html_code_doc_app())

    elseif req.target == "/run" && req.method == "POST"
        data = JSON.parse(String(req.body))
        problem = get(data, "problem", "")
        result = ""

        try
            result = OptimalControlProblems.generate_prompt(problem)
        catch err
            result = "Error during generation: $(err)"
        end

        return HTTP.Response(200, ["Content-Type" => "text/plain"], result)

    elseif req.target == "/quit"
        @async begin
            sleep(0.5)
            exit()
        end
        return HTTP.Response(200, ["Content-Type" => "text/plain"], "Server shutting down.")

    else
        return HTTP.Response(404, ["Content-Type" => "text/plain"], "Not found")
    end
end

"""
$(TYPEDSIGNATURES)

Launches the Julia Docstrings Generator web app on localhost.

Starts an HTTP server on port 8080 and prints the local URL for access.

# Arguments

- `::OptimalControlProblems.DocAppTag`: A dispatch tag used to identify this application.

# Returns

- `nothing`: This function runs the server and does not return a result.

# Example

```julia-repl
julia> OptimalControlProblems.doc_app(OptimalControlProblems.DocAppTag())
Open http://localhost:8080 in your browser.
```
"""
function OptimalControlProblems.doc_app(::OptimalControlProblems.DocAppTag)
    println("Open http://localhost:8080 in your browser.")
    HTTP.serve(handled_doc_app, "127.0.0.1", 8080)
    return nothing
end