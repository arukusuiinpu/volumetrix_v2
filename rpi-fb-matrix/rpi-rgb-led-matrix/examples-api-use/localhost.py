import os
from flask import Flask, request, redirect, url_for, render_template_string, jsonify, make_response
import subprocess

app = Flask(__name__)

MODELS_DIR = "./models"

def get_saved_value(filename, default=""):
    if os.path.exists(filename):
        with open(filename, "r") as f:
            return f.read().strip()
    return default

def list_obj_files():
    if os.path.exists(MODELS_DIR):
        return sorted([f for f in os.listdir(MODELS_DIR) if f.endswith('.obj')])
    return []

@app.route('/execute', methods=['POST'])
def execute():
    command = request.form.get('command')
    try:
        output = subprocess.check_output(command, shell=True, stderr=subprocess.STDOUT, text=True)
    except subprocess.CalledProcessError as e:
        output = e.output
    return jsonify({'output': output})

@app.route('/')
def index():
    current_id = get_saved_value("model_id.txt")
    current_scale = get_saved_value("config.txt", "12.0")
    rx, ry, rz = [get_saved_value(f"rot{i}.txt", "0.0") for i in "XYZ"]
    sx, sy, sz = [get_saved_value(f"set{i}.txt", "0.0") for i in "XYZ"]
    obj_files = list_obj_files()

    return render_template_string('''
    <style>
        body { font-family: sans-serif; max-width: 800px; margin: 20px auto; padding: 0 20px; color: #333; }
        h3 { margin-top: 30px; border-bottom: 1px solid #eee; padding-bottom: 5px; }
        
        /* Unified Button Style */
        .btn {
            display: inline-block;
        }
        
        /* Input Styling */
        input[type="text"], input[type="number"], select {
            padding: 6px;
            border: 1px solid #ccc;
            border-radius: 4px;
        }
        hr { border: 0; border-top: 1px solid #eee; margin: 20px 0; }
    </style>

    <h1>Model Configuration</h1>
    
    <form action="/upload" method="POST" enctype="multipart/form-data" style="margin-bottom: 20px;">
        <h3>Upload Model</h3>
        <input type="file" name="file">
        <input type="submit" value="Upload" class="btn">
    </form>

    <form action="/update" method="POST">
        <p>Select Model: 
            <select id="model_selector" onchange="document.getElementById('manual_id').value = this.value.replace('.obj', '')">
                <option value="" disabled {% if not current_id %}selected{% endif %}>-- Pick a file --</option>
                {% for model in models %}
                    <option value="{{ model }}" {% if model.replace('.obj', '') == current_id %}selected{% endif %}>
                        {{ model.replace('.obj', '') }}
                    </option>
                {% endfor %}
            </select>
        </p>
        <p>Model ID: <input type="text" name="manual_id" id="manual_id" value="{{ current_id }}"></p>
        <p>Model Scale: <input type="number" step="0.1" name="scale" value="{{ current_scale }}" required></p>
        
        <h3>Manual Angle Setup (Degrees)</h3>
        <p>X: <input type="number" step="1" name="setX" value="{{ sx }}"> 
           Y: <input type="number" step="1" name="setY" value="{{ sy }}"> 
           Z: <input type="number" step="1" name="setZ" value="{{ sz }}"></p>

        <h3>Auto-Rotation Speed</h3>
        <p>X: <input type="number" step="0.01" name="rotX" value="{{ rx }}">
           Y: <input type="number" step="0.01" name="rotY" value="{{ ry }}">
           Z: <input type="number" step="0.01" name="rotZ" value="{{ rz }}"></p>
        
        <input type="submit" value="Update Config" class="btn">
    </form>

    <div style="display: flex; gap: 10px; margin-top: 20px;">
        <form action="/reset" method="POST">
            <button type="submit" class="btn">Reset angles</button>
        </form>
        <form action="/restart" method="POST">
            <button type="submit" class="btn" 
                    onclick="return confirm('This will kill the current process and recompile. Proceed?')">
                Recompile & Restart
            </button>
        </form>
    </div>
    
    <h3>Demo Control</h3>
    <div style="display: flex; gap: 10px; margin-bottom: 20px;">
        <button onclick="sendDemoAction('on')" class="btn">Demo On</button>
        <button onclick="sendDemoAction('off')" class="btn">Demo Off</button>
    </div>
    
    <h3>Remote Execute</h3>
    <div">
        <form id="execForm" style="display: flex; gap: 10px;">
            <input type="text" id="cmd" name="command" placeholder="ls -la" style="flex-grow: 1; font-family: monospace; background: #000; color: #fff; border: 1px solid #444; padding: 8px;">
            <button type="submit" class="btn">Run</button>
        </form>
        <pre id="terminal">Output will appear here...</pre>
    </div>
    
    <script>
        async function sendDemoAction(action) {
            const formData = new FormData();
            formData.append('action', action);
            const term = document.getElementById('terminal');
            term.innerText = "Sending " + action.toUpperCase() + " command...";
            try {
                const response = await fetch('/toggle_demo', { method: 'POST', body: formData });
                const data = await response.json();
                term.innerText = data.status;
            } catch (err) { term.innerText = "Error: " + err; }
        }
        document.getElementById('execForm').onsubmit = async (e) => {
            e.preventDefault();
            const term = document.getElementById('terminal');
            const cmdInput = document.getElementById('cmd');
            const formData = new FormData();
            formData.append('command', cmdInput.value);
            term.innerText = "Running...";
            try {
                const response = await fetch('/execute', { method: 'POST', body: formData });
                const data = await response.json();
                term.innerText = data.output || "No output returned.";
            } catch (err) { term.innerText = "Error: " + err; }
        };
    </script>
''', models=obj_files, current_id=current_id, current_scale=current_scale,
                              rx=rx, ry=ry, rz=rz, sx=sx, sy=sy, sz=sz)


@app.route('/get_models')
def get_models():
    return jsonify(list_obj_files())

@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return redirect(request.url)

    file = request.files['file']

    if file.filename == '':
        return redirect(request.url)

    if file:
        save_path = os.path.join(MODELS_DIR, file.filename)
        file.save(save_path)

        with open("model_id.txt", "w") as f:
            f.write(file.filename.replace('.obj', ''))

    return redirect(url_for('index'))

@app.route('/toggle_demo', methods=['POST'])
def toggle_demo():
    action = request.form.get('action')
    try:
        if action == 'off':
            subprocess.run(["pkill", "demo"])
            return jsonify({'status': 'Demo stopped (pkill demo)'})
        elif action == 'on':
            cmd_chain = "compilecode && run &"
            subprocess.Popen(
                ["bash", "-i", "-c", cmd_chain],
                cwd="/opt/volumetrix/rpi-fb-matrix/rpi-rgb-led-matrix/examples-api-use/"
            )
            return jsonify({'status': 'Demo starting...'})
    except Exception as e:
        return jsonify({'status': f'Error: {str(e)}'})
    return jsonify({'status': 'Unknown action'})

@app.route('/restart', methods=['POST'])
def restart_code():
    try:
        cmd_chain = "pkill demo; compilecode && run &"

        subprocess.Popen(
            ["bash", "-i", "-c", cmd_chain],
            cwd="/opt/volumetrix/rpi-fb-matrix/rpi-rgb-led-matrix/examples-api-use/"
        )
        return "Restarting... <a href='/'>Back</a>"
    except Exception as e:
        return f"Error: {str(e)}"

@app.route('/update', methods=['POST'])
def update_file():
    model_id = request.form.get('manual_id', '').strip()
    scale = request.form.get('scale', '12.0').strip()

    print(f"DEBUG: Saving Scale: {scale} | Saving ID: {model_id}")

    if model_id:
        with open("model_id.txt", "w") as f:
            f.write(model_id)

    with open("config.txt", "w") as f:
        f.write(scale)

    for axis in ['rotX', 'rotY', 'rotZ']:
        val = request.form.get(axis, '0.0').strip()
        with open(f"{axis}.txt", "w") as f:
            f.write(val)

    for prefix in ['rot', 'set']:
        for axis in ['X', 'Y', 'Z']:
            field = f"{prefix}{axis}"
            val = request.form.get(field, '0.0').strip()
            with open(f"{field}.txt", "w") as f:
                f.write(val)

    response = make_response(redirect(url_for('index')))
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    return response

@app.route('/reset', methods=['POST'])
def reset_imu():
    with open("reset.txt", "w") as f:
        f.write("1.0")
    return redirect(url_for('index'))

if __name__ == '__main__':
    if not os.path.exists(MODELS_DIR):
        os.makedirs(MODELS_DIR)
    app.run(host='0.0.0.0', port=5000)
