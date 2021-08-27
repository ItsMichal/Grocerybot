from flask import Flask, render_template, request, redirect, url_for, jsonify
app = Flask(__name__)

ready=False
status="Waiting..."
shoplist = []
hint = ""

@app.route('/')
def index():
    global shoplist, status, hint
    print(shoplist) 
    r = render_template('index.html', status=status, hint=hint)
    hint = ""
    return r

@app.route('/await')
def waiting_room():
    global ready
    if ready:
        ready = False
        return jsonify(True)
    else:
        return jsonify(ready)

@app.route('/list')
def shop_list():
    global shoplist

    return jsonify(shoplist)

@app.route('/shop', methods=['GET', 'POST'])
def parse_list():
    global shoplist, hint, status, ready
    shoplist = []
    for item in request.form:
        shoplist.append(item)
    
    ready = True

    if len(request.form) > 0:
        hint = "Sending to robot..."
        return redirect(url_for('index'))
    else:
        hint = "You've got to select something!"
        return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0", port="80")