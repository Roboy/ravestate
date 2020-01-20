from flask import Flask
app = Flask(__name__)


@app.route('/')
def hello():
    # Take a free port
    # Start ravestate on the selected port
    # Render template with port filled in
    pass


if __name__ == '__main__':
    app.run()
