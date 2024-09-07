# app.py

from flask import Flask
from flask_cors import CORS
from routes import routes_blueprint

# Initialize Flask app
app = Flask(__name__)

# Middleware
CORS(app)  # Enable CORS
app.config['JSONIFY_PRETTYPRINT_REGULAR'] = True

# Use the routes from the routes module
app.register_blueprint(routes_blueprint)

