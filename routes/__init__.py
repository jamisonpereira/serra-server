# routes/__init__.py

from flask import Blueprint
from routes.auth import auth_blueprint
from routes.resupply import resupply_blueprint
# from routes.drone import drone_blueprint  # Import drone routes

# Create a Blueprint for all routes
routes_blueprint = Blueprint('routes', __name__)

# Register routes
routes_blueprint.register_blueprint(auth_blueprint, url_prefix='/auth')
routes_blueprint.register_blueprint(resupply_blueprint, url_prefix='/resupply')
# routes_blueprint.register_blueprint(drone_blueprint, url_prefix='/drone')  # Register drone routes