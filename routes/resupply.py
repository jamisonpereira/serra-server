# routes/resupply.py

from flask import Blueprint
from controllers.resupplyController import request_resupply
from middleware.authenticate import authenticate_token

# Create a Blueprint for resupply routes
resupply_blueprint = Blueprint('resupply', __name__)

# Protect the route with the authentication middleware
resupply_blueprint.route('/request', methods=['POST'])(authenticate_token(request_resupply))
