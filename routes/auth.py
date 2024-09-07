# routes/auth.py

from flask import Blueprint
from controllers.authController import login, register, logout

# Create a Blueprint for authentication routes
auth_blueprint = Blueprint('auth', __name__)

# Define authentication routes
auth_blueprint.route('/login', methods=['POST'])(login)
auth_blueprint.route('/register', methods=['POST'])(register)
auth_blueprint.route('/logout', methods=['POST'])(logout)
