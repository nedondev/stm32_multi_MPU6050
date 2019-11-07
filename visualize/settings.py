#settings.py
from dotenv import load_dotenv
import os
load_dotenv()
SERIAL_PORT = os.getenv("SERIAL_PORT")
IP_ADDRESS = os.getenv("IP_ADDRESS")
PORT = int(os.getenv("PORT"))
