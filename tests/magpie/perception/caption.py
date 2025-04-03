'''
@file lm_scan.py
@brief LLM wrapper to retrieve object classification
'''

import os
import requests
import base64
from openai import OpenAI
from PIL import Image
from pillow_heif import register_heif_opener
import numpy as np

class Caption:
    def __init__(self):
        self.api_key = os.environ.get('LLM_API_KEY')
        self.openai = OpenAI(self.api_key)
    
    def encode_image(self, image_path):
        with open(f"{image_path}", "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
