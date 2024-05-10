from PIL import Image
import io
import base64
import numpy as np
from lab_utils.plan_utils import *

def resize_img(source, scale = 1):
    image = source
    width, height = image.size
    resized_image = image.resize((int(width * scale), int(height * scale)),  Image.LANCZOS)
    return resized_image

def img_to_bytes(img):
    # encode a PNG formatted version of image into BASE64
    with io.BytesIO() as bio:
        img.save(bio, format="PNG")
        contents = bio.getvalue()
        encoded = base64.b64encode(contents)
    
    return encoded

def bytes_to_img(bytes_data):
    data = bytes_data.decode('utf-8')
    msg = base64.b64decode(data)
    buf = io.BytesIO(msg)
    img = Image.open(buf)
    return img

def get_map_bmp(map_img):
    image = map_img.convert(mode="1")
    map_img = 1-np.array(image)  
    map_bmp = BMPMap(width=map_img.shape[1], height=map_img.shape[0], mat=map_img)
    return map_bmp
