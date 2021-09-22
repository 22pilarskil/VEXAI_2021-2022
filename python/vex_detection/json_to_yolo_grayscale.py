import json
import os
from pathlib import Path
import glob
import shutil

import numpy as np
from PIL import ExifTags
from tqdm import tqdm
import requests
import yaml
from PIL import Image
from tqdm import tqdm
import cv2


def make_dirs(dir='new_dir/'):
    # Create folders
    dir = Path(dir)
    if dir.exists():
        shutil.rmtree(dir)  # delete dir
    for p in dir, dir / 'labels', dir / 'images':
        p.mkdir(parents=True, exist_ok=True)  # make dir
    return dir
    
def convert(file, zip=True):
    # Convert Labelbox JSON labels to YOLO labels
    names = []  # class names
    file = Path(file)
    save_dir = make_dirs("train_data_grayscale")
    with open(file) as f:
        data = json.load(f)  # load JSON

    for img in tqdm(data, desc=f'Converting {file}'):
        im_path = img['Labeled Data']
        im = Image.open(requests.get(im_path, stream=True).raw if im_path.startswith('http') else im_path)  # open
        im = cv2.cvtColor(np.asarray(im), cv2.COLOR_BGR2GRAY)
        im = Image.fromarray(im)
        width, height = im.size  # image size
        label_path = save_dir / 'labels' / Path(img['External ID']).with_suffix('.txt').name
        image_path = save_dir / 'images' / img['External ID']
        im.save(image_path, quality=95, subsampling=0)
        try:
            for label in img['Label']['objects']:
                # box
                top, left, h, w = label['bbox'].values()  # top, left, height, width
                xywh = [(left + w / 2) / width, (top + h / 2) / height, w / width, h / height]  # xywh normalized

                # class
                cls = label['value']  # class name
                if cls not in names:
                    names.append(cls)

                line = names.index(cls), *xywh  # YOLO format (class_index, xywh)
                with open(label_path, 'a') as f:
                    f.write(('%g ' * len(line)).rstrip() % line + '\n')
        except:
            with open(label_path, 'a') as f:
                    f.write("\n")

    # Save dataset.yaml


    # Zip
    if zip:
        print(f'Zipping as {save_dir}.zip...')
        os.system(f'zip -qr {save_dir}.zip {save_dir}')

    print('Conversion completed successfully!')


if __name__ == '__main__':
    convert('train_data.json')
