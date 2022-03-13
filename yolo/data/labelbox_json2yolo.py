import json
import os
from pathlib import Path

import requests
import yaml
from PIL import Image
from tqdm import tqdm
import labelbox
from utils import make_dirs


def convert(file, zip=True):
    # Convert Labelbox JSON labels to YOLO labels
    
    names = []  # class names
    file = Path(file)
    save_dir = make_dirs(file.stem)
    with open(file) as f:
        data = json.load(f)  # load JSON
    
    for img in tqdm(data, desc=f'Converting {file}'):
        im_path = img['Labeled Data']
        im = Image.open(requests.get(im_path, stream=True).raw if im_path.startswith('http') else im_path)  # open
        width, height = im.size  # image size
        label_path = save_dir / 'labels' / Path(img['External ID']).with_suffix('.txt').name
        image_path = save_dir / 'images' / img['External ID']
        im.save(image_path, quality=95, subsampling=0)
        if(img['Skipped'] == True):
            continue
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

    # Save dataset.yaml
    d = {'path': f"../datasets/",
         'train': "images/train",
         'val': "images/val",
         'test': "",
         'nc': len(names),
         'names': names}  # dictionary

    with open(save_dir / file.with_suffix('.yaml').name, 'w') as f:
        yaml.dump(d, f, sort_keys=False)

    # Zip
    if zip:
        print(f'Zipping as {save_dir}.zip...')
        os.system(f'zip -qr {save_dir}.zip {save_dir}')

    print('Conversion completed successfully!')

