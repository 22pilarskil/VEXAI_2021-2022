import git
import labelbox_json2yolo as l2j
import os
import shutil
import random
import argparse



if __name__ == '__main__':

    os.environ['WANDB_DISABLED'] = 'true'
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", metavar="json", type=str)
    parser.add_argument("--out_file", metavar="out_file", type=str)
    parser.add_argument("--batch_size", metavar="batch_size", type=int, default=8)
    parser.add_argument("--epochs", metavar="epochs", type=int, default=10)
    parser.add_argument("--train_size", metavar="train_size", type=int, default=200)
    parser.add_argument("--weights", metavar="weights", type=str, default="yolov5s.pt")
    args = parser.parse_args()


    name_of_json = args.json
    out_file = args.out_file
    batch_size = args.batch_size
    epochs = args.epochs
    train_size = args.train_size
    weights = args.weights

    
    PATH = os.path.abspath(os.getcwd())

    try: 
        
        git.Git().clone("https://github.com/ultralytics/yolov5")
        l2j.convert(name_of_json + ".json")
        os.mkdir("datasets")
        
        shutil.move(PATH + "/" + name_of_json + "/images", PATH + "/datasets/images")
        shutil.move(PATH + "/" + name_of_json + "/labels", PATH + "/datasets/labels")
        os.rename(PATH + "/" + name_of_json + "/" + name_of_json + ".yaml", PATH + "/yolov5/data/" + name_of_json + ".yaml")
        os.rmdir(name_of_json)
        os.mkdir(PATH + "/datasets/images/train")
        os.mkdir(PATH + "/datasets/images/val")
        os.mkdir(PATH + "/datasets/labels/train")
        os.mkdir(PATH + "/datasets/labels/val")
        
        total_samples = len(os.listdir('datasets/images')) - 2
        x = os.listdir('datasets/images')
        y = os.listdir('datasets/labels')
        print(x)
        
        
        val_size = total_samples - train_size
        random_sample_train = random.sample(range(0, total_samples - 1), train_size)
        random_sample_val = []
        for i in range(total_samples):
            if (not (i in random_sample_train)):
                random_sample_val.append(i)
        
        
        for i in random_sample_train:
            
            root = x[i][:-4]
            if not root.isdigit(): continue
            shutil.copyfile(PATH + "/datasets/images/" + root + '.jpg', PATH + "/datasets/images/train/" + root + '.jpg')
            try:
               shutil.copyfile(PATH + "/datasets/labels/" + root + '.txt', PATH + "/datasets/labels/train/" + root + '.txt')
            except(FileNotFoundError):
                continue
            

        for i in random_sample_val:
            root = x[i][:-4]
            if not root.isdigit(): continue
            shutil.copyfile(PATH + "/datasets/images/" + root + '.jpg',PATH + "/datasets/images/val/" + root + '.jpg')
            try:
                shutil.copyfile(PATH + "/datasets/labels/" + root + '.txt',PATH + "/datasets/labels/val/" + root + '.txt')
            except(FileNotFoundError):
                continue
        
        
        
        os.system('python3 ' + PATH + '/yolov5/train.py --img 640 --batch ' + str(batch_size) + ' --epochs ' + str(epochs) + ' --data ' + PATH + 
                  '/yolov5/data/' + name_of_json + '.yaml --weights ' + weights + ' --cache -3')

        #get best.pt from the yolo folder, delete the yolo folder
        os.rename(PATH + "/yolov5/runs/train/exp/weights/best.pt", PATH + "/../models/weights/" + out_file)

    except Exception as e:
        print(e)
    

    shutil.rmtree(PATH + "/yolov5", ignore_errors=True)
    shutil.rmtree("datasets", ignore_errors=True)
    shutil.rmtree("wandb", ignore_errors=True)
    shutil.rmtree("__pycache__", ignore_errors=True)
    if os.path.exists(weights): os.remove(weights)
    if os.path.exists(name_of_json + ".zip"): os.remove(name_of_json + ".zip")
    
    
    
    
    
