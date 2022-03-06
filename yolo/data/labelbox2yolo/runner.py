import git
import labelbox_json2yolo as l2j
import os
import shutil
import random

if __name__ == '__main__':
    batchsize = 8
    epochs = 1
    train_size = 200
    name_of_json = "export-2022-02-27T00_24_04.064Z"



    
    PATH = os.path.abspath(os.getcwd())
    
    
    git.Git().clone("https://github.com/ultralytics/yolov5")
    l2j.convert(name_of_json + ".json")
    os.mkdir("datasets")
    
    shutil.move(PATH + "/"+name_of_json + "/images", PATH + "/datasets/images")
    shutil.move(PATH + "/"+ name_of_json+"/labels", PATH + "/datasets/labels")
    os.rename(PATH+ "/"+name_of_json+"/" + name_of_json+".yaml",PATH+"/yolov5/data/"+ name_of_json+ ".yaml")
    os.rmdir(name_of_json)
    os.mkdir(PATH+"/datasets/images/train")
    os.mkdir(PATH+"/datasets/images/val")
    os.mkdir(PATH+"/datasets/labels/train")
    os.mkdir(PATH+"/datasets/labels/val")
    
    total_samples = len(os.listdir('datasets/images'))-2
    x = os.listdir('datasets/images')
    y = os.listdir('datasets/labels')
    print(x)
    
    
    val_size = total_samples-train_size
    random_sample_train = random.sample(range(0, total_samples-1), train_size)
    random_sample_val = []
    for i in range(total_samples):
        if(not(i in random_sample_train)):
            random_sample_val.append(i)
    
    
    for i in random_sample_train:
        
        root = x[i][:-4]
        if not root.isdigit(): continue
        shutil.copyfile(PATH+ "/datasets/images/" + root+'.jpg',PATH+"/datasets/images/train/" + root+'.jpg')
        try:
           shutil.copyfile(PATH+ "/datasets/labels/" + root+'.txt',PATH+"/datasets/labels/train/" + root+'.txt')
        except(FileNotFoundError):
            continue
        

    for i in random_sample_val:
        root = x[i][:-4]
        if not root.isdigit(): continue
        shutil.copyfile(PATH+ "/datasets/images/" + root+'.jpg',PATH+"/datasets/images/val/" + root+'.jpg')
        try:
            shutil.copyfile(PATH+ "/datasets/labels/" + root+'.txt',PATH+"/datasets/labels/val/" + root+'.txt')
        except(FileNotFoundError):
            continue
    
    
    
    os.system('python3 ' + PATH + '/yolov5/train.py --img 640 --batch ' + str(batchsize) + ' --epochs ' + str(epochs) + ' --data ' + PATH + 
              "/yolov5/data/"+name_of_json+ ".yaml" + " --weights yolov5s.pt --cache")

    #get best.pt from the yolo folder, delete the yolo folder
    os.rename(PATH + "/yolov5/runs/train/exp/weights/best.pt", PATH + "/best.pt")
    
    shutil.rmtree(PATH + "/yolov5", ignore_errors= True)
    shutil.rmtree("datasets", ignore_errors= True)
    shutil.rmtree("wandb", ignore_errors=True)
    shutil.rmtree("__pycache__", ignore_errors= True)
    os.remove("yolov5s.pt")
    os.remove(name_of_json + ".zip")

    
    
    
    
    
